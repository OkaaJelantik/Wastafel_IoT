/**
 * @file main.cpp
 * @brief Wastafel_IoT — IoT Smart Sink Controller
 *
 * All business logic, state machines, and flow control live in this file
 * for academic evaluation clarity.
 *
 * System Overview:
 *   ┌─────────────────────────────────────────────────────┐
 *   │  SENSOR LAYER  │  HC-SR04 A (Hands) + B (Water)    │
 *   │  ACTUATOR      │  Relay → Water Pump                │
 *   │  DISPLAY       │  1602 LCD via I2C (PCF8574)        │
 *   │  INPUT         │  Touch Pad (GPIO4) → Refill Mode   │
 *   │  IoT           │  MQTT (telemetry) + Blynk (logger) │
 *   └─────────────────────────────────────────────────────┘
 *
 * State Machine:
 *   NORMAL   → Hand < 15cm → pump ON; Hand > 22cm + debounce → pump OFF
 *   CRITICAL → Water level below threshold → hard lock pump OFF
 *   REFILL   → Touch triggered → pump locked OFF, sensor detached
 *
 * Target: ESP32 DevKit V1
 * Framework: ESP-IDF v5.x (C++)
 */

#include <cstdio>
#include <cstring>
#include <cmath>

/* ── FreeRTOS ──────────────────────────────────────────────────────── */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ── ESP-IDF System ────────────────────────────────────────────────── */
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"

/* ── Networking ────────────────────────────────────────────────────── */
#include "esp_wifi.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_http_client.h"

/* ── Peripheral Drivers ────────────────────────────────────────────── */
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/touch_pad.h"


/* ── Project Components ────────────────────────────────────────────── */
#include "hcsr04.h"
#include "i2c_lcd.h"
#include "include/pins.h"

static const char *TAG = "WASTAFEL";

/* ═══════════════════════════════════════════════════════════════════
 *  COMPILE-TIME CONFIGURATION CONSTANTS
 * ═══════════════════════════════════════════════════════════════════ */

// ── Hand Detection Thresholds (cm) ──
static constexpr float HAND_ON_THRESHOLD    = 15.0f;   // Pump ON  when < 15cm
static constexpr float HAND_OFF_THRESHOLD   = 22.0f;   // Pump OFF when > 22cm (hysteresis)
static constexpr uint32_t HAND_OFF_TIMEOUT_MS = 500;   // 500ms hand-away timeout
static uint32_t s_hand_away_start_ms = 0;

// ── Water Level Thresholds (cm) ──
// NOTE: Distance is measured from sensor to water surface.
//       LOWER distance = HIGHER water level.
static constexpr float WATER_CRITICAL_CM    = 16.0f;   // Critical: water too low (far from sensor)
static constexpr float WATER_LOW_CM         = 12.0f;   // Warning threshold
static constexpr float TANK_AREA_CM2       = 56.72f;  // Tank surface area

// ── Stabilization Hardening ──
static constexpr float STABILITY_THRESHOLD_CM = 0.15f; // Max ripple spread allowed
static constexpr uint32_t STABILITY_MIN_MS     = 1000; // Min wait for ripples to subside
static constexpr uint32_t STABILITY_MAX_MS     = 5000; // Max wait before forcing result

// ── Moving Average Filter ──
static constexpr int   WATER_MA_WINDOW      = 10;      // Number of samples for smoothing

// ── Touch Pad ──
static constexpr uint32_t TOUCH_THRESHOLD   = 400;     // Touch detection threshold

// ── Timing (ms) ──
static constexpr int   SENSOR_READ_INTERVAL_MS  = 100;  // Sensor read period
static constexpr int   LCD_UPDATE_INTERVAL_MS   = 500;  // LCD refresh period
static constexpr int   MQTT_PUBLISH_INTERVAL_MS = 2000; // MQTT telemetry period
static constexpr int   BLYNK_PUSH_INTERVAL_MS   = 5000; // Blynk push period

// ── Offline Buffer ──
static constexpr int   OFFLINE_BUFFER_SIZE  = 30;       // Max buffered telemetry entries

// ── WiFi Event Bits ──
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

/* ═══════════════════════════════════════════════════════════════════
 *  SYSTEM STATE ENUMERATIONS
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief Operating mode of the smart sink
 */
enum class SystemState {
    STATE_IDLE,         // Ready, waiting for hands
    STATE_PUMPING,      // Hand detected, pump active
    STATE_STABILIZING,  // Waiting for water surface to settle
    STATE_CALCULATING,  // Showing "Calculating..."
    STATE_SHOW_RESULT,  // Showing final dispensed mL
    STATE_CRITICAL,     // Water low
    STATE_REFILL        // Manual refill mode
};

/**
 * @brief Type of volume change action
 */
enum class VolumeAction {
    NONE,
    DISPENSED,          // Water pumped out
    ADDED               // Water refilled
};

/* ═══════════════════════════════════════════════════════════════════
 *  MOVING AVERAGE FILTER
 *  Smooths water level readings to prevent LCD flicker
 * ═══════════════════════════════════════════════════════════════════ */

class MovingAverage {
public:
    MovingAverage() : _index(0), _count(0), _sum(0.0f) {
        memset(_buffer, 0, sizeof(_buffer));
    }

    /**
     * @brief Add a new sample and return the smoothed average
     * @param value New sensor reading
     * @return Smoothed value (average of last WATER_MA_WINDOW samples)
     */
    float add(float value) {
        // Subtract the oldest value from the running sum
        _sum -= _buffer[_index];
        // Insert the new value
        _buffer[_index] = value;
        _sum += value;
        // Advance circular index
        _index = (_index + 1) % WATER_MA_WINDOW;
        // Track how many valid samples we have (up to window size)
        if (_count < WATER_MA_WINDOW) _count++;
        return _sum / static_cast<float>(_count);
    }

    float get() const {
        if (_count == 0) return 0.0f;
        return _sum / static_cast<float>(_count);
    }

    /**
     * @brief Calculate the difference between max and min in the current window
     * High spread = rippling/unstable surface.
     */
    float get_spread() const {
        if (_count == 0) return 0.0f;
        float min_val = _buffer[0];
        float max_val = _buffer[0];
        for (int i = 1; i < _count; i++) {
            if (_buffer[i] < min_val) min_val = _buffer[i];
            if (_buffer[i] > max_val) max_val = _buffer[i];
        }
        return max_val - min_val;
    }

    /**
     * @brief Check if the current reading is stable (low ripples)
     * @param threshold_cm Max allowed spread in cm
     */
    bool is_stable(float threshold_cm) const {
        if (_count < WATER_MA_WINDOW) return false; // Need a full window
        return get_spread() < threshold_cm;
    }

private:
    float _buffer[WATER_MA_WINDOW];
    int   _index;
    int   _count;
    float _sum;
};

/* ═══════════════════════════════════════════════════════════════════
 *  OFFLINE TELEMETRY BUFFER
 *  Stores sensor data when WiFi is unavailable for later sync
 * ═══════════════════════════════════════════════════════════════════ */

struct TelemetryEntry {
    float hand_distance;
    float water_level;
    bool  pump_on;
    int   state;           // Cast of SystemState
    int64_t timestamp_ms;  // Uptime in ms
};

static TelemetryEntry s_offline_buffer[OFFLINE_BUFFER_SIZE];
static int s_offline_head = 0;   // Next write position
static int s_offline_count = 0;  // Number of buffered entries

/**
 * @brief Buffer a telemetry entry for later transmission
 */
static void buffer_telemetry(float hand_dist, float water_dist, bool pump,
                             SystemState state) {
    TelemetryEntry &e = s_offline_buffer[s_offline_head];
    e.hand_distance = hand_dist;
    e.water_level   = water_dist;
    e.pump_on       = pump;
    e.state         = static_cast<int>(state);
    e.timestamp_ms  = esp_log_timestamp();

    s_offline_head = (s_offline_head + 1) % OFFLINE_BUFFER_SIZE;
    if (s_offline_count < OFFLINE_BUFFER_SIZE) s_offline_count++;
}

/* ═══════════════════════════════════════════════════════════════════
 *  GLOBAL STATE VARIABLES
 * ═══════════════════════════════════════════════════════════════════ */

// ── Hardware Instances ──
static HCSR04  *s_hand_sensor  = nullptr;
static HCSR04  *s_water_sensor = nullptr;
static I2C_LCD *s_lcd          = nullptr;

// ── System State ──
static SystemState s_state      = SystemState::STATE_IDLE;
static bool        s_pump_on    = false;

// ── Volume & Timing ──
static float        s_dispensed_ml     = 0.0f;
static float        s_initial_water_cm = 0.0f; // Level at start of pumping
static uint32_t     s_calc_finish_ms   = 0;
static VolumeAction s_last_action      = VolumeAction::NONE;
static constexpr uint32_t CALC_DISPLAY_MS  = 3000;  // Show result for 3s

// ── Sensor Readings ──
static float s_hand_distance_cm  = -1.0f;   // Raw hand sensor
static float s_water_distance_cm = -1.0f;   // Smoothed water level
static float s_current_vol_ml    = 0.0f;    // Shared volume source of truth
static MovingAverage s_water_filter;

// ── Network State ──
static EventGroupHandle_t s_wifi_event_group = nullptr;
static esp_mqtt_client_handle_t s_mqtt_client = nullptr;
static bool s_wifi_connected = false;
static bool s_mqtt_connected = false;

// ── Touch State ──
static bool s_touch_pressed = false;

/* ═══════════════════════════════════════════════════════════════════
 *  WiFi INITIALIZATION & EVENT HANDLER
 * ═══════════════════════════════════════════════════════════════════ */

static int s_wifi_retry_count = 0;
static constexpr int WIFI_MAX_RETRY = 10;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            s_wifi_connected = false;
            s_mqtt_connected = false;
            if (s_wifi_retry_count < WIFI_MAX_RETRY) {
                esp_wifi_connect();
                s_wifi_retry_count++;
                ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d",
                         s_wifi_retry_count, WIFI_MAX_RETRY);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "WiFi connection failed after %d retries",
                         WIFI_MAX_RETRY);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
        nullptr, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler,
        nullptr, &instance_got_ip));

    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid,
            CONFIG_WASTAFEL_WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password,
            CONFIG_WASTAFEL_WIFI_PASSWORD,
            sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA init complete, connecting to '%s'...",
             CONFIG_WASTAFEL_WIFI_SSID);
}

/* ═══════════════════════════════════════════════════════════════════
 *  MQTT CLIENT
 * ═══════════════════════════════════════════════════════════════════ */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected to broker");
        s_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type: %d", event->error_handle->error_type);
        break;
    default:
        break;
    }
}

static void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = CONFIG_WASTAFEL_MQTT_BROKER_URI;

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client,
                                   (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, nullptr);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(TAG, "MQTT client started → %s", CONFIG_WASTAFEL_MQTT_BROKER_URI);
}

/**
 * @brief Publish telemetry JSON to MQTT topic
 *
 * Topic format: wastafel/{device_id}/telemetry
 * Payload: JSON with hand_cm, water_cm, pump, state
 */
static void mqtt_publish_telemetry(float hand_cm, float water_cm,
                                   bool pump, SystemState state) {
    if (!s_mqtt_connected) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "wastafel/%s/telemetry",
             CONFIG_WASTAFEL_DEVICE_ID);

    const char *state_str;
    switch (state) {
    case SystemState::STATE_IDLE:        state_str = "IDLE";        break;
    case SystemState::STATE_PUMPING:     state_str = "PUMPING";     break;
    case SystemState::STATE_CALCULATING: state_str = "CALCULATING"; break;
    case SystemState::STATE_CRITICAL:    state_str = "CRITICAL";    break;
    case SystemState::STATE_REFILL:      state_str = "REFILL";      break;
    default:                             state_str = "UNKNOWN";     break;
    }

    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"hand_cm\":%.1f,\"water_cm\":%.1f,"
             "\"pump\":%s,\"state\":\"%s\",\"uptime_ms\":%lld}",
             hand_cm, water_cm,
             pump ? "true" : "false",
             state_str,
             (long long)esp_log_timestamp());

    esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);
}

/**
 * @brief Flush offline buffer by publishing all buffered entries via MQTT
 */
static void mqtt_flush_offline_buffer(void) {
    if (!s_mqtt_connected || s_offline_count == 0) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "wastafel/%s/buffered",
             CONFIG_WASTAFEL_DEVICE_ID);

    // Calculate the start index of buffered data
    int start = (s_offline_head - s_offline_count + OFFLINE_BUFFER_SIZE)
                % OFFLINE_BUFFER_SIZE;

    int flushed = 0;
    for (int i = 0; i < s_offline_count; i++) {
        int idx = (start + i) % OFFLINE_BUFFER_SIZE;
        TelemetryEntry &e = s_offline_buffer[idx];

        char payload[256];
        snprintf(payload, sizeof(payload),
                 "{\"hand_cm\":%.1f,\"water_cm\":%.1f,"
                 "\"pump\":%s,\"state\":%d,\"ts_ms\":%lld}",
                 e.hand_distance, e.water_level,
                 e.pump_on ? "true" : "false",
                 e.state, (long long)e.timestamp_ms);

        if (esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 0, 0) >= 0) {
            flushed++;
        }
        // Small delay between publishes to avoid overwhelming broker
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Flushed %d/%d buffered entries", flushed, s_offline_count);
    s_offline_count = 0;
    s_offline_head  = 0;
}

/* ═══════════════════════════════════════════════════════════════════
 *  BLYNK HTTP API (Data Logger)
 *  Secondary IoT platform with offline sync capabilities
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief Push sensor data to Blynk via HTTP API
 *
 * Virtual Pins:
 *   V0 = Hand distance (cm)
 *   V1 = Water level distance (cm)
 *   V2 = Pump state (0/1)
 *   V3 = System state (0=Normal, 1=Critical, 2=Refill)
 */
static void blynk_push_data(float hand_cm, float water_cm,
                            bool pump, SystemState state) {
    // Skip if Blynk auth token is not configured
    const char *auth = CONFIG_WASTAFEL_BLYNK_AUTH;
    if (auth[0] == '\0' || !s_wifi_connected) return;

    char url[256];
    snprintf(url, sizeof(url),
             "https://%s/external/api/batch/update"
             "?token=%s&V0=%.1f&V1=%.1f&V2=%d&V3=%d",
             CONFIG_WASTAFEL_BLYNK_SERVER, auth,
             hand_cm, water_cm,
             pump ? 1 : 0,
             static_cast<int>(state));

    esp_http_client_config_t http_cfg = {};
    http_cfg.url = url;
    http_cfg.method = HTTP_METHOD_GET;
    http_cfg.timeout_ms = 3000;

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (client) {
        esp_err_t err = esp_http_client_perform(client);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Blynk push failed: %s", esp_err_to_name(err));
        }
        esp_http_client_cleanup(client);
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  HARDWARE INITIALIZATION
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the relay (pump) GPIO as output, initially OFF
 */
static void relay_init(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << RELAY_PIN);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(RELAY_PIN, 0);
    ESP_LOGI(TAG, "Relay (pump) initialized on GPIO%d — OFF", RELAY_PIN);
}

/**
 * @brief Set the pump relay state
 * @param on true = pump ON, false = pump OFF
 */
static void pump_set(bool on) {
    s_pump_on = on;
    gpio_set_level(RELAY_PIN, on ? 1 : 0);
}

/**
 * @brief Initialize the capacitive touch pad for refill mode trigger
 */
static void touch_init(void) {
    ESP_LOGI(TAG, "Initializing legacy touch_pad on GPIO%d (PAD%d)",
             TOUCH_REFILL_GPIO, TOUCH_REFILL_PAD);
             
    touch_pad_init();
    touch_pad_config((touch_pad_t)TOUCH_REFILL_PAD, 0);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
}

/**
 * @brief Read touch pad and return true if touched
 */
static bool touch_is_pressed(void) {
    uint16_t val;
    touch_pad_read((touch_pad_t)TOUCH_REFILL_PAD, &val);
    // Log occasionally for calibration if needed
    // ESP_LOGD(TAG, "Touch: %d", val);
    return (val < 400); 
}

/**
 * @brief Initialize I2C bus and LCD display
 */
static void lcd_init(void) {
    // Create I2C master bus
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port     = I2C_NUM_0;
    bus_cfg.sda_io_num   = LCD_SDA_PIN;
    bus_cfg.scl_io_num   = LCD_SCL_PIN;
    bus_cfg.clk_source   = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // Create LCD instance and initialize
    s_lcd = new I2C_LCD(bus, LCD_I2C_ADDR);
    ESP_ERROR_CHECK(s_lcd->init());

    // Show boot message
    s_lcd->clear();
    s_lcd->set_cursor(0, 0);
    s_lcd->print("Wastafel IoT");
    s_lcd->set_cursor(0, 1);
    s_lcd->print("Booting...");
    ESP_LOGI(TAG, "LCD initialized on I2C (SDA=%d, SCL=%d, addr=0x%02X)",
             LCD_SDA_PIN, LCD_SCL_PIN, LCD_I2C_ADDR);
}

/**
 * @brief Initialize both HC-SR04 ultrasonic sensors
 *
 * Sensor A (Hands) uses MCPWM Group 0
 * Sensor B (Water) uses MCPWM Group 1
 * This avoids resource conflicts between the two sensors.
 */
static void sensors_init(void) {
    s_hand_sensor = new HCSR04(HAND_TRIG_PIN, HAND_ECHO_PIN, 0);
    ESP_ERROR_CHECK(s_hand_sensor->init());
    ESP_LOGI(TAG, "Hand sensor initialized (Trig=%d, Echo=%d, Group=0)",
             HAND_TRIG_PIN, HAND_ECHO_PIN);

    s_water_sensor = new HCSR04(WATER_TRIG_PIN, WATER_ECHO_PIN, 1);
    ESP_ERROR_CHECK(s_water_sensor->init());
    ESP_LOGI(TAG, "Water sensor initialized (Trig=%d, Echo=%d, Group=1)",
             WATER_TRIG_PIN, WATER_ECHO_PIN);
}

/* ═══════════════════════════════════════════════════════════════════
 *  TASK: SENSOR READING (Core 0)
 *  Reads both ultrasonic sensors at regular intervals.
 *  Applies moving average filter to water level readings.
 * ═══════════════════════════════════════════════════════════════════ */

static void sensor_task(void *pvParams) {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        /* ── 1. Read Hand Sensor ────────────────────────────────── */
        if (s_hand_sensor) {
            s_hand_sensor->trigger();
            vTaskDelay(pdMS_TO_TICKS(40)); // Wait for echo
            float hand_raw = s_hand_sensor->get_distance_cm();
            if (hand_raw > 0) {
                s_hand_distance_cm = hand_raw;
                ESP_LOGD(TAG, "Hand: %.1f cm", hand_raw);
            }
        }

        /* ── 2. Read Water Sensor ────────────────────────────────── */
        if (s_water_sensor) {
            s_water_sensor->trigger();
            vTaskDelay(pdMS_TO_TICKS(30)); // Wait for echo
            float water_raw = s_water_sensor->get_distance_cm();
            if (water_raw > 0) {
                // Apply moving average filter to suppress noise & LCD flicker
                s_water_distance_cm = s_water_filter.add(water_raw);
                
                // Calculate current volume centrally to ensure consistency
                float h = (WATER_CRITICAL_CM - s_water_distance_cm);
                if (h < 0) h = 0;
                s_current_vol_ml = h * TANK_AREA_CM2;
            }
        }

        /* ── 3. Read touch pad ────────────────────────────────────── */
        s_touch_pressed = touch_is_pressed();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  TASK: CONTROL LOGIC (Core 0)
 *  State machine for pump control and mode management.
 *
 *  State Transitions:
 *    NORMAL → CRITICAL : water_cm > WATER_CRITICAL_CM
 *    NORMAL → REFILL   : touch pressed
 *    CRITICAL → NORMAL : water_cm < WATER_LOW_CM
 *    REFILL → NORMAL   : touch pressed again (toggle)
 *    Any → CRITICAL    : always override if water critically low
 * ═══════════════════════════════════════════════════════════════════ */

static void control_task(void *pvParams) {
    TickType_t last_wake = xTaskGetTickCount();
    bool prev_touch = false;

    while (true) {
        bool touch_now = s_touch_pressed;
        bool touch_rising = (touch_now && !prev_touch);
        prev_touch = touch_now;

        switch (s_state) {

        case SystemState::STATE_IDLE:
            pump_set(false);
            if (s_water_distance_cm > WATER_CRITICAL_CM && s_water_distance_cm > 0) {
                s_initial_water_cm = s_water_distance_cm; 
                s_state = SystemState::STATE_REFILL;
                ESP_LOGW(TAG, "→ REFILL: Water critical, auto-entering refill mode");
                break;
            }
            if (touch_rising) {
                s_initial_water_cm = s_water_distance_cm;
                s_state = SystemState::STATE_REFILL;
                break;
            }
            // Hand detection starts pumping
            if (s_hand_distance_cm > 0 && s_hand_distance_cm < HAND_ON_THRESHOLD) {
                // Use currently smoothed moving average as baseline
                s_initial_water_cm = s_water_filter.get(); 
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
            }
            break;

        case SystemState::STATE_PUMPING:
            // CRITICAL LOCKOUT: Immediately interrupt if water level too low
            if (s_water_distance_cm > WATER_CRITICAL_CM || s_water_distance_cm <= 0) {
                pump_set(false);
                s_initial_water_cm = s_water_distance_cm;
                s_state = SystemState::STATE_REFILL;
                ESP_LOGE(TAG, "→ REFILL: Water critical during pump, auto-entering refill mode");
                break;
            }
            // Hand removal starts stabilization
            if (s_hand_distance_cm > HAND_OFF_THRESHOLD || s_hand_distance_cm < 0) {
                if (s_hand_away_start_ms == 0) {
                    s_hand_away_start_ms = esp_log_timestamp(); // Start timer
                } else if ((esp_log_timestamp() - s_hand_away_start_ms) > HAND_OFF_TIMEOUT_MS) {
                    pump_set(false);
                    s_calc_finish_ms = esp_log_timestamp();
                    s_last_action = VolumeAction::DISPENSED;
                    s_state = SystemState::STATE_STABILIZING;
                    s_hand_away_start_ms = 0;
                }
            } else {
                s_hand_away_start_ms = 0; // Hand re-detected, reset timer
            }
            break;

        case SystemState::STATE_STABILIZING:
            if (s_hand_distance_cm > 0 && s_hand_distance_cm < HAND_ON_THRESHOLD) {
                s_state = SystemState::STATE_PUMPING;
                pump_set(true);
                break;
            }
            
            {
                uint32_t elapsed = esp_log_timestamp() - s_calc_finish_ms;
                bool is_stable = s_water_filter.is_stable(STABILITY_THRESHOLD_CM);
                
                // Adaptive finish: 
                // 1. Must wait at least MIN_MS
                // 2. Either surface is stable OR we reached MAX_MS timeout
                if (elapsed >= STABILITY_MIN_MS && (is_stable || elapsed >= STABILITY_MAX_MS)) {
                    if (elapsed >= STABILITY_MAX_MS && !is_stable) {
                        ESP_LOGW(TAG, "Stabilization timeout reached (%.2f cm spread)", 
                                 s_water_filter.get_spread());
                    }

                    // Calculate based on the centrally managed current volume
                    float final_h = (WATER_CRITICAL_CM - s_water_distance_cm);
                    float initial_h = (WATER_CRITICAL_CM - s_initial_water_cm);
                    
                    if (s_last_action == VolumeAction::DISPENSED) {
                        float used_h = initial_h - final_h;
                        // Hardening: Any used height < 0.3cm (approx 17mL) is treated as noise/zero
                        if (used_h < 0.3f) used_h = 0;
                        s_dispensed_ml = used_h * TANK_AREA_CM2;
                    } else {
                        // Level increases = distance decreases
                        float added_h = final_h - initial_h; 
                        if (added_h < 0) added_h = 0;
                        s_dispensed_ml = added_h * TANK_AREA_CM2;
                    }
                    
                    s_calc_finish_ms = esp_log_timestamp();
                    s_state = SystemState::STATE_CALCULATING;
                }
            }
            break;

        case SystemState::STATE_CALCULATING:
            if (s_hand_distance_cm > 0 && s_hand_distance_cm < HAND_ON_THRESHOLD) {
                s_state = SystemState::STATE_PUMPING;
                pump_set(true);
                break;
            }
            if ((esp_log_timestamp() - s_calc_finish_ms) > 1000) {
                s_calc_finish_ms = esp_log_timestamp();
                s_state = SystemState::STATE_SHOW_RESULT;
            }
            break;

        case SystemState::STATE_SHOW_RESULT:
            if (s_hand_distance_cm > 0 && s_hand_distance_cm < HAND_ON_THRESHOLD) {
                s_state = SystemState::STATE_PUMPING;
                pump_set(true);
                break;
            }
            if ((esp_log_timestamp() - s_calc_finish_ms) > 3000) {
                s_state = SystemState::STATE_IDLE;
            }
            break;

        case SystemState::STATE_CRITICAL:
            // This state is now bypassed by auto-transition to REFILL
            s_state = SystemState::STATE_IDLE;
            break;

        case SystemState::STATE_REFILL:
            pump_set(false);
            // Exit via touch: calculate volume added
            if (touch_rising) {
                if (s_water_distance_cm > WATER_CRITICAL_CM || s_water_distance_cm <= 0) {
                    ESP_LOGW(TAG, "Refill exit blocked: Water level still critical (%.1f cm)", s_water_distance_cm);
                } else {
                    s_calc_finish_ms = esp_log_timestamp();
                    s_last_action = VolumeAction::ADDED;
                    s_state = SystemState::STATE_STABILIZING;
                    ESP_LOGI(TAG, "→ STABILIZING: Refill finished, settling water...");
                }
            }
            break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  TASK: LCD DISPLAY (Core 0)
 *  Updates the 1602 LCD with current system status.
 *
 *  Display Layout:
 *    Row 0: Mode + Hand distance
 *    Row 1: Water level + Pump indicator
 *
 *  REFILL mode overrides with dedicated message.
 * ═══════════════════════════════════════════════════════════════════ */

static void lcd_task(void *pvParams) {
    TickType_t last_wake = xTaskGetTickCount();
    char prev_line0[17] = {0};
    char prev_line1[17] = {0};

    while (true) {
        char line0[17] = {0};
        char line1[17] = {0};

        switch (s_state) {
        case SystemState::STATE_IDLE: {
            float current_height_cm = (s_water_distance_cm > 0) ? (WATER_CRITICAL_CM - s_water_distance_cm) : 0;
            if (current_height_cm < 0) current_height_cm = 0;
            float current_vol_ml = current_height_cm * TANK_AREA_CM2;

            snprintf(line0, sizeof(line0), "SYSTEM READY    ");
            snprintf(line1, sizeof(line1), "Capacity:%4.0f mL", current_vol_ml);
            break;
        }

        case SystemState::STATE_PUMPING:
            snprintf(line0, sizeof(line0), "PUMPING...      ");
            snprintf(line1, sizeof(line1), "Dispensing Water");
            break;

        case SystemState::STATE_STABILIZING:
            snprintf(line0, sizeof(line0), "PLEASE WAIT...  ");
            snprintf(line1, sizeof(line1), "Ripple: %.2f cm", s_water_filter.get_spread());
            break;

        case SystemState::STATE_CALCULATING:
            snprintf(line0, sizeof(line0), "CALCULATING...  ");
            snprintf(line1, sizeof(line1), "Please hold on  ");
            break;

        case SystemState::STATE_SHOW_RESULT:
            if (s_last_action == VolumeAction::DISPENSED) {
                snprintf(line0, sizeof(line0), "DISPENSED:      ");
            } else {
                snprintf(line0, sizeof(line0), "ADDED:          ");
            }
            snprintf(line1, sizeof(line1), "%7.1f mL      ", s_dispensed_ml);
            break;

        case SystemState::STATE_REFILL:
            if (s_water_distance_cm > WATER_CRITICAL_CM || s_water_distance_cm <= 0) {
                snprintf(line0, sizeof(line0), "!! LOW WATER !! ");
                snprintf(line1, sizeof(line1), "PLEASE REFILL   ");
            } else {
                snprintf(line0, sizeof(line0), "REFILL MODE     ");
                snprintf(line1, sizeof(line1), "PUMP LOCKED OFF ");
            }
            break;

        case SystemState::STATE_CRITICAL: {
            snprintf(line0, sizeof(line0), "!! LOW WATER !! ");
            snprintf(line1, sizeof(line1), "REFILL REQUIRED ");
            break;
        }
        }

        /* ── Only update LCD if content changed (prevents flicker) ── */
        if (strncmp(line0, prev_line0, 16) != 0 || strncmp(line1, prev_line1, 16) != 0) {
            s_lcd->clear(); // Force clear on change to avoid artifacts
            s_lcd->set_cursor(0, 0);
            s_lcd->print(line0);
            s_lcd->set_cursor(0, 1);
            s_lcd->print(line1);
            
            strncpy(prev_line0, line0, 16);
            strncpy(prev_line1, line1, 16);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LCD_UPDATE_INTERVAL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  TASK: IoT TELEMETRY (Core 1)
 *  Publishes sensor data via MQTT and Blynk at separate intervals.
 *  Buffers data when offline, flushes when connectivity returns.
 * ═══════════════════════════════════════════════════════════════════ */

static void iot_task(void *pvParams) {
    TickType_t last_mqtt  = xTaskGetTickCount();
    TickType_t last_blynk = xTaskGetTickCount();

    // Wait for WiFi before starting IoT operations
    ESP_LOGI(TAG, "IoT task: waiting for WiFi...");
    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (s_wifi_connected) {
        mqtt_init();
        // Give MQTT time to connect
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    while (true) {
        TickType_t now = xTaskGetTickCount();

        /* ── MQTT Telemetry ──────────────────────────────────────── */
        if ((now - last_mqtt) >= pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL_MS)) {
            last_mqtt = now;

            if (s_mqtt_connected) {
                // Flush any buffered data first
                if (s_offline_count > 0) {
                    mqtt_flush_offline_buffer();
                }
                // Publish live telemetry
                mqtt_publish_telemetry(s_hand_distance_cm,
                                       s_water_distance_cm,
                                       s_pump_on, s_state);
            } else {
                // WiFi/MQTT unavailable → buffer the data
                buffer_telemetry(s_hand_distance_cm, s_water_distance_cm,
                                 s_pump_on, s_state);
                ESP_LOGD(TAG, "Buffered telemetry (%d/%d)",
                         s_offline_count, OFFLINE_BUFFER_SIZE);
            }
        }

        /* ── Blynk Data Logger ───────────────────────────────────── */
        if ((now - last_blynk) >= pdMS_TO_TICKS(BLYNK_PUSH_INTERVAL_MS)) {
            last_blynk = now;
            blynk_push_data(s_hand_distance_cm, s_water_distance_cm,
                            s_pump_on, s_state);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Yield to other tasks
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  APPLICATION ENTRY POINT
 * ═══════════════════════════════════════════════════════════════════ */

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "════════════════════════════════════════");
    ESP_LOGI(TAG, "  Wastafel IoT — Smart Sink Controller");
    ESP_LOGI(TAG, "  Target: ESP32 DevKit V1");
    ESP_LOGI(TAG, "════════════════════════════════════════");

    /* ── 1. Initialize Non-Volatile Storage (required for WiFi) ──── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. Initialize Hardware ──────────────────────────────────── */
    lcd_init();          // I2C LCD display
    sensors_init();      // HC-SR04 ultrasonic sensors (A + B)
    relay_init();        // Pump relay
    touch_init();        // Back-to-basics touch polling

    /* ── 3. Initialize WiFi (non-blocking) ───────────────────────── */
    // wifi_init_sta();

    /* ── 4. Update LCD with "Ready" message after init ───────────── */
    vTaskDelay(pdMS_TO_TICKS(500));
    s_lcd->clear();
    s_lcd->set_cursor(0, 0);
    s_lcd->print("System Ready");
    // s_lcd->set_cursor(0, 1);
    // s_lcd->print("Connecting WiFi");

    /* ── 5. Launch FreeRTOS Tasks ────────────────────────────────── */

    // Sensor task — reads HC-SR04 + touch at 10 Hz
    xTaskCreatePinnedToCore(sensor_task,  "sensor",  4096, nullptr, 5,
                            nullptr, 0);

    // Control task — runs state machine at 10 Hz
    xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, 4,
                            nullptr, 0);

    // LCD task — updates display at 2 Hz
    xTaskCreatePinnedToCore(lcd_task,     "lcd",     4096, nullptr, 3,
                            nullptr, 0);

    // IoT task — MQTT + Blynk telemetry (runs on Core 1 for network isolation)
    // xTaskCreatePinnedToCore(iot_task,     "iot",     8192, nullptr, 2,
    //                         nullptr, 1);

    ESP_LOGI(TAG, "All tasks launched — system running");
}
