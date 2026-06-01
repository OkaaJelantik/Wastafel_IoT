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
static constexpr int   HAND_DEBOUNCE_COUNT  = 3;       // Consecutive readings to confirm OFF

// ── Water Level Thresholds (cm) ──
// NOTE: Distance is measured from sensor to water surface.
//       LOWER distance = HIGHER water level.
static constexpr float WATER_CRITICAL_CM    = 25.0f;   // Critical: water too low (far from sensor)
static constexpr float WATER_LOW_CM         = 20.0f;   // Warning threshold
static constexpr float WATER_FULL_CM        = 5.0f;    // Tank full (close to sensor)

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
 *
 * STATE_NORMAL:   Standard operation — hand detection controls pump
 * STATE_CRITICAL: Water level below critical — pump hard-locked OFF
 * STATE_REFILL:   Manual refill mode — pump locked OFF, sensor detached
 */
enum class SystemState {
    STATE_NORMAL,
    STATE_CRITICAL,
    STATE_REFILL
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
static SystemState s_state      = SystemState::STATE_NORMAL;
static bool        s_pump_on    = false;
static int         s_off_debounce_count = 0;  // Hand-off debounce counter

// ── Sensor Readings ──
static float s_hand_distance_cm  = -1.0f;   // Raw hand sensor
static float s_water_distance_cm = -1.0f;   // Smoothed water level
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
    case SystemState::STATE_NORMAL:   state_str = "NORMAL";   break;
    case SystemState::STATE_CRITICAL: state_str = "CRITICAL"; break;
    case SystemState::STATE_REFILL:   state_str = "REFILL";   break;
    default:                          state_str = "UNKNOWN";  break;
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
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_REFILL_PAD, TOUCH_THRESHOLD));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));  // 10ms filter period
    ESP_LOGI(TAG, "Touch pad initialized on GPIO%d (PAD%d)",
             TOUCH_REFILL_GPIO, TOUCH_REFILL_PAD);
}

/**
 * @brief Read touch pad and return true if touched
 */
static bool touch_is_pressed(void) {
    uint16_t value = 0;
    touch_pad_read_filtered(TOUCH_REFILL_PAD, &value);
    return (value < TOUCH_THRESHOLD);
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
        /* ── 1. Trigger both sensors ─────────────────────────────── */
        s_hand_sensor->trigger();
        s_water_sensor->trigger();

        /* Wait for echo return (max ~25ms for 4m range) */
        vTaskDelay(pdMS_TO_TICKS(30));

        /* ── 2. Read hand sensor (raw) ───────────────────────────── */
        float hand_raw = s_hand_sensor->get_distance_cm();
        if (hand_raw > 0) {
            s_hand_distance_cm = hand_raw;
        }

        /* ── 3. Read water sensor (filtered via moving average) ──── */
        float water_raw = s_water_sensor->get_distance_cm();
        if (water_raw > 0) {
            // Apply moving average filter to suppress noise & LCD flicker
            s_water_distance_cm = s_water_filter.add(water_raw);
        }

        /* ── 4. Read touch pad ────────────────────────────────────── */
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

    // Debounce state for touch toggle detection
    bool prev_touch = false;

    while (true) {
        /* ── 1. Touch toggle detection (rising edge) ────────────── */
        bool touch_now = s_touch_pressed;
        bool touch_rising = (touch_now && !prev_touch);
        prev_touch = touch_now;

        /* ── 2. State Machine ────────────────────────────────────── */
        switch (s_state) {

        case SystemState::STATE_NORMAL:
            /*
             * NORMAL MODE:
             * - Check water level → transition to CRITICAL if too low
             * - Check touch → transition to REFILL
             * - Hand detection with hysteresis controls pump
             */
            if (s_water_distance_cm > WATER_CRITICAL_CM &&
                s_water_distance_cm > 0) {
                // Water critically low → hard lock pump OFF
                pump_set(false);
                s_off_debounce_count = 0;
                s_state = SystemState::STATE_CRITICAL;
                ESP_LOGW(TAG, "→ CRITICAL: Water level too low (%.1f cm)",
                         s_water_distance_cm);
                break;
            }

            if (touch_rising) {
                // Enter refill mode → lock pump OFF
                pump_set(false);
                s_off_debounce_count = 0;
                s_state = SystemState::STATE_REFILL;
                ESP_LOGI(TAG, "→ REFILL: Manual refill mode activated");
                break;
            }

            /* ── Hand detection with hysteresis & debounce ──────── */
            if (s_hand_distance_cm > 0 &&
                s_hand_distance_cm < HAND_ON_THRESHOLD) {
                // Hand detected within range → pump ON immediately
                pump_set(true);
                s_off_debounce_count = 0;
            } else if (s_hand_distance_cm > HAND_OFF_THRESHOLD ||
                       s_hand_distance_cm < 0) {
                // Hand moved away past hysteresis band → debounce OFF
                s_off_debounce_count++;
                if (s_off_debounce_count >= HAND_DEBOUNCE_COUNT) {
                    pump_set(false);
                    s_off_debounce_count = HAND_DEBOUNCE_COUNT; // Clamp
                }
            }
            // Between 15-22cm: maintain current state (hysteresis dead zone)
            break;

        case SystemState::STATE_CRITICAL:
            /*
             * CRITICAL MODE:
             * - Pump is HARD LOCKED OFF regardless of hand detection
             * - Water level must recover above LOW threshold to exit
             * - Touch can still enter REFILL mode
             */
            pump_set(false);  // Enforce pump OFF every cycle

            if (touch_rising) {
                s_state = SystemState::STATE_REFILL;
                ESP_LOGI(TAG, "→ REFILL (from CRITICAL)");
                break;
            }

            if (s_water_distance_cm > 0 &&
                s_water_distance_cm < WATER_LOW_CM) {
                // Water recovered above warning threshold
                s_state = SystemState::STATE_NORMAL;
                ESP_LOGI(TAG, "→ NORMAL: Water level recovered (%.1f cm)",
                         s_water_distance_cm);
            }
            break;

        case SystemState::STATE_REFILL:
            /*
             * REFILL MODE:
             * - Pump is LOCKED OFF (water sensor is detached during refill)
             * - Only exits via touch toggle
             * - Display shows "REFILL MODE - LOCKED"
             */
            pump_set(false);  // STRICT: Force pump OFF during refill

            if (touch_rising) {
                // Exit refill mode → return to normal
                s_state = SystemState::STATE_NORMAL;
                s_off_debounce_count = 0;
                ESP_LOGI(TAG, "→ NORMAL: Refill mode deactivated");
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

    // Buffers for previous LCD content to avoid unnecessary rewrites
    char prev_line0[17] = {0};
    char prev_line1[17] = {0};

    while (true) {
        char line0[17] = {0};
        char line1[17] = {0};

        switch (s_state) {

        case SystemState::STATE_REFILL:
            /* ── REFILL MODE: Show dedicated lockout message ─────── */
            snprintf(line0, sizeof(line0), "REFILL MODE     ");
            snprintf(line1, sizeof(line1), "PUMP LOCKED OFF ");
            break;

        case SystemState::STATE_CRITICAL:
            /* ── CRITICAL: Show warning + water reading ──────────── */
            snprintf(line0, sizeof(line0), "!! LOW WATER !! ");
            snprintf(line1, sizeof(line1), "W:%.0fcm LOCKED ",
                     s_water_distance_cm);
            break;

        case SystemState::STATE_NORMAL:
        default:
            /* ── NORMAL: Show hand/water distances + pump state ──── */
            if (s_hand_distance_cm > 0) {
                snprintf(line0, sizeof(line0), "H:%.0fcm  W:%.0fcm",
                         s_hand_distance_cm, s_water_distance_cm);
            } else {
                snprintf(line0, sizeof(line0), "H:---   W:%.0fcm",
                         s_water_distance_cm);
            }
            snprintf(line1, sizeof(line1), "Pump:%s  %s",
                     s_pump_on ? "ON " : "OFF",
                     (s_water_distance_cm > WATER_LOW_CM) ? "LOW!" : "OK  ");
            break;
        }

        /* ── Only update LCD if content changed (prevents flicker) ── */
        if (strncmp(line0, prev_line0, 16) != 0) {
            s_lcd->set_cursor(0, 0);
            s_lcd->print(line0);
            strncpy(prev_line0, line0, 16);
        }
        if (strncmp(line1, prev_line1, 16) != 0) {
            s_lcd->set_cursor(0, 1);
            s_lcd->print(line1);
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
    touch_init();        // Touch pad for refill mode

    /* ── 3. Initialize WiFi (non-blocking) ───────────────────────── */
    wifi_init_sta();

    /* ── 4. Update LCD with "Ready" message after init ───────────── */
    vTaskDelay(pdMS_TO_TICKS(500));
    s_lcd->clear();
    s_lcd->set_cursor(0, 0);
    s_lcd->print("System Ready");
    s_lcd->set_cursor(0, 1);
    s_lcd->print("Connecting WiFi");

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
    xTaskCreatePinnedToCore(iot_task,     "iot",     8192, nullptr, 2,
                            nullptr, 1);

    ESP_LOGI(TAG, "All tasks launched — system running");
}
