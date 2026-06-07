#include <cstdio>
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "hardware_manager.h"
#include "display_service.h"
#include "iot_service.h"
#include "data_processing.h"

static const char *TAG = "WASTAFEL";

static constexpr float    HAND_ON_THRESHOLD     = 15.0f;  // [Unit: cm]
static constexpr float    HAND_OFF_THRESHOLD    = 22.0f;  // [Unit: cm]
static constexpr uint32_t HAND_OFF_TIMEOUT_MS   = 500;

static constexpr float    WATER_LOW_CM          = 12.0f;  // [Unit: cm]

static constexpr float    STABILITY_THRESHOLD_CM = 0.15f; // [Unit: cm]
static constexpr uint32_t STABILITY_MIN_MS       = 1000;
static constexpr uint32_t STABILITY_MAX_MS       = 5000;

static constexpr float    MIN_DISPENSED_HEIGHT_CM = 0.3f; // [Unit: cm]

static constexpr int SENSOR_READ_INTERVAL_MS  = 100;
static constexpr int LCD_UPDATE_INTERVAL_MS   = 500;
static constexpr int MQTT_PUBLISH_INTERVAL_MS = 2000;
static constexpr int BLYNK_PUSH_INTERVAL_MS   = 5000;
static constexpr int CALC_DISPLAY_MS          = 3000;

enum class SystemState : int {
    STATE_IDLE        = 0,
    STATE_PUMPING     = 1,
    STATE_STABILIZING = 2,
    STATE_CALCULATING = 3,
    STATE_SHOW_RESULT = 4,
    STATE_CRITICAL    = 5,
    STATE_REFILL      = 6,
};

enum class VolumeAction {
    NONE,
    DISPENSED,
    ADDED,
};

static HardwareManager hw;

static SystemState s_state   = SystemState::STATE_IDLE;
static bool        s_pump_on = false;

static float        s_hand_distance_cm  = -1.0f;
static float        s_water_distance_cm = -1.0f;
static float        s_current_vol_ml    =  0.0f;
static MovingAverage s_water_filter;

static float        s_initial_water_cm = 0.0f;
static float        s_dispensed_ml     = 0.0f;
static uint32_t     s_calc_finish_ms   = 0;
static VolumeAction s_last_action      = VolumeAction::NONE;

static uint32_t s_hand_away_start_ms = 0;

static inline void pump_set(bool on) {
    s_pump_on = on;
    hw.set_pump(on);
}

static inline bool is_water_level_critical() {
    return (s_water_distance_cm > WATER_CRITICAL_CM) ||
           (s_water_distance_cm <= 0);
}

static inline bool is_hand_detected() {
    return (s_hand_distance_cm > 0) &&
           (s_hand_distance_cm < HAND_ON_THRESHOLD);
}

static void sensor_task(void *pvParams) {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        float hand_raw = hw.get_hand_distance_cm();
        if (hand_raw > 0) {
            s_hand_distance_cm = hand_raw;
        }

        float water_raw = hw.get_water_distance_cm();
        if (water_raw > 0) {
            s_water_distance_cm = s_water_filter.add(water_raw);
            s_current_vol_ml = volume_from_distance(s_water_distance_cm);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

// [STATE_PUMPING]
static void logic_handle_pumping() {
    if (is_water_level_critical()) {
        pump_set(false);
        s_initial_water_cm = s_water_distance_cm;
        s_state = SystemState::STATE_REFILL;
        return;
    }

    bool hand_is_absent = (s_hand_distance_cm > HAND_OFF_THRESHOLD ||
                           s_hand_distance_cm < 0);

    if (hand_is_absent) {
        if (s_hand_away_start_ms == 0) {
            s_hand_away_start_ms = esp_log_timestamp();
        } else if ((esp_log_timestamp() - s_hand_away_start_ms) > HAND_OFF_TIMEOUT_MS) {
            pump_set(false);
            s_calc_finish_ms     = esp_log_timestamp();
            s_last_action        = VolumeAction::DISPENSED;
            s_hand_away_start_ms = 0;
            s_state = SystemState::STATE_STABILIZING;
        }
    } else {
        s_hand_away_start_ms = 0;
    }
}

// [STATE_STABILIZING]
static void logic_handle_stabilizing() {
    if (is_hand_detected()) {
        pump_set(true);
        s_state = SystemState::STATE_PUMPING;
        return;
    }

    uint32_t elapsed_ms = esp_log_timestamp() - s_calc_finish_ms;
    bool surface_stable = s_water_filter.is_stable(STABILITY_THRESHOLD_CM);

    bool min_wait_met   = (elapsed_ms >= STABILITY_MIN_MS);
    bool timeout_forced = (elapsed_ms >= STABILITY_MAX_MS);

    if (!min_wait_met) return;
    if (!surface_stable && !timeout_forced) return;

    float final_height   = WATER_CRITICAL_CM - s_water_distance_cm;
    float initial_height = WATER_CRITICAL_CM - s_initial_water_cm;

    if (s_last_action == VolumeAction::DISPENSED) {
        float used_height = initial_height - final_height;
        if (used_height < MIN_DISPENSED_HEIGHT_CM) used_height = 0;
        s_dispensed_ml = used_height * TANK_AREA_CM2;
    } else {
        float added_height = final_height - initial_height;
        if (added_height < 0) added_height = 0;
        s_dispensed_ml = added_height * TANK_AREA_CM2;
    }

    s_calc_finish_ms = esp_log_timestamp();
    s_state = SystemState::STATE_CALCULATING;
}

static void control_task(void *pvParams) {
    TickType_t last_wake  = xTaskGetTickCount();
    bool       prev_touch = false;

    while (true) {
        bool touch_now    = hw.is_touch_pressed();
        bool touch_rising = (touch_now && !prev_touch);
        prev_touch        = touch_now;

        switch (s_state) {

        // [STATE_IDLE]
        case SystemState::STATE_IDLE:
            pump_set(false);

            if (is_water_level_critical()) {
                s_initial_water_cm = s_water_distance_cm;
                s_state = SystemState::STATE_REFILL;
                break;
            }

            if (touch_rising) {
                s_initial_water_cm = s_water_distance_cm;
                s_state = SystemState::STATE_REFILL;
                break;
            }

            if (is_hand_detected()) {
                s_initial_water_cm = s_water_filter.get();
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
            }
            break;

        case SystemState::STATE_PUMPING:
            logic_handle_pumping();
            break;

        case SystemState::STATE_STABILIZING:
            logic_handle_stabilizing();
            break;

        // [STATE_CALCULATING]
        case SystemState::STATE_CALCULATING:
            if (is_hand_detected()) {
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
                break;
            }
            if ((esp_log_timestamp() - s_calc_finish_ms) > 1000) {
                s_calc_finish_ms = esp_log_timestamp();
                s_state = SystemState::STATE_SHOW_RESULT;
            }
            break;

        // [STATE_SHOW_RESULT]
        case SystemState::STATE_SHOW_RESULT:
            if (is_hand_detected()) {
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
                break;
            }
            if ((esp_log_timestamp() - s_calc_finish_ms) > CALC_DISPLAY_MS) {
                s_state = SystemState::STATE_IDLE;
            }
            break;

        case SystemState::STATE_CRITICAL:
            s_state = SystemState::STATE_IDLE;
            break;

        // [STATE_REFILL]
        case SystemState::STATE_REFILL:
            pump_set(false);

            if (touch_rising) {
                if (!is_water_level_critical()) {
                    s_calc_finish_ms = esp_log_timestamp();
                    s_last_action    = VolumeAction::ADDED;
                    s_state          = SystemState::STATE_STABILIZING;
                }
            }
            break;

        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

static void lcd_task(void *pvParams) {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        DisplayData data;
        data.water_distance_cm  = s_water_distance_cm;
        data.water_spread_cm    = s_water_filter.get_spread();
        data.volume_result_ml   = s_dispensed_ml;
        data.result_is_dispense = (s_last_action == VolumeAction::DISPENSED);

        DisplayService::update(static_cast<int>(s_state), data);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LCD_UPDATE_INTERVAL_MS));
    }
}

// [IOT_TASK]
static void iot_task(void *pvParams) {
    TickType_t last_mqtt  = xTaskGetTickCount();
    TickType_t last_blynk = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();

        if ((now - last_mqtt) >= pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL_MS)) {
            last_mqtt = now;
            float vol_ml = s_current_vol_ml;
            float delta_ml = s_dispensed_ml;
            IoTService::publish(s_hand_distance_cm, s_water_distance_cm,
                                s_pump_on, static_cast<int>(s_state), vol_ml, delta_ml);
        }

        if ((now - last_blynk) >= pdMS_TO_TICKS(BLYNK_PUSH_INTERVAL_MS)) {
            last_blynk = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void) {
    // [APP_MAIN]
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(hw.init());
    ESP_ERROR_CHECK(DisplayService::init());

    // [IOT_INIT]
    ESP_ERROR_CHECK(IoTService::init());

    vTaskDelay(pdMS_TO_TICKS(500));
    DisplayService::show_ready();

    xTaskCreatePinnedToCore(sensor_task,  "sensor",  4096, nullptr, 5,
                            nullptr, 0);
    xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, 4,
                            nullptr, 0);
    xTaskCreatePinnedToCore(lcd_task,     "lcd",     4096, nullptr, 3,
                            nullptr, 0);
    xTaskCreatePinnedToCore(IoTService::run_task, "iot_net", 4096, nullptr, 2,
                            nullptr, 1);
    xTaskCreatePinnedToCore(iot_task,     "iot_pub", 4096, nullptr, 2,
                            nullptr, 0);

    ESP_LOGI(TAG, "All tasks launched — system running.");
}
