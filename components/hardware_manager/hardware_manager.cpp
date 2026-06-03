/**
 * @file hardware_manager.cpp
 * @brief Hardware Manager Implementation
 *
 * Wraps raw ESP-IDF peripheral drivers behind a clean OOP interface.
 * All GPIO, sensor, relay, and touch-pad details live here so that
 * main.cpp and the service components stay free of raw register calls.
 */

#include "hardware_manager.h"

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *HW_TAG = "HW_MGR";

/* ─────────────────────────────────────────────────────────────────── */

HardwareManager::~HardwareManager() {
    delete _hand_sensor;
    delete _water_sensor;
}

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC: init()
 *  Call once from app_main() before launching any FreeRTOS task.
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t HardwareManager::init() {
    ESP_LOGI(HW_TAG, "Initialising hardware peripherals...");

    ESP_ERROR_CHECK(_init_relay());
    ESP_ERROR_CHECK(_init_touch());
    ESP_ERROR_CHECK(_init_sensors());

    ESP_LOGI(HW_TAG, "All hardware initialised successfully.");
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC: Ultrasonic Sensor Accessors
 * ═══════════════════════════════════════════════════════════════════ */

float HardwareManager::get_hand_distance_cm() {
    if (!_hand_sensor) return -1.0f;
    _hand_sensor->trigger();
    vTaskDelay(pdMS_TO_TICKS(40)); // Mandatory echo settle time for HC-SR04
    float d = _hand_sensor->get_distance_cm();
    return (d > 0) ? d : -1.0f;
}

float HardwareManager::get_water_distance_cm() {
    if (!_water_sensor) return -1.0f;
    _water_sensor->trigger();
    vTaskDelay(pdMS_TO_TICKS(30)); // Water sensor uses slightly shorter wait
    float d = _water_sensor->get_distance_cm();
    return (d > 0) ? d : -1.0f;
}

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC: Relay (Pump) Control
 * ═══════════════════════════════════════════════════════════════════ */

void HardwareManager::set_pump(bool on) {
    _pump_on = on;
    gpio_set_level(RELAY_PIN, on ? 1 : 0);
    ESP_LOGD(HW_TAG, "Pump → %s", on ? "ON" : "OFF");
}

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC: Touch Pad
 * ═══════════════════════════════════════════════════════════════════ */

bool HardwareManager::is_touch_pressed() {
    uint16_t raw_value = 0;
    touch_pad_read(static_cast<touch_pad_t>(TOUCH_REFILL_PAD), &raw_value);
    return (raw_value < TOUCH_THRESHOLD_VAL);
}

/* ═══════════════════════════════════════════════════════════════════
 *  PRIVATE: Peripheral Init Helpers
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t HardwareManager::_init_relay() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask  = (1ULL << RELAY_PIN);
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Ensure pump is OFF on boot (fail-safe)
    gpio_set_level(RELAY_PIN, 0);
    _pump_on = false;

    ESP_LOGI(HW_TAG, "Relay (pump) ready on GPIO%d — default OFF", RELAY_PIN);
    return ESP_OK;
}

esp_err_t HardwareManager::_init_touch() {
    ESP_LOGI(HW_TAG, "Touch pad on GPIO%d (PAD%d, threshold<%u)",
             TOUCH_REFILL_GPIO, TOUCH_REFILL_PAD, TOUCH_THRESHOLD_VAL);

    touch_pad_init();
    touch_pad_config(static_cast<touch_pad_t>(TOUCH_REFILL_PAD), 0);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    return ESP_OK;
}

esp_err_t HardwareManager::_init_sensors() {
    // Sensor A: Hand detection — uses MCPWM capture group 0
    _hand_sensor = new HCSR04(HAND_TRIG_PIN, HAND_ECHO_PIN, 0);
    ESP_ERROR_CHECK(_hand_sensor->init());
    ESP_LOGI(HW_TAG, "Hand sensor (HC-SR04 A): Trig=GPIO%d, Echo=GPIO%d",
             HAND_TRIG_PIN, HAND_ECHO_PIN);

    // Sensor B: Water level — uses MCPWM capture group 1 to avoid conflicts
    _water_sensor = new HCSR04(WATER_TRIG_PIN, WATER_ECHO_PIN, 1);
    ESP_ERROR_CHECK(_water_sensor->init());
    ESP_LOGI(HW_TAG, "Water sensor (HC-SR04 B): Trig=GPIO%d, Echo=GPIO%d",
             WATER_TRIG_PIN, WATER_ECHO_PIN);

    return ESP_OK;
}
