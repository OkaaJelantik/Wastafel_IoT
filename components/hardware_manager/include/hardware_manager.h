/**
 * @file hardware_manager.h
 * @brief Hardware Manager — Clean API over raw ESP32 peripheral drivers
 *
 * Abstracts:
 *   - HC-SR04 ultrasonic sensors (hand + water level)
 *   - Relay GPIO (water pump)
 *   - Capacitive touch pad (manual refill trigger)
 *
 * All init is centralised in HardwareManager::init().
 * Callers (main.cpp) only use the semantic accessors:
 *
 *   hw.get_hand_distance_cm()    → float
 *   hw.get_water_distance_cm()   → float (raw, unfiltered)
 *   hw.is_touch_pressed()        → bool
 *   hw.set_pump(true/false)
 *   hw.is_pump_on()              → bool
 */

#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "hcsr04.h"
#include "include/pins.h"

class HardwareManager {
public:
    HardwareManager() = default;
    ~HardwareManager();

    /**
     * @brief Initialise all peripheral drivers.
     *        Must be called once from app_main() before tasks are launched.
     * @return ESP_OK on success; panics via ESP_ERROR_CHECK on failure.
     */
    esp_err_t init();

    // ── Ultrasonic Sensors ───────────────────────────────────────────

    /**
     * @brief Trigger HC-SR04 A and return measured hand distance.
     *
     * Includes the mandatory 40 ms echo wait internally.
     * Returns -1.0f when the sensor reports an out-of-range reading.
     */
    float get_hand_distance_cm();

    /**
     * @brief Trigger HC-SR04 B and return raw water-level distance.
     *
     * Returns -1.0f on sensor error.
     * Caller (sensor_task) is responsible for passing the result
     * through a MovingAverage filter before using it for logic.
     */
    float get_water_distance_cm();

    // ── Relay (Water Pump) ───────────────────────────────────────────

    /**
     * @brief Drive the relay to switch the water pump on or off.
     * @param on true = energise relay (pump ON); false = de-energise (pump OFF)
     */
    void set_pump(bool on);

    /** @brief Returns the last commanded pump state. */
    bool is_pump_on() const { return _pump_on; }

    // ── Touch Pad ────────────────────────────────────────────────────

    /**
     * @brief Read capacitive touch pad; returns true when touched.
     *        Threshold: raw value < TOUCH_THRESHOLD (400).
     */
    bool is_touch_pressed();

private:
    esp_err_t _init_relay();
    esp_err_t _init_touch();
    esp_err_t _init_sensors();

    HCSR04 *_hand_sensor  = nullptr;
    HCSR04 *_water_sensor = nullptr;
    bool    _pump_on      = false;

    static constexpr uint32_t TOUCH_THRESHOLD_VAL = 400;
};
