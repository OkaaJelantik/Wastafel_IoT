/**
 * @file hcsr04.h
 * @brief HC-SR04 Ultrasonic Distance Sensor Driver (MCPWM Capture)
 *
 * Pure hardware driver — no business logic. Uses the ESP-IDF v5.x MCPWM
 * capture peripheral for non-blocking, hardware-timed echo pulse measurement.
 *
 * Principle of operation:
 *   1. A 10 µs HIGH pulse is sent on the TRIG pin.
 *   2. The sensor emits an ultrasonic burst and raises ECHO HIGH.
 *   3. ECHO goes LOW when the burst echo returns.
 *   4. The MCPWM capture ISR records timestamps on both edges.
 *   5. Pulse width (in µs) is converted to distance: d = t_us / 58.
 *
 * @note Each instance occupies one MCPWM capture channel.  ESP32 has
 *       2 MCPWM groups × 3 capture channels = 6 channels total.
 *       Use the group_id constructor parameter to spread sensors across groups.
 */

#pragma once

#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Nothing needed from C — class is C++ only */

#ifdef __cplusplus
}
#endif

/**
 * @class HCSR04
 * @brief HC-SR04 Ultrasonic Distance Sensor Driver
 *
 * Uses MCPWM Capture for non-blocking, hardware-timed pulse measurement.
 * This is a raw hardware driver — no business logic.
 */
class HCSR04 {
public:
    /**
     * @brief Construct HC-SR04 driver
     *
     * @param trig_pin  GPIO number for the trigger pulse output
     * @param echo_pin  GPIO number for the echo pulse input (MCPWM capture)
     * @param group_id  MCPWM group index (0 or 1). Use different groups when
     *                  running multiple sensors to avoid resource conflicts,
     *                  or share a group if channels are available.
     */
    HCSR04(gpio_num_t trig_pin, gpio_num_t echo_pin, int group_id = 0);

    /**
     * @brief Initialize MCPWM capture and GPIO
     *
     * Sets up the capture timer at 1 MHz resolution (1 tick = 1 µs) and
     * configures a capture channel with ISR callback for rising + falling
     * edge detection on the echo pin.
     *
     * @return ESP_OK on success, or an esp_err_t error code
     */
    esp_err_t init();

    /**
     * @brief Send a 10 µs trigger pulse to start a measurement
     *
     * The result will be available via get_distance_cm() after the echo
     * returns (typically < 30 ms for distances under 4 m).
     *
     * @return ESP_OK on success
     */
    esp_err_t trigger();

    /**
     * @brief Get the last measured distance in centimetres
     *
     * @return Distance in cm, or -1.0f if no valid reading is available
     */
    float get_distance_cm() const;

    /**
     * @brief Check whether the last measurement is valid
     *
     * A measurement is invalid if:
     *   - No echo was received yet
     *   - The computed distance is outside the sensor's 2–400 cm range
     *
     * @return true if the last reading is within the valid range
     */
    bool is_valid() const;

private:
    /* ── Pin & peripheral configuration ──────────────────────────── */
    gpio_num_t _trig_pin;                       ///< Trigger output GPIO
    gpio_num_t _echo_pin;                       ///< Echo input GPIO
    int        _group_id;                       ///< MCPWM group index (0 or 1)
    mcpwm_cap_timer_handle_t   _cap_timer;      ///< MCPWM capture timer handle
    mcpwm_cap_channel_handle_t _cap_channel;    ///< MCPWM capture channel handle

    /* ── ISR-shared data (volatile for cross-context visibility) ── */
    volatile uint32_t _capture_val_begin;   ///< Capture timestamp at rising edge
    volatile uint32_t _capture_val_end;     ///< Capture timestamp at falling edge
    volatile uint32_t _pulse_ticks;         ///< Computed pulse width (timer ticks)
    volatile bool     _measurement_done;    ///< true when a complete measurement exists
    volatile bool     _wait_for_falling;    ///< true while waiting for falling edge

    /**
     * @brief MCPWM capture ISR callback (runs in IRAM)
     *
     * Registered for BOTH edges.  On the rising edge it records the start
     * timestamp; on the falling edge it computes the pulse width and sets
     * the measurement-done flag.
     *
     * @param cap_chan  Capture channel handle (unused)
     * @param edata    Event data containing cap_value and cap_edge
     * @param user_ctx Pointer to the owning HCSR04 instance (`this`)
     * @return false — no high-priority task is woken
     */
    static bool capture_callback(
        mcpwm_cap_channel_handle_t cap_chan,
        const mcpwm_capture_event_data_t *edata,
        void *user_ctx);
};
