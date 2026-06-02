/**
 * @file hcsr04.cpp
 * @brief HC-SR04 Ultrasonic Distance Sensor — MCPWM Capture Implementation
 *
 * Pure hardware driver using the ESP-IDF v5.x MCPWM capture API.
 *
 * Timing reference (HC-SR04 datasheet):
 *   - Trigger pulse:  10 µs HIGH on TRIG pin
 *   - Echo pulse:     proportional to object distance
 *   - Speed of sound: 343 m/s → 0.0343 cm/µs
 *   - Round-trip:     distance_cm = echo_us / 58.0
 *   - Valid range:    2 cm – 400 cm
 */

#include "hcsr04.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"       // for ESP_RETURN_ON_ERROR
#include "esp_timer.h"
#include "esp_rom_sys.h"     // for esp_rom_delay_us (safe to call from task ctx)

static const char *TAG = "HCSR04";

/* ══════════════════════════════════════════════════════════════════
 *  Physical constants & driver parameters
 * ══════════════════════════════════════════════════════════════ */

/// Round-trip divisor: distance_cm = echo_µs / 58
static constexpr float US_TO_CM = 58.0f;

/// HC-SR04 minimum measurable distance (cm)
static constexpr float MIN_DISTANCE_CM = 2.0f;

/// HC-SR04 maximum measurable distance (cm)
static constexpr float MAX_DISTANCE_CM = 400.0f;

/// MCPWM capture timer resolution — 1 MHz gives 1 tick per µs
static constexpr uint32_t CAPTURE_RESOLUTION_HZ = 1000000;

/* ══════════════════════════════════════════════════════════════════
 *  Constructor
 * ══════════════════════════════════════════════════════════════ */

HCSR04::HCSR04(gpio_num_t trig_pin, gpio_num_t echo_pin, int group_id)
    : _trig_pin(trig_pin)
    , _echo_pin(echo_pin)
    , _group_id(group_id)
    , _cap_timer(nullptr)
    , _cap_channel(nullptr)
    , _capture_val_begin(0)
    , _capture_val_end(0)
    , _pulse_ticks(0)
    , _measurement_done(false)
    , _wait_for_falling(false)
{
}

/* ══════════════════════════════════════════════════════════════════
 *  ISR callback  (placed in IRAM for deterministic latency)
 * ══════════════════════════════════════════════════════════════ */

/**
 * The capture channel is configured for BOTH edges, so this callback
 * fires twice per measurement cycle:
 *
 *   1. Rising edge  → record start timestamp
 *   2. Falling edge → compute pulse width, set done flag
 *
 * The edge type is determined by edata->cap_edge.
 */
bool HCSR04::capture_callback(
    mcpwm_cap_channel_handle_t cap_chan,
    const mcpwm_capture_event_data_t *edata,
    void *user_ctx)
{
    HCSR04 *self = static_cast<HCSR04 *>(user_ctx);

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        /* ── Rising edge: echo pulse started ────────────────────── */
        self->_capture_val_begin = edata->cap_value;
        self->_wait_for_falling  = true;

    } else if (self->_wait_for_falling) {
        /* ── Falling edge: echo pulse ended ─────────────────────── */
        self->_capture_val_end  = edata->cap_value;
        self->_pulse_ticks      = self->_capture_val_end
                                - self->_capture_val_begin;
        self->_measurement_done = true;
        self->_wait_for_falling = false;
    }

    return false;  // No high-priority task woken
}

/* ══════════════════════════════════════════════════════════════════
 *  Initialisation
 * ══════════════════════════════════════════════════════════════ */

esp_err_t HCSR04::init()
{
    /* ── 1. Configure TRIG pin as push-pull output, initially LOW ─ */
    gpio_config_t trig_conf = {};
    trig_conf.pin_bit_mask  = (1ULL << _trig_pin);
    trig_conf.mode          = GPIO_MODE_OUTPUT;
    trig_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    trig_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    trig_conf.intr_type     = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&trig_conf), TAG,
                        "Failed to configure trigger GPIO %d", _trig_pin);
    gpio_set_level(_trig_pin, 0);

    /* ── 2. Create MCPWM capture timer (1 MHz → 1 µs resolution) ─ */
    mcpwm_capture_timer_config_t cap_timer_cfg = {};
    cap_timer_cfg.clk_src    = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    cap_timer_cfg.group_id   = _group_id;
    cap_timer_cfg.resolution_hz = CAPTURE_RESOLUTION_HZ;
    ESP_RETURN_ON_ERROR(mcpwm_new_capture_timer(&cap_timer_cfg, &_cap_timer),
                        TAG, "Failed to create capture timer (group %d)",
                        _group_id);

    /* ── 3. Create capture channel on the ECHO pin ───────────────
     *  Capture on BOTH edges so the ISR can distinguish rising
     *  (echo start) from falling (echo end) via edata->cap_edge.
     *  Internal pull-down keeps the line stable when idle or disconnected. */
    mcpwm_capture_channel_config_t cap_ch_cfg = {};
    cap_ch_cfg.gpio_num        = _echo_pin;
    cap_ch_cfg.prescale        = 1;           // No additional prescaling
    cap_ch_cfg.flags.neg_edge  = true;        // Capture falling edge
    cap_ch_cfg.flags.pos_edge  = true;        // Capture rising edge
    ESP_RETURN_ON_ERROR(
        mcpwm_new_capture_channel(_cap_timer, &cap_ch_cfg, &_cap_channel),
        TAG, "Failed to create capture channel on GPIO %d", _echo_pin);
    
    // Explicitly set pull-down on Echo pin to prevent floating state
    gpio_set_pull_mode(_echo_pin, GPIO_PULLDOWN_ONLY);

    /* ── 4. Register ISR callback with `this` as user context ──── */
    mcpwm_capture_event_callbacks_t cbs = {};
    cbs.on_cap = capture_callback;
    ESP_RETURN_ON_ERROR(
        mcpwm_capture_channel_register_event_callbacks(
            _cap_channel, &cbs, this),
        TAG, "Failed to register capture callback");

    /* ── 5. Enable channel and start timer ─────────────────────── */
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_enable(_cap_channel),
                        TAG, "Failed to enable capture channel");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_enable(_cap_timer),
                        TAG, "Failed to enable capture timer");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_start(_cap_timer),
                        TAG, "Failed to start capture timer");

    ESP_LOGI(TAG, "Initialised — TRIG=GPIO%d  ECHO=GPIO%d  group=%d",
             _trig_pin, _echo_pin, _group_id);
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════
 *  Trigger a measurement
 * ══════════════════════════════════════════════════════════════ */

esp_err_t HCSR04::trigger()
{
    /*
     * Reset state so the ISR starts fresh for this measurement cycle.
     * Then send a 10 µs HIGH pulse on the TRIG pin — this tells the
     * HC-SR04 to emit its ultrasonic burst.
     */
    _measurement_done  = false;
    _wait_for_falling  = false;

    gpio_set_level(_trig_pin, 1);
    esp_rom_delay_us(10);           // Datasheet requires ≥ 10 µs
    gpio_set_level(_trig_pin, 0);

    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════
 *  Read-back helpers
 * ══════════════════════════════════════════════════════════════ */

float HCSR04::get_distance_cm() const
{
    if (!_measurement_done) {
        return -1.0f;   // No measurement available yet
    }

    /*
     * Get actual timer resolution (ESP32 is fixed at 80MHz).
     * Convert raw ticks to microseconds based on this resolution.
     */
    uint32_t resolution = 0;
    mcpwm_capture_timer_get_resolution(_cap_timer, &resolution);
    
    float pulse_us = (static_cast<float>(_pulse_ticks) * 1000000.0f) / static_cast<float>(resolution);
    float distance = pulse_us / US_TO_CM;

    /* Reject out-of-range readings (sensor spec: 2–400 cm) */
    if (distance < MIN_DISTANCE_CM || distance > MAX_DISTANCE_CM) {
        return -1.0f;
    }

    return distance;
}

bool HCSR04::is_valid() const
{
    float d = get_distance_cm();
    return (d >= MIN_DISTANCE_CM) && (d <= MAX_DISTANCE_CM);
}
