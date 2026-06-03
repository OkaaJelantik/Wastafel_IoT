/**
 * @file data_processing.h
 * @brief Data Processing Component — MovingAverage filter & Volume calculations
 *
 * Provides:
 *   - MovingAverage: Circular buffer filter to suppress sensor noise
 *   - TelemetryEntry: POD struct for offline buffering
 *   - volume_from_distance(): Converts sensor distance to mL
 *
 * Used by: hardware_manager (filtering), main (volume calculation)
 */

#pragma once

#include <cstring>

// ── Window size for the water-level moving average ──
static constexpr int DATA_MA_WINDOW = 10;

// ── Physical tank dimensions ──
static constexpr float TANK_AREA_CM2    = 56.72f; // Tank base surface area (cm²)
static constexpr float WATER_CRITICAL_CM = 16.0f; // Sensor→water distance at "empty" (cm)

/* ════════════════════════════════════════════════════════════════════
 *  CLASS: MovingAverage
 *  Circular-buffer low-pass filter for ultrasonic readings.
 *  Thread-safe: caller must guard if accessed from multiple tasks.
 * ════════════════════════════════════════════════════════════════════ */
class MovingAverage {
public:
    MovingAverage() : _index(0), _count(0), _sum(0.0f) {
        memset(_buffer, 0, sizeof(_buffer));
    }

    /**
     * @brief Push a new sample; returns the smoothed average.
     * @param value Raw sensor reading (cm)
     * @return Running mean of the last DATA_MA_WINDOW samples
     */
    float add(float value) {
        _sum -= _buffer[_index];
        _buffer[_index] = value;
        _sum += value;
        _index = (_index + 1) % DATA_MA_WINDOW;
        if (_count < DATA_MA_WINDOW) _count++;
        return _sum / static_cast<float>(_count);
    }

    /** @brief Return the current smoothed value without adding a sample. */
    float get() const {
        if (_count == 0) return 0.0f;
        return _sum / static_cast<float>(_count);
    }

    /**
     * @brief Spread = max − min in the current window.
     *        High spread → water surface still rippling.
     */
    float get_spread() const {
        if (_count == 0) return 0.0f;
        float lo = _buffer[0], hi = _buffer[0];
        for (int i = 1; i < _count; i++) {
            if (_buffer[i] < lo) lo = _buffer[i];
            if (_buffer[i] > hi) hi = _buffer[i];
        }
        return hi - lo;
    }

    /**
     * @brief Returns true when the window is full AND spread < threshold_cm.
     *        Used by the STABILIZING state to confirm ripples have subsided.
     */
    bool is_stable(float threshold_cm) const {
        if (_count < DATA_MA_WINDOW) return false;
        return get_spread() < threshold_cm;
    }

private:
    float _buffer[DATA_MA_WINDOW];
    int   _index;
    int   _count;
    float _sum;
};

/* ════════════════════════════════════════════════════════════════════
 *  STRUCT: TelemetryEntry
 *  Snapshot stored in the offline ring-buffer when WiFi is unavailable.
 * ════════════════════════════════════════════════════════════════════ */
struct TelemetryEntry {
    float   hand_distance_cm;
    float   water_distance_cm;
    bool    pump_active;
    int     system_state;   // Cast of SystemState enum
    int64_t timestamp_ms;   // esp_log_timestamp() at time of capture
};

/* ════════════════════════════════════════════════════════════════════
 *  FREE FUNCTIONS: Volume Helpers
 * ════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convert sensor-to-surface distance to water volume (mL).
 *
 * The sensor sits above the tank; smaller distance = more water.
 * Water height  = WATER_CRITICAL_CM − distance_cm
 * Volume (mL)   = height × TANK_AREA_CM2
 *
 * @param distance_cm  Smoothed distance reading (cm)
 * @return Volume in mL; clamped to 0 if distance exceeds critical level
 */
inline float volume_from_distance(float distance_cm) {
    float height = WATER_CRITICAL_CM - distance_cm;
    if (height < 0.0f) height = 0.0f;
    return height * TANK_AREA_CM2;
}

/**
 * @brief Calculate volume delta between two distance readings.
 *
 * Positive result = water was added (refill).
 * Negative result = water was consumed (dispensed) — caller must abs() it.
 *
 * @param before_cm  Distance at start of action
 * @param after_cm   Distance at end of action
 * @return Signed volume change in mL
 */
inline float volume_delta_ml(float before_cm, float after_cm) {
    // A smaller distance after means more water (refill); larger = dispensed.
    return (before_cm - after_cm) * TANK_AREA_CM2;
}
