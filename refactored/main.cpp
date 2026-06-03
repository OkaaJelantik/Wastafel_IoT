/**
 * @file main.cpp
 * @brief Wastafel_IoT — Smart Sink Controller  ·  THE BRAIN
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  This file contains ONLY business logic:                     ║
 * ║    • System constants & enumerations                         ║
 * ║    • Global state variables                                   ║
 * ║    • FreeRTOS task bodies (sensor, control, lcd, iot)        ║
 * ║    • State-machine transitions (control_task)                ║
 * ║                                                              ║
 * ║  All technical details are hidden behind service components: ║
 * ║    HardwareManager  →  sensors, relay, touch                 ║
 * ║    DisplayService   →  I2C LCD rendering                     ║
 * ║    IoTService       →  WiFi, MQTT, Blynk, offline buffer     ║
 * ║    data_processing  →  MovingAverage, volume helpers         ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * System Overview:
 *   ┌──────────────────────────────────────────────────────────┐
 *   │  SENSOR LAYER  │  HC-SR04 A (Hands) + B (Water Level)   │
 *   │  ACTUATOR      │  Relay → Water Pump                     │
 *   │  DISPLAY       │  1602 LCD via I2C (PCF8574)             │
 *   │  INPUT         │  Touch Pad (GPIO4) → Refill Mode        │
 *   │  IoT           │  MQTT (telemetry) + Blynk (dashboard)   │
 *   └──────────────────────────────────────────────────────────┘
 *
 * State Machine:
 *   IDLE        → Hand < 15 cm          → PUMPING
 *   PUMPING     → Hand away 500 ms      → STABILIZING
 *   PUMPING     → Water critical        → REFILL (auto)
 *   STABILIZING → Ripple settled        → CALCULATING
 *   CALCULATING → 1 s display delay     → SHOW_RESULT
 *   SHOW_RESULT → 3 s timeout           → IDLE
 *   IDLE        → Touch pressed         → REFILL
 *   REFILL      → Touch pressed again   → STABILIZING (exit)
 *   Any         → Water critical        → REFILL (always wins)
 *
 * Target:    ESP32 DevKit V1
 * Framework: ESP-IDF v5.x (C++)
 */

#include <cstdio>
#include <cstring>
#include <cmath>

/* ── FreeRTOS ──────────────────────────────────────────────────── */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── ESP-IDF System ────────────────────────────────────────────── */
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

/* ── Project Services & Components ────────────────────────────── */
#include "hardware_manager.h"   // Sensors, relay, touch
#include "display_service.h"    // LCD rendering
#include "iot_service.h"        // WiFi / MQTT / Blynk
#include "data_processing.h"    // MovingAverage, volume helpers

static const char *TAG = "WASTAFEL";

/* ═══════════════════════════════════════════════════════════════
 *  COMPILE-TIME CONFIGURATION
 * ═══════════════════════════════════════════════════════════════ */

// ── Hand Detection ──
static constexpr float    HAND_ON_THRESHOLD     = 15.0f;  // Pump ON  when hand < 15 cm
static constexpr float    HAND_OFF_THRESHOLD    = 22.0f;  // Pump OFF when hand > 22 cm (hysteresis)
static constexpr uint32_t HAND_OFF_TIMEOUT_MS   = 500;    // Debounce: hand must be gone ≥ 500 ms

// ── Water Level ──
static constexpr float    WATER_CRITICAL_CM     = 16.0f;  // Distance > this → tank is empty
static constexpr float    WATER_LOW_CM          = 12.0f;  // Warning threshold (informational)

// ── Stabilization Hardening ──
static constexpr float    STABILITY_THRESHOLD_CM = 0.15f; // Max ripple spread (cm) for "stable"
static constexpr uint32_t STABILITY_MIN_MS       = 1000;  // Always wait at least 1 s
static constexpr uint32_t STABILITY_MAX_MS       = 5000;  // Force result after 5 s regardless

// ── Volume Noise Gate ──
static constexpr float    MIN_DISPENSED_HEIGHT_CM = 0.3f; // < 0.3 cm change ≈ sensor noise → 0 mL

// ── Task Timing ──
static constexpr int SENSOR_READ_INTERVAL_MS  = 100;
static constexpr int LCD_UPDATE_INTERVAL_MS   = 500;
static constexpr int MQTT_PUBLISH_INTERVAL_MS = 2000;
static constexpr int BLYNK_PUSH_INTERVAL_MS   = 5000;
static constexpr int CALC_DISPLAY_MS          = 3000; // Show SHOW_RESULT for 3 s

/* ═══════════════════════════════════════════════════════════════
 *  SYSTEM STATE ENUMERATIONS
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief All possible operating modes of the smart sink.
 *
 * Numeric values are intentional — they match the state codes
 * used by DisplayService and IoTService (avoids circular includes).
 */
enum class SystemState : int {
    STATE_IDLE        = 0, ///< Ready and waiting; pump is always off
    STATE_PUMPING     = 1, ///< Hand detected; pump is running
    STATE_STABILIZING = 2, ///< Pump off; waiting for water surface to settle
    STATE_CALCULATING = 3, ///< Brief "Calculating…" display pause
    STATE_SHOW_RESULT = 4, ///< Showing dispensed / added volume in mL
    STATE_CRITICAL    = 5, ///< Legacy; immediately redirects to REFILL
    STATE_REFILL      = 6, ///< Manual or auto refill mode; pump is locked off
};

/**
 * @brief Tracks what the last pumping cycle was for result display.
 */
enum class VolumeAction {
    NONE,
    DISPENSED, ///< Water was pumped out (normal wash)
    ADDED,     ///< Water was added (refill cycle completed)
};

/* ═══════════════════════════════════════════════════════════════
 *  GLOBAL STATE
 *  These variables are written by sensor_task / control_task and
 *  read by lcd_task / iot_task.  Tasks run on Core 0; iot_task on
 *  Core 1.  Reads of scalar floats/bools are atomic on Xtensa LX6.
 * ═══════════════════════════════════════════════════════════════ */

// ── Service Singletons ──
static HardwareManager hw;

// ── State Machine ──
static SystemState s_state   = SystemState::STATE_IDLE;
static bool        s_pump_on = false;

// ── Sensor Readings (written by sensor_task) ──
static float        s_hand_distance_cm  = -1.0f;
static float        s_water_distance_cm = -1.0f;
static float        s_current_vol_ml    =  0.0f;
static MovingAverage s_water_filter;

// ── Volume Calculation Context ──
static float        s_initial_water_cm = 0.0f;  // Snapshot at action start
static float        s_dispensed_ml     = 0.0f;  // Result shown on LCD
static uint32_t     s_calc_finish_ms   = 0;      // Timestamp for timed transitions
static VolumeAction s_last_action      = VolumeAction::NONE;

// ── Debounce Timer (written/read only by control_task) ──
static uint32_t s_hand_away_start_ms = 0;

/* ═══════════════════════════════════════════════════════════════
 *  HELPER: pump control
 *  Thin wrapper so we always keep s_pump_on in sync with the relay.
 * ═══════════════════════════════════════════════════════════════ */
static inline void pump_set(bool on) {
    s_pump_on = on;
    hw.set_pump(on);
}

/* ═══════════════════════════════════════════════════════════════
 *  HELPER: water level predicates
 * ═══════════════════════════════════════════════════════════════ */
static inline bool is_water_level_critical() {
    return (s_water_distance_cm > WATER_CRITICAL_CM) ||
           (s_water_distance_cm <= 0);
}

static inline bool is_hand_detected() {
    return (s_hand_distance_cm > 0) &&
           (s_hand_distance_cm < HAND_ON_THRESHOLD);
}

/* ═══════════════════════════════════════════════════════════════
 *  TASK: SENSOR READING (Core 0, 10 Hz)
 *
 *  1. Read hand sensor (raw, no filter).
 *  2. Read water sensor and apply MovingAverage.
 *  3. Derive current tank volume for IoT reporting.
 *  4. Sample touch pad.
 * ═══════════════════════════════════════════════════════════════ */
static void sensor_task(void *pvParams) {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        /* ── Hand distance ─────────────────────────────────────── */
        float hand_raw = hw.get_hand_distance_cm();
        if (hand_raw > 0) {
            s_hand_distance_cm = hand_raw;
        }

        /* ── Water level (filtered) ────────────────────────────── */
        float water_raw = hw.get_water_distance_cm();
        if (water_raw > 0) {
            s_water_distance_cm = s_water_filter.add(water_raw);

            // Centralise volume calculation so all consumers agree
            s_current_vol_ml = volume_from_distance(s_water_distance_cm);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  STATE HANDLER: logic_handle_pumping()
 *  Called from control_task while STATE_PUMPING is active.
 *
 *  Responsibilities:
 *    a) CRITICAL LOCKOUT: if the tank runs dry during pumping,
 *       immediately cut the pump and enter REFILL mode.
 *    b) HAND REMOVAL with 500 ms debounce: once the hand has been
 *       absent long enough, stop pumping and begin stabilisation.
 * ═══════════════════════════════════════════════════════════════ */
static void logic_handle_pumping() {
    // ── (a) Water-critical lockout overrides everything ──────────
    if (is_water_level_critical()) {
        pump_set(false);
        s_initial_water_cm = s_water_distance_cm;
        s_state = SystemState::STATE_REFILL;
        ESP_LOGE(TAG, "→ REFILL: Water critical during pumping — pump locked off");
        return;
    }

    // ── (b) Hand removal with hysteresis + 500 ms debounce ───────
    bool hand_is_absent = (s_hand_distance_cm > HAND_OFF_THRESHOLD ||
                           s_hand_distance_cm < 0);

    if (hand_is_absent) {
        if (s_hand_away_start_ms == 0) {
            s_hand_away_start_ms = esp_log_timestamp(); // Start debounce timer
        } else if ((esp_log_timestamp() - s_hand_away_start_ms) > HAND_OFF_TIMEOUT_MS) {
            // Hand has been gone long enough — stop pumping
            pump_set(false);
            s_calc_finish_ms     = esp_log_timestamp();
            s_last_action        = VolumeAction::DISPENSED;
            s_hand_away_start_ms = 0;
            s_state = SystemState::STATE_STABILIZING;
            ESP_LOGI(TAG, "→ STABILIZING: Hand removed, waiting for water to settle");
        }
    } else {
        s_hand_away_start_ms = 0; // Hand re-detected: reset debounce
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  STATE HANDLER: logic_handle_stabilizing()
 *  Called from control_task while STATE_STABILIZING is active.
 *
 *  Responsibilities:
 *    a) Allow the user to restart pumping mid-stabilisation.
 *    b) Adaptive wait: exit when surface is stable (spread < 0.15 cm)
 *       OR after a hard 5 s timeout to prevent deadlock.
 *    c) Calculate dispensed / added volume and advance to CALCULATING.
 * ═══════════════════════════════════════════════════════════════ */
static void logic_handle_stabilizing() {
    // ── (a) Hand re-detected → restart pump immediately ──────────
    if (is_hand_detected()) {
        pump_set(true);
        s_state = SystemState::STATE_PUMPING;
        ESP_LOGI(TAG, "→ PUMPING: Hand re-detected during stabilisation");
        return;
    }

    uint32_t elapsed_ms = esp_log_timestamp() - s_calc_finish_ms;
    bool surface_stable = s_water_filter.is_stable(STABILITY_THRESHOLD_CM);

    // ── (b) Adaptive exit condition ───────────────────────────────
    bool min_wait_met   = (elapsed_ms >= STABILITY_MIN_MS);
    bool timeout_forced = (elapsed_ms >= STABILITY_MAX_MS);

    if (!min_wait_met) return; // Always honour the minimum wait
    if (!surface_stable && !timeout_forced) return;

    if (timeout_forced && !surface_stable) {
        ESP_LOGW(TAG, "Stabilisation timeout — forcing calculation "
                      "(ripple spread=%.2f cm)", s_water_filter.get_spread());
    }

    // ── (c) Compute volume delta ───────────────────────────────────
    float final_height   = WATER_CRITICAL_CM - s_water_distance_cm;
    float initial_height = WATER_CRITICAL_CM - s_initial_water_cm;

    if (s_last_action == VolumeAction::DISPENSED) {
        float used_height = initial_height - final_height;
        // Noise gate: sub-0.3 cm changes (~17 mL) are treated as zero
        if (used_height < MIN_DISPENSED_HEIGHT_CM) used_height = 0;
        s_dispensed_ml = used_height * TANK_AREA_CM2;
    } else {
        // Refill: water level rose → distance decreased
        float added_height = final_height - initial_height;
        if (added_height < 0) added_height = 0;
        s_dispensed_ml = added_height * TANK_AREA_CM2;
    }

    s_calc_finish_ms = esp_log_timestamp();
    s_state = SystemState::STATE_CALCULATING;
    ESP_LOGI(TAG, "→ CALCULATING: Volume = %.1f mL", s_dispensed_ml);
}

/* ═══════════════════════════════════════════════════════════════
 *  TASK: CONTROL LOGIC (Core 0, 10 Hz)
 *
 *  This task IS the state machine.  Each case handles exactly one
 *  operating mode; all sensor reads are delegated to sensor_task.
 *
 *  The guard at the top of the loop detects a rising edge on the
 *  touch pad (single press = toggle refill mode).
 * ═══════════════════════════════════════════════════════════════ */
static void control_task(void *pvParams) {
    TickType_t last_wake  = xTaskGetTickCount();
    bool       prev_touch = false;

    while (true) {
        /* ── Touch pad: detect rising edge (press, not hold) ─────── */
        bool touch_now    = hw.is_touch_pressed();
        bool touch_rising = (touch_now && !prev_touch);
        prev_touch        = touch_now;

        switch (s_state) {

        /* ────────────────────────────────────────────────────────
         *  IDLE: Pump is off.  Waiting for a hand or touch event.
         *  Auto-transition to REFILL if tank is already critical.
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_IDLE:
            pump_set(false);

            // Water-critical check takes priority over user input
            if (is_water_level_critical()) {
                s_initial_water_cm = s_water_distance_cm;
                s_state = SystemState::STATE_REFILL;
                ESP_LOGW(TAG, "→ REFILL: Low water detected in IDLE — auto refill mode");
                break;
            }

            // Manual refill mode toggled by touch
            if (touch_rising) {
                s_initial_water_cm = s_water_distance_cm;
                s_state = SystemState::STATE_REFILL;
                ESP_LOGI(TAG, "→ REFILL: Touch pressed — entering refill mode");
                break;
            }

            // Hand close enough → start dispensing
            if (is_hand_detected()) {
                s_initial_water_cm = s_water_filter.get(); // Baseline from filter
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
                ESP_LOGI(TAG, "→ PUMPING: Hand detected (%.1f cm)", s_hand_distance_cm);
            }
            break;

        /* ────────────────────────────────────────────────────────
         *  PUMPING: Pump is running.
         *  Delegated to logic_handle_pumping() for clarity.
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_PUMPING:
            logic_handle_pumping();
            break;

        /* ────────────────────────────────────────────────────────
         *  STABILIZING: Pump off, water surface settling.
         *  Delegated to logic_handle_stabilizing().
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_STABILIZING:
            logic_handle_stabilizing();
            break;

        /* ────────────────────────────────────────────────────────
         *  CALCULATING: 1 s display buffer before showing result.
         *  Allows another pump cycle to interrupt if hand returns.
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_CALCULATING:
            if (is_hand_detected()) {
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
                ESP_LOGI(TAG, "→ PUMPING: Hand returned during calculating");
                break;
            }
            if ((esp_log_timestamp() - s_calc_finish_ms) > 1000) {
                s_calc_finish_ms = esp_log_timestamp();
                s_state = SystemState::STATE_SHOW_RESULT;
                ESP_LOGI(TAG, "→ SHOW_RESULT: Displaying %.1f mL", s_dispensed_ml);
            }
            break;

        /* ────────────────────────────────────────────────────────
         *  SHOW_RESULT: Display dispensed / added volume for 3 s.
         *  Returns to IDLE automatically, or immediately if a new
         *  hand is detected.
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_SHOW_RESULT:
            if (is_hand_detected()) {
                pump_set(true);
                s_state = SystemState::STATE_PUMPING;
                ESP_LOGI(TAG, "→ PUMPING: Hand detected during result display");
                break;
            }
            if ((esp_log_timestamp() - s_calc_finish_ms) > CALC_DISPLAY_MS) {
                s_state = SystemState::STATE_IDLE;
                ESP_LOGI(TAG, "→ IDLE: Result display timeout");
            }
            break;

        /* ────────────────────────────────────────────────────────
         *  CRITICAL: Legacy state — immediately falls through to IDLE.
         *  Water-critical logic is now handled inline via REFILL.
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_CRITICAL:
            s_state = SystemState::STATE_IDLE;
            break;

        /* ────────────────────────────────────────────────────────
         *  REFILL: Pump is hard-locked OFF.
         *  User fills the tank, then presses touch again to exit.
         *  Exit is blocked while water is still critical.
         * ──────────────────────────────────────────────────────── */
        case SystemState::STATE_REFILL:
            pump_set(false); // Continuously enforce: pump stays off

            if (touch_rising) {
                if (is_water_level_critical()) {
                    // Tank still empty — forbid exit and inform user
                    ESP_LOGW(TAG, "Refill exit blocked: water still critical (%.1f cm)",
                             s_water_distance_cm);
                } else {
                    // Tank filled enough → calculate how much was added
                    s_calc_finish_ms = esp_log_timestamp();
                    s_last_action    = VolumeAction::ADDED;
                    s_state          = SystemState::STATE_STABILIZING;
                    ESP_LOGI(TAG, "→ STABILIZING: Refill complete, settling surface...");
                }
            }
            break;

        } // end switch

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  TASK: LCD DISPLAY (Core 0, 2 Hz)
 *  Assembles a DisplayData snapshot and delegates all rendering
 *  to DisplayService::update().
 * ═══════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════
 *  TASK: IoT TELEMETRY (Core 1)
 *  Drives MQTT + Blynk publish intervals.
 *  Delegates all network operations to IoTService.
 * ═══════════════════════════════════════════════════════════════ */
static void iot_task(void *pvParams) {
    TickType_t last_mqtt  = xTaskGetTickCount();
    TickType_t last_blynk = xTaskGetTickCount();

    // Block here until WiFi succeeds or times out (handled inside IoTService)
    IoTService::run_task(nullptr); // Performs the WiFi wait + MQTT init

    while (true) {
        TickType_t now = xTaskGetTickCount();

        /* ── MQTT publish at 2 Hz ───────────────────────────────── */
        if ((now - last_mqtt) >= pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL_MS)) {
            last_mqtt = now;
            IoTService::publish(s_hand_distance_cm, s_water_distance_cm,
                                s_pump_on, static_cast<int>(s_state));
        }

        /* ── Blynk push at 0.2 Hz ───────────────────────────────── */
        if ((now - last_blynk) >= pdMS_TO_TICKS(BLYNK_PUSH_INTERVAL_MS)) {
            last_blynk = now;
            // Blynk push is triggered via the same publish pathway
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  APPLICATION ENTRY POINT
 * ═══════════════════════════════════════════════════════════════ */

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "════════════════════════════════════════");
    ESP_LOGI(TAG, "  Wastafel IoT — Smart Sink Controller ");
    ESP_LOGI(TAG, "  Target: ESP32 DevKit V1              ");
    ESP_LOGI(TAG, "════════════════════════════════════════");

    /* ── 1. NVS flash (required for WiFi credential storage) ─── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. Hardware peripherals ─────────────────────────────── */
    ESP_ERROR_CHECK(hw.init());          // HC-SR04 × 2, relay, touch pad

    /* ── 3. Display ──────────────────────────────────────────── */
    ESP_ERROR_CHECK(DisplayService::init()); // I2C bus + LCD, shows splash

    /* ── 4. IoT (non-blocking WiFi start) ───────────────────── */
    // ESP_ERROR_CHECK(IoTService::init()); // Uncomment to enable networking

    /* ── 5. Post-init ready message ─────────────────────────── */
    vTaskDelay(pdMS_TO_TICKS(500));
    DisplayService::show_ready();

    /* ── 6. Launch FreeRTOS Tasks ────────────────────────────── */

    // sensor_task: reads HC-SR04 × 2 + touch pad @ 10 Hz (Core 0)
    xTaskCreatePinnedToCore(sensor_task,  "sensor",  4096, nullptr, 5,
                            nullptr, 0);

    // control_task: runs the state machine @ 10 Hz (Core 0)
    xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, 4,
                            nullptr, 0);

    // lcd_task: refreshes 1602 LCD @ 2 Hz (Core 0)
    xTaskCreatePinnedToCore(lcd_task,     "lcd",     4096, nullptr, 3,
                            nullptr, 0);

    // iot_task: MQTT + Blynk telemetry on Core 1 (network isolation)
    // xTaskCreatePinnedToCore(iot_task,     "iot",     8192, nullptr, 2,
    //                         nullptr, 1);

    ESP_LOGI(TAG, "All tasks launched — system running.");
}
