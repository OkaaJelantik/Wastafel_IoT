/**
 * @file display_service.h
 * @brief Display Service — I2C LCD 1602 via PCF8574
 *
 * Wraps the raw I2C_LCD driver and provides a single high-level call:
 *
 *   DisplayService::update(state, data);
 *
 * Layout (16×2):
 *   Row 0: Mode / status label
 *   Row 1: Numeric reading or action prompt
 *
 * Only redraws when content changes to prevent LCD flicker.
 */

#pragma once

#include "esp_err.h"
#include "i2c_lcd.h"

/* ── DisplayData: snapshot passed from main.cpp on each update ── */
struct DisplayData {
    float   water_distance_cm;  // Raw smoothed reading
    float   water_spread_cm;    // Current ripple spread (MovingAverage)
    float   volume_result_ml;   // Volume dispensed or added (SHOW_RESULT)
    bool    result_is_dispense; // true = dispensed, false = added (refill)
};

/* ════════════════════════════════════════════════════════════════════
 *  CLASS: DisplayService (static singleton façade)
 * ════════════════════════════════════════════════════════════════════ */
class DisplayService {
public:
    DisplayService() = delete; // Pure static

    /**
     * @brief Initialise the I2C master bus and LCD.
     *        Shows a "Booting…" splash.  Call once from app_main().
     */
    static esp_err_t init();

    /**
     * @brief Show the "System Ready" message after all hardware init completes.
     */
    static void show_ready();

    /**
     * @brief Refresh the display for the given system state.
     *        Call from lcd_task() at LCD_UPDATE_INTERVAL_MS.
     *
     * @param state   Cast of SystemState as int (avoids circular include)
     * @param data    Sensor snapshot for numeric fields
     */
    static void update(int state, const DisplayData &data);

private:
    static void _write_lines(const char *line0, const char *line1);

    static I2C_LCD *s_lcd;
    static char     s_prev_line0[17];
    static char     s_prev_line1[17];
};
