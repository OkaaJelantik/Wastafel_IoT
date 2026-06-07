/**
 * @file display_service.cpp
 * @brief Display Service Implementation
 *
 * All snprintf formatting, set_cursor calls, and flicker-prevention
 * logic lives here.  main.cpp just calls DisplayService::update().
 */

#include "display_service.h"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "include/pins.h"
#include "../data_processing/include/data_processing.h"

static const char *DISP_TAG = "DISP_SVC";

/* ── SystemState numeric codes — must match SystemState enum in main.cpp ── */
static constexpr int STATE_IDLE        = 0;
static constexpr int STATE_PUMPING     = 1;
static constexpr int STATE_STABILIZING = 2;
static constexpr int STATE_CALCULATING = 3;
static constexpr int STATE_SHOW_RESULT = 4;
static constexpr int STATE_CRITICAL    = 5;
static constexpr int STATE_REFILL      = 6;

/* ── Static Member Definitions ── */
I2C_LCD *DisplayService::s_lcd       = nullptr;
char     DisplayService::s_prev_line0[17] = {0};
char     DisplayService::s_prev_line1[17] = {0};

/* ══════════════════════════════════════════════════════════════════
 *  PUBLIC: init()
 * ══════════════════════════════════════════════════════════════════ */

esp_err_t DisplayService::init() {
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port              = I2C_NUM_0;
    bus_cfg.sda_io_num            = LCD_SDA_PIN;
    bus_cfg.scl_io_num            = LCD_SCL_PIN;
    bus_cfg.clk_source            = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt     = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    s_lcd = new I2C_LCD(bus, LCD_I2C_ADDR);
    ESP_ERROR_CHECK(s_lcd->init());

    // Splash screen during boot
    s_lcd->clear();
    s_lcd->set_cursor(0, 0); s_lcd->print("Wastafel IoT");
    s_lcd->set_cursor(0, 1); s_lcd->print("Booting...");

    ESP_LOGI(DISP_TAG, "LCD ready (SDA=%d, SCL=%d, addr=0x%02X)",
             LCD_SDA_PIN, LCD_SCL_PIN, LCD_I2C_ADDR);
    return ESP_OK;
}

/* ── Show once after all hardware init completes ── */
void DisplayService::show_ready() {
    if (!s_lcd) return;
    s_lcd->clear();
    s_lcd->set_cursor(0, 0); s_lcd->print("System Ready");
}

/* ══════════════════════════════════════════════════════════════════
 *  PUBLIC: update()
 *  Build the two 16-char strings for the current state and push to
 *  the LCD only when content has changed (flicker prevention).
 * ══════════════════════════════════════════════════════════════════ */

void DisplayService::update(int state, const DisplayData &data) {
    char line0[17] = {0};
    char line1[17] = {0};

    switch (state) {

    case STATE_IDLE: {
        // Derive a friendly remaining-capacity figure for the idle screen
        float height_cm = WATER_CRITICAL_CM - data.water_distance_cm;
        if (height_cm < 0) height_cm = 0;
        float vol_ml = height_cm * TANK_AREA_CM2;

        snprintf(line0, sizeof(line0), "SYSTEM READY    ");
        snprintf(line1, sizeof(line1), "Capacity:%4.0f mL", vol_ml);
        break;
    }

    case STATE_PUMPING:
        snprintf(line0, sizeof(line0), "PUMPING...      ");
        snprintf(line1, sizeof(line1), "Dispensing Water");
        break;

    case STATE_STABILIZING:
        snprintf(line0, sizeof(line0), "PLEASE WAIT...  ");
        snprintf(line1, sizeof(line1), "Ripple:%.2f cm  ", data.water_spread_cm);
        break;

    case STATE_CALCULATING:
        snprintf(line0, sizeof(line0), "CALCULATING...  ");
        snprintf(line1, sizeof(line1), "Please hold on  ");
        break;

    case STATE_SHOW_RESULT:
        snprintf(line0, sizeof(line0), data.result_is_dispense
                 ? "DISPENSED:      "
                 : "ADDED:          ");
        snprintf(line1, sizeof(line1), "%7.1f mL      ", data.volume_result_ml);
        break;

    case STATE_REFILL:
        if (data.water_distance_cm > WATER_CRITICAL_CM
                || data.water_distance_cm <= 0) {
            snprintf(line0, sizeof(line0), "!! LOW WATER !! ");
            snprintf(line1, sizeof(line1), "PLEASE REFILL   ");
        } else {
            snprintf(line0, sizeof(line0), "REFILL MODE     ");
            snprintf(line1, sizeof(line1), "PUMP LOCKED OFF ");
        }
        break;

    case STATE_CRITICAL:
    default:
        snprintf(line0, sizeof(line0), "!! LOW WATER !! ");
        snprintf(line1, sizeof(line1), "REFILL REQUIRED ");
        break;
    }

    _write_lines(line0, line1);
}

/* ══════════════════════════════════════════════════════════════════
 *  PRIVATE: _write_lines()
 *  Only sends I2C commands when at least one line has changed.
 * ══════════════════════════════════════════════════════════════════ */

void DisplayService::_write_lines(const char *line0, const char *line1) {
    if (!s_lcd) return;

    if (strncmp(line0, s_prev_line0, 16) == 0 &&
        strncmp(line1, s_prev_line1, 16) == 0) {
        return; // Content unchanged — skip I2C traffic
    }

    s_lcd->clear();
    s_lcd->set_cursor(0, 0); s_lcd->print(line0);
    s_lcd->set_cursor(0, 1); s_lcd->print(line1);

    strncpy(s_prev_line0, line0, 16);
    strncpy(s_prev_line1, line1, 16);
}
