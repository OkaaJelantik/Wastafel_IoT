#include "i2c_lcd.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "I2C_LCD";

// Row DDRAM address offsets for 1602 LCD
static constexpr uint8_t ROW_OFFSETS[] = {0x00, 0x40};

I2C_LCD::I2C_LCD(i2c_master_bus_handle_t bus, uint8_t addr)
    : _bus(bus)
    , _dev(nullptr)
    , _addr(addr)
    , _backlight(LCD_BL_BIT)  // Backlight ON by default
{}

void I2C_LCD::_write_pcf8574(uint8_t data) {
    uint8_t buf = data | _backlight;
    i2c_master_transmit(_dev, &buf, 1, 100);
}

void I2C_LCD::_pulse_enable(uint8_t data) {
    _write_pcf8574(data | LCD_EN_BIT);   // EN high
    esp_rom_delay_us(1);                  // Hold >450ns
    _write_pcf8574(data & ~LCD_EN_BIT);  // EN low
    esp_rom_delay_us(50);                 // Command execution time
}

void I2C_LCD::_send_nibble(uint8_t nibble, uint8_t mode) {
    // Nibble is in upper 4 bits (bits 4-7), mode in lower bits
    uint8_t data = (nibble & 0xF0) | mode;
    _pulse_enable(data);
}

void I2C_LCD::_send_byte(uint8_t byte, uint8_t mode) {
    // Send high nibble first, then low nibble
    _send_nibble(byte & 0xF0, mode);          // High nibble
    _send_nibble((byte << 4) & 0xF0, mode);   // Low nibble
}

void I2C_LCD::_write_cmd(uint8_t cmd) {
    _send_byte(cmd, 0);  // RS=0 for command
}

void I2C_LCD::_write_data(uint8_t data) {
    _send_byte(data, LCD_RS_BIT);  // RS=1 for data
}

esp_err_t I2C_LCD::init() {
    // Add PCF8574 device to I2C bus
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = _addr;
    dev_cfg.scl_speed_hz = 100000;  // 100 kHz standard mode
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(_bus, &dev_cfg, &_dev),
        TAG, "Failed to add LCD device to I2C bus"
    );

    // HD44780 initialization sequence for 4-bit mode
    // Must follow specific timing per datasheet
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait >40ms after power-on

    // Step 1: Send 0x30 three times to ensure 8-bit mode first
    _send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));   // Wait >4.1ms
    _send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(1));   // Wait >100µs
    _send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 2: Switch to 4-bit mode
    _send_nibble(0x20, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 3: Function Set — 4-bit, 2 lines, 5x8 font
    _write_cmd(LCD_CMD_FUNCTION_SET);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 4: Display ON, cursor OFF, blink OFF
    _write_cmd(LCD_CMD_DISPLAY_ON);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 5: Clear display
    _write_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));  // Clear takes ~1.52ms

    // Step 6: Entry mode — increment cursor, no shift
    _write_cmd(LCD_CMD_ENTRY_MODE);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "LCD initialized at I2C address 0x%02X", _addr);
    return ESP_OK;
}

void I2C_LCD::clear() {
    _write_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));  // Clear command needs ~1.52ms
}

void I2C_LCD::set_cursor(uint8_t col, uint8_t row) {
    if (row > 1) row = 1;    // Clamp to valid range
    if (col > 15) col = 15;
    _write_cmd(LCD_CMD_SET_DDRAM | (col + ROW_OFFSETS[row]));
}

void I2C_LCD::print(const char* str) {
    while (*str) {
        _write_data(static_cast<uint8_t>(*str));
        str++;
    }
}

void I2C_LCD::printf(const char* fmt, ...) {
    char buf[64];  // 1602 LCD max is 32 chars, 64 is safe buffer
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    print(buf);
}

void I2C_LCD::set_backlight(bool on) {
    _backlight = on ? LCD_BL_BIT : 0;
    _write_pcf8574(0);  // Send current backlight state
}
