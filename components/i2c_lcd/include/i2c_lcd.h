#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <cstdarg>
#include <cstdint>

// HD44780 Commands
#define LCD_CMD_CLEAR           0x01
#define LCD_CMD_HOME            0x02
#define LCD_CMD_ENTRY_MODE      0x06  // Increment cursor, no shift
#define LCD_CMD_DISPLAY_ON      0x0C  // Display on, cursor off, blink off
#define LCD_CMD_FUNCTION_SET    0x28  // 4-bit mode, 2 lines, 5x8 font
#define LCD_CMD_SET_DDRAM       0x80  // Set DDRAM address

// PCF8574 Bit Mapping for LCD backpack
#define LCD_RS_BIT    (1 << 0)  // Register Select: 0=command, 1=data
#define LCD_RW_BIT    (1 << 1)  // Read/Write: 0=write (always)
#define LCD_EN_BIT    (1 << 2)  // Enable pulse
#define LCD_BL_BIT    (1 << 3)  // Backlight control
// D4-D7 are on bits 4-7 of PCF8574

/**
 * @brief HD44780 1602 LCD Driver via PCF8574 I2C Backpack
 * Uses the new ESP-IDF v5.x I2C master driver.
 * This is a raw hardware driver — no business logic.
 */
class I2C_LCD {
public:
    /**
     * @brief Construct LCD driver
     * @param bus I2C master bus handle (caller must create and own the bus)
     * @param addr PCF8574 I2C address (default 0x27)
     */
    I2C_LCD(i2c_master_bus_handle_t bus, uint8_t addr = 0x27);

    /**
     * @brief Initialize LCD in 4-bit mode
     * Performs the HD44780 initialization sequence with proper delays.
     * @return ESP_OK on success
     */
    esp_err_t init();

    /** @brief Clear display and return cursor to home */
    void clear();

    /**
     * @brief Set cursor position
     * @param col Column (0-15)
     * @param row Row (0-1)
     */
    void set_cursor(uint8_t col, uint8_t row);

    /**
     * @brief Print a string at current cursor position
     * @param str Null-terminated string
     */
    void print(const char* str);

    /**
     * @brief Printf-style formatted print
     * @param fmt Format string
     * @param ... Arguments
     */
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    /**
     * @brief Control backlight
     * @param on true=backlight on, false=backlight off
     */
    void set_backlight(bool on);

private:
    i2c_master_bus_handle_t _bus;
    i2c_master_dev_handle_t _dev;
    uint8_t _addr;
    uint8_t _backlight;  // Current backlight state (LCD_BL_BIT or 0)

    /** @brief Write a single byte to PCF8574 */
    void _write_pcf8574(uint8_t data);

    /** @brief Pulse the Enable pin (E) to latch data */
    void _pulse_enable(uint8_t data);

    /** @brief Send a 4-bit nibble to LCD
     * @param nibble Upper 4 bits contain the data
     * @param mode 0 for command, LCD_RS_BIT for data
     */
    void _send_nibble(uint8_t nibble, uint8_t mode);

    /** @brief Send a full byte to LCD in 4-bit mode
     * @param byte The byte to send
     * @param mode 0 for command, LCD_RS_BIT for data
     */
    void _send_byte(uint8_t byte, uint8_t mode);

    /** @brief Send a command byte */
    void _write_cmd(uint8_t cmd);

    /** @brief Send a data byte */
    void _write_data(uint8_t data);
};
