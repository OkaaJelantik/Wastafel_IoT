/**
 * @file pins.h
 * @brief Centralized Pin Definitions for Wastafel_IoT
 *
 * All hardware GPIO assignments are defined here for easy modification.
 * Matches the hardware mapping specified in the Genesis Prompt.
 *
 * Target: ESP32 DevKit V1
 */

#pragma once

#define TOUCH_PAD_VERSION_SUPPRESS_DEPRECATION_WARNING
#include "driver/gpio.h"
#include "driver/touch_sens.h"

// ============================================================================
// HC-SR04 Sensor A — Hand Detection
// ============================================================================
#define HAND_TRIG_PIN       GPIO_NUM_32
#define HAND_ECHO_PIN       GPIO_NUM_33

// ============================================================================
// HC-SR04 Sensor B — Water Level Monitoring
// ============================================================================
#define WATER_TRIG_PIN      GPIO_NUM_14
#define WATER_ECHO_PIN      GPIO_NUM_27

// ============================================================================
// I2C LCD 1602 via PCF8574 (Address 0x27)
// ============================================================================
#define LCD_SDA_PIN         GPIO_NUM_21
#define LCD_SCL_PIN         GPIO_NUM_22
#define LCD_I2C_ADDR        0x27

// ============================================================================
// Relay — Water Pump Control (Active HIGH)
// ============================================================================
#define RELAY_PIN           GPIO_NUM_25

// ============================================================================
// Touch Pad — Manual Refill Mode Trigger
// GPIO 4 = TOUCH_PAD_NUM0 on ESP32
// ============================================================================
#define TOUCH_REFILL_PAD    0
#define TOUCH_REFILL_GPIO   GPIO_NUM_4
