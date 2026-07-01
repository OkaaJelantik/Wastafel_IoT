#pragma once

#include "driver/gpio.h"

// HC-SR04 Sensor A — Hand Detection
#define HAND_TRIG_PIN       GPIO_NUM_32
#define HAND_ECHO_PIN       GPIO_NUM_33

// HC-SR04 Sensor B — Water Level Monitoring
#define WATER_TRIG_PIN      GPIO_NUM_5
#define WATER_ECHO_PIN      GPIO_NUM_18

// I2C LCD 1602 via PCF8574 (Address 0x27)
#define LCD_SDA_PIN         GPIO_NUM_21
#define LCD_SCL_PIN         GPIO_NUM_22
#define LCD_I2C_ADDR        0x27

// Relay — Water Pump Control (Active HIGH)
#define RELAY_PIN           GPIO_NUM_26

// Touch Pad — Manual Refill Mode Trigger
#define TOUCH_REFILL_PAD    0
#define TOUCH_REFILL_GPIO   GPIO_NUM_4
