# Perplexity Research Brief: Wastafel_IoT Project

**Role:** Expert Embedded Systems Researcher
**Context:** I am building an IoT Smart Sink using **ESP32 DevKit V1** with the **ESP-IDF v6.2.0** framework. The project is written in **C++** with a focus on high-level abstraction (OOP).

**Hardware Stack:**
- 2x HC-SR04 Ultrasonic Sensors
- 1x I2C LCD 1602 (PCF8574 adapter, Address 0x27)
- 2x Relay Module (SRD-05VDC-SL-C)
- ESP32 DevKit V1

**Specific Research Goals:**

1. **HC-SR04 C++ Driver for ESP-IDF:**
   - Find or recommend a non-blocking C++ implementation for HC-SR04.
   - It must handle 2 sensors concurrently without using `vTaskDelay` that blocks other sensor readings.
   - Preferred method: Using RMT or MCPWM peripherals for precise timing in ESP-IDF.

2. **I2C LCD 1602 C++ Driver:**
   - Recommend the most stable C++ wrapper/library for a 1602 LCD via I2C on ESP-IDF.
   - Must support basic functions: `init`, `clear`, `setCursor`, `print`.

3. **Blynk & MQTT Integration (C++):**
   - Provide a simplified architecture for using MQTT (telemetry) and Blynk (data logging/fallback) together in C++ on ESP-IDF.
   - Explain how to handle Blynk "Offline" data logging as a fallback.

4. **Modular Architecture Best Practices:**
   - Provide a template or snippet for a C++ Class structure for an ESP-IDF component (e.g., a `Sensor` class that wraps the driver).

**Instructions for Output:**
- Prioritize code snippets compatible with modern ESP-IDF (v5.x/v6.x).
- Avoid Arduino-specific libraries unless they are easily portable to pure ESP-IDF.
- Explain any specific menuconfig requirements for the recommended libraries.
