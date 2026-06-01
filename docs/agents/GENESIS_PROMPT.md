# Genesis Prompt: Wastafel_IoT Project Initialization

**Role:** Expert Embedded Systems Engineer (ESP-IDF & C++ Specialist)
**Context:** Initialize a professional, modular ESP-IDF project for an IoT Smart Sink.
**Framework:** ESP-IDF (Latest stable)
**Language:** C++ (Object-Oriented)
**Target:** ESP32 DevKit V1

## 1. Project Structure
- `components/hcsr04/`: C++ Class using MCPWM Capture for non-blocking distance measurement.
- `components/i2c_lcd/`: C++ Class for 1602 LCD via PCF8574 (Address 0x27).
- `main/main.cpp`: **CRITICAL:** All business logic, state machines, and flow control must be here for academic evaluation.
- `main/include/pins.h`: Centralized pin definitions.

## 2. Hardware Mapping (STRICT ADHERENCE)
- **HC-SR04 A (Hands):** Trig=12, Echo=13
- **HC-SR04 B (Water):** Trig=14, Echo=27
- **I2C LCD:** SDA=21, SCL=22
- **Relay (Pump):** GPIO 25
- **Touch (Refill):** GPIO 4

## 3. Core Logic Requirements (In main.cpp)
- **Hand Detection:** Distance < 15cm -> Pump ON. Hysteresis: Off-limit > 22cm + debounce.
- **Water Management:** If Level < Critical -> Hard Lock Pump OFF.
- **Smoothing:** Implement a Moving Average Filter for water level readings to prevent LCD flicker.
- **Refill Mode:** Triggered via Touch Pin. **STRICT REQUIREMENT:** Force Pump OFF (Lockout) during refill because the water level sensor will be detached. Display "REFILL MODE - LOCKED" on LCD.

## 4. IoT Strategy
- **MQTT:** Primary telemetry for real-time monitoring.
- **Blynk:** Use as a secondary data logger with offline sync capabilities.
- **Reliability:** Implement a simple buffer for sensor data when Wi-Fi is unavailable.

## 5. Output Format
- Provide the complete directory structure.
- Provide all source and header files separately.
- Include a `CMakeLists.txt` for the root and components.
- Include brief comments explaining the logic flow in `main.cpp`.
