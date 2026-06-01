# Project: Wastafel_IoT
**Status:** Initialization
**Date:** 2026-06-01

## 1. Vision
An automated smart sink system using ESP32 (C++) with a modular object-oriented architecture.

## 2. Technical Stack
- **Language:** C++ (OOP for modularity)
- **Framework:** ESP-IDF
- **IoT Platforms:** MQTT & Blynk (Basic)
- **Modularity:** Separate classes for `Sensor`, `Pump`, `Display`.

## 3. Logic & State Management (from project_overview.md)
### Hand Detection (HC-SR04 A)
- **Trigger:** Distance < 15cm -> Pump ON.
- **Hysteresis:** Off-limit > 22cm + quick countdown (debounce) to confirm exit.
- **Safety:** Must check Water Level before activation.

### Water Level Management (HC-SR04 B)
- **Monitoring:** Real-time level display.
- **Critical Path:** If level < MIN_THRESHOLD -> Hard Disable Pump.
### Refill Mode (Touch Pin Triggered)
- **Condition:** Touch Pin (GPIO 4) Active.
- **Safety Action:** **Force Pump OFF (Hard Lockout).** 
- **Reason:** Water Level sensor is detached during refill; readings are invalid.
- **LCD Status:** Display "REFILL MODE" and "PUMP LOCKED".

## 4. Proposed IoT Features (For Selection)
- **Telemetry:** Real-time water level % and Pump state.
- **Alerts:** "Low Water" push notification via Blynk.
- **Remote Toggle:** Manual Override Pump ON/OFF.
- **Stats:** Total "Pumping Time" or estimated water usage.

## 3. Milestones
1. **Foundation:** Framework setup and workflow validation. [IN PROGRESS]
2. **Research:** Hardware component selection and pinout design.
3. **Initialization:** Claude Opus generates core boilerplate.
4. **Development:** Iterative feature implementation.

## 4. Active Task
Defining the initial Research Brief for Perplexity.
