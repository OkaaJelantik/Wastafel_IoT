# Design Spec: Wastafel_IoT Clean Architecture Refactor
**Date:** 2026-06-03
**Status:** Approved

## 1. Overview
The goal is to refactor the current "God File" `main.cpp` into a clean, service-oriented architecture. This ensures that the academic evaluation focuses on the **Business Logic** and **System Flow** while technical details (drivers, networking, data filtering) are isolated into specialized components.

## 2. Component Architecture

### A. The Brain (`main/main.cpp`)
- **Role:** Orchestration, State Machine, and System Hardening.
- **Contents:**
    - `SystemState` Enum.
    - Global variables for system status (Current Volume, Current State, etc.).
    - `control_task`: The heart of the state machine.
    - `logic_functions`: High-level functions like `process_hand_detection()`, `enforce_safety_limits()`.
    - `app_main`: System initialization call (delegating to services).

### B. Hardware Wrapper (`components/hardware_manager`)
- **Role:** Abstraction for all physical IO.
- **Interface:**
    - `float get_hand_distance()`
    - `float get_water_distance_raw()`
    - `void set_pump_state(bool on)`
    - `bool is_refill_button_pressed()`
- **Implementation:** Wraps `HCSR04`, `Relay` (GPIO), and `TouchPad`.

### C. IoT Service (`components/iot_service`)
- **Role:** Handles all external communication.
- **Interface:**
    - `void iot_init()`
    - `void iot_publish_status(float hand, float water, bool pump, int state)`
    - `bool iot_is_connected()`
- **Implementation:** Wraps WiFi, MQTT, Blynk, and the Offline Buffer logic.

### D. Display Service (`components/display_service`)
- **Role:** Handles the 1602 LCD.
- **Interface:**
    - `void display_init()`
    - `void display_status(SystemState state, float value1, float value2)`
- **Implementation:** Wraps `I2C_LCD` and manages string formatting/line management to reduce `main.cpp` clutter.

### E. Data Processor (`components/data_processing`)
- **Role:** Mathematical filters and calculations.
- **Contents:**
    - `MovingAverage` class.
    - Volume calculation logic (Area * Height).

## 3. Interaction Flow
1. `app_main` calls `init()` on all Services.
2. `sensor_task` updates the `HardwareWrapper` and applies `DataProcessor` filters.
3. `control_task` (The Brain) reads high-level data from `HardwareWrapper`.
4. `control_task` makes decisions (e.g., turn on pump) and commands `HardwareWrapper`.
5. `iot_task` periodically grabs state from `main.cpp` and sends via `IoTService`.
6. `lcd_task` calls `DisplayService` based on current `SystemState`.

## 4. Academic Benefits
- **Readability:** A lecturer can read `main.cpp` and understand the entire logic without being distracted by MQTT JSON formatting or I2C bus initialization.
- **Traceability:** Logic paths (e.g., "Why did the pump stop?") are explicitly coded in `main.cpp` using descriptive function names.
- **Modularity:** Demonstrates professional software engineering practices (SOLID principles).
