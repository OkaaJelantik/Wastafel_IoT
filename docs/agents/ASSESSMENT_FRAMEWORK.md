# Sensor Assessment Framework (Wastafel IoT)

This framework provides a structured, incremental approach to validate hardware components and their drivers in the Wastafel IoT project.

## 1. Testing Strategy
- **Incremental Validation:** Add and test one component at a time.
- **Isolation:** Do not proceed to the next component until the current one is verified.
- **Environment:** Build, flash, and monitor logs using `idf.py monitor`.

## 2. Assessment Steps

### Step 1: Core System & Logs
- **Action:** Ensure the base system boots and log output is functional.
- **Verification:** Look for `TAG: "Wastafel_IoT"` messages in serial monitor.

### Step 2: I2C LCD Component
- **Action:** Initialize I2C driver and LCD component.
- **Verification:** Display a test message ("LCD Ready") on the screen.
- **Diagnostic Checklist (Failure Modes):**
  - [ ] **Backlight but no text?** Adjust the potentiometer on the back of the I2C backpack.
  - [ ] **Gibberish / Random Characters?** 
    - Check for loose SDA/SCL wires.
    - Verify I2C pull-up resistors (ESP32 internal might not be enough; use 4.7kΩ external).
    - Lower I2C frequency in `i2c_lcd.cpp` (current: 100kHz).
  - [ ] **Dislocation (Text not centered)?** 
    - Check row/column offsets in `i2c_lcd.cpp`.
    - Verify if it's a 16x2 or 20x4 model.
  - [ ] **No Response?** Use `i2c_scanner` (logic can be added) to verify address (0x27 or 0x3F).

### Step 3: HC-SR04 A (Hands Sensor)
- **Action:** Initialize hand sensor and print distance readings.
- **Verification:** Monitor `TAG: "HCSR04"` and check if distance readings change when a hand is placed in front of the sensor.
- **Diagnostic Checklist (Failure Modes):**
  - [ ] **Constant 0cm or 400cm?** 
    - Check if sensor is powered by 5V (most HC-SR04 need 5V).
    - Ensure Echo pin has a voltage divider if connecting a 5V echo to 3.3V ESP32.
  - [ ] **Jitter / Fluctuating Readings?**
    - Shield the wires (ultrasonic pulses are sensitive to noise).
    - Add a small capacitor (100uF) across sensor VCC and GND.
  - [ ] **"Frozen" readings?** 
    - Verify TRIG pulse width (10us).
    - Check if MCPWM capture group is shared incorrectly.

### Step 4: HC-SR04 B (Water Level Sensor)
- **Action:** Initialize water level sensor independently.
- **Verification:** Same as HC-SR04 A.
- **Diagnostic Checklist:**
  - [ ] **Crosstalk?** Ensure sensors are pointed away from each other or increase the delay between measurements to allow echoes to dissipate.

### Step 5: Touch Pad (Manual Refill)
- **Action:** Initialize touch driver and print touch state.
- **Verification:** Monitor touch sensor read value during press/release.
- **Checklist:**
  - [ ] Threshold sensitivity calibrated?

### Step 6: Full System Integration
- **Action:** Enable all components and verify main loop logic.
- **Verification:** Functional testing of sink automation.
