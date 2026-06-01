# Proposed Pinout (Breadboard Optimized)

## Goal: High physical organization and EMI reduction.

### 1. Left Rail (Sensors - Grouped)
- **HC-SR04 A (Hands):** GPIO 12 (Trig), GPIO 13 (Echo)
- **HC-SR04 B (Water):** GPIO 14 (Trig), GPIO 27 (Echo)
*Rationale: Pins 12, 13, 14, 27 are all together on the left side.*

### 2. Right Rail (Control & Display - Spaced)
- **I2C LCD:** GPIO 21 (SDA), GPIO 22 (SCL)
- **Relay 1 (Pump):** GPIO 25
- **Relay 2 (Spare):** GPIO 26
- **Touch Pin (Refill):** GPIO 4
*Rationale: Keeps I2C separate from switching noise of relays.*

### 3. Power Distribution
- **5V Rail:** To Relays & LCD.
- **3.3V Rail:** To HC-SR04 (Check compatibility, some need 5V).
- **GND:** Central star-ground on breadboard rail.
