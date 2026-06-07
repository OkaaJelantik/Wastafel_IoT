# HARDWARE.md: Pinout & Integration

This document defines the physical interface requirements for the `Wastafel_RRQSahabatan` project.

## 1. Pin Mapping (ESP32 DevKit V1)

| Peripheral | Component | GPIO |
| :--- | :--- | :--- |
| **Hand Sensor** | HC-SR04 | TRIG: 32, ECHO: 33 |
| **Water Sensor** | HC-SR04 | TRIG: 5, ECHO: 18 |
| **I2C LCD** | PCF8574 | SDA: 21, SCL: 22 (0x27) |
| **Relay** | Water Pump | 26 (Active High) |
| **Touch Pad** | Refill Trigger| 4 (Pad 0) |

## 2. Hardware Debugging Strategy
*   **Sensor Strategy**: Real-time behavior is critical. Debugging employs isolated tracing (via UART/Serial) to separate sensor data ingestion from the LCD display loop, ensuring performance stability during debugging.
