# IOT_SPEC.md: Telemetry & Communication

This document details the IoT communication strategy.

## 1. Connectivity
*   **Protocol**: Wi-Fi enabled telemetry.

## 2. Telemetry Strategy
*   **Primary Metric**: Water Level (demonstrated to be the most stable data point).
*   **Fallback Mechanism**: In scenarios where "Water Outflow" metrics conflict with level sensor data, the water level readings take precedence for system validation.
*   **Data Points**:
    *   Water Level (Primary)
    *   Water Inflow (Tracking)
    *   Water Outflow (Tracking)
