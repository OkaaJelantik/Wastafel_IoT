# ORCHESTRA.md: Master Vision & Architecture

This document acts as the central hub for the `Wastafel_RRQSahabatan` project, defining the core architectural vision and system-level guidelines.

## 1. Project Vision
*   **Project Name**: Wastafel_RRQSahabatan
*   **Core Purpose**: Implementasi matkul Sensor & IoT.
*   **Fundamental Philosophy**: Data Stability. Real-time sensor data (especially HC-SR04) is inherently volatile. The system must decouple raw sensor ingestion from actionable logic (UI/Database reporting).

## 2. Core Concepts
*   **Calculated vs Real-time**:
    *   **Trigger Actions (Pump/Safety)**: Use raw/filtered real-time data for immediate response.
    *   **Reporting (UI/DB)**: Use stabilized, calculated values to prevent artifacts (e.g., negative volume during ripples).
*   **Threading Isolation**: Calculation tasks are isolated from the main control loop, ensuring sensor noise does not cause system-level misbehavior.

## 3. System Components
*   [HARDWARE.md](./HARDWARE.md): Pinout and hardware integration strategies.
*   [MAIN_FLOW.md](./MAIN_FLOW.md): State machine and threading logic.
*   [IOT_SPEC.md](./IOT_SPEC.md): Telemetry and cloud communication.
*   [DISPLAY_SPEC.md](./DISPLAY_SPEC.md): UI feedback strategies.
*   [DATA_PROCESSING.md](./DATA_PROCESSING.md): Filtering and calculation algorithms.
