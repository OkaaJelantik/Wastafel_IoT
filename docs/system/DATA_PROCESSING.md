# DATA_PROCESSING.md: Filtering & Calculation

This document outlines how sensor data is processed.

## 1. Data Filtering
*   **Sensor Noise Mitigation**: HC-SR04 readings undergo filtering to remove outliers caused by surface ripples in `STATE_STABILIZING`.
*   **Volume Calculation**: Delta volume is calculated by comparing stable states before and after pumping, ensuring accurate measurement.
