# MAIN_FLOW.md: Logic & Threading

This document outlines the state machine behavior and the threading isolation strategy for volume calculation.

## 1. State Machine Definitions

*   **[LABEL: STATE_IDLE]**: Normal system operation, waiting for interaction.
*   **[LABEL: STATE_PUMPING]**: Water is being dispensed. A snapshot of the tank volume is taken at the onset of this state to facilitate delta calculation.
*   **[LABEL: STATE_STABILIZING]**: Post-pumping phase. The system waits for surface ripples to subside before finalizing volume calculations.
*   **[LABEL: STATE_CALCULATING]**: Computation phase. Isolated from the main loop to prevent noise from affecting critical decisions.
*   **[LABEL: STATE_SHOW_RESULT]**: UI update phase displaying dispensed volume.
*   **[LABEL: STATE_REFILL]**: Handling water inflow, distinct from outflow logic.

## 2. Threading & Calculation Isolation
To maintain data integrity, volume calculations are performed in a dedicated context:
1.  **Isolation**: Prevents sensor fluctuations from causing negative volume readings or erratic pump behavior.
2.  **Concurrency**: Allows the pump to continue operation while calculations are processed, preventing system lockups.
3.  **Persistence**: The calculation state is maintained even if pumping is interrupted, avoiding restart loops.

## 3. Critical Constraints
*   **Safety Rule A**: Ensure pump tracking persists despite movement detected by the sensor.
*   **Safety Rule B**: Enforce hard floor limits for water level to prevent pump damage.
