#ifndef VEHICLE_TASK_H
#define VEHICLE_TASK_H

typedef struct {
    int accel_raw;
    int brake_raw;
    int brake_active;
    int sensor_ok;
    int drive_enabled;
    int reverse;
    int command_accel;
    int command_brake;
    int command_estop;
} VehicleUiState;
/* Read-only UI snapshot. Does not change vehicle control decisions. */
void VehicleReadUiState(VehicleUiState *out);

extern int vehicle_control_ready;
extern volatile int ui_pressure_percent;

void VehicleControlPoll(void);
/* Call from the main loop after publishing a completed CNN steering result. */
void VehicleCnnUpdateRequest(void);
/* Consume JA1 IRQs: TEST is diagnostic-only; DEMO controls drive permission. */
void VehicleJa1Poll(void);

#endif /* VEHICLE_TASK_H */
