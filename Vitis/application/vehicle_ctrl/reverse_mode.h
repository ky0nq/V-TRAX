#ifndef ADDITIONALFEATURE_H
#define ADDITIONALFEATURE_H

/*
 * reverse_mode.h
 *
 * JA2-only reverse-mode feature.
 *
 * JA1 is intentionally NOT referenced here.
 * JA1 remains owned by vehicle_ctrl.c for vehicle ARM/STOP.
 *
 * JA2 behavior:
 *   press 1 -> NORMAL -> REVERSE
 *   press 2 -> REVERSE -> NORMAL
 *   press 3 -> NORMAL -> REVERSE
 *   ...
 *
 * Reverse mode:
 *   - steering uses the selected steering value supplied by vehicle_ctrl.c
 *   - accelerator uses an independent reverse-mode threshold table
 *   - brake uses an independent reverse-mode threshold table
 *   - CommandPacket.flags bit 1 marks REVERSE mode
 *
 * The existing 8-byte CommandPacket size is unchanged.
 */

#include <stdint.h>


// ==================================================
// Packet flag extension
//
// bit 0 : FLAG_EMERGENCY_STOP (vehicle.h)
// bit 1 : reverse mode
// ==================================================
#ifndef FLAG_REVERSE_MODE
#define FLAG_REVERSE_MODE 0x02U
#endif


// ==================================================
// JA2 IRQ cooldown is configured by JA_GPIO_COOLDOWN_MS
// in driver/video_pipeline_driver/video_pipeline_driver.h.
// ==================================================


// ==================================================
// Reverse steering sign
//
// 0 : use the selected steering value exactly as supplied
// 1 : invert the supplied steering sign in reverse mode
//
// Start at 0. Change only after real-vehicle testing if needed.
// ==================================================
#define ADDITIONAL_FEATURE_INVERT_CNN_STEERING 0U


typedef enum
{
    ADDITIONAL_FEATURE_NORMAL = 0,
    ADDITIONAL_FEATURE_REVERSE = 1

} AdditionalFeatureMode;


// ==================================================
// JA2 mode control
// ==================================================

/*
 * Initialize JA2 mode handling.
 *
 * Startup mode is always NORMAL.
 * The first observed IRQ is accepted immediately; further IRQs on JA2
 * are ignored for 300 ms after the last accepted event.
 */
void AdditionalFeatureInit(void);


/*
 * Consume one latched JA2 press event.
 *
 * Call this frequently from the main super-loop, independently
 * from VehicleJa1Poll().
 *
 * Return:
 *   1 = mode changed on this call
 *   0 = no mode change
 */
int AdditionalFeaturePoll(void);


/*
 * 1 = REVERSE
 * 0 = NORMAL
 */
int AdditionalFeatureIsActive(void);


AdditionalFeatureMode AdditionalFeatureGetMode(void);


const char *AdditionalFeatureModeName(void);


// ==================================================
// Reverse-mode pedal conversion
//
// Threshold values live in reverse_mode.c so they can be
// calibrated independently from the forward-mode thresholds in
// vehicle_ctrl.c.
// ==================================================
uint8_t AdditionalFeatureAccelLevel(int16_t raw);

uint8_t AdditionalFeatureBrakeLevel(int16_t raw);


// ==================================================
// Apply reverse-mode command transformation
//
// NORMAL:
//   no values are changed.
//
// REVERSE:
//   steering   = supplied steering, optionally sign-inverted as configured above
//   accelLevel = reverse accelerator threshold mapping
//   brakeLevel = reverse brake threshold mapping
//   flags     |= FLAG_REVERSE_MODE
//
// Brake has priority over accelerator.
//
// IMPORTANT:
//   PC E-stop, sensor-link safety, CNN freshness, vehicle_armed,
//   and JA1 state remain owned by vehicle_ctrl.c.
//   This function does not read or modify JA1.
// ==================================================
void AdditionalFeatureApply(
    int16_t accelRaw,
    int16_t brakeRaw,
    int8_t cnnSteering,
    int8_t *steering,
    uint8_t *accelLevel,
    uint8_t *brakeLevel,
    uint8_t *flags
);

#endif /* ADDITIONALFEATURE_H */
