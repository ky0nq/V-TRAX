#ifndef VEHICLE_DRIVER_H
#define VEHICLE_DRIVER_H

#include "xil_types.h"
#include "xtime_l.h"
#include <stdint.h>

/* Raw byte I/O + wire-protocol framing for both UARTs.
 * (Everything the outside world needs is already in vehicle.h;
 *  this header only exposes what vehicle_ctrl.c additionally needs
 *  to drive the Driver layer directly.) */

extern int8_t  pcSteering;
extern uint8_t pcEmergencyStop;

extern int16_t latestAccelRaw;
extern int16_t latestBrakeRaw;
extern uint8_t latestSensorSequence;
extern int     sensorHistoryValid;

/* Defined in vehicle_ctrl.c and initialized by vehicleInit()
 * in vehicle_driver.c. */
extern XTime lastPcCommandTime;
extern XTime lastSensorPacketTime;

int processPCSerial(void);
int processSensorUART(void);

#endif /* VEHICLE_DRIVER_H */
