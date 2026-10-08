#ifndef VEHICLE_H
#define VEHICLE_H

// ==================================================
// Zybo command synthesis -- public interface
//
// The Zybo sits between two links it does not control: pedal
// samples arrive over a wireless hop, and steering comes from a
// source that is swapped at build time. Everything below is what
// main() needs to drive one control cycle; the decoding, the
// link timing and the validation stay in vehicle.c.
//
// Wiring:
//   UART0  TX -> ESP32 #1 -> ESP32 #2      (CommandPacket)
//          RX <- ESP32 #1 <- ESP32 #3      (SensorPacket)
//   UART1  PC keyboard and debug output
// ==================================================

#include <stdint.h>

#include "xuartps.h"
#include "xtime_l.h"


// ==================================================
// Safety
// ==================================================
#define FLAG_EMERGENCY_STOP        0x01

// Wall-clock timeouts.
//
// These used to be loop-iteration counts, which tied the safety
// timing to how long one pass of the loop happened to take. The
// sensor node sends at a fixed 50 Hz from its own clock, so any
// drift between the two rates changed how long "lost" took to
// trigger. Real time removes that coupling.
#define PC_COMMAND_TIMEOUT_MS      500
#define SENSOR_TIMEOUT_MS          200


// ==================================================
// Steering Source
//
// 0 = PC keyboard over UART1 (development)
// 1 = CNN accelerator RESULT register
//
// The CNN base address only exists in xparameters.h once the
// accelerator is instantiated in the block design, so the
// register access stays behind this switch. With it at 0 the
// whole pipeline still runs on keyboard steering, which is how
// the FSR thresholds get calibrated.
//
// The PC keeps the manual stop in both modes.
// ==================================================
#define STEERING_SOURCE_CNN        1


// How long a CNN result stays usable.
//
// The RESULT register holds its last value indefinitely, so
// without this a stalled accelerator would steer the car on a
// frozen reading while every other link still looks healthy.
#define CNN_TIMEOUT_MS             200


#if STEERING_SOURCE_CNN
// Register map of the CNN team's AXI4-Lite slave.
//
// CONTROL  bit0 start / bit1 irq clear / bit2 irq enable
// STATUS   bit0 busy  / bit1 ready     / bit2 done / bit3 pending
// RESULT   bits 7:0 carry the steering angle, zero-extended
//
// CONTROL bit0 acts on the data being written rather than on the
// stored register, so writing the start mask repeatedly is how a
// new inference is requested.
//
// CNN_BASEADDR is a placeholder: the real macro only appears in
// xparameters.h once the accelerator is in the block design, and
// it is named after the IP (AXI4_Lite_interconnect).
#endif

// Command output rate to ESP32 #1: 10 ms = 100 Hz.
//
// This runs faster than anything feeding it. The sensor node and
// the PC both send at 50 Hz, and ESP32 #2 drives the motors at
// 50 Hz, so every other command repeats the previous sample and
// is consumed without effect. What it buys is latency: an input
// that changes just after a send waits 10 ms instead of 20 ms.
//
// Raising the sensor node to match is not just a constant. It
// reads two ADS1115 channels per cycle, and each conversion is
// waited out open-loop (delay(2) against 1.16 ms at 860 SPS),
// which with the I2C transactions costs roughly 5-6 ms of a
// 10 ms budget. Going to 100 Hz there means shortening that
// wait or reading one channel per cycle.
#define CONTROL_PERIOD_US          10000


// ==================================================
// Sensor Plausibility Limits
//
// The sensor path only checked SOF and CRC, so a corrupt but
// CRC-valid sample, a wrong PGA setting, or broken FSR wiring
// could be read straight through as full throttle.
//
// ADS1115 at PGA +/-4.096 V gives 32767 counts for 4.096 V. The
// dividers run from 3.3 V, so nothing above ~26400 counts is
// physically reachable; anything higher means a fault.
// ==================================================
#define SENSOR_RAW_MAX             26400
#define SENSOR_RAW_MIN             (-1000)

// Largest believable change between two 20 ms samples. A pedal
// pressed as fast as a person can manage still takes ~80 ms to
// travel full scale (~6600 counts per sample), so this leaves
// more than 2x headroom while still rejecting an instantaneous
// zero-to-full jump.
#define SENSOR_MAX_STEP            26000


// ==================================================
// VEHICLE COMMAND PACKET
//
// Zybo
//   -> UART0 TX
//   -> ESP32 #1
//   -> ESP-NOW
//   -> ESP32 #2
//
// 8 bytes
//
// Byte 0 : 0xAA
// Byte 1 : 0x55
// Byte 2 : sequence
// Byte 3 : steering
// Byte 4 : accel
// Byte 5 : brake
// Byte 6 : flags
// Byte 7 : CRC8
// ==================================================
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;

    uint8_t sequence;

    int8_t steering;

    uint8_t accel;
    uint8_t brake;

    uint8_t flags;

    uint8_t crc;

} CommandPacket;


// ==================================================
// SENSOR PACKET
//
// ESP32 #3
//   -> ESP-NOW
//   -> ESP32 #1
//   -> UART0 RX
//   -> Zybo
//
// 8 bytes
//
// Byte 0 : 0xA5
// Byte 1 : 0x5A
// Byte 2 : sequence
// Byte 3~4 : accelRaw
// Byte 5~6 : brakeRaw
// Byte 7 : CRC8
// ==================================================
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;

    uint8_t sequence;

    int16_t accelRaw;
    int16_t brakeRaw;

    uint8_t crc;

} SensorPacket;


// ==================================================
// Pedal input for one cycle
//
// levels are already 0 when the link is down, so a caller can
// use them without checking linkOK first.
// ==================================================
typedef struct
{
    int      linkOK;
    int16_t  accelRaw;
    int16_t  brakeRaw;
    uint8_t  sequence;
    uint8_t  accelLevel;
    uint8_t  brakeLevel;

} SensorState;


// ==================================================
// PC input for one cycle
//
// The PC carries the manual stop in both steering modes, so
// emergencyStop is meaningful even when the CNN owns steering.
// ==================================================
typedef struct
{
    int     linkOK;
    int8_t  steering;
    int     emergencyStop;

} PcState;


// ==================================================
// Setup
//
// Brings up both UARTs and starts every link in the lost state,
// so nothing moves until real traffic arrives.
// ==================================================
int vehicleInit(void);


// ==================================================
// One cycle of input
//
// Each drains its UART, refreshes its link timestamp and reports
// what the control loop should act on.
// ==================================================
void vehiclePollPC(PcState *out);

void vehiclePollSensor(SensorState *out);


// ==================================================
// Steering source
//
// Returns 1 when *steering holds a usable value, 0 when the
// source is stale and the caller must raise E-Stop. Which source
// is read depends on STEERING_SOURCE_CNN.
// ==================================================
int getSteering(int pcLinkOK, int8_t *steering);

// ==================================================
// CNN Steering Update
//
// main.c owns the CNN accelerator.
// When a new CNN result is ready, main.c passes it
// to vehicle.c through this function.
// ==================================================
#if STEERING_SOURCE_CNN
void vehicleSetCnnSteering(
    int8_t steering,
    XTime completed_at
);
#endif

// ==================================================
// Output
// ==================================================
void sendCommandPacket(
    int8_t  steering,
    uint8_t accel,
    uint8_t brake,
    uint8_t flags
);


// ==================================================
// Loop timing
//
// holdControlPeriod sleeps only the remainder of the period, so
// the work above it does not push the rate around.
// ==================================================
uint32_t elapsedMs(XTime since);

uint32_t elapsedUs(XTime since);

void holdControlPeriod(XTime cycleStart);


#endif  // VEHICLE_H
