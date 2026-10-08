#ifndef UI_SERIAL_WIRE_H
#define UI_SERIAL_WIRE_H
#include <stdio.h>
#include <stdint.h>
#include "../vehicle_ctrl/vehicle_ctrl.h"
/* CRC-16/CCITT-FALSE: corrupted/interleaved debug lines are ignored by PC. */
static int UiSerialFormat(char *dst, unsigned capacity, int pressure, int angle,
                          int valid, unsigned age, const VehicleUiState *s, int demo)
{
    unsigned i, bit;
    uint16_t crc = 0xffffU;
    int count = snprintf(dst, capacity,
        "HUD4,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        pressure, angle, valid, age, s->accel_raw, s->brake_raw,
        s->sensor_ok, s->drive_enabled, s->reverse, s->command_accel,
        s->command_brake, s->command_estop, s->brake_active, demo);
    if (count < 0 || (unsigned)count + 6U > capacity) return -1;
    for (i = 0; i < (unsigned)count; ++i) {
        crc ^= (uint16_t)((unsigned char)dst[i]) << 8;
        for (bit = 0; bit < 8U; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? (crc << 1) ^ 0x1021U : crc << 1);
    }
    return count + snprintf(dst + count, capacity - count, "*%04X", (unsigned)crc);
}
#endif
