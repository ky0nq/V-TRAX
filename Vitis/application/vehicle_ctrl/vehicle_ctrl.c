#include "vehicle_ctrl.h"
#include "../../driver/vehicle_driver/vehicle.h"
#include "../../driver/vehicle_driver/vehicle_driver.h"
#include "../capture_cnn_ctrl/capture_cnn_ctrl.h"
#include "../../driver/video_pipeline_driver/video_pipeline_driver.h"
#include "../../driver/gpio_driver/gpio_driver.h"
#include "xil_printf.h"
#include "xtime_l.h"
#include <stdint.h>
#include "reverse_mode.h"

int vehicle_control_ready = 0;
volatile int ui_pressure_percent = 0;

/* JA1 press events are latched by the GPIO IRQ handler in
 * video_pipeline_driver.c using a per-pin cooldown. */
static unsigned int vehicle_armed = 0U;
static unsigned int vehicle_start_pending = 0U;
static unsigned int vehicle_stop_pending = 0U;
/* Main-loop request: send the newly published CNN steering without a new tick. */
static unsigned int vehicle_cnn_update_pending = 0U;
static int ui_accel_raw, ui_brake_raw, ui_brake_active, ui_sensor_ok;
static int ui_command_accel, ui_command_brake, ui_command_estop = 1;

void VehicleReadUiState(VehicleUiState *out)
{
    if (!out) return;
    out->accel_raw = ui_accel_raw;
    out->brake_raw = ui_brake_raw;
    out->brake_active = ui_brake_active;
    out->sensor_ok = ui_sensor_ok;
    out->drive_enabled = vehicle_control_ready && vehicle_armed &&
        CaptureGetMode() == CAPTURE_MODE_DEMO;
    out->reverse = AdditionalFeatureIsActive();
    out->command_accel = ui_command_accel;
    out->command_brake = ui_command_brake;
    out->command_estop = ui_command_estop;
}

static const int16_t accelThreshold[5] = {
    7000, 10000, 14000, 17000, 20000
};
uint8_t rawToLevel(int16_t raw, const int16_t threshold[5]);

void VehicleJa1Poll(void)
{
    static unsigned int initialized = 0U;
    static CaptureMode mode_seen = CAPTURE_MODE_TEST;
    CaptureMode mode = CaptureGetMode();

    /* Retain the existing startup/TEST/DEMO transition policy. */
    if (!initialized || mode != mode_seen || mode == CAPTURE_MODE_TEST) {
        vehicle_armed = 0U;
        vehicle_start_pending = 0U;
        vehicle_stop_pending = mode == CAPTURE_MODE_DEMO ? 1U : 0U;
        Ja1PressDiscard();
        mode_seen = mode;
        initialized = 1U;
        return;
    }

    /* Leave later events queued until the previous request is processed. */
    if (vehicle_start_pending || vehicle_stop_pending)
        return;
    if (!Ja1PressTake())
        return;

    if (vehicle_armed) {
        vehicle_armed = 0U;
        vehicle_start_pending = 0U;
        vehicle_stop_pending = 1U;
    } else {
        /* Button release cannot cancel an accepted request.
         * VehicleControlPoll still checks E-stop, sensors, steering and accel.
         */
        vehicle_start_pending = 1U;
    }
}

void VehicleCnnUpdateRequest(void)
{
    if (CaptureGetMode() == CAPTURE_MODE_DEMO)
        vehicle_cnn_update_pending = 1U;
}

void VehicleControlPoll(void)
{
    static u32 timer_seen = 0U;
    static unsigned int timer_initialized = 0U;
    static unsigned int debug_counter = 0U;
    u32 tick;
    PcState pc;
    SensorState sensor;
    int8_t steering = 0;
    int steering_ok;
    uint8_t accel_level;
    uint8_t brake_level;
    uint8_t flags = 0U;

    if (!vehicle_control_ready)
        return;

    tick = timer_irq_count;
    /* Discard initialization ticks, then service each observed IRQ event.
     * This consumer is independent of CaptureTimerPoll's tick tracking. */
    if (!timer_initialized) {
        timer_seen = tick;
        timer_initialized = 1U;
        if (!vehicle_stop_pending && !vehicle_start_pending &&
            !vehicle_cnn_update_pending)
            return;
    }
    if (tick == timer_seen && !vehicle_stop_pending && !vehicle_start_pending &&
        !vehicle_cnn_update_pending)
        return;
    /* Coalesce missed ticks instead of sending stale commands in a burst. */
    timer_seen = tick;
    /* Consume once; all normal input validation and stop overrides still apply. */
    vehicle_cnn_update_pending = 0U;

    vehiclePollSensor(&sensor);
    /* Publish sensor input BEFORE brake/arming overrides motor commands. */
    ui_accel_raw = sensor.accelRaw;
    ui_brake_raw = sensor.brakeRaw;
    ui_sensor_ok = sensor.linkOK;
    ui_brake_active = sensor.linkOK && sensor.brakeLevel > 0U;
    ui_pressure_percent = !sensor.linkOK || sensor.accelRaw <= 7000 ? 0 :
        sensor.accelRaw >= 22000 ? 100 :
        ((int)sensor.accelRaw - 7000) * 100 / (22000 - 7000);

    /* TEST keeps ESP reception but leaves console RX and vehicle TX alone. */
    if (CaptureGetMode() == CAPTURE_MODE_TEST)
        return;

    vehiclePollPC(&pc);

    steering_ok = getSteering(pc.linkOK, &steering);
    accel_level = sensor.accelLevel;
    brake_level = sensor.brakeLevel;

    AdditionalFeatureApply(
        sensor.accelRaw,
        sensor.brakeRaw,
        steering,
        &steering,
        &accel_level,
        &brake_level,
        &flags
    );

    if (pc.emergencyStop)
        flags |= FLAG_EMERGENCY_STOP;
    if (!sensor.linkOK)
        flags |= FLAG_EMERGENCY_STOP;
    if (!steering_ok)
        flags |= FLAG_EMERGENCY_STOP;

    if (vehicle_start_pending) {
        vehicle_start_pending = 0U;
        /* Use raw accel: accelLevel is also zero when the brake overrides it. */

        int accel_released =
            AdditionalFeatureIsActive()
                ? (AdditionalFeatureAccelLevel(sensor.accelRaw) == 0U)
                : (rawToLevel(sensor.accelRaw, accelThreshold) == 0U);

        if ((flags & FLAG_EMERGENCY_STOP) == 0U && accel_released) {
            vehicle_armed = 1U;
            xil_printf("JA1: vehicle armed (%s)\r\n", AdditionalFeatureModeName());
        } else {
            xil_printf("JA1 denied: PC_STOP=%d SENSOR_OK=%d STEERING_OK=%d ACC_RELEASED=%d ACC_RAW=%d BRAKE_RAW=%d\r\n",
                pc.emergencyStop ? 1 : 0, sensor.linkOK ? 1 : 0,
                steering_ok ? 1 : 0, accel_released,
                sensor.accelRaw, sensor.brakeRaw);
        }
    }
    /* A fault also disarms: recovery alone must not restart the vehicle. */
    if (flags & FLAG_EMERGENCY_STOP)
        vehicle_armed = 0U;
    if (!vehicle_armed)
        flags |= FLAG_EMERGENCY_STOP;

    if (flags & FLAG_EMERGENCY_STOP) {
        accel_level = 0U;
        brake_level = 0U;
    }

    /* Mirror the final packet after all control overrides. */
    ui_command_accel = accel_level;
    ui_command_brake = brake_level;
    ui_command_estop = (flags & FLAG_EMERGENCY_STOP) != 0U;
    sendCommandPacket(steering, accel_level, brake_level, flags);
    if (vehicle_stop_pending) {
        vehicle_stop_pending = 0U;
        xil_printf("JA1: vehicle stopped; release then press again to arm\r\n");
    }
// ======================= Commented out for now ===============================
    /* About 10 Hz diagnostic output at a 100 Hz control rate. */
    /*if ((debug_counter % 10U) == 0U) {
        xil_printf("SENSOR_SEQ=%d "
                   "ACC_RAW=%d A=%d "
                   "BRAKE_RAW=%d B=%d "
                   "SENSOR=%s "
                   "STEER=%d "
                   "STEER_SRC=%s:%s "
                   "ESTOP=%d "
                   "PC=%s\r\n",
                   sensor.sequence,
                   sensor.accelRaw, accel_level,
                   sensor.brakeRaw, brake_level,
                   sensor.linkOK ? "OK" : "LOST",
                   steering,
#if STEERING_SOURCE_CNN
                   "CNN",
#else
                   "PC",
#endif
                   steering_ok ? "OK" : "STALE",
                   (flags & FLAG_EMERGENCY_STOP) ? 1 : 0,
                   pc.linkOK ? "OK" : "LOST");
    }*/

    // =========================================================
    ++debug_counter;

    /*xil_printf(
        "ESTOP=%d "
        "E_PC=%d E_SENSOR=%d E_CNN=%d "
        "PC=%s\r\n",

        (flags & FLAG_EMERGENCY_STOP) ? 1 : 0,

        pc.emergencyStop ? 1 : 0,
        !sensor.linkOK ? 1 : 0,
        !steering_ok ? 1 : 0,

        pc.linkOK ? "OK" : "LOST"
    );*/
}

/* Vehicle input state, validation and control-policy helpers. */

static const int16_t brakeThreshold[5] =
{
    10000,
    11000,
    12000,
    13000,
    14000
};



int pcLinkEverSeen = 0;


XTime lastPcCommandTime;

static int8_t cnnSteering = 0;


static int cnnEverSeen = 0;


static XTime lastCnnUpdateTime;

int sensorLinkEverSeen = 0;


XTime lastSensorPacketTime;

uint8_t rawToLevel(
    int16_t raw,
    const int16_t threshold[5]
)
{
    if (
        raw
        <
        0
    )
    {
        raw =
            0;
    }


    if (
        raw
        <
        threshold[0]
    )
    {
        return 0;
    }


    if (
        raw
        <
        threshold[1]
    )
    {
        return 1;
    }


    if (
        raw
        <
        threshold[2]
    )
    {
        return 2;
    }


    if (
        raw
        <
        threshold[3]
    )
    {
        return 3;
    }


    if (
        raw
        <
        threshold[4]
    )
    {
        return 4;
    }


    return 5;
}

void vehicleSetCnnSteering(
    int8_t steering,
    XTime completed_at
)
{
#if STEERING_SOURCE_CNN

    int16_t value = steering;

    /*
     * CNN continuous regression result
     * -> vehicle protocol 10-degree step
     *
     *  27 -> 30
     *  24 -> 20
     * -27 -> -30
     * -24 -> -20
     */
    if (value >= 0)
        value = ((value + 5) / 10) * 10;
    else
        value = ((value - 5) / 10) * 10;

    if (value > 90)
        value = 90;

    if (value < -90)
        value = -90;

    cnnSteering = (int8_t)value;

    lastCnnUpdateTime = completed_at;

    cnnEverSeen = 1;

#else
    (void)steering;
    (void)completed_at;
#endif
}

int getSteering(
    int pcLinkOK,
    int8_t *steering
)
{
#if STEERING_SOURCE_CNN

    (void)pcLinkOK;

    /*
     * No valid CNN result has arrived yet.
     */
    if (
        !cnnEverSeen
    )
    {
        *steering = 0;
        return 0;
    }

    /*
     * A CNN result must keep being refreshed.
     * Otherwise an old steering angle could remain
     * active indefinitely.
     */
    if (
        elapsedMs(
            lastCnnUpdateTime
        )
        >
        CNN_TIMEOUT_MS
    )
    {
        *steering = 0;
        return 0;
    }

    *steering =
        cnnSteering;

    return 1;

#else

    if (
        pcLinkOK
    )
    {
        *steering =
            pcSteering;

        return 1;
    }

    *steering = 0;

    return 0;

#endif
}

void vehiclePollPC(
    PcState *out
)
{
    if (
        processPCSerial()
    )
    {
        XTime_GetTime(
            &lastPcCommandTime
        );


        pcLinkEverSeen =
            1;
    }


    int linkOK =
        (
            pcLinkEverSeen
            &&
            elapsedMs(
                lastPcCommandTime
            )
            <=
            PC_COMMAND_TIMEOUT_MS
        );


    if (
        !linkOK
    )
    {
        /* PC-link timeout stop disabled; retain for future use.
        pcEmergencyStop =
            1;
        */


#if !STEERING_SOURCE_CNN
        pcSteering =
            0;
#endif
    }


    out->linkOK =
        linkOK;


    out->steering =
        pcSteering;


    out->emergencyStop =
        (pcEmergencyStop != 0);
}

void vehiclePollSensor(
    SensorState *out
)
{
    if (
        processSensorUART()
    )
    {
        XTime_GetTime(
            &lastSensorPacketTime
        );


        sensorLinkEverSeen =
            1;
    }


    int linkOK =
        (
            sensorLinkEverSeen
            &&
            elapsedMs(
                lastSensorPacketTime
            )
            <=
            SENSOR_TIMEOUT_MS
        );


    // Drop the slew reference so the first sample after a
    // recovery is not measured against a stale one.
    if (
        !linkOK
    )
    {
        sensorHistoryValid =
            0;
    }


    out->linkOK =
        linkOK;


    out->accelRaw =
        latestAccelRaw;


    out->brakeRaw =
        latestBrakeRaw;


    out->sequence =
        latestSensorSequence;


    out->accelLevel =
        0;


    out->brakeLevel =
        0;


    if (
        linkOK
    )
    {
        out->accelLevel =
            rawToLevel(
                latestAccelRaw,
                accelThreshold
            );


        out->brakeLevel =
            rawToLevel(
                latestBrakeRaw,
                brakeThreshold
            );


        // Brake wins over accel.
        if (
            out->brakeLevel
            >
            0
        )
        {
            out->accelLevel =
                0;
        }
    }
}
