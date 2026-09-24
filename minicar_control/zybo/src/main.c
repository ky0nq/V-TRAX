#include "xparameters.h"
#include "xuartps.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"
#include "xtime_l.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>


// ==================================================
// UART
//
// UART0:
//   TX -> ESP32 #1 -> ESP32 #2 Vehicle
//   RX <- ESP32 #1 <- ESP32 #3 Sensor
//
// UART1:
//   PC Keyboard Control
//   Debug Output
// ==================================================
XUartPs Uart0;
XUartPs Uart1;


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

// Command output rate to ESP32 #1: 20 ms = 50 Hz, matching the
// sensor node so commands carry fresh samples.
#define CONTROL_PERIOD_US          20000


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
#define SENSOR_MAX_STEP            15000


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
// Command Sequence
// ==================================================
static uint8_t commandSequence = 0;


// ==================================================
// FSR Threshold
//
// Placeholder values.
// Recalibrate once the cushion/plate is fitted.
// ==================================================
static const int16_t accelThreshold[5] =
{
    1000,
    4000,
    8000,
    13000,
    19000
};


static const int16_t brakeThreshold[5] =
{
    1000,
    4000,
    8000,
    13000,
    19000
};


// ==================================================
// PC Control State
// ==================================================
static int8_t pcSteering = 0;


// Start latched in E-STOP for safety.
static uint8_t pcEmergencyStop = 1;


static char pcRxLine[32];


static uint32_t pcRxIndex = 0;


// Cleared until the first valid line arrives, so the link reads
// as lost at boot instead of looking fresh against a zeroed
// timestamp.
static int pcLinkEverSeen = 0;


static XTime lastPcCommandTime;


// ==================================================
// Wireless Sensor State
// ==================================================
static int16_t latestAccelRaw = 0;


static int16_t latestBrakeRaw = 0;


static uint8_t latestSensorSequence = 0;


static int sensorLinkEverSeen = 0;


static XTime lastSensorPacketTime;


// Cleared whenever the link drops, so the first sample after a
// recovery is not rejected for jumping away from a stale one.
static int sensorHistoryValid = 0;


// ==================================================
// UART0 Sensor Packet Parser
// ==================================================
static uint8_t sensorRxBuffer[8];


static uint32_t sensorRxIndex = 0;


// ==================================================
// CRC-8
//
// Polynomial = 0x07
// Initial = 0x00
// ==================================================
uint8_t calculateCRC8(
    const uint8_t *data,
    uint32_t length
)
{
    uint8_t crc = 0x00;


    for (
        uint32_t i = 0;
        i < length;
        i++
    )
    {
        crc ^= data[i];


        for (
            int bit = 0;
            bit < 8;
            bit++
        )
        {
            if (
                crc
                &
                0x80
            )
            {
                crc =
                    (crc << 1)
                    ^
                    0x07;
            }
            else
            {
                crc <<=
                    1;
            }
        }
    }


    return crc;
}


// ==================================================
// Milliseconds elapsed since a global timer timestamp
// ==================================================
uint32_t elapsedMs(
    XTime since
)
{
    XTime now;

    XTime_GetTime(
        &now
    );


    return
        (uint32_t)(
            ((now - since) * 1000U)
            /
            COUNTS_PER_SECOND
        );
}


// ==================================================
// Microseconds elapsed since a global timer timestamp
//
// Only valid for short intervals. The 1e6 scaling overflows a
// 64-bit count after roughly 15 hours, so this covers one loop
// pass while elapsedMs covers the link timeouts.
// ==================================================
uint32_t elapsedUs(
    XTime since
)
{
    XTime now;

    XTime_GetTime(
        &now
    );


    return
        (uint32_t)(
            ((now - since) * 1000000U)
            /
            COUNTS_PER_SECOND
        );
}


// ==================================================
// Hold the control loop period
//
// A bare usleep() is added on top of the work above it, so the
// real period would drift with UART and debug load. This sleeps
// only the remainder of the period instead.
// ==================================================
void holdControlPeriod(
    XTime cycleStart
)
{
    uint32_t usedUs =
        elapsedUs(
            cycleStart
        );


    if (
        usedUs
        <
        CONTROL_PERIOD_US
    )
    {
        usleep(
            CONTROL_PERIOD_US - usedUs
        );
    }
}


// ==================================================
// UART0 Init
//
// UART0:
//   TX = MIO15 / JF10
//   RX = MIO14 / JF9
//
// Baud = 115200
// ==================================================
int initUART0(void)
{
    XUartPs_Config *Config =
        XUartPs_LookupConfig(
            XPAR_XUARTPS_0_DEVICE_ID
        );


    if (
        Config
        ==
        NULL
    )
    {
        return XST_FAILURE;
    }


    int Status =
        XUartPs_CfgInitialize(
            &Uart0,
            Config,
            Config->BaseAddress
        );


    if (
        Status
        !=
        XST_SUCCESS
    )
    {
        return XST_FAILURE;
    }


    return
        XUartPs_SetBaudRate(
            &Uart0,
            115200
        );
}


// ==================================================
// UART1 Init
//
// UART1 = PC / COM10
// Baud = 115200
// ==================================================
int initUART1(void)
{
    XUartPs_Config *Config =
        XUartPs_LookupConfig(
            XPAR_XUARTPS_1_DEVICE_ID
        );


    if (
        Config
        ==
        NULL
    )
    {
        return XST_FAILURE;
    }


    int Status =
        XUartPs_CfgInitialize(
            &Uart1,
            Config,
            Config->BaseAddress
        );


    if (
        Status
        !=
        XST_SUCCESS
    )
    {
        return XST_FAILURE;
    }


    return
        XUartPs_SetBaudRate(
            &Uart1,
            115200
        );
}


// ==================================================
// Send Vehicle Command
//
// Zybo UART0 TX
//      v
// ESP32 #1 GPIO16 RX
//      v
// ESP-NOW
//      v
// ESP32 #2
// ==================================================
void sendCommandPacket(
    int8_t steering,
    uint8_t accel,
    uint8_t brake,
    uint8_t flags
)
{
    CommandPacket packet;


    packet.sof1 =
        0xAA;


    packet.sof2 =
        0x55;


    packet.sequence =
        commandSequence++;


    packet.steering =
        steering;


    packet.accel =
        accel;


    packet.brake =
        brake;


    packet.flags =
        flags;


    packet.crc =
        calculateCRC8(
            (const uint8_t *)&packet,
            7
        );


    XUartPs_Send(
        &Uart0,
        (uint8_t *)&packet,
        sizeof(packet)
    );


    while (
        XUartPs_IsSending(
            &Uart0
        )
    );
}


// ==================================================
// Sensor Raw Plausibility
//
// Rejects readings the hardware cannot actually produce.
// ==================================================
int sensorRawInRange(
    int16_t raw
)
{
    return (
        raw >= SENSOR_RAW_MIN
        &&
        raw <= SENSOR_RAW_MAX
    );
}


// ==================================================
// Absolute difference between two raw readings
// ==================================================
int32_t sensorRawStep(
    int16_t a,
    int16_t b
)
{
    int32_t diff =
        (int32_t)a
        -
        (int32_t)b;


    return (
        diff < 0
            ? -diff
            : diff
    );
}


// ==================================================
// Validate Sensor Packet
// ==================================================
int validateSensorPacket(
    const SensorPacket *packet
)
{
    // Header
    if (
        packet->sof1
        !=
        0xA5
    )
    {
        return 0;
    }


    if (
        packet->sof2
        !=
        0x5A
    )
    {
        return 0;
    }


    // CRC
    uint8_t expectedCRC =
        calculateCRC8(
            (const uint8_t *)packet,
            7
        );


    if (
        expectedCRC
        !=
        packet->crc
    )
    {
        return 0;
    }


    // Range
    //
    // A CRC only proves the bytes survived the link, not that
    // the reading means anything.
    if (
        !sensorRawInRange(
            packet->accelRaw
        )
        ||
        !sensorRawInRange(
            packet->brakeRaw
        )
    )
    {
        return 0;
    }


    return 1;
}


// ==================================================
// Process Wireless Sensor UART
//
// ESP32 #1 GPIO17 TX
//      v
// Zybo JF9 / MIO14 / UART0 RX
//
// Sensor packet header:
//
// A5 5A
//
// Return:
//   1 = valid packet received
//   0 = no new valid packet
// ==================================================
int processSensorUART(void)
{
    int validPacketReceived = 0;


    // Drain the UART0 RX FIFO completely.
    while (
        XUartPs_IsReceiveData(
            Uart0.Config.BaseAddress
        )
    )
    {
        uint8_t data =
            (uint8_t)
            XUartPs_ReadReg(
                Uart0.Config.BaseAddress,
                XUARTPS_FIFO_OFFSET
            );


        // ==========================================
        // Byte 0
        // Find 0xA5
        // ==========================================
        if (
            sensorRxIndex
            ==
            0
        )
        {
            if (
                data
                ==
                0xA5
            )
            {
                sensorRxBuffer[0] =
                    data;


                sensorRxIndex =
                    1;
            }


            continue;
        }


        // ==========================================
        // Byte 1
        // Confirm 0x5A
        // ==========================================
        if (
            sensorRxIndex
            ==
            1
        )
        {
            if (
                data
                ==
                0x5A
            )
            {
                sensorRxBuffer[1] =
                    data;


                sensorRxIndex =
                    2;
            }
            else
            {
                // Bad header
                sensorRxIndex =
                    0;


                // If this byte is itself 0xA5 it can start
                // a new packet.
                if (
                    data
                    ==
                    0xA5
                )
                {
                    sensorRxBuffer[0] =
                        data;


                    sensorRxIndex =
                        1;
                }
            }


            continue;
        }


        // ==========================================
        // Byte 2 ~ Byte 7
        // ==========================================
        sensorRxBuffer[
            sensorRxIndex
        ] =
            data;


        sensorRxIndex++;


        // ==========================================
        // Complete 8-byte SensorPacket
        // ==========================================
        if (
            sensorRxIndex
            ==
            8
        )
        {
            SensorPacket packet;


            memcpy(
                &packet,
                sensorRxBuffer,
                sizeof(packet)
            );


            // --------------------------------------
            // Header + CRC
            // --------------------------------------
            if (
                validateSensorPacket(
                    &packet
                )
            )
            {
                // --------------------------------------
                // Slew
                //
                // A reading can sit inside the valid range
                // and still be impossible. An FSR that goes
                // open circuit pulls its divider to the rail
                // within one sample, and only the size of
                // the jump gives that away.
                // --------------------------------------
                int accept = 1;


                if (
                    sensorHistoryValid
                )
                {
                    if (
                        sensorRawStep(
                            packet.accelRaw,
                            latestAccelRaw
                        )
                        >
                        SENSOR_MAX_STEP
                        ||
                        sensorRawStep(
                            packet.brakeRaw,
                            latestBrakeRaw
                        )
                        >
                        SENSOR_MAX_STEP
                    )
                    {
                        accept = 0;
                    }
                }


                if (accept)
                {
                    latestAccelRaw =
                        packet.accelRaw;


                    latestBrakeRaw =
                        packet.brakeRaw;


                    latestSensorSequence =
                        packet.sequence;


                    sensorHistoryValid =
                        1;


                    validPacketReceived =
                        1;
                }
            }


            // Ready for the next packet
            sensorRxIndex =
                0;
        }
    }


    return validPacketReceived;
}


// ==================================================
// FSR RAW -> Level 0~5
// ==================================================
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


// ==================================================
// Parse PC Command
//
// Format:
//
// K,<steering>,<estop>
//
// Example:
//
// K,0,0
// K,-30,0
// K,40,0
// K,0,1
// ==================================================
int parsePCCommand(
    const char *line
)
{
    int steeringValue;


    int estopValue;


    if (
        sscanf(
            line,
            "K,%d,%d",
            &steeringValue,
            &estopValue
        )
        !=
        2
    )
    {
        return 0;
    }


    // Steering Range
    if (
        steeringValue < -90
        ||
        steeringValue > 90
    )
    {
        return 0;
    }


    // Steering 10-degree step
    if (
        (
            steeringValue
            %
            10
        )
        !=
        0
    )
    {
        return 0;
    }


    // E-stop
    if (
        estopValue != 0
        &&
        estopValue != 1
    )
    {
        return 0;
    }


    pcSteering =
        (int8_t)
        steeringValue;


    pcEmergencyStop =
        (uint8_t)
        estopValue;


    return 1;
}


// ==================================================
// Process PC UART1
// ==================================================
int processPCSerial(void)
{
    int validCommandReceived =
        0;


    while (
        XUartPs_IsReceiveData(
            Uart1.Config.BaseAddress
        )
    )
    {
        uint8_t c =
            (uint8_t)
            XUartPs_ReadReg(
                Uart1.Config.BaseAddress,
                XUARTPS_FIFO_OFFSET
            );


        // CR ignore
        if (
            c
            ==
            '\r'
        )
        {
            continue;
        }


        // End of line
        if (
            c
            ==
            '\n'
        )
        {
            pcRxLine[
                pcRxIndex
            ] =
                '\0';


            if (
                parsePCCommand(
                    pcRxLine
                )
            )
            {
                validCommandReceived =
                    1;
            }


            pcRxIndex =
                0;


            continue;
        }


        // Normal character
        if (
            pcRxIndex
            <
            sizeof(pcRxLine) - 1
        )
        {
            pcRxLine[
                pcRxIndex++
            ] =
                (char)c;
        }
        else
        {
            // overflow protection
            pcRxIndex =
                0;
        }
    }


    return validCommandReceived;
}


// ==================================================
// MAIN
// ==================================================
int main(void)
{
    int Status;


    // ==================================================
    // UART0
    //
    // Zybo <-> ESP32 #1
    // ==================================================
    Status =
        initUART0();


    if (
        Status
        !=
        XST_SUCCESS
    )
    {
        return XST_FAILURE;
    }


    // ==================================================
    // UART1
    //
    // PC <-> Zybo
    // ==================================================
    Status =
        initUART1();


    if (
        Status
        !=
        XST_SUCCESS
    )
    {
        return XST_FAILURE;
    }


    // ==================================================
    // Runtime State
    // ==================================================
    uint32_t debugCounter =
        0;


    XTime_GetTime(
        &lastPcCommandTime
    );


    XTime_GetTime(
        &lastSensorPacketTime
    );


    // ==================================================
    // Startup Debug
    // ==================================================
    xil_printf(
        "\r\n"
        "========================================\r\n"
        " ZYBO WIRELESS SENSOR VEHICLE CONTROL\r\n"
        "========================================\r\n"
        "PC Steering : UART1 / COM10\r\n"
        "Sensor RX   : UART0 RX / JF9 / MIO14\r\n"
        "Vehicle TX  : UART0 TX / JF10 / MIO15\r\n"
        "\r\n"
        "Sensor path:\r\n"
        "FSR -> ADS1115 -> ESP32 #3\r\n"
        "    -> ESP-NOW -> ESP32 #1 -> Zybo\r\n"
        "\r\n"
        "Vehicle path:\r\n"
        "Zybo -> ESP32 #1 -> ESP-NOW -> ESP32 #2\r\n"
        "========================================\r\n"
    );


    // ==================================================
    // MAIN LOOP
    // ==================================================
    while (1)
    {
        XTime cycleStart;

        XTime_GetTime(
            &cycleStart
        );


        // ==============================================
        // 1. PC Steering / E-Stop
        // ==============================================
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


        int pcLinkOK =
            (
                pcLinkEverSeen
                &&
                elapsedMs(
                    lastPcCommandTime
                )
                <=
                PC_COMMAND_TIMEOUT_MS
            );


        // PC communication timeout
        if (
            !pcLinkOK
        )
        {
            pcSteering =
                0;


            pcEmergencyStop =
                1;
        }


        // ==============================================
        // 2. Wireless Sensor Receive
        //
        // ESP32 #1 GPIO17
        //       v
        // Zybo UART0 RX
        // ==============================================
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


        // ==============================================
        // 3. Sensor Link State
        //
        // No accepted packet for SENSOR_TIMEOUT_MS
        // => Sensor LOST
        // ==============================================
        int sensorLinkOK =
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
            !sensorLinkOK
        )
        {
            sensorHistoryValid =
                0;
        }


        // ==============================================
        // 4. RAW -> Level
        // ==============================================
        uint8_t accelLevel =
            0;


        uint8_t brakeLevel =
            0;


        if (
            sensorLinkOK
        )
        {
            accelLevel =
                rawToLevel(
                    latestAccelRaw,
                    accelThreshold
                );


            brakeLevel =
                rawToLevel(
                    latestBrakeRaw,
                    brakeThreshold
                );


            // ==========================================
            // Brake Priority
            // ==========================================
            if (
                brakeLevel
                >
                0
            )
            {
                accelLevel =
                    0;
            }
        }


        // ==============================================
        // 5. Safety / E-STOP
        // ==============================================
        uint8_t flags =
            0;


        // PC E-stop
        if (
            pcEmergencyStop
        )
        {
            flags |=
                FLAG_EMERGENCY_STOP;
        }


        // Sensor wireless link lost
        if (
            !sensorLinkOK
        )
        {
            flags |=
                FLAG_EMERGENCY_STOP;
        }


        // Any E-stop
        if (
            flags
            &
            FLAG_EMERGENCY_STOP
        )
        {
            accelLevel =
                0;


            brakeLevel =
                0;
        }


        // ==============================================
        // 6. Vehicle Command Send
        //
        // Zybo
        //   -> ESP32 #1
        //   -> ESP32 #2
        // ==============================================
        sendCommandPacket(
            pcSteering,
            accelLevel,
            brakeLevel,
            flags
        );


        // ==============================================
        // 7. Debug
        //
        // 10 loops -> about 5 Hz
        // ==============================================
        if (
            (
                debugCounter
                %
                10
            )
            ==
            0
        )
        {
            xil_printf(
                "SENSOR_SEQ=%d "
                "ACC_RAW=%d A=%d "
                "BRAKE_RAW=%d B=%d "
                "SENSOR=%s "
                "STEER=%d "
                "ESTOP=%d "
                "PC=%s\r\n",

                latestSensorSequence,

                latestAccelRaw,
                accelLevel,

                latestBrakeRaw,
                brakeLevel,

                sensorLinkOK
                    ?
                    "OK"
                    :
                    "LOST",

                pcSteering,

                (
                    flags
                    &
                    FLAG_EMERGENCY_STOP
                )
                    ?
                    1
                    :
                    0,

                pcLinkOK
                    ?
                    "OK"
                    :
                    "LOST"
            );
        }


        debugCounter++;


        // ==============================================
        // 50 Hz
        // ==============================================
        holdControlPeriod(
            cycleStart
        );
    }


    return 0;
}
