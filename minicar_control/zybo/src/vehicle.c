#include "vehicle.h"

#include "xparameters.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"
#include "xil_io.h"

#include <stdio.h>
#include <string.h>


// ==================================================
// UART instances
//
// UART0 faces ESP32 #1 in both directions; UART1 faces the PC.
// ==================================================
XUartPs Uart0;
XUartPs Uart1;


// ==================================================
// Command Sequence
// ==================================================
static uint8_t commandSequence = 0;


// ==================================================
// FSR Threshold
//
// Measured per pedal. The two differ because the mechanics
// differ -- the same force reaches each sensor differently.
// ==================================================
static const int16_t accelThreshold[5] =
{
    7000,
    10000,
    14000,
    19000,
    22000
};


static const int16_t brakeThreshold[5] =
{
    13000,
    17000,
    19000,
    21000,
    23000
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


// ==================================================
// CNN Steering State
// ==================================================
static int8_t cnnSteering = 0;


static int cnnEverSeen = 0;


static XTime lastCnnUpdateTime;


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
// Read the CNN accelerator RESULT register
//
// Return:
//   1 = a fresh, in-range steering label was read
//   0 = nothing new, or the value was rejected
//
// The range and 10-degree-step checks live here so a bad
// inference can never reach the packet. A rejected value is
// treated as "no update", which lets it expire through
// CNN_TIMEOUT_MS instead of freezing the last good reading.
//
// Must not block: the control loop budget is CONTROL_PERIOD_US.
// ==================================================
int readCNNSteering(
    int8_t *steering
)
{
#if STEERING_SOURCE_CNN

    uint32_t status =
        Xil_In32(
            CNN_BASEADDR + CNN_REG_STATUS
        );


    if (
        (status & CNN_STATUS_DONE_MASK)
        ==
        0
    )
    {
        // Either an inference is running or none has been asked
        // for. done_status clears on start, so without a start
        // the DONE bit never comes back and the steering source
        // expires through CNN_TIMEOUT_MS.
        if (
            status & CNN_STATUS_READY_MASK
        )
        {
            Xil_Out32(
                CNN_BASEADDR + CNN_REG_CONTROL,
                CNN_CONTROL_START_MASK
            );
        }


        return 0;
    }


    // The accelerator's result is 8 bits, and the AXI slave
    // zero-extends it into the 32-bit register. Reading all 32
    // bits therefore loses the sign: -90 arrives as 0x000000A6
    // and reads back as 166, which the range check below would
    // throw away. Keeping the low byte and casting it to int8_t
    // restores the two's complement.
    int8_t value =
        (int8_t)(
            Xil_In32(
                CNN_BASEADDR + CNN_REG_RESULT
            )
            &
            0xFF
        );


    // Queue the next inference before validating this one, so a
    // reading that gets rejected still leaves the accelerator
    // working on the frame after it.
    Xil_Out32(
        CNN_BASEADDR + CNN_REG_CONTROL,
        CNN_CONTROL_START_MASK
    );


    if (
        value < -90
        ||
        value > 90
    )
    {
        return 0;
    }


    if (
        (value % 10)
        !=
        0
    )
    {
        return 0;
    }


    *steering =
        value;


    return 1;

#else

    (void)steering;

    return 0;

#endif
}


// ==================================================
// Steering source
//
// Return:
//   1 = *steering holds a usable value
//   0 = the source is stale; caller must raise E-Stop
//
// This is the only place that decides where steering comes
// from. Swapping the source means flipping STEERING_SOURCE_CNN,
// not editing the control loop.
// ==================================================
int getSteering(
    int pcLinkOK,
    int8_t *steering
)
{
#if STEERING_SOURCE_CNN

    (void)pcLinkOK;


    int8_t fresh;


    if (
        readCNNSteering(
            &fresh
        )
    )
    {
        cnnSteering =
            fresh;


        XTime_GetTime(
            &lastCnnUpdateTime
        );


        cnnEverSeen =
            1;
    }


    if (
        cnnEverSeen
        &&
        elapsedMs(
            lastCnnUpdateTime
        )
        <=
        CNN_TIMEOUT_MS
    )
    {
        *steering =
            cnnSteering;


        return 1;
    }


    return 0;

#else

    if (pcLinkOK)
    {
        *steering =
            pcSteering;


        return 1;
    }


    return 0;

#endif
}


// ==================================================
// MAIN

// ==================================================
// Setup
// ==================================================
int vehicleInit(void)
{
    int Status =
        initUART0();


    if (
        Status
        !=
        XST_SUCCESS
    )
    {
        return Status;
    }


    Status =
        initUART1();


    if (
        Status
        !=
        XST_SUCCESS
    )
    {
        return Status;
    }


    // Both links start in the lost state. The everSeen flags do
    // the real work; the timestamps just need to be defined.
    XTime_GetTime(
        &lastPcCommandTime
    );


    XTime_GetTime(
        &lastSensorPacketTime
    );


    return XST_SUCCESS;
}


// ==================================================
// One cycle of PC input
//
// Losing the PC always latches E-Stop, because the PC carries
// the manual stop in both steering modes. It only clears the
// steering value when the PC is also the steering source --
// otherwise a PC dropout would overwrite good CNN steering.
// ==================================================
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
        pcEmergencyStop =
            1;


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


// ==================================================
// One cycle of pedal input
//
// Levels are produced only while the link is up, so a stale
// reading can never be converted into throttle.
// ==================================================
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
