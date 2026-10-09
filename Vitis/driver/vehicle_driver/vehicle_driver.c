#include "vehicle_driver.h"
#include "vehicle.h"
#include "xparameters.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"
#include <stdio.h>
#include <string.h>

XUartPs Uart0;
XUartPs Uart1;

static uint8_t commandSequence = 0;

static char pcRxLine[32];


static uint32_t pcRxIndex = 0;

int8_t pcSteering = 0;

uint8_t pcEmergencyStop = 0;

int16_t latestAccelRaw = 0;


int16_t latestBrakeRaw = 0;


uint8_t latestSensorSequence = 0;

int sensorHistoryValid = 0;

static uint8_t sensorRxBuffer[8];


static uint32_t sensorRxIndex = 0;

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


    /* UART1 is also stdout. Finish queued boot logs before CfgInitialize
     * and SetBaudRate reset TX and change its baud-rate registers. */
    XTime tx_wait_start;
    XTime tx_wait_now;
    u32 tx_status;
    XTime_GetTime(&tx_wait_start);
    for (;;) {
        tx_status = XUartPs_ReadReg(Config->BaseAddress, XUARTPS_SR_OFFSET);
        if ((tx_status & XUARTPS_SR_TXEMPTY) != 0U &&
            (tx_status & XUARTPS_SR_TACTIVE) == 0U)
            break;
        XTime_GetTime(&tx_wait_now);
        if (tx_wait_now - tx_wait_start >= COUNTS_PER_SECOND)
            return XST_FAILURE;
        usleep(100U);
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
