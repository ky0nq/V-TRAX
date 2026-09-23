#include "xparameters.h"
#include "xuartps.h"
#include "xuartps_hw.h"
#include "xiicps.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"
#include "xtime_l.h"

#include <stdint.h>
#include <stdio.h>


XUartPs Uart0;   // Zybo -> ESP32 #1
XUartPs Uart1;   // PC <-> Zybo
XIicPs Iic;      // ADS1115


#define FLAG_EMERGENCY_STOP       0x01

#define ADS1115_ADDR              0x48
#define ADS1115_REG_CONVERSION    0x00
#define ADS1115_REG_CONFIG        0x01

#define I2C_CLOCK_HZ              100000

// Wall-clock timeout. The control loop period is not constant
// (ADS1115 conversion waits and the periodic debug line both add
// to it), so counting loop iterations would make this safety
// constant drift with CPU load.
#define PC_COMMAND_TIMEOUT_MS     500

// Command rate to ESP32 #1: 10 ms period = 100 Hz.
#define CONTROL_PERIOD_US         10000

// Consecutive ADS1115 failures tolerated before the I2C
// controller is reset.
#define I2C_FAILURE_LIMIT         5


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


static uint8_t sequenceNumber = 0;


// ==================================================
// FSR thresholds
// Placeholder levels. Calibrate against real FSR readings.
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
// PC Command State
// ==================================================
static int8_t pcSteering = 0;

static uint8_t pcEmergencyStop = 1;

static char pcRxLine[32];

static uint32_t pcRxIndex = 0;

static XTime lastPcCommandTime;


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
// Only valid for short intervals. The 1e6 scaling overflows
// a 64-bit count after roughly 15 hours, so this is used for
// intra-cycle timing while elapsedMs covers the long ones.
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
// A bare usleep() is added on top of the work above it, so
// the real period would drift with I2C and UART load. This
// sleeps only the remainder of the period instead.
// ==================================================
void holdControlPeriod(
    XTime cycleStart
)
{
    uint32_t usedUs =
        elapsedUs(
            cycleStart
        );


    if (usedUs < CONTROL_PERIOD_US)
    {
        usleep(
            CONTROL_PERIOD_US - usedUs
        );
    }
}


// ==================================================
// CRC-8
// ==================================================
uint8_t calculateCRC8(
    const uint8_t *data,
    uint32_t length
)
{
    uint8_t crc = 0x00;

    for (uint32_t i = 0;
         i < length;
         i++)
    {
        crc ^= data[i];

        for (int bit = 0;
             bit < 8;
             bit++)
        {
            if (crc & 0x80)
            {
                crc =
                    (crc << 1)
                    ^
                    0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


// ==================================================
// UART0
// Zybo -> ESP32 #1
// ==================================================
int initUART0(void)
{
    XUartPs_Config *Config =
        XUartPs_LookupConfig(
            XPAR_XUARTPS_0_DEVICE_ID
        );

    if (Config == NULL)
    {
        return XST_FAILURE;
    }


    int Status =
        XUartPs_CfgInitialize(
            &Uart0,
            Config,
            Config->BaseAddress
        );


    if (Status != XST_SUCCESS)
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
// UART1
// PC <-> Zybo
// ==================================================
int initUART1(void)
{
    XUartPs_Config *Config =
        XUartPs_LookupConfig(
            XPAR_XUARTPS_1_DEVICE_ID
        );


    if (Config == NULL)
    {
        return XST_FAILURE;
    }


    int Status =
        XUartPs_CfgInitialize(
            &Uart1,
            Config,
            Config->BaseAddress
        );


    if (Status != XST_SUCCESS)
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
// UART0 -> ESP32 #1 Packet
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
        sequenceNumber++;

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
            (uint8_t *)&packet,
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
// I2C1
// ==================================================
int initI2C(void)
{
    XIicPs_Config *Config =
        XIicPs_LookupConfig(
            XPAR_XIICPS_0_DEVICE_ID
        );


    if (Config == NULL)
    {
        return XST_FAILURE;
    }


    int Status =
        XIicPs_CfgInitialize(
            &Iic,
            Config,
            Config->BaseAddress
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    Status =
        XIicPs_SelfTest(
            &Iic
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    Status =
        XIicPs_SetSClk(
            &Iic,
            I2C_CLOCK_HZ
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    xil_printf(
        "I2C1 INIT OK\r\n"
    );


    return XST_SUCCESS;
}


// ==================================================
// I2C Bus Recovery
//
// Aborts any transfer in progress and returns the PS I2C
// controller to a known state, so a transient glitch on the
// bus cannot wedge it until the next power cycle.
//
// XIicPs_Reset restores register defaults, so the serial
// clock has to be programmed again afterwards.
// ==================================================
int recoverI2C(void)
{
    XIicPs_Reset(
        &Iic
    );


    return
        XIicPs_SetSClk(
            &Iic,
            I2C_CLOCK_HZ
        );
}


// ==================================================
// ADS1115 Config
// ==================================================
int ADS1115_WriteConfig(
    uint16_t config
)
{
    uint8_t tx[3];


    tx[0] =
        ADS1115_REG_CONFIG;

    tx[1] =
        (uint8_t)(
            (config >> 8)
            &
            0xFF
        );

    tx[2] =
        (uint8_t)(
            config
            &
            0xFF
        );


    int Status =
        XIicPs_MasterSendPolled(
            &Iic,
            tx,
            3,
            ADS1115_ADDR
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    while (
        XIicPs_BusIsBusy(
            &Iic
        )
    );


    return XST_SUCCESS;
}


// ==================================================
// ADS1115 Conversion Read
// ==================================================
int ADS1115_ReadConversion(
    int16_t *value
)
{
    uint8_t pointer =
        ADS1115_REG_CONVERSION;

    uint8_t rx[2];


    int Status =
        XIicPs_MasterSendPolled(
            &Iic,
            &pointer,
            1,
            ADS1115_ADDR
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    while (
        XIicPs_BusIsBusy(
            &Iic
        )
    );


    Status =
        XIicPs_MasterRecvPolled(
            &Iic,
            rx,
            2,
            ADS1115_ADDR
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    while (
        XIicPs_BusIsBusy(
            &Iic
        )
    );


    *value =
        (int16_t)(
            (
                (uint16_t)rx[0]
                <<
                8
            )
            |
            rx[1]
        );


    return XST_SUCCESS;
}


// ==================================================
// ADS1115 Channel Read
// ==================================================
int ADS1115_ReadChannel(
    uint8_t channel,
    int16_t *value
)
{
    if (channel > 3)
    {
        return XST_FAILURE;
    }


    uint16_t mux =
        (uint16_t)(
            4
            +
            channel
        );


    uint16_t config =
          0x8000
        | (mux << 12)
        | 0x0200
        | 0x0100
        | 0x00E0
        | 0x0003;


    int Status =
        ADS1115_WriteConfig(
            config
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    usleep(
        2000
    );


    return
        ADS1115_ReadConversion(
            value
        );
}


// ==================================================
// FSR Read
// A0 = Accel
// A1 = Brake
// ==================================================
int readPressureSensors(
    int16_t *accelRaw,
    int16_t *brakeRaw
)
{
    int Status =
        ADS1115_ReadChannel(
            0,
            accelRaw
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    Status =
        ADS1115_ReadChannel(
            1,
            brakeRaw
        );


    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    return XST_SUCCESS;
}


// ==================================================
// RAW -> Level 0~5
// ==================================================
uint8_t rawToLevel(
    int16_t raw,
    const int16_t threshold[5]
)
{
    if (raw < 0)
    {
        raw = 0;
    }


    if (raw < threshold[0])
        return 0;

    if (raw < threshold[1])
        return 1;

    if (raw < threshold[2])
        return 2;

    if (raw < threshold[3])
        return 3;

    if (raw < threshold[4])
        return 4;


    return 5;
}


// ==================================================
// PC command:
// K,<steering>,<estop>
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


    if (
        steeringValue < -90 ||
        steeringValue > 90
    )
    {
        return 0;
    }


    if (
        (steeringValue % 10)
        !=
        0
    )
    {
        return 0;
    }


    if (
        estopValue != 0 &&
        estopValue != 1
    )
    {
        return 0;
    }


    pcSteering =
        (int8_t)steeringValue;

    pcEmergencyStop =
        (uint8_t)estopValue;


    return 1;
}


// ==================================================
// UART1 PC RX
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


        if (c == '\r')
        {
            continue;
        }


        if (c == '\n')
        {
            pcRxLine[pcRxIndex] =
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


        if (
            pcRxIndex
            <
            sizeof(pcRxLine) - 1
        )
        {
            pcRxLine[pcRxIndex++] =
                (char)c;
        }
        else
        {
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


    // UART0 -> ESP32
    Status =
        initUART0();

    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    // UART1 -> PC
    Status =
        initUART1();

    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }


    // ADS1115
    Status =
        initI2C();

    if (Status != XST_SUCCESS)
    {
        xil_printf(
            "I2C1 INIT FAILED\r\n"
        );

        return XST_FAILURE;
    }


    int16_t accelRaw =
        0;

    int16_t brakeRaw =
        0;


    uint32_t debugCounter =
        0;

    uint32_t i2cFailureCount =
        0;


    XTime_GetTime(
        &lastPcCommandTime
    );


    xil_printf(
        "\r\n"
        "========================================\r\n"
        " ZYBO HYBRID VEHICLE CONTROL\r\n"
        "========================================\r\n"
        "LEFT/RIGHT : PC Steering\r\n"
        "SPACE      : Emergency Stop\r\n"
        "FSR A0     : Acceleration\r\n"
        "FSR A1     : Brake\r\n"
        "UART0      : ESP32 #1 Command\r\n"
        "UART1      : PC Control + Debug\r\n"
        "========================================\r\n"
    );


    while (1)
    {
        XTime cycleStart;

        XTime_GetTime(
            &cycleStart
        );


        // ==========================================
        // PC steering / E-stop
        // ==========================================
        if (
            processPCSerial()
        )
        {
            XTime_GetTime(
                &lastPcCommandTime
            );
        }


        uint32_t pcSilenceMs =
            elapsedMs(
                lastPcCommandTime
            );


        // PC link timeout
        if (
            pcSilenceMs
            >
            PC_COMMAND_TIMEOUT_MS
        )
        {
            pcSteering =
                0;

            pcEmergencyStop =
                1;
        }


        // ==========================================
        // FSR read
        // ==========================================
        Status =
            readPressureSensors(
                &accelRaw,
                &brakeRaw
            );


        if (Status != XST_SUCCESS)
        {
            sendCommandPacket(
                0,
                0,
                0,
                FLAG_EMERGENCY_STOP
            );


            i2cFailureCount++;


            if (
                i2cFailureCount
                >=
                I2C_FAILURE_LIMIT
            )
            {
                i2cFailureCount =
                    0;


                if (
                    recoverI2C()
                    ==
                    XST_SUCCESS
                )
                {
                    xil_printf(
                        "I2C BUS RESET\r\n"
                    );
                }
                else
                {
                    xil_printf(
                        "I2C BUS RESET FAILED\r\n"
                    );
                }
            }


            if (
                (debugCounter % 20)
                ==
                0
            )
            {
                xil_printf(
                    "ADS1115 READ ERROR -> E-STOP\r\n"
                );
            }


            debugCounter++;


            holdControlPeriod(
                cycleStart
            );


            continue;
        }


        i2cFailureCount =
            0;


        // ==========================================
        // FSR RAW -> Level
        // ==========================================
        uint8_t accelLevel =
            rawToLevel(
                accelRaw,
                accelThreshold
            );


        uint8_t brakeLevel =
            rawToLevel(
                brakeRaw,
                brakeThreshold
            );


        // Brake priority
        if (brakeLevel > 0)
        {
            accelLevel =
                0;
        }


        // ==========================================
        // Emergency stop
        // ==========================================
        uint8_t flags =
            0;


        if (pcEmergencyStop)
        {
            flags |=
                FLAG_EMERGENCY_STOP;

            accelLevel =
                0;

            brakeLevel =
                0;
        }


        // ==========================================
        // Final command -> ESP32 #1
        // ==========================================
        sendCommandPacket(
            pcSteering,
            accelLevel,
            brakeLevel,
            flags
        );


        // ==========================================
        // Debug -> COM10
        //
        // Every 20th cycle keeps this near 5 Hz at a 100 Hz
        // control rate, so the debug line does not double the
        // UART1 load when the period is shortened.
        // ==========================================
        if (
            (debugCounter % 20)
            ==
            0
        )
        {
            xil_printf(
                "FSR ACC_RAW=%d A=%d BRAKE_RAW=%d B=%d STEER=%d ESTOP=%d PC=%d\r\n",
                accelRaw,
                accelLevel,
                brakeRaw,
                brakeLevel,
                pcSteering,
                pcEmergencyStop,
                (
                    pcSilenceMs
                    <=
                    PC_COMMAND_TIMEOUT_MS
                )
            );
        }


        debugCounter++;


        holdControlPeriod(
            cycleStart
        );
    }


    return 0;
}
