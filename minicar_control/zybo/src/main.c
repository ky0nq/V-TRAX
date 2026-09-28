#include "vehicle.h"

#include "xil_printf.h"
#include "xstatus.h"


// ==================================================
// MAIN
//
// One pass of the loop is one command to the vehicle. The order
// is fixed: read both inputs, decide whether either has gone
// stale, then send -- so a command never leaves carrying an
// input the Zybo already knows it cannot trust.
//
// Three conditions raise E-Stop here. A fourth lives on
// ESP32 #2, which stops on its own if these packets stop
// arriving at all, and that one covers the cases the Zybo
// cannot see: its own death, ESP32 #1's, or the radio between
// them.
// ==================================================
int main(void)
{
    if (
        vehicleInit()
        !=
        XST_SUCCESS
    )
    {
        xil_printf(
            "VEHICLE INIT FAILED\r\n"
        );


        // Bare metal: returning from main is undefined, and a
        // board that cannot talk to the vehicle should sit still
        // rather than run an uninitialised loop.
        while (1)
        {
        }
    }


    uint32_t debugCounter =
        0;


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


    while (1)
    {
        XTime cycleStart;

        XTime_GetTime(
            &cycleStart
        );


        // ==============================================
        // 1. Inputs
        // ==============================================
        PcState pc;

        vehiclePollPC(
            &pc
        );


        SensorState sensor;

        vehiclePollSensor(
            &sensor
        );


        // ==============================================
        // 2. Steering source
        //
        // Stays 0 when the source is stale, so a frozen reading
        // can never keep the wheels turned.
        // ==============================================
        int8_t steering =
            0;


        int steeringOK =
            getSteering(
                pc.linkOK,
                &steering
            );


        // ==============================================
        // 3. Safety
        // ==============================================
        uint8_t accelLevel =
            sensor.accelLevel;


        uint8_t brakeLevel =
            sensor.brakeLevel;


        uint8_t flags =
            0;


        // Manual stop from the PC
        if (
            pc.emergencyStop
        )
        {
            flags |=
                FLAG_EMERGENCY_STOP;
        }


        // Pedal sensor wireless link lost
        if (
            !sensor.linkOK
        )
        {
            flags |=
                FLAG_EMERGENCY_STOP;
        }


        // Steering source stale
        if (
            !steeringOK
        )
        {
            flags |=
                FLAG_EMERGENCY_STOP;
        }


        // ESP32 #2 stops the motors on the flag alone, before it
        // looks at these, but sending a pedal command alongside
        // an E-Stop would still be a lie about what was asked.
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
        // 4. Send
        // ==============================================
        sendCommandPacket(
            steering,
            accelLevel,
            brakeLevel,
            flags
        );


        // ==============================================
        // 5. Debug
        //
        // 10 loops -> about 10 Hz at a 100 Hz control rate.
        //
        // STEER_SRC tells which of the three E-Stop triggers
        // fired, which is otherwise invisible from ESTOP alone.
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
                "STEER_SRC=%s:%s "
                "ESTOP=%d "
                "PC=%s\r\n",

                sensor.sequence,

                sensor.accelRaw,
                accelLevel,

                sensor.brakeRaw,
                brakeLevel,

                sensor.linkOK
                    ?
                    "OK"
                    :
                    "LOST",

                steering,

#if STEERING_SOURCE_CNN
                "CNN",
#else
                "PC",
#endif

                steeringOK
                    ?
                    "OK"
                    :
                    "STALE",

                (
                    flags
                    &
                    FLAG_EMERGENCY_STOP
                )
                    ?
                    1
                    :
                    0,

                pc.linkOK
                    ?
                    "OK"
                    :
                    "LOST"
            );
        }


        debugCounter++;


        holdControlPeriod(
            cycleStart
        );
    }


    return 0;
}
