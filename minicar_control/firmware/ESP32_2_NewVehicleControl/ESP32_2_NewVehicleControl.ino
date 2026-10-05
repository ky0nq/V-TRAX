#include <WiFi.h>

#include <esp_now.h>

#include <esp_wifi.h>

#include <ACB_SmartCar_V2.h>


ACB_SmartCar_V2 ACB_SmartCar;


// ==================================================

// ESP32 #1 BRIDGE MAC

// B0:3F:D3:75:17:50

// ==================================================

const uint8_t BRIDGE_MAC[6] =

{

  0xB0,

  0x3F,

  0xD3,

  0x75,

  0x17,

  0x50

};


// ==================================================

// Protocol

// ==================================================

#define FLAG_EMERGENCY_STOP 0x01
#define FLAG_REVERSE_MODE   0x02


struct __attribute__((packed)) CommandPacket

{

  uint8_t sof1;

  uint8_t sof2;


  uint8_t sequence;


  int8_t steering;     // -90 ~ +90 normalized steering command

  uint8_t accel;       // 0 ~ 5

  uint8_t brake;       // 0 ~ 5


  uint8_t flags;


  uint8_t crc;

};


static_assert(

  sizeof(CommandPacket) == 8,

  "CommandPacket must be exactly 8 bytes"

);


// ==================================================

// Application Command

// ==================================================

struct DriverCommand

{

  int8_t steering;

  uint8_t accel;

  uint8_t brake;


  bool emergencyStop;

  bool reverseMode;

};


DriverCommand command =

{

  0,      // steering

  0,      // accel

  0,      // brake

  false,  // emergencyStop

  false   // reverseMode

};


// ==================================================

// ESP-NOW receive buffer

//

// The receive callback and loop() may run on different tasks,

// so the shared packet is guarded by a short critical section.

// ==================================================

portMUX_TYPE packetMux =

  portMUX_INITIALIZER_UNLOCKED;


CommandPacket pendingPacket;


volatile bool newPacketAvailable = false;


// ==================================================

// Control State
//
// vehicleSpeed:
//   0.0 ~ 255.0 = ACEBOTT motor command magnitude
//
// currentSteering:
//   -90 ~ +90 = normalized steering command
//   It is NOT a physical wheel angle.
// ==================================================

float vehicleSpeed = 0.0f;

float currentSteering = 0.0f;


// ==================================================
// Direction-change interlock
//
// JA2 mode selection is intentionally independent from JA1 ARM/STOP.
// Therefore ESP32 #2 locally prevents an instantaneous direction reversal.
//
// When FORWARD <-> REVERSE changes:
//   1. hardStop() immediately
//   2. require accelerator level 0 once
//   3. only then allow motion in the new direction
// ==================================================

bool directionChangeInterlock = false;


// ==================================================

// Communication State

// ==================================================

const unsigned long FAILSAFE_MS = 300;


unsigned long lastPacketTime = 0;


bool communicationActive = false;


uint8_t lastSequence = 0;

bool sequenceInitialized = false;


// ==================================================

// Control Period

// ==================================================

// ACB_SmartCar_V2 sends four motor commands through a 9600-baud
// serial link. 25 ms gives the four commands enough transmission margin.
const unsigned long CONTROL_PERIOD_MS = 25;


unsigned long lastControlTime = 0;


// ==================================================
// ACEBOTT QD001 V2 Speed Domain
//
// The ACEBOTT motor API uses -255 ~ +255.
// vehicleSpeed is the forward command magnitude: 0 ~ 255.
//
// Accel / brake are still application-level 0 ~ 5 commands.
// The rates below are expressed directly in ACEBOTT command units/sec.
// They preserve the previous time-to-full-scale behavior after changing
// the speed domain from 0~100 to 0~255. They are NOT minimum-PWM
// compensation values and can be tuned later on the real QD001 V2.
// ==================================================

const float ACEBOTT_SPEED_MAX = 255.0f;

// Measured on this QD001 V2:
//   110 : some wheels may move, unstable
//   120 : vehicle starts moving on the floor
//   125 : chosen minimum reliable drive command with margin
//
// Any wheel that is intentionally moving is commanded at >= 125.
// This avoids the unstable partial-wheel region seen around 110.
const int ACEBOTT_MIN_DRIVE = 125;

// Car-like differential steering.
//
// Both left and right sides keep rotating FORWARD while steering.
// Steering changes only the speed difference between the two sides.
//
// 0.30 means that at full steering:
//   outer target = base * 1.30
//   inner target = base * 0.70
//
// Final commands are clamped to the measured reliable range:
//   125 ~ 255
//
// This value is a starting tuning value for the ACEBOTT QD001 V2.
// It is NOT the old L298N TURN_GAIN rule.
const float ACEBOTT_STEER_DIFF_GAIN = 0.30f;


// ==================================================
// Acceleration Parameter
//
// unit = ACEBOTT speed-command units / second
// ==================================================

const float accelRateTable[6] =
{
  0.0f,
  20.4f,     // A1 : 0 -> 255 ~= 12.5 sec
  35.7f,     // A2 : ~= 7.1 sec
  56.1f,     // A3 : ~= 4.5 sec
  81.6f,     // A4 : ~= 3.1 sec
  114.8f     // A5 : ~= 2.2 sec
};


// ==================================================
// Brake Parameter
//
// unit = ACEBOTT speed-command units / second
// ==================================================

const float brakeRateTable[6] =
{
  0.0f,
  25.5f,     // B1 : 255 -> 0 = 10 sec
  45.9f,     // B2 : ~= 5.6 sec
  76.5f,     // B3 : ~= 3.3 sec
  255.0f,    // B4 : 1 sec
  318.75f     // B5 : ~= 0.8 sec
};


// ==================================================
// Natural coast
// ==================================================

const float COAST_RATE = 10.2f;


// ==================================================
// Steering Parameter
//
// Steering remains -90 ~ +90 only because this is the existing
// packet/CNN command domain. It is NOT a physical steering angle.
//
// 180 command-units/sec means:
// center (0) -> full command (+/-90) ~= 0.5 sec.
// ==================================================

const float STEERING_RATE = 180.0f;


// ==================================================

// CRC-8

// Polynomial = 0x07

// Initial = 0x00

// ==================================================

uint8_t calculateCRC8(

  const uint8_t* data,

  size_t length

)

{

  uint8_t crc = 0x00;


  for (size_t i = 0; i < length; i++)

  {

    crc ^= data[i];


    for (int bit = 0; bit < 8; bit++)

    {

      if (crc & 0x80)

      {

        crc =

          (crc << 1) ^ 0x07;

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

// Packet Validation

// ==================================================

bool validatePacket(

  const CommandPacket& packet

)

{

  // ----------------------------------------------

  // Header

  // ----------------------------------------------

  if (

    packet.sof1 != 0xAA ||

    packet.sof2 != 0x55

  )

  {

    return false;

  }


  // ----------------------------------------------

  // CRC

  // ----------------------------------------------

  uint8_t expectedCRC =

    calculateCRC8(

      (const uint8_t*)&packet,

      7

    );


  if (packet.crc != expectedCRC)

  {

    return false;

  }


  // ----------------------------------------------

  // Command range

  // ----------------------------------------------

  if (

    packet.steering < -90 ||

    packet.steering > 90

  )

  {

    return false;

  }


  // Steering must land on a whole 10-unit command step.

  if ((packet.steering % 10) != 0)

  {

    return false;

  }


  if (packet.accel > 5)

  {

    return false;

  }


  if (packet.brake > 5)

  {

    return false;

  }


  return true;

}


// ==================================================

// ESP-NOW callback

//

// This only:

// 1. checks the sender MAC

// 2. checks the length

// 3. copies the packet

//

// No vehicle control is done here.

// ==================================================

void onDataRecv(

  const esp_now_recv_info_t* info,

  const uint8_t* data,

  int len

)

{

  // ----------------------------------------------

  // Accept only packets sent by the ESP32 #1 bridge.

  // ----------------------------------------------

  if (

    memcmp(

      info->src_addr,

      BRIDGE_MAC,

      6

    ) != 0

  )

  {

    return;

  }


  // ----------------------------------------------

  // Packet size

  // ----------------------------------------------

  if (len != sizeof(CommandPacket))

  {

    return;

  }


  // ----------------------------------------------

  // Copy into the shared buffer

  // ----------------------------------------------

  portENTER_CRITICAL(&packetMux);


  memcpy(

    &pendingPacket,

    data,

    sizeof(CommandPacket)

  );


  newPacketAvailable = true;


  portEXIT_CRITICAL(&packetMux);

}


// ==================================================

// Utility

// ==================================================

float moveToward(

  float currentValue,

  float targetValue,

  float maxStep

)

{

  if (currentValue < targetValue)

  {

    currentValue += maxStep;


    if (currentValue > targetValue)

      currentValue = targetValue;

  }

  else if (currentValue > targetValue)

  {

    currentValue -= maxStep;


    if (currentValue < targetValue)

      currentValue = targetValue;

  }


  return currentValue;

}


// ==================================================
// v5 Minimal Motor Output Cache
//
// IMPORTANT:
// Everything related to ESP-NOW receive, packet sequence,
// packet validation, vehicle state, steering and failsafe
// remains exactly as v4.
//
// Only repeated ACEBOTT motor serial writes are suppressed.
// ==================================================

int lastMotorLeftCommand = 0;
int lastMotorRightCommand = 0;

bool motorOutputInitialized = false;

uint32_t motorTransmitCount = 0;
uint32_t motorDuplicateSkipCount = 0;


// ==================================================
// ACEBOTT QD001 V2 Motor Output
//
// Library motor numbering:
//   1 = Front Left
//   2 = Rear Left
//   3 = Front Right
//   4 = Rear Right
//
// Command range:
//   -255 = reverse
//      0 = stop
//   +255 = forward
// ==================================================

void writeMotorCommands(
  int leftCommand,
  int rightCommand,
  bool forceTransmit
)
{
  leftCommand =
    constrain(
      leftCommand,
      -255,
      255
    );

  rightCommand =
    constrain(
      rightCommand,
      -255,
      255
    );

  // Skip only when the actual motor command is identical.
  if (
    !forceTransmit
    &&
    motorOutputInitialized
    &&
    leftCommand == lastMotorLeftCommand
    &&
    rightCommand == lastMotorRightCommand
  )
  {
    motorDuplicateSkipCount++;
    return;
  }

  ACB_SmartCar.motorControl(1, leftCommand);
  ACB_SmartCar.motorControl(2, leftCommand);
  ACB_SmartCar.motorControl(3, rightCommand);
  ACB_SmartCar.motorControl(4, rightCommand);

  lastMotorLeftCommand = leftCommand;
  lastMotorRightCommand = rightCommand;
  motorOutputInitialized = true;

  motorTransmitCount++;
}


void motorControl(
  int leftCommand,
  int rightCommand
)
{
  writeMotorCommands(
    leftCommand,
    rightCommand,
    false
  );
}


void forceMotorControl(
  int leftCommand,
  int rightCommand
)
{
  writeMotorCommands(
    leftCommand,
    rightCommand,
    true
  );
}


// ==================================================
// Speed State -> ACEBOTT Base Command
//
// vehicleSpeed is an internal 0~255 state.
// The real QD001 V2 cannot move reliably at very small motor commands.
//
// Therefore:
//   speed == 0       -> motor command 0
//   speed > 0        -> 125 ~ 255
//
// This is NOT the old L298N "65% PWM" rule.
// 125 comes from the measured QD001 V2 minimum reliable drive region.
// ==================================================

int speedToBaseCommand(
  float speed
)
{
  if (speed <= 0.0f)
  {
    return 0;
  }

  speed =
    constrain(
      speed,
      0.0f,
      ACEBOTT_SPEED_MAX
    );

  float ratio =
    speed / ACEBOTT_SPEED_MAX;

  float commandValue =
    ACEBOTT_MIN_DRIVE
    +
    ratio *
    (
      ACEBOTT_SPEED_MAX -
      ACEBOTT_MIN_DRIVE
    );

  return constrain(
    (int)roundf(commandValue),
    ACEBOTT_MIN_DRIVE,
    255
  );
}


// ==================================================
// Clamp a moving ACEBOTT motor command
//
// During normal driving we do not use the unstable 1~124 region.
// A moving wheel is therefore always commanded at 125~255.
//
// Steering never reverses a wheel in this car-like mode.
// ==================================================

int clampMovingCommand(
  float commandValue
)
{
  if (commandValue <= 0.0f)
  {
    return 0;
  }

  int output =
    (int)roundf(commandValue);

  if (output < ACEBOTT_MIN_DRIVE)
  {
    output = ACEBOTT_MIN_DRIVE;
  }

  if (output > 255)
  {
    output = 255;
  }

  return output;
}


// ==================================================
// ACEBOTT Car-like Steering Mixer
//
// Unlike ACEBOTT's Rotate commands, steering here does NOT reverse
// either side of the vehicle.
//
// steering = 0:
//   Left  = base
//   Right = base
//
// steering > 0 (right turn):
//   Left  = faster  (outer side)
//   Right = slower  (inner side)
//
// steering < 0 (left turn):
//   Left  = slower  (inner side)
//   Right = faster  (outer side)
//
// At full steering with ACEBOTT_STEER_DIFF_GAIN = 0.30:
//
//   outer target = base * 1.30
//   inner target = base * 0.70
//
// After that, commands are limited to 125~255.
//
// -90 / +90 are normalized steering-command endpoints.
// They are NOT physical wheel angles.
// ==================================================

void calculateMotorCommands(
  float speed,
  float steering,
  int& baseCommand,
  int& leftCommand,
  int& rightCommand
)
{
  baseCommand =
    speedToBaseCommand(
      speed
    );

  if (baseCommand == 0)
  {
    leftCommand = 0;
    rightCommand = 0;
    return;
  }

  float steeringFactor =
    constrain(
      steering / 90.0f,
      -1.0f,
      1.0f
    );

  float turnAmount =
    ACEBOTT_STEER_DIFF_GAIN *
    fabsf(steeringFactor);

  float leftTarget =
    (float)baseCommand;

  float rightTarget =
    (float)baseCommand;

  if (steeringFactor > 0.0f)
  {
    // Right turn:
    // left side is outer, right side is inner.
    leftTarget =
      baseCommand *
      (1.0f + turnAmount);

    rightTarget =
      baseCommand *
      (1.0f - turnAmount);
  }
  else if (steeringFactor < 0.0f)
  {
    // Left turn:
    // right side is outer, left side is inner.
    leftTarget =
      baseCommand *
      (1.0f - turnAmount);

    rightTarget =
      baseCommand *
      (1.0f + turnAmount);
  }

  leftCommand =
    clampMovingCommand(
      leftTarget
    );

  rightCommand =
    clampMovingCommand(
      rightTarget
    );
}


// ==================================================
// Speed + Steering -> ACEBOTT Left / Right Commands
// ==================================================

void driveVehicle(
  float speed,
  float steering
)
{
  int baseCommand = 0;
  int leftCommand = 0;
  int rightCommand = 0;

  calculateMotorCommands(
    speed,
    steering,
    baseCommand,
    leftCommand,
    rightCommand
  );

  // ------------------------------------------------
  // Reverse mode
  //
  // calculateMotorCommands() always calculates positive
  // car-like steering magnitudes. Reverse mode changes
  // only the final motor direction.
  //
  // CNN steering sign is NOT changed here. If reverse
  // steering needs to be mirrored after real-car testing,
  // use ADDITIONAL_FEATURE_INVERT_CNN_STEERING on Zybo.
  // ------------------------------------------------
  if (command.reverseMode)
  {
    leftCommand =
      -leftCommand;

    rightCommand =
      -rightCommand;
  }

  motorControl(
    leftCommand,
    rightCommand
  );
}


// ==================================================

// Hard Stop

//

// Used by Failsafe and Emergency Stop.

// ==================================================

void hardStop()

{

  vehicleSpeed = 0.0f;


  motorControl(

    0,

    0

  );

}


// ==================================================

// Vehicle Control

// ==================================================

void updateVehicleControl(

  float dt

)

{

  // ----------------------------------------------

  // Emergency Stop

  // ----------------------------------------------

  if (command.emergencyStop)

  {

    hardStop();


    return;

  }


  // ----------------------------------------------
  // Direction-change interlock
  //
  // After FORWARD <-> REVERSE changes, the accelerator
  // must be released once before motion can resume.
  // ----------------------------------------------
  if (directionChangeInterlock)

  {

    hardStop();


    if (command.accel == 0)

    {

      directionChangeInterlock = false;


      Serial.println(

        "DIRECTION INTERLOCK RELEASED"

      );

    }


    return;

  }

  // ==================================================

  // 1. Steering State Update

  // ==================================================

  float steeringStep =

    STEERING_RATE * dt;


  currentSteering =

    moveToward(

      currentSteering,

      (float)command.steering,

      steeringStep

    );


  // ==================================================

  // 2. Longitudinal Vehicle State

  //

  // Brake > Accel

  // ==================================================


  if (command.brake > 0)

  {

    // ----------------------------------------------

    // Brake

    // ----------------------------------------------

    float deceleration =

      brakeRateTable[

        command.brake

      ];


    vehicleSpeed -=

      deceleration * dt;


    if (vehicleSpeed < 0.0f)

    {

      vehicleSpeed = 0.0f;

    }

  }


  else if (command.accel > 0)

  {

    // ----------------------------------------------

    // Acceleration

    // ----------------------------------------------

    float acceleration =

      accelRateTable[

        command.accel

      ];


    vehicleSpeed +=

      acceleration * dt;


    if (vehicleSpeed > ACEBOTT_SPEED_MAX)

    {

      vehicleSpeed = ACEBOTT_SPEED_MAX;

    }

  }


  else

  {

    // ----------------------------------------------

    // Coast

    // Accel = 0

    // Brake = 0

    // ----------------------------------------------

    vehicleSpeed -=

      COAST_RATE * dt;


    if (vehicleSpeed < 0.0f)

    {

      vehicleSpeed = 0.0f;

    }

  }


  // ==================================================

  // 3. Motor Output

  // ==================================================

  driveVehicle(

    vehicleSpeed,

    currentSteering

  );

}


// ==================================================

// Received packet -> DriverCommand

// ==================================================

void applyReceivedPacket()

{

  if (!newPacketAvailable)

  {

    return;

  }


  CommandPacket packet;


  // ----------------------------------------------

  // Take the shared packet

  // ----------------------------------------------

  portENTER_CRITICAL(&packetMux);


  memcpy(

    &packet,

    &pendingPacket,

    sizeof(packet)

  );


  newPacketAvailable = false;


  portEXIT_CRITICAL(&packetMux);


  // ----------------------------------------------

  // Header / CRC / range check

  // ----------------------------------------------

  if (!validatePacket(packet))

  {

    Serial.println(

      "INVALID PACKET"

    );


    return;

  }


  // ----------------------------------------------

  // Sequence check

  // ----------------------------------------------

  if (sequenceInitialized)

  {

    uint8_t expected =

      lastSequence + 1;


    if (

      packet.sequence != expected

    )

    {

      Serial.print(

        "SEQ GAP: expected="

      );


      Serial.print(

        expected

      );


      Serial.print(

        " received="

      );


      Serial.println(

        packet.sequence

      );

    }

  }


  lastSequence =

    packet.sequence;


  sequenceInitialized =

    true;


  // ----------------------------------------------

  // Update command

  // ----------------------------------------------

  bool newReverseMode =

    (

      packet.flags &

      FLAG_REVERSE_MODE

    ) != 0;


  bool directionChanged =

    newReverseMode

    !=

    command.reverseMode;


  command.steering =

    packet.steering;


  command.accel =

    packet.accel;


  command.brake =

    packet.brake;


  command.emergencyStop =

    (

      packet.flags &

      FLAG_EMERGENCY_STOP

    ) != 0;


  command.reverseMode =

    newReverseMode;


  // ----------------------------------------------
  // Direction changed
  //
  // Stop immediately. Because JA2 does not disarm JA1,
  // the accelerator must be released once before driving
  // in the opposite direction.
  // ----------------------------------------------
  if (directionChanged)

  {

    hardStop();


    directionChangeInterlock = true;


    Serial.print(

      "DRIVE MODE -> "

    );


    Serial.println(

      command.reverseMode

        ? "REVERSE"

        : "FORWARD"

    );

  }


  // ----------------------------------------------

  // Communication State

  // ----------------------------------------------

  lastPacketTime =

    millis();


  communicationActive =

    true;


  // ----------------------------------------------

  // Emergency is handled immediately

  // ----------------------------------------------

  if (command.emergencyStop)

  {

    hardStop();


    Serial.println(

      "EMERGENCY STOP"

    );

  }

}


// ==================================================

// Communication Failsafe

// ==================================================

void checkFailsafe()

{

  if (!communicationActive)

  {

    return;

  }


  if (

    millis() - lastPacketTime

    >

    FAILSAFE_MS

  )

  {

    communicationActive =

      false;


    currentSteering = 0.0f;


    hardStop();


    Serial.println(

      "FAILSAFE -> MOTOR STOP"

    );

  }

}


// ==================================================

// Debug

// ==================================================

void printStatus()

{

  static unsigned long lastPrintTime = 0;


  if (

    millis() - lastPrintTime

    <

    200

  )

  {

    return;

  }


  lastPrintTime =

    millis();


  Serial.print(

    "SEQ="

  );


  Serial.print(

    lastSequence

  );


  Serial.print(

    " CMD[S="

  );


  Serial.print(

    command.steering

  );


  Serial.print(

    " A="

  );


  Serial.print(

    command.accel

  );


  Serial.print(

    " B="

  );


  Serial.print(

    command.brake

  );


  Serial.print(

    " MODE="

  );


  Serial.print(

    command.reverseMode

      ? "REV"

      : "FWD"

  );


  Serial.print(

    " ILK="

  );


  Serial.print(

    directionChangeInterlock

      ? 1

      : 0

  );


  Serial.print(

    "] STATE[V="

  );


  Serial.print(

    vehicleSpeed,

    1

  );


  Serial.print(

    " Steering="

  );


  Serial.print(

    currentSteering,

    1

  );


  int debugBase = 0;
  int debugLeft = 0;
  int debugRight = 0;

  calculateMotorCommands(
    vehicleSpeed,
    currentSteering,
    debugBase,
    debugLeft,
    debugRight
  );

  if (command.reverseMode)
  {
    debugLeft =
      -debugLeft;

    debugRight =
      -debugRight;
  }

  Serial.print(
    "] Base="
  );

  Serial.print(
    debugBase
  );

  Serial.print(
    " Motor[L="
  );

  Serial.print(
    debugLeft
  );

  Serial.print(
    " R="
  );

  Serial.print(
    debugRight
  );

  Serial.print(
    "] Link="
  );


  Serial.print(

    communicationActive ?

    "OK" :

    "STOP"

  );


  Serial.print(
    " MotorTX="
  );

  Serial.print(
    motorTransmitCount
  );


  Serial.print(
    " MotorSkip="
  );

  Serial.println(
    motorDuplicateSkipCount
  );

}


// ==================================================

// SETUP

// ==================================================

void setup()

{

  Serial.begin(

    115200

  );


    ACB_SmartCar.Init();


    forceMotorControl(

        0,

        0

    );


  // ==================================================

  // Wi-Fi / ESP-NOW

  // ==================================================

  WiFi.mode(
    WIFI_STA
  );

  while (!WiFi.STA.started())
  {
    delay(10);
  }


  esp_wifi_set_channel(

    1,

    WIFI_SECOND_CHAN_NONE

  );


  if (

    esp_now_init()

    !=

    ESP_OK

  )

  {

    Serial.println(

      "ESP-NOW INIT FAILED"

    );


    while (true)

    {

      motorControl(

        0,

        0

      );


      delay(

        1000

      );

    }

  }


  esp_now_register_recv_cb(

    onDataRecv

  );


  // ==================================================

  // Start

  // ==================================================

  lastControlTime =

    millis();


  Serial.println();

  Serial.println(

    "================================"

  );


  Serial.println(

    " ACEBOTT QD001 V2 VEHICLE CONTROLLER"

  );


  Serial.println(

    "================================"

  );


  Serial.println(

    "Reverse flag = 0x02"

  );


  Serial.println(

    "Direction-change interlock = ENABLED"

  );


  Serial.print(

    "Vehicle MAC = "

  );


  Serial.println(

    WiFi.STA.macAddress()

  );


  Serial.println(

    "ESP-NOW Channel = 1"

  );


  Serial.print(
    "ACEBOTT Min Drive = "
  );

  Serial.println(
    ACEBOTT_MIN_DRIVE
  );


  Serial.print(
    "ACEBOTT Steering Diff Gain = "
  );

  Serial.println(
    ACEBOTT_STEER_DIFF_GAIN,
    2
  );

  Serial.println(

    "Waiting for Zybo Command..."

  );

}


// ==================================================

// LOOP

// ==================================================

void loop()

{

  // ------------------------------------------------

  // 1. Apply new ESP-NOW packet

  // ------------------------------------------------

  applyReceivedPacket();


  // ------------------------------------------------

  // 2. Communication failsafe

  // ------------------------------------------------

  checkFailsafe();


  // ------------------------------------------------

  // 3. 40 Hz Vehicle Control

  // ------------------------------------------------

  unsigned long now =

    millis();


  if (

    now - lastControlTime

    >=

    CONTROL_PERIOD_MS

  )

  {

    float dt =

      (

        now -

        lastControlTime

      )

      /

      1000.0f;


    lastControlTime =

      now;


    if (communicationActive)

    {

      updateVehicleControl(

        dt

      );

    }

    else

    {

      motorControl(

        0,

        0

      );

    }

  }


  // ------------------------------------------------

  // 4. Debug

  // ------------------------------------------------

  printStatus();

}
