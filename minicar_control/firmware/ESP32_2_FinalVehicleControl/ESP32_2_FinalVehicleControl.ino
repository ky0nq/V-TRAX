#include <WiFi.h>

#include <esp_now.h>

#include <esp_wifi.h>

// ==================================================
// MDD10A Motor Driver Pinout
//
// CH1 = Left side  (FL + RL in parallel)
// CH2 = Right side (FR + RR in parallel)
//
// Verified on the real vehicle:
//   LEFT  : DIR LOW  = Forward, DIR HIGH = Reverse
//   RIGHT : DIR HIGH = Forward, DIR LOW  = Reverse
// ==================================================

#define LEFT_PWM    16
#define LEFT_DIR    17
#define RIGHT_PWM   18
#define RIGHT_DIR   19

#define LEFT_FORWARD   LOW
#define LEFT_REVERSE   HIGH
#define RIGHT_FORWARD  HIGH
#define RIGHT_REVERSE  LOW

// Motor-terminal DMM readings depend on battery state and load.
// PWM 230 measured roughly 5.8 to 6.3 V during no-load wheel tests.
// Keep 230 as the initial hardware PWM limit instead of 255.
const int MDD10A_PWM_MAX = 230;


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
//   0.0 ~ 255.0 = internal vehicle speed-command magnitude
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

// Direct PWM no longer has the old 9600-baud Car_Shield bottleneck.
// Keep a 25 ms (40 Hz) control period so steering, braking,
// target-speed ramping, and communication behavior remain predictable.
const unsigned long CONTROL_PERIOD_MS = 25;


unsigned long lastControlTime = 0;


// ==================================================
// Vehicle Control Speed Domain
//
// vehicleSpeed remains an internal 0 ~ 255 command magnitude.
// Accel and brake remain application-level 0 ~ 5 commands.
//
// Accelerator levels select target speeds from targetSpeedTable[].
// vehicleSpeed approaches the selected target smoothly using
// SPEED_RISE_RATE and SPEED_FALL_RATE.
//
// The final hardware PWM is scaled separately to 0 ~ MDD10A_PWM_MAX.
// ==================================================

const float VEHICLE_SPEED_MAX = 255.0f;

// Minimum internal motor command used while a wheel is commanded to move.
// This value is intentionally lower than the A1 target to provide some
// low-speed headroom for steering and deceleration.
//
// command 60 -> actual PWM approximately 54
// A1 target 67 -> actual PWM approximately 60
const int MOTOR_COMMAND_MIN = 60;

// Car-like differential steering.
//
// Both left and right sides keep rotating FORWARD while steering.
// Steering changes only the speed difference between the two sides.
//
// 0.30 means that at full steering:
//   outer target = base * 1.30
//   inner target = base * 0.70
//
// Final moving commands are clamped to the configured range:
//   MOTOR_COMMAND_MIN ~ 255
//
// Car-like differential steering gain.
const float STEER_DIFF_GAIN = 0.30f;


// ==================================================
// Accelerator Level -> Target Speed
//
// Approximate final PWM targets during straight driving:
//   A1 -> PWM 60
//   A2 -> PWM 90
//   A3 -> PWM 125
//   A4 -> PWM 170
//   A5 -> PWM 230
//
// These are internal command targets, not direct PWM values.
// commandToPwm() performs the final 0 ~ 255 to 0 ~ 230 scaling.
// ==================================================

const float targetSpeedTable[6] =
{
  0.0f,    // A0 -> no accelerator target
  67.0f,   // A1 -> actual PWM approximately 60
  100.0f,  // A2 -> actual PWM approximately 90
  139.0f,  // A3 -> actual PWM approximately 125
  188.0f,  // A4 -> actual PWM approximately 170
  255.0f   // A5 -> actual PWM 230
};

// Ramp rates used to move vehicleSpeed toward a nonzero accelerator target.
// unit = internal command units per second
//
// SPEED_RISE_RATE controls how smoothly speed increases.
// SPEED_FALL_RATE controls how quickly speed falls when a lower
// nonzero accelerator target is selected.
const float SPEED_RISE_RATE = 50.0f;
const float SPEED_FALL_RATE = 80.0f;


// ==================================================
// Brake Parameter
//
// unit = internal speed-command units / second
// ==================================================

const float brakeRateTable[6] =
{
  0.0f,
  25.5f,     // B1 : 255 -> 0 ~= 10.0 sec
  45.9f,     // B2 : 255 -> 0 ~= 5.6 sec
  76.5f,     // B3 : 255 -> 0 ~= 3.3 sec
  255.0f,     // B4 : 255 -> 0 ~= 1.0 sec
  318.75f    // B5 : 255 -> 0 ~= 0.8 sec
};


// ==================================================
// Natural Coast
//
// Applied only when both accelerator and brake are zero.
// This remains separate from SPEED_FALL_RATE, which is used when a lower
// nonzero accelerator target is selected.
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
// MDD10A Direct Motor Output
//
// Upper control still uses -255 ~ +255 commands.
// The magnitude is scaled to 0 ~ MDD10A_PWM_MAX (230).
// Direction is converted to the verified DIR polarity for each side.
// ==================================================

int lastMotorLeftCommand = 0;
int lastMotorRightCommand = 0;
int lastMotorLeftPwm = 0;
int lastMotorRightPwm = 0;


int commandToPwm(int command)
{
  command = constrain(command, -255, 255);

  int magnitude = abs(command);

  if (magnitude == 0)
  {
    return 0;
  }

  return map(
    magnitude,
    0,
    255,
    0,
    MDD10A_PWM_MAX
  );
}


void motorInit()
{
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Safe boot state: PWM off first.
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  digitalWrite(LEFT_DIR, LEFT_FORWARD);
  digitalWrite(RIGHT_DIR, RIGHT_FORWARD);

  lastMotorLeftCommand = 0;
  lastMotorRightCommand = 0;
  lastMotorLeftPwm = 0;
  lastMotorRightPwm = 0;
}


void setLeftMotor(int command)
{
  command = constrain(command, -255, 255);

  if (command == 0)
  {
    analogWrite(LEFT_PWM, 0);
    lastMotorLeftCommand = 0;
    lastMotorLeftPwm = 0;
    return;
  }

  digitalWrite(
    LEFT_DIR,
    command > 0 ? LEFT_FORWARD : LEFT_REVERSE
  );

  int pwm = commandToPwm(command);
  analogWrite(LEFT_PWM, pwm);

  lastMotorLeftCommand = command;
  lastMotorLeftPwm = pwm;
}


void setRightMotor(int command)
{
  command = constrain(command, -255, 255);

  if (command == 0)
  {
    analogWrite(RIGHT_PWM, 0);
    lastMotorRightCommand = 0;
    lastMotorRightPwm = 0;
    return;
  }

  digitalWrite(
    RIGHT_DIR,
    command > 0 ? RIGHT_FORWARD : RIGHT_REVERSE
  );

  int pwm = commandToPwm(command);
  analogWrite(RIGHT_PWM, pwm);

  lastMotorRightCommand = command;
  lastMotorRightPwm = pwm;
}


void motorControl(
  int leftCommand,
  int rightCommand
)
{
  setLeftMotor(leftCommand);
  setRightMotor(rightCommand);
}


void forceMotorControl(
  int leftCommand,
  int rightCommand
)
{
  // Kept for compatibility with the existing setup flow.
  // Direct PWM does not need the old serial-output cache.
  motorControl(leftCommand, rightCommand);
}


// ==================================================
// Speed State -> Base Motor Command
//
// vehicleSpeed is already expressed in the same internal 0 ~ 255 command
// domain used by the target-speed table. Do not remap it a second time.
//
// Therefore:
//   speed == 0       -> motor command 0
//   speed > 0        -> MOTOR_COMMAND_MIN ~ 255
//
// MOTOR_COMMAND_MIN is an internal command value, not a direct PWM value.
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
      VEHICLE_SPEED_MAX
    );

  int commandValue =
    (int)roundf(speed);

  if (commandValue < MOTOR_COMMAND_MIN)
  {
    commandValue = MOTOR_COMMAND_MIN;
  }

  return constrain(
    commandValue,
    MOTOR_COMMAND_MIN,
    255
  );
}


// ==================================================
// Clamp a Moving Motor Command
//
// Any positive moving command below MOTOR_COMMAND_MIN is raised to
// MOTOR_COMMAND_MIN. This avoids very small commands that may not produce
// stable wheel motion.
//
// Current configured moving-command range:
//   MOTOR_COMMAND_MIN ~ 255
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

  if (output < MOTOR_COMMAND_MIN)
  {
    output = MOTOR_COMMAND_MIN;
  }

  if (output > 255)
  {
    output = 255;
  }

  return output;
}


// ==================================================
// Car-like Steering Mixer
//
// Steering here does NOT reverse
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
// At full steering with STEER_DIFF_GAIN = 0.30:
//
//   outer target = base * 1.30
//   inner target = base * 0.70
//
// After steering mixing, positive commands are limited to
// MOTOR_COMMAND_MIN ~ 255.
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
    STEER_DIFF_GAIN *
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
// Speed + Steering -> Left / Right Commands
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


  // ----------------------------------------------
  // Brake behavior is shared by FORWARD and REVERSE.
  //
  // vehicleSpeed is always a positive magnitude.
  // Reverse direction is applied later in driveVehicle()
  // by negating the final left/right motor commands.
  //
  // Therefore both directions use the same brakeRateTable[].
  // ----------------------------------------------

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

    // Target-Speed Throttle Control

    // ----------------------------------------------

    float targetSpeed =

      targetSpeedTable[

        command.accel

      ];


    float speedRate;


    if (vehicleSpeed < targetSpeed)

    {

      speedRate = SPEED_RISE_RATE;

    }

    else

    {

      speedRate = SPEED_FALL_RATE;

    }


    vehicleSpeed =

      moveToward(

        vehicleSpeed,

        targetSpeed,

        speedRate * dt

      );

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
    "] PWM[L="
  );

  Serial.print(
    lastMotorLeftPwm
  );

  Serial.print(
    " R="
  );

  Serial.print(
    lastMotorRightPwm
  );

  Serial.print(
    "] Link="
  );


  Serial.println(

    communicationActive ?

    "OK" :

    "STOP"

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


  motorInit();

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

    " MDD10A VEHICLE CONTROLLER"

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
    "MDD10A PWM Max = "
  );

  Serial.println(
    MDD10A_PWM_MAX
  );


  Serial.print(
    "Motor Command Min = "
  );

  Serial.println(
    MOTOR_COMMAND_MIN
  );


  Serial.print(
    "Steering Diff Gain = "
  );

  Serial.println(
    STEER_DIFF_GAIN,
    2
  );


  Serial.println(
    "Build = v8 MDD10A PWM230 + REVERSE + B4 1.0s/B5 0.8s"
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
