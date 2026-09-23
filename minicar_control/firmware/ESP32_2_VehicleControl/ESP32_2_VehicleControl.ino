#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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
// L298N
// ==================================================
#define IN1 25
#define IN2 26
#define ENA 27

#define IN3 32
#define IN4 14
#define ENB 13


// ==================================================
// PWM
// ==================================================
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;


// ==================================================
// Protocol
// ==================================================
#define FLAG_EMERGENCY_STOP 0x01

struct __attribute__((packed)) CommandPacket
{
  uint8_t sof1;
  uint8_t sof2;

  uint8_t sequence;

  int8_t steering;     // -90 ~ +90
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
};


DriverCommand command =
{
  0,      // steering
  0,      // accel
  0,      // brake
  false   // emergencyStop
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
// ==================================================
float vehicleSpeed = 0.0f;
float currentSteering = 0.0f;


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
const unsigned long CONTROL_PERIOD_MS = 20;

unsigned long lastControlTime = 0;


// ==================================================
// Acceleration Parameter
//
// virtual speed unit / second
// ==================================================
const float accelRateTable[6] =
{
  0.0f,

  8.0f,     // A1
  14.0f,    // A2
  22.0f,    // A3
  32.0f,    // A4
  45.0f     // A5
};


// ==================================================
// Brake Parameter
//
// virtual speed unit / second
// ==================================================
const float brakeRateTable[6] =
{
  0.0f,

  10.0f,    // B1
  18.0f,    // B2
  30.0f,    // B3
  50.0f,    // B4
  80.0f     // B5
};


// ==================================================
// Natural coast
// ==================================================
const float COAST_RATE = 4.0f;


// ==================================================
// Steering Parameter
//
// 180 deg/sec
// 0 -> 90 degree is about 0.5 sec
// ==================================================
const float STEERING_RATE = 180.0f;


// ==================================================
// Steering Mixing
//
// At full steering the inner motor runs at about 65%.
// ==================================================
const float TURN_GAIN = 0.35f;


// ==================================================
// Current 4.8V + L298N compensation
//
// Keeps the virtual vehicleSpeed separate from the PWM command.
// ==================================================
const float MOTOR_START_SPEED = 10.0f;
const int MOTOR_MIN_PWM = 65;


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


  // Steering must land on a whole 10 degree step.
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
// Virtual Speed -> Motor Command
//
// 0 ~ 10   : Motor OFF
// 10 ~ 100 : PWM Command 65 ~ 100
//
// Compensates for the current 4.8V + L298N setup.
// ==================================================
int speedToMotorCommand(
  float speed
)
{
  if (speed < MOTOR_START_SPEED)
  {
    return 0;
  }


  speed =
    constrain(
      speed,
      MOTOR_START_SPEED,
      100.0f
    );


  float ratio =
    (speed - MOTOR_START_SPEED)
    /
    (100.0f - MOTOR_START_SPEED);


  float commandValue =
    MOTOR_MIN_PWM
    +
    ratio *
    (100 - MOTOR_MIN_PWM);


  return (int)commandValue;
}


// ==================================================
// LEFT MOTOR
// ==================================================
void setLeftMotor(
  int speed
)
{
  speed =
    constrain(
      speed,
      -100,
      100
    );


  int pwm =
    map(
      abs(speed),
      0,
      100,
      0,
      255
    );


  if (speed > 0)
  {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  else if (speed < 0)
  {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  else
  {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }


  ledcWrite(
    ENA,
    pwm
  );
}


// ==================================================
// RIGHT MOTOR
//
// Logic is inverted because of the physical mounting direction.
// ==================================================
void setRightMotor(
  int speed
)
{
  speed =
    constrain(
      speed,
      -100,
      100
    );


  int pwm =
    map(
      abs(speed),
      0,
      100,
      0,
      255
    );


  if (speed > 0)
  {
    // Forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  else if (speed < 0)
  {
    // Backward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  else
  {
    // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }


  ledcWrite(
    ENB,
    pwm
  );
}


// ==================================================
// Motor Control
// ==================================================
void motorControl(
  int leftSpeed,
  int rightSpeed
)
{
  setLeftMotor(
    leftSpeed
  );

  setRightMotor(
    rightSpeed
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
// Speed + Steering -> Left / Right Motor
// ==================================================
void driveVehicle(
  float speed,
  float steering
)
{
  int baseCommand =
    speedToMotorCommand(
      speed
    );


  if (baseCommand == 0)
  {
    motorControl(
      0,
      0
    );

    return;
  }


  float steeringFactor =
    steering / 90.0f;


  steeringFactor =
    constrain(
      steeringFactor,
      -1.0f,
      1.0f
    );


  float left =
    baseCommand;

  float right =
    baseCommand;


  // ----------------------------------------------
  // Left Turn
  // ----------------------------------------------
  if (steeringFactor < 0.0f)
  {
    left =
      baseCommand *
      (
        1.0f
        -
        TURN_GAIN *
        (-steeringFactor)
      );
  }


  // ----------------------------------------------
  // Right Turn
  // ----------------------------------------------
  else if (steeringFactor > 0.0f)
  {
    right =
      baseCommand *
      (
        1.0f
        -
        TURN_GAIN *
        steeringFactor
      );
  }


  motorControl(
    (int)left,
    (int)right
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


    if (vehicleSpeed > 100.0f)
    {
      vehicleSpeed = 100.0f;
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


  Serial.print(
    "] MotorBase="
  );

  Serial.print(
    speedToMotorCommand(
      vehicleSpeed
    )
  );


  Serial.print(
    " Link="
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


  // ==================================================
  // Motor GPIO
  // ==================================================
  pinMode(
    IN1,
    OUTPUT
  );

  pinMode(
    IN2,
    OUTPUT
  );

  pinMode(
    IN3,
    OUTPUT
  );

  pinMode(
    IN4,
    OUTPUT
  );


  // ==================================================
  // PWM
  // ==================================================
  ledcAttach(
    ENA,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  ledcAttach(
    ENB,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  // Stopped at startup
  motorControl(
    0,
    0
  );


  // ==================================================
  // Wi-Fi / ESP-NOW
  // ==================================================
  WiFi.mode(
    WIFI_STA
  );


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
    " ESP32 #2 VEHICLE CONTROLLER"
  );

  Serial.println(
    "================================"
  );


  Serial.print(
    "Vehicle MAC = "
  );

  Serial.println(
    WiFi.macAddress()
  );


  Serial.println(
    "ESP-NOW Channel = 1"
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
  // 3. 50 Hz Vehicle Control
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
