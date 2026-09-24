#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


// ==================================================
// ESP32 #3 SENSOR NODE
//
// Accel FSR -> ADS1115 A0
// Brake FSR -> ADS1115 A1
//
// ADS1115 -> ESP32 #3 via I2C
// ESP32 #3 -> ESP32 #1 via ESP-NOW
//
// ESP-NOW Channel = 1
// ==================================================


// ==================================================
// I2C
// ==================================================
#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_CLOCK_HZ 100000


// ==================================================
// ADS1115
//
// ADDR -> GND
// Address = 0x48
// ==================================================
#define ADS1115_ADDR            0x48

#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01


// ==================================================
// ESP32 #1 Bridge MAC
//
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
// Sensor Packet
//
// 8 bytes total
//
// Byte 0 : 0xA5
// Byte 1 : 0x5A
// Byte 2 : sequence
// Byte 3~4 : accelRaw
// Byte 5~6 : brakeRaw
// Byte 7 : CRC8
// ==================================================
struct __attribute__((packed)) SensorPacket
{
  uint8_t sof1;
  uint8_t sof2;

  uint8_t sequence;

  int16_t accelRaw;
  int16_t brakeRaw;

  uint8_t crc;
};


static_assert(
  sizeof(SensorPacket) == 8,
  "SensorPacket must be exactly 8 bytes"
);


uint8_t sensorSequence = 0;


// ==================================================
// CRC-8
//
// Polynomial = 0x07
// Initial    = 0x00
//
// Computed over bytes 0..6
// ==================================================
uint8_t calculateCRC8(
  const uint8_t* data,
  size_t length
)
{
  uint8_t crc = 0x00;


  for (size_t i = 0;
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
// ADS1115 Register Write
// ==================================================
bool ADS1115_WriteRegister(
  uint8_t reg,
  uint16_t value
)
{
  Wire.beginTransmission(
    ADS1115_ADDR
  );


  Wire.write(
    reg
  );


  Wire.write(
    (uint8_t)(
      value >> 8
    )
  );


  Wire.write(
    (uint8_t)(
      value
      &
      0xFF
    )
  );


  uint8_t result =
    Wire.endTransmission();


  return (
    result == 0
  );
}


// ==================================================
// ADS1115 Conversion Register Read
// ==================================================
bool ADS1115_ReadConversion(
  int16_t &value
)
{
  Wire.beginTransmission(
    ADS1115_ADDR
  );


  Wire.write(
    ADS1115_REG_CONVERSION
  );


  // repeated start
  if (
    Wire.endTransmission(false)
    !=
    0
  )
  {
    return false;
  }


  int count =
    Wire.requestFrom(
      ADS1115_ADDR,
      (uint8_t)2
    );


  if (count != 2)
  {
    return false;
  }


  uint8_t msb =
    Wire.read();


  uint8_t lsb =
    Wire.read();


  value =
    (int16_t)(
      (
        (uint16_t)msb
        <<
        8
      )
      |
      lsb
    );


  return true;
}


// ==================================================
// ADS1115 Single-Ended Read
//
// Channel 0 = A0
// Channel 1 = A1
// Channel 2 = A2
// Channel 3 = A3
//
// PGA = +/-4.096 V
// Data Rate = 860 SPS
// ==================================================
bool ADS1115_ReadChannel(
  uint8_t channel,
  int16_t &value
)
{
  if (channel > 3)
  {
    return false;
  }


  // Single-ended MUX
  //
  // A0-GND = 100
  // A1-GND = 101
  // A2-GND = 110
  // A3-GND = 111
  uint16_t mux =
    4
    +
    channel;


  uint16_t config =
      0x8000          // OS = conversion start
    | (mux << 12)     // MUX
    | 0x0200          // PGA = +/-4.096V
    | 0x0100          // Single-shot
    | 0x00E0          // 860 SPS
    | 0x0003;         // Comparator disabled


  if (
    !ADS1115_WriteRegister(
      ADS1115_REG_CONFIG,
      config
    )
  )
  {
    return false;
  }


  // 860 SPS is about 1.16 ms
  // 2 ms leaves margin
  delay(2);


  return
    ADS1115_ReadConversion(
      value
    );
}


// ==================================================
// ADS1115 Connection Check
// ==================================================
bool ADS1115_Check()
{
  Wire.beginTransmission(
    ADS1115_ADDR
  );


  return (
    Wire.endTransmission()
    ==
    0
  );
}


// ==================================================
// ESP-NOW Setup
// ==================================================
bool setupESPNow()
{
  // ----------------------------------------------
  // Wi-Fi Station Mode
  // ----------------------------------------------
  WiFi.mode(
    WIFI_STA
  );


  // ----------------------------------------------
  // Channel = 1
  // ----------------------------------------------
  esp_err_t channelResult =
    esp_wifi_set_channel(
      1,
      WIFI_SECOND_CHAN_NONE
    );


  if (
    channelResult
    !=
    ESP_OK
  )
  {
    Serial.print(
      "SET CHANNEL FAILED: "
    );


    Serial.println(
      (int)channelResult
    );


    return false;
  }


  // ----------------------------------------------
  // ESP-NOW Initialize
  // ----------------------------------------------
  if (
    esp_now_init()
    !=
    ESP_OK
  )
  {
    Serial.println(
      "ESP-NOW INIT FAILED"
    );


    return false;
  }


  // ----------------------------------------------
  // ESP32 #1 Bridge Peer
  // ----------------------------------------------
  esp_now_peer_info_t peerInfo = {};


  memcpy(
    peerInfo.peer_addr,
    BRIDGE_MAC,
    6
  );


  peerInfo.channel =
    1;


  peerInfo.encrypt =
    false;


  peerInfo.ifidx =
    WIFI_IF_STA;


  // Remove any stale peer registration
  if (
    esp_now_is_peer_exist(
      BRIDGE_MAC
    )
  )
  {
    esp_now_del_peer(
      BRIDGE_MAC
    );
  }


  // Register the ESP32 #1 peer
  if (
    esp_now_add_peer(
      &peerInfo
    )
    !=
    ESP_OK
  )
  {
    Serial.println(
      "ADD BRIDGE PEER FAILED"
    );


    return false;
  }


  return true;
}


// ==================================================
// SETUP
// ==================================================
void setup()
{
  Serial.begin(
    115200
  );


  delay(
    1000
  );


  // ==================================================
  // I2C
  // ==================================================
  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );


  Wire.setClock(
    I2C_CLOCK_HZ
  );


  Serial.println();
  Serial.println(
    "================================"
  );


  Serial.println(
    " ESP32 #3 SENSOR NODE"
  );


  Serial.println(
    "================================"
  );


  Serial.println(
    "I2C SDA = GPIO21"
  );


  Serial.println(
    "I2C SCL = GPIO22"
  );


  Serial.println(
    "ADS1115 Address = 0x48"
  );


  // ==================================================
  // ADS1115 Check
  // ==================================================
  if (
    !ADS1115_Check()
  )
  {
    Serial.println(
      "ADS1115 NOT FOUND!"
    );


    Serial.println(
      "Check VDD / GND / SDA / SCL / ADDR"
    );


    while (true)
    {
      delay(
        1000
      );
    }
  }


  Serial.println(
    "ADS1115 FOUND!"
  );


  // ==================================================
  // ESP-NOW
  // ==================================================
  if (
    !setupESPNow()
  )
  {
    Serial.println(
      "ESP-NOW SETUP FAILED"
    );


    while (true)
    {
      delay(
        1000
      );
    }
  }


  // ==================================================
  // Start Information
  // ==================================================
  Serial.print(
    "ESP32 #3 MAC = "
  );


  Serial.println(
    WiFi.macAddress()
  );


  Serial.println(
    "ESP32 #1 MAC = B0:3F:D3:75:17:50"
  );


  Serial.println(
    "ESP-NOW Channel = 1"
  );


  Serial.println(
    "ADS1115 A0 = Accel FSR"
  );


  Serial.println(
    "ADS1115 A1 = Brake FSR"
  );


  Serial.println();


  Serial.println(
    "Sensor transmission started."
  );


  Serial.println();
}


// ==================================================
// LOOP
//
// Reads the sensors and transmits at about 50 Hz
// ==================================================
void loop()
{
  static unsigned long lastCycleTime =
    0;


  static uint32_t debugDivider =
    0;


  unsigned long now =
    millis();


  // ==================================================
  // 50 Hz
  // ==================================================
  if (
    now
    -
    lastCycleTime
    <
    20
  )
  {
    return;
  }


  lastCycleTime =
    now;


  // ==================================================
  // 1. ADS1115 Read
  // ==================================================
  int16_t accelRaw =
    0;


  int16_t brakeRaw =
    0;


  bool accelOK =
    ADS1115_ReadChannel(
      0,
      accelRaw
    );


  bool brakeOK =
    ADS1115_ReadChannel(
      1,
      brakeRaw
    );


  if (
    !accelOK
    ||
    !brakeOK
  )
  {
    Serial.println(
      "ADS1115 READ ERROR"
    );


    return;
  }


  // ==================================================
  // 2. Build the sensor packet
  // ==================================================
  SensorPacket packet;


  packet.sof1 =
    0xA5;


  packet.sof2 =
    0x5A;


  packet.sequence =
    sensorSequence++;


  packet.accelRaw =
    accelRaw;


  packet.brakeRaw =
    brakeRaw;


  packet.crc =
    calculateCRC8(
      (const uint8_t*)&packet,
      7
    );


  // ==================================================
  // 3. ESP-NOW -> ESP32 #1
  // ==================================================
  esp_err_t result =
    esp_now_send(
      BRIDGE_MAC,
      (const uint8_t*)&packet,
      sizeof(packet)
    );


  // ==================================================
  // 4. Serial Debug
  //
  // Wireless runs at about 50 Hz
  // Serial output is about 5 Hz
  // ==================================================
  debugDivider++;


  if (
    debugDivider
    >=
    10
  )
  {
    debugDivider =
      0;


    Serial.print(
      "SENSOR -> BRIDGE  SEQ="
    );


    Serial.print(
      packet.sequence
    );


    Serial.print(
      " ACC="
    );


    Serial.print(
      packet.accelRaw
    );


    Serial.print(
      " BRAKE="
    );


    Serial.print(
      packet.brakeRaw
    );


    Serial.print(
      " SEND="
    );


    if (
      result
      ==
      ESP_OK
    )
    {
      Serial.println(
        "OK"
      );
    }
    else
    {
      Serial.print(
        "ERROR("
      );


      Serial.print(
        (int)result
      );


      Serial.println(
        ")"
      );
    }
  }
}