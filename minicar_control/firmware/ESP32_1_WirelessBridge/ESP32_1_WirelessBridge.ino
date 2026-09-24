#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>


// ==================================================
// ESP32 #1 WIRELESS HUB
//
// [Sensor direction]
//
// ESP32 #3
//   v ESP-NOW
// ESP32 #1
//   v UART2 TX GPIO17
// Zybo UART0 RX / JF9 / MIO14
//
//
// [Vehicle direction]
//
// Zybo UART0 TX / JF10 / MIO15
//   v
// ESP32 #1 UART2 RX GPIO16
//   v ESP-NOW
// ESP32 #2 Vehicle
//
// UART      = 115200 8N1
// ESP-NOW   = Channel 1
// ==================================================


// ==================================================
// UART2
// ==================================================
HardwareSerial ZyboUART(2);

#define UART_RX 16
#define UART_TX 17


// ==================================================
// ESP32 #2 Vehicle MAC
//
// B0:3F:D3:64:04:14
// ==================================================
const uint8_t VEHICLE_MAC[6] =
{
  0xB0,
  0x3F,
  0xD3,
  0x64,
  0x04,
  0x14
};


// ==================================================
// ESP32 #3 Sensor Node MAC
//
// 38:3E:51:CB:14:40
// ==================================================
const uint8_t SENSOR_MAC[6] =
{
  0x38,
  0x3E,
  0x51,
  0xCB,
  0x14,
  0x40
};


// ==================================================
// Vehicle Command Packet
//
// Zybo
//   -> ESP32 #1
//   -> ESP32 #2
//
// 8 Bytes
//
// Byte 0 : AA
// Byte 1 : 55
// Byte 2 : sequence
// Byte 3 : steering
// Byte 4 : accel
// Byte 5 : brake
// Byte 6 : flags
// Byte 7 : CRC
// ==================================================
struct __attribute__((packed)) CommandPacket
{
  uint8_t sof1;
  uint8_t sof2;

  uint8_t sequence;

  int8_t steering;

  uint8_t accel;
  uint8_t brake;

  uint8_t flags;

  uint8_t crc;
};


static_assert(
  sizeof(CommandPacket) == 8,
  "CommandPacket must be 8 bytes"
);


// ==================================================
// Sensor Packet
//
// ESP32 #3
//   -> ESP32 #1
//   -> Zybo
//
// 8 Bytes
//
// Byte 0 : A5
// Byte 1 : 5A
// Byte 2 : sequence
// Byte 3~4 : accelRaw
// Byte 5~6 : brakeRaw
// Byte 7 : CRC
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
  "SensorPacket must be 8 bytes"
);


// ==================================================
// Sensor Plausibility Limits
//
// ADS1115 at PGA +/-4.096 V gives 32767 counts for 4.096 V.
// The dividers run from 3.3 V, so nothing above ~26400 counts
// is physically reachable and anything higher means a fault.
//
// The Zybo checks this too. Dropping a bad reading here keeps
// it off the UART entirely.
// ==================================================
#define SENSOR_RAW_MAX  26400
#define SENSOR_RAW_MIN  (-1000)


// ==================================================
// CRC-8
//
// Polynomial = 0x07
// Initial = 0x00
// ==================================================
uint8_t calculateCRC8(
  const uint8_t* data,
  size_t length
)
{
  uint8_t crc =
    0x00;


  for (
    size_t i = 0;
    i < length;
    i++
  )
  {
    crc ^=
      data[i];


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
// Vehicle Command Validation
// ==================================================
bool validateCommandPacket(
  const CommandPacket& packet
)
{
  // Header
  if (
    packet.sof1 != 0xAA
    ||
    packet.sof2 != 0x55
  )
  {
    return false;
  }


  // CRC
  uint8_t expectedCRC =
    calculateCRC8(
      (const uint8_t*)&packet,
      7
    );


  if (
    expectedCRC
    !=
    packet.crc
  )
  {
    return false;
  }


  // Steering Range
  if (
    packet.steering < -90
    ||
    packet.steering > 90
  )
  {
    return false;
  }


  // Steering = 10 degree step
  if (
    (
      packet.steering
      %
      10
    )
    !=
    0
  )
  {
    return false;
  }


  // Accel Range
  if (
    packet.accel
    >
    5
  )
  {
    return false;
  }


  // Brake Range
  if (
    packet.brake
    >
    5
  )
  {
    return false;
  }


  return true;
}


// ==================================================
// Sensor Packet Validation
// ==================================================
bool validateSensorPacket(
  const SensorPacket& packet
)
{
  // Header
  if (
    packet.sof1 != 0xA5
    ||
    packet.sof2 != 0x5A
  )
  {
    return false;
  }


  // CRC
  uint8_t expectedCRC =
    calculateCRC8(
      (const uint8_t*)&packet,
      7
    );


  if (
    expectedCRC
    !=
    packet.crc
  )
  {
    return false;
  }


  // Range
  //
  // A CRC only proves the bytes survived the link, not that
  // the reading means anything.
  if (
    packet.accelRaw < SENSOR_RAW_MIN
    ||
    packet.accelRaw > SENSOR_RAW_MAX
    ||
    packet.brakeRaw < SENSOR_RAW_MIN
    ||
    packet.brakeRaw > SENSOR_RAW_MAX
  )
  {
    return false;
  }


  return true;
}


// ==================================================
// ESP32 #3 Sensor Receive Buffer
//
// The ESP-NOW callback and loop() can run on
// different tasks, so the shared packet is guarded
// by a short critical section.
// ==================================================
portMUX_TYPE sensorMux =
  portMUX_INITIALIZER_UNLOCKED;


SensorPacket pendingSensorPacket;


volatile bool newSensorPacketAvailable =
  false;


volatile uint32_t sensorPacketCount =
  0;


volatile uint32_t invalidSensorPacketCount =
  0;


// ==================================================
// ESP-NOW Receive Callback
//
// ESP32 #3
//    v
// ESP32 #1
//
// The callback does not touch the UART; it only
// copies the packet.
// ==================================================
void onDataRecv(
  const esp_now_recv_info_t* info,
  const uint8_t* data,
  int len
)
{
  // ----------------------------------------------
  // NULL check
  // ----------------------------------------------
  if (
    info == nullptr
    ||
    data == nullptr
  )
  {
    return;
  }


  // ----------------------------------------------
  // Accept only packets sent by ESP32 #3.
  // ----------------------------------------------
  if (
    memcmp(
      info->src_addr,
      SENSOR_MAC,
      6
    )
    !=
    0
  )
  {
    return;
  }


  // ----------------------------------------------
  // Packet length
  // ----------------------------------------------
  if (
    len
    !=
    sizeof(SensorPacket)
  )
  {
    invalidSensorPacketCount++;

    return;
  }


  // ----------------------------------------------
  // Local packet copy
  // ----------------------------------------------
  SensorPacket packet;


  memcpy(
    &packet,
    data,
    sizeof(packet)
  );


  // ----------------------------------------------
  // Header / CRC validation
  // ----------------------------------------------
  if (
    !validateSensorPacket(
      packet
    )
  )
  {
    invalidSensorPacketCount++;

    return;
  }


  // ----------------------------------------------
  // Shared Buffer Update
  // ----------------------------------------------
  portENTER_CRITICAL(
    &sensorMux
  );


  pendingSensorPacket =
    packet;


  newSensorPacketAvailable =
    true;


  sensorPacketCount++;


  portEXIT_CRITICAL(
    &sensorMux
  );
}


// ==================================================
// ESP32 #3 Sensor
//       v ESP-NOW
// ESP32 #1
//       v UART TX
// Zybo
//
// GPIO17 -> Zybo JF9 / MIO14
// ==================================================
void processSensorToZybo()
{
  SensorPacket packet;

  bool havePacket =
    false;


  // ----------------------------------------------
  // Receive Buffer Copy
  // ----------------------------------------------
  portENTER_CRITICAL(
    &sensorMux
  );


  if (
    newSensorPacketAvailable
  )
  {
    packet =
      pendingSensorPacket;


    newSensorPacketAvailable =
      false;


    havePacket =
      true;
  }


  portEXIT_CRITICAL(
    &sensorMux
  );


  if (
    !havePacket
  )
  {
    return;
  }


  // ----------------------------------------------
  // Binary SensorPacket -> Zybo
  //
  // ESP32 GPIO17
  //      v
  // Zybo JF9 / MIO14 / UART0 RX
  // ----------------------------------------------
  ZyboUART.write(
    (const uint8_t*)&packet,
    sizeof(packet)
  );


  // ----------------------------------------------
  // Debug
  //
  // About 5 Hz
  // ----------------------------------------------
  static uint32_t debugDivider =
    0;


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
      "SENSOR -> ZYBO  SEQ="
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


    // Rejected packets were counted but never shown, which
    // left no way to see the link degrading.
    Serial.print(
      " RX="
    );


    Serial.print(
      sensorPacketCount
    );


    Serial.print(
      " REJECT="
    );


    Serial.println(
      invalidSensorPacketCount
    );
  }
}


// ==================================================
// Zybo Command UART Parser
//
// Zybo JF10 / MIO15
//      v
// ESP32 GPIO16
//
// Header = AA 55
// ==================================================
uint8_t commandRxBuffer[8];


int commandRxIndex =
  0;


// ==================================================
// Zybo
//   v UART
// ESP32 #1
//   v ESP-NOW
// ESP32 #2
// ==================================================
void processZyboCommand()
{
  while (
    ZyboUART.available()
  )
  {
    uint8_t data =
      ZyboUART.read();


    // ==============================================
    // Byte 0
    // Find 0xAA
    // ==============================================
    if (
      commandRxIndex
      ==
      0
    )
    {
      if (
        data
        ==
        0xAA
      )
      {
        commandRxBuffer[0] =
          data;


        commandRxIndex =
          1;
      }


      continue;
    }


    // ==============================================
    // Byte 1
    // Confirm 0x55
    // ==============================================
    if (
      commandRxIndex
      ==
      1
    )
    {
      if (
        data
        ==
        0x55
      )
      {
        commandRxBuffer[1] =
          data;


        commandRxIndex =
          2;
      }
      else
      {
        // Search for a header again
        commandRxIndex =
          0;


        // If this byte is itself AA it can start
        // a new header
        if (
          data
          ==
          0xAA
        )
        {
          commandRxBuffer[0] =
            data;


          commandRxIndex =
            1;
        }
      }


      continue;
    }


    // ==============================================
    // Remaining Bytes
    // ==============================================
    commandRxBuffer[
      commandRxIndex
    ] =
      data;


    commandRxIndex++;


    // ==============================================
    // Complete 8 Byte CommandPacket
    // ==============================================
    if (
      commandRxIndex
      ==
      8
    )
    {
      CommandPacket packet;


      memcpy(
        &packet,
        commandRxBuffer,
        sizeof(packet)
      );


      // ------------------------------------------
      // Validate
      // ------------------------------------------
      if (
        validateCommandPacket(
          packet
        )
      )
      {
        // ----------------------------------------
        // ESP-NOW -> Vehicle
        // ----------------------------------------
        esp_err_t result =
          esp_now_send(
            VEHICLE_MAC,
            (const uint8_t*)&packet,
            sizeof(packet)
          );


        if (
          result
          ==
          ESP_OK
        )
        {
          // --------------------------------------
          // Debug ~5Hz
          // --------------------------------------
          static uint32_t debugDivider =
            0;


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
              "ZYBO -> VEHICLE  SEQ="
            );


            Serial.print(
              packet.sequence
            );


            Serial.print(
              " S="
            );


            Serial.print(
              packet.steering
            );


            Serial.print(
              " A="
            );


            Serial.print(
              packet.accel
            );


            Serial.print(
              " B="
            );


            Serial.print(
              packet.brake
            );


            Serial.print(
              " FLAG="
            );


            Serial.println(
              packet.flags
            );
          }
        }
        else
        {
          Serial.print(
            "VEHICLE SEND ERROR: "
          );


          Serial.println(
            (int)result
          );
        }
      }
      else
      {
        Serial.println(
          "INVALID ZYBO COMMAND PACKET"
        );
      }


      // ------------------------------------------
      // Next packet
      // ------------------------------------------
      commandRxIndex =
        0;
    }
  }
}


// ==================================================
// ESP-NOW Setup
// ==================================================
bool setupESPNow()
{
  // Wi-Fi Station
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
  // Initialize ESP-NOW
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


  // ==============================================
  // ESP32 #2 Vehicle Peer
  // ==============================================
  esp_now_peer_info_t vehiclePeer = {};


  memcpy(
    vehiclePeer.peer_addr,
    VEHICLE_MAC,
    6
  );


  vehiclePeer.channel =
    1;


  vehiclePeer.encrypt =
    false;


  vehiclePeer.ifidx =
    WIFI_IF_STA;


  // Remove any stale peer registration
  if (
    esp_now_is_peer_exist(
      VEHICLE_MAC
    )
  )
  {
    esp_now_del_peer(
      VEHICLE_MAC
    );
  }


  // Register the vehicle peer
  if (
    esp_now_add_peer(
      &vehiclePeer
    )
    !=
    ESP_OK
  )
  {
    Serial.println(
      "ADD VEHICLE PEER FAILED"
    );


    return false;
  }


  // ==============================================
  // ESP32 #3 Sensor Receive Callback
  // ==============================================
  if (
    esp_now_register_recv_cb(
      onDataRecv
    )
    !=
    ESP_OK
  )
  {
    Serial.println(
      "REGISTER RECEIVE CALLBACK FAILED"
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
  // ==================================================
  // USB Serial Debug
  // COM12
  // ==================================================
  Serial.begin(
    115200
  );


  delay(
    500
  );


  // ==================================================
  // UART2 <-> Zybo
  //
  // GPIO16 RX <- Zybo JF10/MIO15 TX
  // GPIO17 TX -> Zybo JF9/MIO14 RX
  // ==================================================
  ZyboUART.begin(
    115200,
    SERIAL_8N1,
    UART_RX,
    UART_TX
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
  // Boot Information
  // ==================================================
  Serial.println();


  Serial.println(
    "================================"
  );


  Serial.println(
    " ESP32 #1 WIRELESS HUB READY"
  );


  Serial.println(
    "================================"
  );


  Serial.print(
    "ESP32 #1 MAC = "
  );


  Serial.println(
    WiFi.macAddress()
  );


  Serial.println(
    "ESP-NOW Channel = 1"
  );


  Serial.println();


  Serial.println(
    "Sensor Node:"
  );


  Serial.println(
    "  MAC = 38:3E:51:CB:14:40"
  );


  Serial.println(
    "  ESP32 #3 -> ESP32 #1"
  );


  Serial.println();


  Serial.println(
    "Vehicle:"
  );


  Serial.println(
    "  MAC = B0:3F:D3:64:04:14"
  );


  Serial.println(
    "  ESP32 #1 -> ESP32 #2"
  );


  Serial.println();


  Serial.println(
    "UART:"
  );


  Serial.println(
    "  GPIO16 RX <- Zybo JF10/MIO15"
  );


  Serial.println(
    "  GPIO17 TX -> Zybo JF9/MIO14"
  );


  Serial.println();


  Serial.println(
    "Waiting for Sensor + Zybo..."
  );


  Serial.println();
}


// ==================================================
// LOOP
// ==================================================
void loop()
{
  // ==================================================
  // ESP32 #3
  //    v ESP-NOW
  // ESP32 #1
  //    v UART
  // Zybo
  // ==================================================
  processSensorToZybo();


  // ==================================================
  // Zybo
  //    v UART
  // ESP32 #1
  //    v ESP-NOW
  // ESP32 #2
  // ==================================================
  processZyboCommand();
}