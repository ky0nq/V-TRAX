#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>

HardwareSerial ZyboUART(2);

#define UART_RX 16
#define UART_TX 17

// ESP32 #2 Vehicle MAC: B0:3F:D3:64:04:14
uint8_t vehicleMAC[6] = {
  0xB0, 0x3F, 0xD3, 0x64, 0x04, 0x14
};

struct __attribute__((packed)) CommandPacket {
  uint8_t sof1;
  uint8_t sof2;
  uint8_t sequence;
  int8_t steering;
  uint8_t accel;
  uint8_t brake;
  uint8_t flags;
  uint8_t crc;
};

static_assert(sizeof(CommandPacket) == 8, "Packet must be 8 bytes");

uint8_t calculateCRC8(const uint8_t* data, size_t length)
{
  uint8_t crc = 0x00;

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }

  return crc;
}

bool validatePacket(const CommandPacket& packet)
{
  if (packet.sof1 != 0xAA || packet.sof2 != 0x55)
    return false;

  uint8_t crc = calculateCRC8((const uint8_t*)&packet, 7);
  if (crc != packet.crc)
    return false;

  if (packet.steering < -90 || packet.steering > 90)
    return false;

  if ((packet.steering % 10) != 0)
    return false;

  if (packet.accel > 5 || packet.brake > 5)
    return false;

  return true;
}

uint8_t rxBuffer[8];
int rxIndex = 0;

void processUART()
{
  while (ZyboUART.available()) {
    uint8_t data = ZyboUART.read();

    // Find 0xAA
    if (rxIndex == 0) {
      if (data == 0xAA) {
        rxBuffer[0] = data;
        rxIndex = 1;
      }
      continue;
    }

    // Confirm 0x55
    if (rxIndex == 1) {
      if (data == 0x55) {
        rxBuffer[1] = data;
        rxIndex = 2;
      } else {
        rxIndex = 0;

        if (data == 0xAA) {
          rxBuffer[0] = data;
          rxIndex = 1;
        }
      }
      continue;
    }

    rxBuffer[rxIndex++] = data;

    if (rxIndex == 8) {
      CommandPacket packet;
      memcpy(&packet, rxBuffer, sizeof(packet));

      if (validatePacket(packet)) {
        esp_err_t result = esp_now_send(
          vehicleMAC,
          (uint8_t*)&packet,
          sizeof(packet)
        );

        if (result == ESP_OK) {
          // Zybo sends at 100 Hz, so every 20th packet is ~5 Hz.
          static int debugDivider = 0;
          debugDivider++;

          if (debugDivider >= 20) {
            debugDivider = 0;

            Serial.print("UART -> ESPNOW  SEQ=");
            Serial.print(packet.sequence);
            Serial.print(" S=");
            Serial.print(packet.steering);
            Serial.print(" A=");
            Serial.print(packet.accel);
            Serial.print(" B=");
            Serial.print(packet.brake);
            Serial.print(" FLAG=");
            Serial.println(packet.flags);
          }
        } else {
          Serial.print("ESP-NOW SEND ERROR: ");
          Serial.println((int)result);
        }
      } else {
        Serial.println("INVALID UART PACKET");
      }

      rxIndex = 0;
    }
  }
}

void setup()
{
  Serial.begin(115200);

  ZyboUART.begin(
    115200,
    SERIAL_8N1,
    UART_RX,
    UART_TX
  );

  WiFi.mode(WIFI_STA);

  esp_err_t chResult = esp_wifi_set_channel(
    1,
    WIFI_SECOND_CHAN_NONE
  );

  if (chResult != ESP_OK) {
    Serial.println("SET CHANNEL FAILED");
    while (true) delay(1000);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, vehicleMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_is_peer_exist(vehicleMAC)) {
    esp_now_del_peer(vehicleMAC);
  }

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ADD PEER FAILED");
    while (true) delay(1000);
  }

  Serial.println();
  Serial.println("========================");
  Serial.println(" ESP32 #1 BRIDGE READY");
  Serial.println("========================");
  Serial.print("Bridge MAC = ");
  Serial.println(WiFi.macAddress());
  Serial.println("UART2 RX = GPIO16 / 115200");
  Serial.println("ESP-NOW Channel = 1");
  Serial.println("Waiting for Zybo packet...");
}

void loop()
{
  processUART();
}
