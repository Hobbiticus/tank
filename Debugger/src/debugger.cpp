#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t TurretMAC[6] = { 0x94, 0xb9, 0x7e, 0xd4, 0xb7, 0x18 };

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent data!" : "Failed to send data");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  char str[256] = {0};
  snprintf(str, sizeof(str), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx :: %s", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (char*)incomingData);
  Serial.println(str);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Failed to init ESP-NOW");
    return;
  }
  Serial.println(WiFi.macAddress());

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, TurretMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK)
  {
    Serial.println("Failed to register receive callback\n");
    return;
  }
  Serial.println("ESP-NOW started");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
}