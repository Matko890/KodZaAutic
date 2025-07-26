#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Joystick pins (both on ADC1, so they still work under Wi-Fi)
const int joyYPin = 32; // vertical axis (forward/reverse)
const int joyXPin = 33; // horizontal axis (left/right turning)

// Packet structure: two 12-bit readings (0–4095)
typedef struct {
  uint16_t joyY;  // Forward/Reverse
  uint16_t joyX;  // Left/Right turning
} JoyPacket;

JoyPacket packet;

// (Replace with your XIAO_C3's MAC address)
uint8_t slaveAddress[] = { 0x40, 0x4C, 0xCA, 0xFA, 0x5D, 0x98 };

void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Optional: monitor send status here
}

// Function to get turning direction and amount
String getTurningInfo(uint16_t rawValue) {
  int center = 2048; // Middle of 12-bit range
  int deadZone = 100; // Dead zone to prevent drift
  
  int deviation = rawValue - center;
  
  if (abs(deviation) < deadZone) {
    return "Straight";
  }
  
  // Calculate percentage (0-100%)
  int percent = map(abs(deviation), deadZone, 2048, 0, 100);
  percent = constrain(percent, 0, 100);
  
  if (deviation < 0) {
    return "Left: " + String(percent) + "%";
  } else {
    return "Right: " + String(percent) + "%";
  }
}

// Function to get forward/reverse info
String getMovementInfo(uint16_t rawValue) {
  int center = 2048; // Middle of 12-bit range
  int deadZone = 100; // Dead zone to prevent drift
  
  int deviation = rawValue - center;
  
  if (abs(deviation) < deadZone) {
    return "Stopped";
  }
  
  // Calculate percentage (0-100%)
  int percent = map(abs(deviation), deadZone, 2048, 0, 100);
  percent = constrain(percent, 0, 100);
  
  if (deviation < 0) {
    return "Reverse: " + String(percent) + "%";
  } else {
    return "Forward: " + String(percent) + "%";
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Print this WROOM's MAC so you can verify in Serial Monitor
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("WROOM MAC: ");
  Serial.println(WiFi.macAddress());

  // Configure ADC attenuation so 0–3.3V → 0–4095
  analogSetPinAttenuation(joyYPin, ADC_11db);
  analogSetPinAttenuation(joyXPin, ADC_11db);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }

  esp_now_register_send_cb(onDataSent);

  // Register XIAO_C3 as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, slaveAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true) {}
  }
}

void loop() {
  // Read vertical (Y) and horizontal (X) axes
  packet.joyY = analogRead(joyYPin);
  packet.joyX = analogRead(joyXPin);

  // Get movement and turning descriptions
  String movement = getMovementInfo(packet.joyY);
  String turning = getTurningInfo(packet.joyX);

  // Print formatted output showing both movement and turning
  Serial.print(movement);
  Serial.print(" | ");
  Serial.print(turning);
  Serial.print(" | Raw Y: ");
  Serial.print(packet.joyY);
  Serial.print(" X: ");
  Serial.println(packet.joyX);

  // Send over ESP-NOW
  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&packet, sizeof(packet));
  if (result != ESP_OK) {
    Serial.println("Send Error");
  }

  delay(50);
}
