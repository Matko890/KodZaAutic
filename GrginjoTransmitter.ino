#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_pm.h>

// Joystick pins (both on ADC1, so they still work under Wi-Fi)
const int joyYPin = 32; // vertical axis (forward/reverse)
const int joyXPin = 33; // horizontal axis (left/right turning)
const int lightsPin = 25; // GPIO 25 for lights control

// Energy saving variables
unsigned long lastTransmissionTime = 0;
const unsigned long transmissionInterval = 50; // Normal interval
const unsigned long slowTransmissionInterval = 200; // Slow interval when no changes
uint16_t lastJoyY = 2048;
uint16_t lastJoyX = 2048;
bool lastLightsState = false;
const int significantChange = 50; // Threshold for significant joystick change

// Packet structure: two 12-bit readings (0–4095) + lights control
typedef struct {
  uint16_t joyY;  // Forward/Reverse
  uint16_t joyX;  // Left/Right turning
  bool lightsOn;  // Lights control
} JoyPacket;

JoyPacket packet;

// (Replace with your XIAO_C3's MAC address)
uint8_t slaveAddress[] = { 0x40, 0x4C, 0xCA, 0xFA, 0x5D, 0x98 };

void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Optional: monitor send status here
}

// Function to configure power management for energy saving
void configurePowerManagement() {
  // Set CPU frequency to 80MHz instead of 240MHz for energy saving
  setCpuFrequencyMhz(80);
  
  // Configure power management
  esp_pm_config_t pm_config;
  pm_config.max_freq_mhz = 80;
  pm_config.min_freq_mhz = 10;
  pm_config.light_sleep_enable = true;
  esp_pm_configure(&pm_config);
  
  // Reduce WiFi power consumption
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

// Function to check if transmission is needed
bool shouldTransmit(uint16_t currentY, uint16_t currentX, bool currentLights) {
  // Always transmit if lights state changed
  if (currentLights != lastLightsState) {
    return true;
  }
  
  // Check for significant joystick changes
  if (abs(currentY - lastJoyY) > significantChange || 
      abs(currentX - lastJoyX) > significantChange) {
    return true;
  }
  
  // If no significant changes, use slower transmission rate
  return false;
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

  // Configure lights control pin
  pinMode(lightsPin, INPUT_PULLUP);

  // Configure power management for energy saving
  configurePowerManagement();

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

  Serial.println("Energy-optimized transmitter ready");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read vertical (Y) and horizontal (X) axes
  uint16_t currentJoyY = analogRead(joyYPin);
  uint16_t currentJoyX = analogRead(joyXPin);
  bool currentLights = !digitalRead(lightsPin); // Active low (button pressed = lights on)

  // Check if we should transmit
  bool needTransmit = shouldTransmit(currentJoyY, currentJoyX, currentLights);
  
  // Determine transmission interval
  unsigned long interval = needTransmit ? transmissionInterval : slowTransmissionInterval;
  
  if (currentTime - lastTransmissionTime >= interval) {
    // Update packet
    packet.joyY = currentJoyY;
    packet.joyX = currentJoyX;
    packet.lightsOn = currentLights;

    // Get movement and turning descriptions
    String movement = getMovementInfo(packet.joyY);
    String turning = getTurningInfo(packet.joyX);

    // Print formatted output showing both movement and turning
    Serial.print(movement);
    Serial.print(" | ");
    Serial.print(turning);
    Serial.print(" | Lights: ");
    Serial.print(packet.lightsOn ? "ON" : "OFF");
    Serial.print(" | Raw Y: ");
    Serial.print(packet.joyY);
    Serial.print(" X: ");
    Serial.print(packet.joyX);
    Serial.print(" | CPU: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println("MHz");

    // Send over ESP-NOW
    esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)&packet, sizeof(packet));
    if (result != ESP_OK) {
      Serial.println("Send Error");
    }

    // Update last values
    lastJoyY = currentJoyY;
    lastJoyX = currentJoyX;
    lastLightsState = currentLights;
    lastTransmissionTime = currentTime;
  }

  // Short delay to allow light sleep between operations
  delay(10);
}
