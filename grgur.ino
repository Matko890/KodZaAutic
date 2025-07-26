#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// Pin assignments - GPIOs for XIAO C3
const int forwardPwmPin    = D6;  // GPIO6
const int reversePwmPin    = 0;   // GPIO0
const int forwardEnablePin = D7;  // GPIO7
const int reverseEnablePin = 1;   // GPIO1
const int servoPin         = D4;  // GPIO4 (servo)

// PWM channels (not needed for ESP32 Core 3.x - pins are used directly)
// const int forwardPwmChannel = 0;
// const int reversePwmChannel = 1;

// Motor PWM parameters - Use lower frequency to avoid servo conflicts
const int pwmFreq       = 1000;  // 1 kHz for motors (much lower than servo's 50Hz)
const int pwmResolution = 8;     // 8-bit

// Joystick dead-band thresholds
const int centerLow  = 1700;
const int centerHigh = 2000;

// Servo pulse parameters
const int servoMinPulse = 1000;  // 1ms
const int servoCenterPulse = 1500; // 1.5ms
const int servoMaxPulse = 2000;  // 2ms
const int servoDeadZone = 250;

// Packet format
typedef struct {
  uint16_t joyY;
  uint16_t joyX;
} JoyPacket;

// Latest joystick values
volatile uint16_t latestJoyY = 2048;
volatile uint16_t latestJoyX = 2048;

Servo steeringServo;  // Create Servo object

// ESP-NOW receive callback
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(JoyPacket)) {
    JoyPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    latestJoyY = packet.joyY;
    latestJoyX = packet.joyX;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting XIAO C3 Receiver with proper PWM control...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }
  esp_now_register_recv_cb(onDataRecv);

  // Setup enable pins
  pinMode(forwardEnablePin, OUTPUT);
  pinMode(reverseEnablePin, OUTPUT);
  pinMode(forwardPwmPin, OUTPUT);    // Simple digital output
  pinMode(reversePwmPin, OUTPUT);    // Simple digital output
  digitalWrite(forwardEnablePin, LOW);
  digitalWrite(reverseEnablePin, LOW);
  digitalWrite(forwardPwmPin, LOW);
  digitalWrite(reversePwmPin, LOW);

  // Don't setup PWM for motors - use simple on/off control
  // This completely avoids PWM conflicts with the servo

  // Initialize servo at 50Hz (ESP32Servo handles this automatically)
  steeringServo.setPeriodHertz(50);  // Explicitly set 50Hz
  steeringServo.attach(servoPin, servoMinPulse, servoMaxPulse);
  steeringServo.writeMicroseconds(servoCenterPulse);  // Start at center

  Serial.println("Setup complete - Motors at 1kHz, Servo at 50Hz");
}

void loop() {
  uint16_t joyY = latestJoyY;
  uint16_t joyX = latestJoyX;

  int fwdDuty = 0, revDuty = 0;

  // Motor control - simple on/off (no PWM conflicts)
  if (joyY > centerHigh) {
    digitalWrite(forwardEnablePin, HIGH);
    digitalWrite(reverseEnablePin, LOW);
    digitalWrite(forwardPwmPin, HIGH);  // Full speed forward
    digitalWrite(reversePwmPin, LOW);
  } else if (joyY < centerLow) {
    digitalWrite(forwardEnablePin, LOW);
    digitalWrite(reverseEnablePin, HIGH);
    digitalWrite(forwardPwmPin, LOW);
    digitalWrite(reversePwmPin, HIGH);  // Full speed reverse
  } else {
    digitalWrite(forwardEnablePin, LOW);
    digitalWrite(reverseEnablePin, LOW);
    digitalWrite(forwardPwmPin, LOW);
    digitalWrite(reversePwmPin, LOW);
  }

  // Servo control
  int deviation = joyX - 2048;
  int servoPulse = servoCenterPulse;

  if (abs(deviation) > servoDeadZone) {
    if (deviation < 0) {
      servoPulse = map(joyX, 0, 2048 - servoDeadZone, servoMinPulse, servoCenterPulse);
    } else {
      servoPulse = map(joyX, 2048 + servoDeadZone, 4095, servoCenterPulse, servoMaxPulse);
    }
    servoPulse = constrain(servoPulse, servoMinPulse, servoMaxPulse);
  }

  steeringServo.writeMicroseconds(servoPulse);

  Serial.printf("Y=%4u X=%4u | Fwd=%s Rev=%s | Servo=%4d us\n", 
                joyY, joyX, 
                (digitalRead(forwardPwmPin) ? "ON" : "OFF"),
                (digitalRead(reversePwmPin) ? "ON" : "OFF"),
                servoPulse);
  delay(50);
}