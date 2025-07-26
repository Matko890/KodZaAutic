#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <esp_pm.h>
#include <esp_wifi.h>

// Pin assignments - GPIOs for XIAO C3
const int forwardPwmPin    = D6;  // GPIO6
const int reversePwmPin    = 0;   // GPIO0
const int forwardEnablePin = D7;  // GPIO7
const int reverseEnablePin = 1;   // GPIO1
const int servoPin         = D4;  // GPIO4 (servo)
const int redLightsPin     = D10; // GPIO10 (red brake lights)
const int headLightsPin    = D9;  // GPIO9 (headlights)

// Note: ESP32-C3 uses simplified PWM API - no channels needed

// Motor PWM parameters - Use lower frequency to avoid servo conflicts
const int pwmFreq       = 1000;  // 1 kHz for motors (much lower than servo's 50Hz)
const int pwmResolution = 8;     // 8-bit
const int lightsPwmFreq = 5000;  // 5 kHz for lights PWM
const int lightsPwmResolution = 8; // 8-bit

// Joystick dead-band thresholds
const int centerLow  = 1700;
const int centerHigh = 2000;

// Servo pulse parameters
const int servoMinPulse = 1000;  // 1ms
const int servoCenterPulse = 1500; // 1.5ms
const int servoMaxPulse = 2000;  // 2ms
const int servoDeadZone = 250;

// Brake lights parameters
const int minBrakeIntensity = 50;  // Minimum brightness when not braking
const int maxBrakeIntensity = 255; // Maximum brightness when braking
const int headLightsIntensity = 200; // Headlights intensity when on

// Packet format
typedef struct {
  uint16_t joyY;
  uint16_t joyX;
  bool lightsOn;  // Lights control
} JoyPacket;

// Latest joystick values
volatile uint16_t latestJoyY = 2048;
volatile uint16_t latestJoyX = 2048;
volatile bool latestLightsOn = false;

// Energy saving variables
unsigned long lastReceiveTime = 0;
const unsigned long timeoutPeriod = 500; // 500ms timeout
bool motorsActive = false;

Servo steeringServo;  // Create Servo object

// Function to configure power management for energy saving
void configurePowerManagement() {
  // Set CPU frequency to 80MHz instead of 160MHz for energy saving
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

// Function to calculate brake light intensity based on throttle
int calculateBrakeIntensity(uint16_t joyY) {
  int center = 2048;
  int deadZone = 100;
  
  // If in dead zone (not moving), show minimum brake lights
  if (abs(joyY - center) < deadZone) {
    return maxBrakeIntensity; // Stronger when stopped (braking effect)
  }
  
  // If moving forward, reduce brake lights
  if (joyY > center + deadZone) {
    // Moving forward - minimal brake lights
    return minBrakeIntensity;
  }
  
  // If reversing, medium brake lights
  return (minBrakeIntensity + maxBrakeIntensity) / 2;
}

// Function to control lights based on current state
void controlLights(uint16_t joyY, bool lightsOn) {
  if (lightsOn) {
    // Headlights on
    ledcWrite(headLightsPin, headLightsIntensity);
    
    // Brake lights with intensity based on throttle
    int brakeIntensity = calculateBrakeIntensity(joyY);
    ledcWrite(redLightsPin, brakeIntensity);
  } else {
    // All lights off
    ledcWrite(headLightsPin, 0);
    ledcWrite(redLightsPin, 0);
  }
}

// Function to safely stop motors (energy saving)
void stopMotors() {
  digitalWrite(forwardEnablePin, LOW);
  digitalWrite(reverseEnablePin, LOW);
  digitalWrite(forwardPwmPin, LOW);
  digitalWrite(reversePwmPin, LOW);
  motorsActive = false;
}

// ESP-NOW receive callback
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(JoyPacket)) {
    JoyPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    latestJoyY = packet.joyY;
    latestJoyX = packet.joyX;
    latestLightsOn = packet.lightsOn;
    lastReceiveTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Energy-Optimized XIAO C3 Receiver with Lights Control...");

  // Configure power management for energy saving
  configurePowerManagement();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }
  esp_now_register_recv_cb(onDataRecv);

  // Setup motor enable pins
  pinMode(forwardEnablePin, OUTPUT);
  pinMode(reverseEnablePin, OUTPUT);
  pinMode(forwardPwmPin, OUTPUT);    // Simple digital output
  pinMode(reversePwmPin, OUTPUT);    // Simple digital output
  
  // Setup lights pins
  pinMode(redLightsPin, OUTPUT);
  pinMode(headLightsPin, OUTPUT);
  
  // Initialize all outputs to LOW
  digitalWrite(forwardEnablePin, LOW);
  digitalWrite(reverseEnablePin, LOW);
  digitalWrite(forwardPwmPin, LOW);
  digitalWrite(reversePwmPin, LOW);

  // Setup PWM for lights control (ESP32-C3 uses newer API)
  ledcAttach(redLightsPin, lightsPwmFreq, lightsPwmResolution);
  ledcAttach(headLightsPin, lightsPwmFreq, lightsPwmResolution);
  
  // Initialize lights to off
  ledcWrite(redLightsPin, 0);
  ledcWrite(headLightsPin, 0);

  // Initialize servo at 50Hz (ESP32Servo handles this automatically)
  steeringServo.setPeriodHertz(50);  // Explicitly set 50Hz
  steeringServo.attach(servoPin, servoMinPulse, servoMaxPulse);
  steeringServo.writeMicroseconds(servoCenterPulse);  // Start at center

  lastReceiveTime = millis();
  Serial.println("Setup complete - Motors at 1kHz, Servo at 50Hz, Lights PWM at 5kHz");
  Serial.print("CPU Frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println("MHz");
}

void loop() {
  unsigned long currentTime = millis();
  uint16_t joyY = latestJoyY;
  uint16_t joyX = latestJoyX;
  bool lightsOn = latestLightsOn;

  // Check for communication timeout (safety feature + energy saving)
  if (currentTime - lastReceiveTime > timeoutPeriod) {
    if (motorsActive) {
      stopMotors();
      Serial.println("Communication timeout - Motors stopped for safety");
    }
    // Keep lights control active even during timeout
    controlLights(2048, lightsOn); // Use center position for brake lights
    delay(100); // Longer delay during timeout to save energy
    return;
  }

  // Motor control - simple on/off (no PWM conflicts)
  if (joyY > centerHigh) {
    digitalWrite(forwardEnablePin, HIGH);
    digitalWrite(reverseEnablePin, LOW);
    digitalWrite(forwardPwmPin, HIGH);  // Full speed forward
    digitalWrite(reversePwmPin, LOW);
    motorsActive = true;
  } else if (joyY < centerLow) {
    digitalWrite(forwardEnablePin, LOW);
    digitalWrite(reverseEnablePin, HIGH);
    digitalWrite(forwardPwmPin, LOW);
    digitalWrite(reversePwmPin, HIGH);  // Full speed reverse
    motorsActive = true;
  } else {
    digitalWrite(forwardEnablePin, LOW);
    digitalWrite(reverseEnablePin, LOW);
    digitalWrite(forwardPwmPin, LOW);
    digitalWrite(reversePwmPin, LOW);
    motorsActive = false;
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

  // Control lights based on current state
  controlLights(joyY, lightsOn);

  // Calculate brake light intensity for display
  int brakeIntensity = calculateBrakeIntensity(joyY);

  Serial.printf("Y=%4u X=%4u | Fwd=%s Rev=%s | Servo=%4d us | Lights=%s | Brake=%3d | Head=%3d\n", 
                joyY, joyX, 
                (digitalRead(forwardPwmPin) ? "ON" : "OFF"),
                (digitalRead(reversePwmPin) ? "ON" : "OFF"),
                servoPulse,
                (lightsOn ? "ON" : "OFF"),
                (lightsOn ? brakeIntensity : 0),
                (lightsOn ? headLightsIntensity : 0));
  
  delay(25); // Reduced delay for better responsiveness
}