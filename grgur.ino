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

// Motor PWM parameters - Use lower frequency to avoid servo conflicts
const int pwmFreq       = 1000;  // 1 kHz for motors (much lower than servo's 50Hz)
const int pwmResolution = 8;     // 8-bit
// No PWM for lights to avoid servo conflicts - using simple on/off control

// Joystick dead-band thresholds
const int centerLow  = 1700;
const int centerHigh = 2000;

// Servo pulse parameters
const int servoMinPulse = 1000;  // 1ms
const int servoCenterPulse = 1500; // 1.5ms
const int servoMaxPulse = 2000;  // 2ms
const int servoDeadZone = 100;

// Lights control - simple on/off to avoid servo conflicts
// Using digital pins to control transistors directly

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

// Function to determine brake light state based on throttle
bool shouldBrakeLightsBeOn(uint16_t joyY) {
  int center = 2048;
  int deadZone = 100;
  
  // Brake lights ON when not accelerating (stopped or reversing)
  if (joyY <= center + deadZone) {
    return true;  // Braking effect when stopped or reversing
  }
  
  return false;  // Off when moving forward
}

// Function to control lights based on current state
void controlLights(uint16_t joyY, bool lightsOn) {
  if (lightsOn) {
    // Headlights on (constant)
    digitalWrite(headLightsPin, HIGH);
    
    // Brake lights based on throttle (braking effect)
    bool brakeState = shouldBrakeLightsBeOn(joyY);
    digitalWrite(redLightsPin, brakeState ? HIGH : LOW);
    
    // Debug lights output
    Serial.print("Lights ON - Head: ON, Brake: ");
    Serial.println(brakeState ? "ON" : "OFF");
  } else {
    // All lights off
    digitalWrite(headLightsPin, LOW);
    digitalWrite(redLightsPin, LOW);
    Serial.println("Lights OFF");
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

  // Initialize lights to off (simple digital control)
  digitalWrite(redLightsPin, LOW);
  digitalWrite(headLightsPin, LOW);

  // Initialize servo at 50Hz (ESP32Servo handles this automatically)
  steeringServo.setPeriodHertz(50);  // Explicitly set 50Hz
  steeringServo.attach(servoPin, servoMinPulse, servoMaxPulse);
  steeringServo.writeMicroseconds(servoCenterPulse);  // Start at center

  lastReceiveTime = millis();
  Serial.println("Setup complete - Motors simple on/off, Servo at 50Hz, Lights digital control");
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

  // Servo control - Fixed logic
  int deviation = joyX - 2048;
  int servoPulse = servoCenterPulse;

  if (abs(deviation) > servoDeadZone) {
    if (deviation < 0) {
      // Left turn
      servoPulse = map(joyX, 0, 2048 - servoDeadZone, servoMinPulse, servoCenterPulse);
    } else {
      // Right turn
      servoPulse = map(joyX, 2048 + servoDeadZone, 4095, servoCenterPulse, servoMaxPulse);
    }
    servoPulse = constrain(servoPulse, servoMinPulse, servoMaxPulse);
  }

  steeringServo.writeMicroseconds(servoPulse);

  // Control lights based on current state
  controlLights(joyY, lightsOn);

  // Get brake light state for display
  bool brakeState = shouldBrakeLightsBeOn(joyY);

  Serial.printf("Y=%4u X=%4u | Fwd=%s Rev=%s | Servo=%4d us | Lights=%s | Brake=%s | Head=%s\n", 
                joyY, joyX, 
                (digitalRead(forwardPwmPin) ? "ON" : "OFF"),
                (digitalRead(reversePwmPin) ? "ON" : "OFF"),
                servoPulse,
                (lightsOn ? "ON" : "OFF"),
                (lightsOn && brakeState ? "ON" : "OFF"),
                (lightsOn ? "ON" : "OFF"));
  
  delay(25); // Reduced delay for better responsiveness
}