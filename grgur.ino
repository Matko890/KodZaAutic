#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
// Don't include ESP32Servo - it doesn't work properly with ESP32-C3
#include <esp_pm.h>
#include <esp_wifi.h>

// Pin assignments - GPIOs for XIAO C3
const int forwardPwmPin    = D6;  // GPIO6
const int reversePwmPin    = 0;   // GPIO0
const int forwardEnablePin = D7;  // GPIO7
const int reverseEnablePin = 1;   // GPIO1
const int servoPin         = D3;  // GPIO4 (servo)
const int redLightsPin     = D10; // GPIO10 (red brake lights)
const int headLightsPin    = D9;  // GPIO9 (headlights)

// PWM parameters - ESP32-C3 specific settings
const int pwmFreq       = 1000;  // 1 kHz for motors
const int pwmResolution = 8;     // 8-bit
const int lightsPwmFreq = 1000;  // 1 kHz for lights PWM
const int lightsPwmResolution = 8; // 8-bit

// Servo PWM parameters - ESP32-C3 LEDC manual control
const int servoPwmFreq = 50;     // 50Hz for servo (separate from other PWM)
const int servoPwmResolution = 16; // 16-bit for better servo precision

// Joystick dead-band thresholds
const int centerLow  = 1700;
const int centerHigh = 2000;

// Servo pulse parameters - Adjusted for better compatibility
const int servoMinPulse = 500;   // 0.5ms - wider range
const int servoCenterPulse = 1500; // 1.5ms
const int servoMaxPulse = 2500;  // 2.5ms - wider range
const int servoDeadZone = 100;

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
volatile bool latestLightsButton = false;

// Lights toggle state management
bool previousLightsButton = false;
bool lightsState = false; // Current lights on/off state

// Energy saving variables
unsigned long lastReceiveTime = 0;
const unsigned long timeoutPeriod = 500; // 500ms timeout
bool motorsActive = false;

// Remove Servo object - use manual LEDC control instead

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
  
  // If in dead zone (not moving), show maximum brake lights (braking effect)
  if (abs(joyY - center) < deadZone) {
    return maxBrakeIntensity;
  }
  
  // If moving forward, reduce brake lights
  if (joyY > center + deadZone) {
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
    
    // Debug lights output
    Serial.print("Lights ON - Head: ");
    Serial.print(headLightsIntensity);
    Serial.print(" Brake: ");
    Serial.println(brakeIntensity);
  } else {
    // All lights off
    ledcWrite(headLightsPin, 0);
    ledcWrite(redLightsPin, 0);
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
    latestLightsButton = packet.lightsOn;
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

  // Initialize servo - ESP32-C3 MANUAL LEDC METHOD (the only way that works!)
  Serial.println("Initializing servo with manual LEDC control...");
  
  // Setup servo PWM channel manually using LEDC
  ledcAttach(servoPin, servoPwmFreq, servoPwmResolution);
  
  // Calculate duty cycle for center position (1500us pulse at 50Hz)
  // At 50Hz: period = 20ms = 20000us
  // For 1500us pulse: duty = (1500/20000) * (2^16-1) = 0.075 * 65535 = 4915
  int centerDuty = 4915; // 1500us pulse width
  ledcWrite(servoPin, centerDuty);
  
  delay(500); // Give servo time to reach center position
  
  Serial.println("Servo initialized with manual LEDC at 50Hz, 16-bit resolution");

  lastReceiveTime = millis();
  Serial.println("Setup complete - Motors/Lights at 1kHz, Servo at 50Hz with manual LEDC");
  Serial.print("CPU Frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println("MHz");
}

void loop() {
  unsigned long currentTime = millis();
  uint16_t joyY = latestJoyY;
  uint16_t joyX = latestJoyX;
  bool lightsButton = latestLightsButton;

  // Handle lights toggle (button press detection)
  if (lightsButton && !previousLightsButton) {
    // Button was just pressed (rising edge)
    lightsState = !lightsState; // Toggle lights state
    Serial.print("Lights toggled: ");
    Serial.println(lightsState ? "ON" : "OFF");
  }
  previousLightsButton = lightsButton; // Remember button state for next loop

  // Check for communication timeout (safety feature + energy saving)
  if (currentTime - lastReceiveTime > timeoutPeriod) {
    if (motorsActive) {
      stopMotors();
      Serial.println("Communication timeout - Motors stopped for safety");
    }
    // Keep lights control active even during timeout
    controlLights(2048, lightsState); // Use center position for brake lights
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

  // Servo control - Manual LEDC with proper pulse width calculation
  int deviation = (int)joyX - 2048;
  int servoDuty = 4915; // Center position (1500us)

  if (abs(deviation) > servoDeadZone) {
    int pulseWidth; // in microseconds
    
    if (deviation < 0) {
      // Left turn - map to 1000us (min pulse)
      pulseWidth = map(joyX, 0, 2048 - servoDeadZone, 1000, 1500);
    } else {
      // Right turn - map to 2000us (max pulse)  
      pulseWidth = map(joyX, 2048 + servoDeadZone, 4095, 1500, 2000);
    }
    
    // Convert pulse width to duty cycle for 50Hz, 16-bit
    // duty = (pulseWidth / 20000) * 65535
    servoDuty = (pulseWidth * 65535) / 20000;
    servoDuty = constrain(servoDuty, 3277, 6553); // 1000us to 2000us range
  }

  // Send PWM signal to servo
  ledcWrite(servoPin, servoDuty);

  // Control lights based on current toggle state
  controlLights(joyY, lightsState);

  // Calculate brake light intensity for display
  int brakeIntensity = calculateBrakeIntensity(joyY);

  // Enhanced debug output with servo duty cycle and pulse width
  int currentPulseWidth = (servoDuty * 20000) / 65535; // Convert back to microseconds for display
  Serial.printf("Y=%4u X=%4u | Fwd=%s Rev=%s | Servo=%4dus (%5d duty) | Lights=%s | Brake=%3d | Head=%3d\n", 
                joyY, joyX, 
                (digitalRead(forwardPwmPin) ? "ON" : "OFF"),
                (digitalRead(reversePwmPin) ? "ON" : "OFF"),
                currentPulseWidth, servoDuty,
                (lightsState ? "ON" : "OFF"),
                (lightsState ? brakeIntensity : 0),
                (lightsState ? headLightsIntensity : 0));
  
  delay(25); // Reduced delay for better responsiveness
}
