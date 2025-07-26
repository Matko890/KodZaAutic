# Energy-Optimized ESP32 RC Car with Lighting Control

This project implements an energy-efficient RC car control system using ESP32-NOW communication between two ESP32 microcontrollers.

## Hardware Setup

### Transmitter (ESP32 WROOM)
- **Joystick Y-axis**: GPIO 32 (ADC1_CH4)
- **Joystick X-axis**: GPIO 33 (ADC1_CH5)
- **Lights Control Button**: GPIO 25 (with internal pull-up, active low)

### Receiver (ESP32 C3 XIAO)
- **Forward PWM**: D6 (GPIO6)
- **Reverse PWM**: GPIO0
- **Forward Enable**: D7 (GPIO7)
- **Reverse Enable**: GPIO1
- **Servo Control**: D4 (GPIO4)
- **Red Brake Lights**: D10 (GPIO10) - PWM controlled
- **Headlights**: D9 (GPIO9) - PWM controlled

## Lighting Circuit

### For both D10 (Red Lights) and D9 (Headlights):
```
ESP32 GPIO → 5kΩ → Base of 2N2222 Transistor
                    ↓ Collector
                    LED+ → 200Ω → LED- → GND
                    ↑
                   +5V/VCC
```

## Energy Saving Features

### Transmitter Optimizations:
1. **Reduced CPU Frequency**: 80MHz instead of 240MHz
2. **Smart Transmission**: Only transmits when significant changes occur
3. **Variable Transmission Rate**: 50ms for active control, 200ms for idle
4. **Light Sleep Mode**: Enabled between operations
5. **WiFi Power Saving**: Minimum modem power consumption

### Receiver Optimizations:
1. **Reduced CPU Frequency**: 80MHz instead of 240MHz
2. **Motor Timeout**: Automatically stops motors after 500ms of no communication
3. **Light Sleep Mode**: Enabled between operations
4. **Efficient Delay**: Reduced loop delay for better responsiveness

## Lighting Control Features

### Brake Lights (D10 - Red):
- **Stopped/Braking**: Maximum intensity (255/255)
- **Moving Forward**: Minimum intensity (50/255)
- **Reversing**: Medium intensity (152/255)
- **Lights Off**: Completely off (0/255)

### Headlights (D9):
- **Lights On**: Constant intensity (200/255)
- **Lights Off**: Completely off (0/255)

## Usage

1. **Power On**: Both devices start in energy-saving mode
2. **Lights Control**: Press button on GPIO 25 (transmitter) to toggle lights
3. **Motor Control**: Use joystick normally - Y-axis for forward/reverse, X-axis for steering
4. **Brake Effect**: Red lights automatically brighten when not accelerating
5. **Safety**: Motors automatically stop if communication is lost for >500ms

## Communication Protocol

The system uses ESP-NOW for low-latency, energy-efficient communication:

```c
typedef struct {
  uint16_t joyY;    // Forward/Reverse (0-4095)
  uint16_t joyX;    // Left/Right turning (0-4095)
  bool lightsOn;    // Lights control state
} JoyPacket;
```

## Power Consumption Improvements

- **CPU Power**: ~60% reduction (240MHz → 80MHz)
- **Transmission Power**: ~75% reduction (smart transmission)
- **Idle Power**: Light sleep mode between operations
- **Safety**: Automatic motor shutdown prevents runaway power consumption

## Transistor Circuit Details

The 2N2222 transistors drive the LEDs with:
- **Base Resistor**: 5kΩ (current limiting for GPIO)
- **LED Resistor**: 200Ω (current limiting for LEDs)
- **PWM Control**: 5kHz frequency for smooth dimming
- **Current Capacity**: Up to ~100mA per channel

## Troubleshooting

1. **Lights not working**: Check transistor connections and resistor values
2. **High power consumption**: Verify CPU frequency is set to 80MHz
3. **Communication issues**: Check MAC address in transmitter code
4. **Motor timeout**: Normal safety feature - check transmitter battery/connection