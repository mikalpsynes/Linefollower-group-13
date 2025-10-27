# ESP32 Line Follower Robot

This is an ESP32-based line follower robot using PID control and QTR reflectance sensors.

## Hardware Setup

### Components
- **ESP32 Development Board** (ESP32vn IoT Uno)
- **QTR Sensor Array** - 6 reflectance sensors
- **Motor Driver** - Dual motor controller
- **2 DC Motors**

### Pin Configuration

#### QTR Sensors (GPIO pins)
- Sensor 1: GPIO 36
- Sensor 2: GPIO 39
- Sensor 3: GPIO 34
- Sensor 4: GPIO 35
- Sensor 5: GPIO 32
- Sensor 6: GPIO 33
- Emitter Pin: GPIO 2

#### Motor A (Left Motor)
- AIN1: GPIO 13
- AIN2: GPIO 12
- PWMA: GPIO 11

#### Motor B (Right Motor)
- BIN1: GPIO 8
- BIN2: GPIO 9
- PWMB: GPIO 10

## Software Setup

### Prerequisites
1. **PlatformIO** installed (via VSCode extension or CLI)
   - Or use the PlatformIO plugin in CLion

### Building and Uploading

#### Using PlatformIO CLI:
```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor

# Upload and monitor
pio run --target upload && pio device monitor
```

#### Using CLion:
1. Install the PlatformIO plugin for CLion
2. Open this folder as a PlatformIO project
3. Use the build/upload buttons in the toolbar

## Usage

### Calibration Process
1. **Upload the code** to your ESP32
2. **Wait for LED to turn ON** - this signals calibration has started
3. **Move the robot** slowly over the line for about 8 seconds
4. **LED turns OFF** - calibration complete
5. The robot will start following the line automatically

### Serial Monitor
Connect to serial at **9600 baud** to see:
- Calibration progress
- Sensor minimum/maximum values
- Debugging information

## PID Tuning

Current PID parameters (adjust in `main.cpp`):
```cpp
float Kp = 0.22;   // Proportional gain
float Ki = 0.015;  // Integral gain
float Kd = 2.1;    // Derivative gain
```

### Speed Settings
```cpp
const uint8_t maxSpeedA = 250;   // Max speed motor A
const uint8_t maxSpeedB = 250;   // Max speed motor B
const uint8_t baseSpeedA = 225;  // Base speed motor A
const uint8_t baseSpeedB = 225;  // Base speed motor B
```

## Features

- **PID Control** with filtered derivative term
- **Anti-windup** protection for integral term
- **Leaky integrator** to prevent accumulation
- **Deadband** for straight line detection
- **Speed reduction in curves** for better control
- **Slew-rate limiter** for smoother acceleration

## Troubleshooting

### Robot doesn't follow the line
- Check sensor wiring and connections
- Re-calibrate by resetting the ESP32
- Adjust PID parameters
- Verify motor directions

### Compilation errors
- Make sure QTRSensors library is installed: `pio lib install "pololu/QTRSensors@^4.0.0"`
- Clean and rebuild: `pio run --target clean && pio run`

### Upload fails
- Check USB connection
- Verify correct COM port
- Hold BOOT button while uploading (if required by your board)

## Project Structure

```
linefollower/
├── platformio.ini      # PlatformIO configuration
├── src/
│   └── main.cpp       # Main robot code
├── include/           # Header files (if needed)
├── lib/               # Custom libraries
└── test/              # Test files
```

## License

Educational project for NTNU - H2025

