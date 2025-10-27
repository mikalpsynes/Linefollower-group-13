pio run
```

### 3. Connect Your ESP32
- Connect ESP32 to your computer via USB
- Windows should detect it and install drivers

### 4. Upload the Code
```bash
pio run --target upload
```

If you get an error about COM port, you can specify it:
```bash
pio run --target upload --upload-port COM3
```

### 5. Monitor Serial Output
```bash
pio device monitor
```

Or combine upload and monitor:
```bash
pio run -t upload && pio device monitor
```

## Common Commands

| Command | Description |
|---------|-------------|
| `pio run` | Build the project |
| `pio run -t upload` | Upload to ESP32 |
| `pio run -t clean` | Clean build files |
| `pio device monitor` | Open serial monitor |
| `pio device list` | List connected devices |
| `pio lib list` | List installed libraries |

## Workflow

### Normal Development Cycle:
1. Edit code in `src/main.cpp`
2. Build: `pio run`
3. Upload: `pio run -t upload`
4. Test on robot
5. Monitor output: `pio device monitor`

### Calibrating the Robot:
1. Upload code to ESP32
2. Place robot so sensors are over the line
3. When built-in LED turns ON, slowly move robot back and forth over the line
4. After ~8 seconds, LED turns OFF - calibration complete
5. Place robot on track - it will start following the line

### Tuning PID Parameters:
Edit these values in `src/main.cpp`:
```cpp
float Kp = 0.22;   // Try 0.1 to 0.5
float Ki = 0.015;  // Try 0.0 to 0.05
float Kd = 2.1;    // Try 1.0 to 5.0
```

Start with:
1. Set Ki=0, Kd=0, adjust Kp until robot oscillates
2. Add Kd to reduce oscillation
3. Add small Ki to eliminate steady-state error

## Troubleshooting

**Build fails:**
```bash
pio run -t clean
pio run
```

**Can't find COM port:**
```bash
pio device list
```
Then use the correct port:
```bash
pio run -t upload --upload-port COMX
```

**Robot goes the wrong way:**
- Swap motor wires
- Or modify the `rightMotor()` and `leftMotor()` functions

**Sensors don't work:**
- Check wiring matches pin configuration
- Look at calibration values in serial monitor
- Should see different values over black vs white

## Next Steps

- Adjust PID parameters for your track
- Modify base speed for faster/slower operation
- Experiment with sensor configuration
- Add features like line detection, speed control, etc.
# Quick Start Guide

## First Time Setup

### 1. Install PlatformIO
If you haven't already, install PlatformIO:
- **CLion**: Install the PlatformIO plugin
- **VSCode**: Install PlatformIO IDE extension
- **CLI**: `pip install platformio`

### 2. Build the Project
```bash
cd C:\dev\NTNUdev\H2025\Linjefølger\fase2\linefollower

