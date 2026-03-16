# SmartErgoMeter — Hammer Cardio XT5 Smart Trainer

Converts a **Hammer Cardio XT5** ergometer into a smart trainer compatible with fitness apps like **KinoMap**, **Zwift**, **Rouvy** etc. via BLE FTMS (Fitness Machine Service).

## Hardware

### Microcontroller
- **ESP32-S3 DevKitC-1** (primary, tested ✅)
- **ESP32-C3 DevKitM-1** (also works ✅)

### Ergometer Components Used
| Component | Description |
|-----------|-------------|
| Reed contact | Speed sensor on flywheel (also used for cadence estimation) |
| Heart rate sensor | 5V digital pulse output |
| Potentiometer | Resistance position feedback (only powered when motor runs) |
| DC Motor | 3.3–5V bidirectional, controls resistance |
| EM78P510 MCU | Original ergometer controller |

### External Circuit
| Component | Purpose |
|-----------|---------|
| **DRV8833** | H-Bridge motor driver (replaces direct motor control) |
| **BS170** N-MOSFET | Switches potentiometer 5V supply |
| **BAT46** Schottky ×2 | OR-circuit for potentiometer supply (ESP32 + EM78P510) |
| **10kΩ / 18kΩ resistors** | Voltage dividers 5V → 3.3V (Reed, HR sensor, Potentiometer) |

### Pin Mapping (ESP32-S3)
| GPIO | Signal | Notes |
|------|--------|-------|
| 6 | Reed contact | Speed + Cadence, 5V via 10k/18k divider |
| 7 | Heart rate sensor | 5V via 10k/18k divider |
| 3 | Potentiometer (ADC) | 5V via 10k/18k divider |
| 4 | MOSFET gate (BS170) | Potentiometer 5V enable |
| 8 | EM78P510 Motor-Out A | 3.3V direct |
| 9 | EM78P510 Motor-Out B | 3.3V direct |
| 10 | DRV8833 IN1 | Motor UP |
| 11 | DRV8833 IN2 | Motor DOWN |

### Wiring Diagram
```
ESP32-S3
│
├── GPIO6 ──[10kΩ]──┬── Reed contact ── GND
│                   └──[18kΩ]── GND
│
├── GPIO7 ──[10kΩ]──┬── HR Sensor output (5V)
│                   └──[18kΩ]── GND
│
├── GPIO3 ──[10kΩ]──┬── Potentiometer wiper
│                   └──[18kΩ]── GND
│                               Potentiometer GND ── GND
│
├── GPIO4 ──[10kΩ pull-down]── Gate (BS170)
│                               Drain ──[BAT46]──┬── Potentiometer VCC (5V)
│                               Source ── GND     │
│                                                 │
│                   EM78P510 motor supply ──[BAT46]─┘
│
├── GPIO8 ─────────────────── EM78P510 Motor-Out A (3.3V)
├── GPIO9 ─────────────────── EM78P510 Motor-Out B (3.3V)
│
├── GPIO10 ────────────────── DRV8833 IN1
├── GPIO11 ────────────────── DRV8833 IN2
│                              DRV8833 OUT1 ── Motor+
│                              DRV8833 OUT2 ── Motor−
│                              DRV8833 VCC  ── 5V
│                              DRV8833 GND  ── GND
```

## Software

### BLE FTMS Profile
The device advertises as a **Fitness Machine Service (0x1826)** Indoor Bike with:

| Characteristic | UUID | Properties |
|----------------|------|------------|
| Fitness Machine Feature | 0x2ACC | READ |
| Indoor Bike Data | 0x2AD2 | NOTIFY (1/sec) |
| Training Status | 0x2AD3 | NOTIFY, READ |
| Supported Resistance Level Range | 0x2AD6 | READ |
| Fitness Machine Control Point | 0x2AD9 | WRITE, INDICATE |
| Fitness Machine Status | 0x2ADA | NOTIFY |

Device name: `FTMS_XXXXXX` (last 3 bytes of BLE MAC address)

### Control Modes
| Mode | Active when | Motor control |
|------|-------------|---------------|
| **KINOMAP** | BLE connected | ESP32 drives motor to target position via potentiometer feedback |
| **MANUAL** | BLE disconnected | EM78P510 buttons control motor (passthrough) |

When BLE connects → ESP32 gets priority (KINOMAP mode)  
When BLE disconnects → EM78P510 buttons take over (MANUAL mode)

### Supported FTMS Opcodes
| OpCode | Function |
|--------|----------|
| 0x00 | Request Control |
| 0x07 | Start / Resume |
| 0x08 | Stop / Pause |
| 0x04 | Set Target Resistance Level |
| 0x11 | Set Indoor Bike Simulation Parameters (grade → resistance) |

### Calibration
The following values need to be calibrated per device:

```cpp
// Flywheel circumference: magnet radius × 2π
// Measure radius from axle to magnet center (mm)
// e.g. 60mm radius → 377mm circumference
#define WHEEL_CIRCUMFERENCE_MM  377.0

// Flywheel to crank gear ratio
// Calibrate: count flywheel pulses during 10 full crank revolutions
// GEAR_RATIO = pulse_count / 10
#define GEAR_RATIO  9.0

// Rider weight in kg (affects power calculation)
#define RIDER_WEIGHT_KG  80.0
```

## Development Setup

### Requirements
- [PlatformIO](https://platformio.org/) (VS Code extension)
- ESP32-S3 or ESP32-C3 DevKit board

### Build & Flash
```bash
# Build main firmware
pio run -e esp32-s3-devkitc-1

# Flash
pio run -e esp32-s3-devkitc-1 --target upload

# Serial monitor
pio device monitor -e esp32-s3-devkitc-1

# Build blink test (hardware check)
pio run -e blink-test --target upload
```

### platformio.ini Environments
| Environment | Board | Purpose |
|-------------|-------|---------|
| `esp32-s3-devkitc-1` | ESP32-S3 | Main firmware |
| `esp32-c3-devkitm-1` | ESP32-C3 | Alternative board |
| `esp32-c3-debug` | ESP32-C3 | With USB CDC for Serial Monitor |
| `blink-test` | ESP32-C3 | Hardware sanity check (RGB LED blink) |

### Testing without Hardware
Enable fake data mode in [src/Smartergoeter.ino](src/Smartergoeter.ino):
```cpp
#define FAKE_DATA 1
```
Generates synthetic speed, cadence and heart rate data for BLE testing with apps.

### Tested Apps
| App | Platform | Result |
|-----|----------|--------|
| nRF Connect | Android | ✅ Indoor Bike Data visible |
| KinoMap | Android/iOS | ✅ Connects as Indoor Bike |
| Wahoo Fitness | Android/iOS | ✅ Connects as Indoor Bike |
| Zwift | Android/iOS | ✅ FTMS supported |

## Roadmap
- [ ] BLE Custom Characteristic for calibration (WHEEL_CIRCUMFERENCE, GEAR_RATIO, RIDER_WEIGHT) with NVS persistence
- [ ] Home Assistant integration
- [ ] Real sensor integration (Reed, HR, Potentiometer)
- [ ] Potentiometer MIN/MAX ADC calibration
- [ ] GEAR_RATIO auto-calibration mode

## License
MIT
