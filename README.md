# SmartErgoMeter — Hammer Cardio XT5 Smart Trainer

Converts a **Hammer Cardio XT5** ergometer into a smart trainer compatible with fitness apps like **KinoMap**, **Zwift**, **Rouvy** etc. via BLE FTMS (Fitness Machine Service).

## Hardware

### Microcontroller
- **ESP32-C3 SuperMini** (tested ✅)
- **ESP32-S3 Zero** (target platform)

### Ergometer Components Used
The original ergometer PCB (EM78P510 controller) is left **untouched**. The ESP32 add-on board taps into existing signals.

| Component | Description |
|-----------|-------------|
| Reed contact | Speed + cadence sensor on crank (1 pulse = 1 revolution) |
| Heart rate sensor | 5V digital pulse output |
| Potentiometer | Resistance position feedback; PNP transistor on original PCB switches its supply |
| DC motor H-bridge | Original NPN H-bridge on PCB; ESP32 drives transistor bases via 8.2kΩ |
| EM78P510 MCU | Original controller; coexists via OR circuit and high-Z motor pins |

### External Circuit (add-on board)
| Component | Purpose |
|-----------|---------|
| **BAT43** Schottky ×2 | OR-circuit: EM78-OUT1 + EM78-OUT2 → GPIO1 (EM78 activity detect) |
| **33kΩ / 68kΩ resistors** | Voltage divider: poti wiper (max ~5V) → GPIO3 (max 3.37V, safe for ESP32) |
| **10kΩ resistor** | Series resistor: reed contact line (4.7V) → GPIO2 (protects ESP32, no divider to GND) |
| **10kΩ / 18kΩ resistors** | Voltage divider: HR sensor output (5V) → GPIO7 |
| **8.2kΩ resistor ×2** | Motor base resistors: GPIO10 → H-bridge T1 base, GPIO20 → H-bridge T2 base |
| **8.2kΩ + 100kΩ resistors** | EM78_ACTIVE signal: BAT43 output → 8.2kΩ → GPIO1 → 100kΩ → GND |

### Pin Mapping
| GPIO | Signal | Notes |
|------|--------|-------|
| 1 | EM78_ACTIVE | OR: EM78-OUT1 + EM78-OUT2 via 2× BAT43 → 8.2kΩ → GPIO1, 100kΩ pulldown |
| 2 | Reed contact | 4.7V line via 10kΩ series, INPUT_PULLUP, FALLING interrupt |
| 3 | Potentiometer (ADC) | Wiper via 33kΩ/68kΩ divider → max 3.37V |
| 5 | Poti enable (PNP base) | Active LOW: GPIO5 LOW → PNP on → poti powered |
| 7 | Heart rate sensor | 5V via 10kΩ/18kΩ divider, INPUT, RISING interrupt |
| 10 | Motor IN_DOWN | HIGH → reduces resistance; 8.2kΩ to H-bridge transistor base |
| 20 | Motor IN_UP | HIGH → increases resistance; 8.2kΩ to H-bridge transistor base |

### Wiring Diagram
```
ESP32
│
├── GPIO2  ──[10kΩ]──────────── Reed contact line (4.7V from original PCB)
│           (INPUT_PULLUP)      (no divider to GND, EM78P510 stays unloaded)
│
├── GPIO7  ──[10kΩ]──┬───────── HR sensor output (5V)
│                    └─[18kΩ]── GND
│
├── GPIO3  ──────────┬───────── Poti wiper (via original PCB PNP switch)
│          [33kΩ between wiper and junction]
│                    └─[68kΩ]── GND
│
├── GPIO5  ── PNP base (on original PCB, via existing base resistor)
│           LOW = PNP on → poti powered
│           HIGH = PNP off → poti unpowered
│           (EM78P510 also drives same PNP via its own resistor)
│
├── GPIO1  ──[8.2kΩ]──┬─[BAT43]── EM78P510 Motor-OUT1
│          [100kΩ]    └─[BAT43]── EM78P510 Motor-OUT2
│           to GND                 (OR circuit: HIGH = EM78 driving motor)
│
├── GPIO10 ──[8.2kΩ]──────────── H-bridge transistor 1 base (Motor DOWN)
├── GPIO20 ──[8.2kΩ]──────────── H-bridge transistor 2 base (Motor UP)
│           KINOMAP: OUTPUT (ESP32 drives)
│           MANUAL:  INPUT / high-Z (EM78P510 has full control)
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

Device name: `FTMSXXXXXXXX` (4 bytes of BLE MAC address)

### Control Modes
| Mode | Active when | Motor control |
|------|-------------|---------------|
| **KINOMAP** | BLE connected | ESP32 drives motor to target position via potentiometer feedback |
| **MANUAL** | BLE disconnected | EM78P510 buttons control motor (motor pins set to high-Z) |

When BLE connects → ESP32 gets priority (KINOMAP mode)  
When BLE disconnects → EM78P510 buttons take over (MANUAL mode)

Collision protection: if EM78_ACTIVE goes HIGH while ESP32 is driving, motor stops immediately.

### Supported FTMS Opcodes
| OpCode | Function |
|--------|----------|
| 0x00 | Request Control |
| 0x07 | Start / Resume |
| 0x08 | Stop / Pause |
| 0x04 | Set Target Resistance Level (SINT16, resolution 0.1, value 0–1000 = 0–100%) |
| 0x11 | Set Indoor Bike Simulation Parameters (grade → resistance mapping) |

### Calibration

The following values are calibrated for this specific Hammer Cardio XT5 unit:

```cpp
// Effective distance per crank revolution (reed is on crank, 1 pulse = 1 rev)
// Gear ratio is implicitly included — no separate GEAR_RATIO needed
// Calibration: WHEEL_CIRCUMFERENCE_MM = current_value × (display_km_h / app_km_h)
#define WHEEL_CIRCUMFERENCE_MM  6264.0   // calibrated (display=21.6 km/h, app=1.3 km/h → 377×16.6≈6264)

// Potentiometer ADC limits (12-bit: 0–4095)
// Drive motor to each end stop, read ADC from Serial: "Poti: X% (ADC=YYYY)"
#define POTI_ADC_MIN  1246   // ADC at minimum resistance end stop
#define POTI_ADC_MAX  4059   // ADC at maximum resistance end stop

// Grade → resistance mapping (OpCode 0x11)
// -10% to +20% grade → 0–100% resistance
// Adjust range after real route testing
```

## Development Setup

### Requirements
- [PlatformIO](https://platformio.org/) (VS Code extension)
- ESP32-C3 SuperMini or ESP32-S3 Zero board

### Build & Flash
```bash
# Build for ESP32-C3
pio run -e esp32-c3-devkitm-1

# Flash
pio run -e esp32-c3-devkitm-1 --target upload

# Debug build with USB CDC Serial Monitor
pio run -e esp32-c3-debug --target upload
pio device monitor -e esp32-c3-debug

# Build for ESP32-S3
pio run -e esp32-s3-devkitc-1 --target upload
```

### platformio.ini Environments
| Environment | Board | Purpose |
|-------------|-------|---------|
| `esp32-c3-devkitm-1` | ESP32-C3 | Main firmware (no USB CDC, stable BLE) |
| `esp32-c3-debug` | ESP32-C3 | With USB CDC for Serial Monitor |
| `esp32-s3-devkitc-1` | ESP32-S3 | Target platform |

### Testing without Hardware
Enable fake data mode in [src/Smartergometer.ino](src/Smartergometer.ino):
```cpp
#define FAKE_DATA 1
```
Generates synthetic speed, cadence and heart rate data for BLE testing with apps.

### Tested Apps
| App | Platform | Result |
|-----|----------|--------|
| nRF Connect | Android | ✅ Indoor Bike Data visible |
| KinoMap | Android / Fire TV | ✅ Resistance control working |

## Schematics
KiCad schematic files are in the [`schematics/`](schematics/) folder.

## Roadmap
- [ ] BLE Custom Characteristic for calibration (WHEEL_CIRCUMFERENCE, POTI_ADC_MIN/MAX) with NVS persistence
- [ ] Home Assistant integration
- [ ] Grade → resistance curve tuning after real route test

## License
MIT