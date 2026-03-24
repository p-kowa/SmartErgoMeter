#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "esp_mac.h"

// ============================================================
// CONFIGURATION
// ============================================================
#define FAKE_DATA 0                    // 1 = test data, 0 = real sensors

boolean serial_debug         = true;  // false when no PC connected
boolean write_startup_message = true;

// Effective distance per crank revolution (reed is on the crank, 1 pulse = 1 revolution)
// Implicitly includes the ergometer gear ratio → no separate GEAR_RATIO needed
// Calibration: WHEEL_CIRCUMFERENCE_MM = current_value × (display_km_h / app_km_h)
// Measured: display=21.6 km/h, app=1.3 km/h → 377 × (21.6/1.3) ≈ 6264 mm
#define WHEEL_CIRCUMFERENCE_MM  6264.0

// Rider weight in kg (used for power calculation)
#define RIDER_WEIGHT_KG  80.0

// Potentiometer ADC limits (0-4095)
// Calibration: drive motor manually to each end stop, read raw ADC from Serial Monitor
// Serial output: "Poti: X% (ADC=YYYY)" → note YYYY at minimum and maximum end stop
#define POTI_ADC_MIN     1246  // ADC value at minimum resistance (measured: 1245-1247)
#define POTI_ADC_MAX     4059  // ADC value at maximum resistance (measured: 4058-4060)

// ============================================================
// PIN DEFINITIONS (ESP32-S3)
// ============================================================
#define PIN_REED        2    // Reed contact → speed + cadence
                             // Hardware: reed line (4.7V) ──10kΩ──GPIO2
                             // 10kΩ series resistor protects ESP32 (max 3.6V)
                             // No divider to GND → EM78P510 remains unloaded
                             // GPIO6=JTAG/MOSI on ESP32-C3 → spurious interrupts, use GPIO2 instead
#define PIN_HEARTRATE   7    // Heart rate sensor (5V via 10k/18k divider) - GPIO7 works
#define PIN_POTI        3    // ADC potentiometer wiper
                             // Hardware: poti wiper ──32kΩ──GPIO3──66kΩ──GND (voltage divider)
                             // Poti VCC ~5V → divider: 5V × 66/(32+66) = 3.34V → safe for ESP32 (max 3.6V)
#define PIN_POTI_EN     5    // PNP transistor base control (active LOW!)
                             // GPIO5 LOW  → PNP conducts → poti powered (measurement)
                             // GPIO5 HIGH → PNP off → poti unpowered (idle)
                             // EM78P510 also drives PNP via 10kΩ → both share the transistor
                             // Contention non-critical: 10kΩ + 12kΩ in series (~0.2mA)
#define PIN_EM78_ACTIVE 1    // EM78P510 active (OR: motor-OUT1 + motor-OUT2 via 2× BAT43 → GPIO1)
                             // GPIO8 = RGB LED on C3 Super Mini → do not use!
                             // Hardware: EM78-OUT1 ──BAT43──┐
                             //           EM78-OUT2 ──BAT43──┴──8.2kΩ──GPIO1──100kΩ──GND
                             // 5V - 0.35V(BAT43) = 4.65V → clamp diode: 0.16mA → safe
                             // HIGH = EM78P510 driving motor → ESP32 must stop immediately
#define PIN_MOTOR_IN_DOWN   10   // H-bridge transistor 1 base (ESP32 via 12kΩ, EM78P510 via 10kΩ)
#define PIN_MOTOR_IN_UP     20   // H-bridge transistor 2 base (ESP32 via 12kΩ, EM78P510 via 10kΩ)
                             // KINOMAP: OUTPUT (ESP32 drives HIGH/LOW)
                             // MANUAL:  INPUT  (high-Z → EM78P510 has full control)

// ============================================================
// MOTOR CONTROL
// ============================================================
enum ControlMode { KINOMAP, MANUAL };
ControlMode motorMode        = MANUAL; // On startup without BLE: manual
int targetResistance         = 0;      // 0-100% target resistance from KinoMap
int currentResistance        = 0;      // Current position (from poti)
unsigned long lastMotorCmd   = 0;
#define MOTOR_TIMEOUT        5000      // ms without KinoMap command → switch back to MANUAL
#define POTI_TOLERANCE       80        // ADC tolerance band (0-4095)
#define MOTOR_DRIVE_INTERVAL 50        // ms between poti readings while driving

void motorDown() {
  pinMode(PIN_MOTOR_IN_DOWN, OUTPUT);
  pinMode(PIN_MOTOR_IN_UP, OUTPUT);
  digitalWrite(PIN_MOTOR_IN_DOWN, HIGH);
  digitalWrite(PIN_MOTOR_IN_UP, LOW);
}

void motorUp() {
  pinMode(PIN_MOTOR_IN_DOWN, OUTPUT);
  pinMode(PIN_MOTOR_IN_UP, OUTPUT);
  digitalWrite(PIN_MOTOR_IN_DOWN, LOW);
  digitalWrite(PIN_MOTOR_IN_UP, HIGH);
}

void motorStop() {
  digitalWrite(PIN_MOTOR_IN_DOWN, LOW);
  digitalWrite(PIN_MOTOR_IN_UP, LOW);
  pinMode(PIN_MOTOR_IN_DOWN, INPUT);  // high-Z → EM78P510 can control freely
  pinMode(PIN_MOTOR_IN_UP, INPUT);
}

// Briefly power poti and read ADC
// PNP logic: LOW = ON, HIGH = OFF
int readPotiPosition() {
  digitalWrite(PIN_POTI_EN, LOW);   // turn PNP on → power poti
  delay(5);
  int val = analogRead(PIN_POTI);
  digitalWrite(PIN_POTI_EN, HIGH);  // turn PNP off → unpower poti
  return val; // 0-4095
}

// Drive motor to target position (blocking, max 5 sec)
// Aborts immediately if EM78P510 presses a button (collision protection)
void driveToPosition(int targetPct) {
  int targetAdc  = map(targetPct, 0, 100, POTI_ADC_MIN, POTI_ADC_MAX);
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {

    // Collision protection: EM78P510 active → stop immediately
    if (digitalRead(PIN_EM78_ACTIVE)) {
      motorStop();
      if (serial_debug) Serial.println("Motor: EM78P510 active, motor stopped!");
      return;
    }

    int currentAdc = readPotiPosition();
    currentResistance = map(constrain(currentAdc, POTI_ADC_MIN, POTI_ADC_MAX), POTI_ADC_MIN, POTI_ADC_MAX, 0, 100);

    if (abs(currentAdc - targetAdc) <= POTI_TOLERANCE) {
      motorStop();
      if (serial_debug) {
        Serial.printf("Motor: target reached %d%% (ADC=%d)\n", targetPct, currentAdc);
      }
      return;
    }

    if (currentAdc < targetAdc) motorUp();
    else                        motorDown();

    delay(MOTOR_DRIVE_INTERVAL);
  }

  motorStop(); // timeout
  if (serial_debug) Serial.println("Motor: timeout!");
}

// MANUAL mode: EM78P510 drives motor directly via its own H-bridge
// Motor pins kept high-Z so there is no conflict
void handleManualMode() {
  motorStop(); // keep pins high-Z
}

// ============================================================
// SENSORS
// ============================================================
volatile unsigned long speed_counter          = 0;
volatile unsigned long speed_timer            = 0;
unsigned long          speed_counter_prev     = 0;
unsigned long          speed_timer_prev       = 0;
double                 instantaneous_speed    = 0; // m/s

volatile unsigned long cadence_timer          = 0;
volatile unsigned long cadence_previous_timer = 0;
unsigned int           instantaneous_cadence  = 0; // RPM

volatile unsigned long hr_timer               = 0;
volatile unsigned long hr_previous_timer      = 0;
uint8_t                HeartRate              = 0;

double instantaneous_power = 0;
float  grade               = 0;
float  wind_speed          = 0;
float  crr                 = 0;
float  cw                  = 0;

// Reed contact interrupt → speed + cadence
// Debounce 50ms: prevents multiple triggers from contact bounce
void IRAM_ATTR reedInterrupt() {
  unsigned long now = millis();
  if (now - speed_timer < 50) return;  // debounce
  speed_counter++;
  speed_timer            = now;
  cadence_previous_timer = cadence_timer;
  cadence_timer          = now;
}

// Heart rate interrupt
void IRAM_ATTR heartRateInterrupt() {
  hr_previous_timer = hr_timer;
  hr_timer          = millis();
}

double calculateSpeed() {
  // Not yet two valid measurements available
  if (speed_timer == 0 || speed_timer_prev == 0) return 0.0;
  // No signal for >3 sec → speed = 0
  if (millis() - speed_timer > 3000) return 0.0;
  if (speed_timer == speed_timer_prev) return 0.0;
  unsigned long delta_count = speed_counter - speed_counter_prev;
  unsigned long delta_time  = speed_timer   - speed_timer_prev;
  if (delta_count == 0 || delta_time == 0 || delta_time > 5000) return 0.0;
  return (delta_count * WHEEL_CIRCUMFERENCE_MM / 1000.0) / (delta_time / 1000.0); // m/s
}

unsigned int calculateCadence() {
  // Reed on crank → 1 pulse = 1 crank revolution → direct RPM
  unsigned long interval = cadence_timer - cadence_previous_timer;
  if (interval < 200 || interval > 5000) return 0;
  return (unsigned int)round(60000.0 / interval);
}

uint8_t calculateHeartRate() {
  // No signal for >3 sec → HR = 0 (hands off sensor)
  if (millis() - hr_timer > 3000) return 0;
  unsigned long interval = hr_timer - hr_previous_timer;
  if (interval < 300 || interval > 3000) return 0;
  return (uint8_t)(60000 / interval);
}

double calculatePower(double speed_ms) {
  if (speed_ms < 0.1) return 0;
  const int   ELEVATION = 40;
  double rho  = 1.225 * exp(-0.00011856 * ELEVATION);
  float  cdA  = 0.324;
  double w    = RIDER_WEIGHT_KG;
  return speed_ms * (
    (w * 9.80655 * sin(atan(grade / 100.0))) +
    (crr * w * 9.80655 * cos(atan(grade / 100.0))) +
    (0.5 * cdA * rho * pow(speed_ms + wind_speed, 2))
  );
}

// ============================================================
// BLE FTMS
// ============================================================
#define FMCP_DATA_SIZE 19

typedef struct __attribute__((packed)) {
  uint8_t OPCODE;
  uint8_t OCTETS[FMCP_DATA_SIZE - 1];
} fmcp_data_t;

typedef union {
  fmcp_data_t values;
  uint8_t     bytes[FMCP_DATA_SIZE];
} fmcp_data_ut;

fmcp_data_ut fmcpData;
short        fmcpValueLength          = 0;
volatile long lastControlPointEvent   = 0;
long          previousControlPointEvent = 0;

// Feature flags
unsigned char ftmfBuffer[4] = { 0b10000111, 0b01000100, 0, 0 };
unsigned char ibdBuffer[9]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char srlrBuffer[4] = { 0, 100, 0, 1 };  // Min=0, Max=100, Step=1
unsigned char ftmsBuffer[2] = { 0, 0 };
unsigned char tsBuffer[2]   = { 0x00, 0x00 };
unsigned char ftmcpBuffer[20];

// Indoor Bike Data Flags
const uint16_t flagInstantaneousCadence = 4;
const uint16_t flagIntantaneousPower    = 64;
const uint16_t flagHeartRate            = 512;

// FMCP Opcodes
const uint8_t fmcpRequestControl                       = 0x00;
const uint8_t fmcpReset                                = 0x01;
const uint8_t fmcpSetTargetSpeed                       = 0x02;
const uint8_t fmcpSetTargetInclination                 = 0x03;
const uint8_t fmcpSetTargetResistanceLevel             = 0x04;
const uint8_t fmcpSetTargetPower                       = 0x05;
const uint8_t fmcpSetTargetHeartRate                   = 0x06;
const uint8_t fmcpStartOrResume                        = 0x07;
const uint8_t fmcpStopOrPause                          = 0x08;
const uint8_t fmcpSetIndoorBikeSimulationParameters    = 0x11;
const uint8_t fmcpResponseCode                         = 0x80;

int  ble_connected = LOW;

BLEServer         *pServer                        = nullptr;
BLECharacteristic *pFitnessMachineFeature         = nullptr;
BLECharacteristic *pIndoorBikeData                = nullptr;
BLECharacteristic *pTrainingStatus                = nullptr;
BLECharacteristic *pSupportedResistanceLevelRange = nullptr;
BLECharacteristic *pFitnessMachineControlPoint    = nullptr;
BLECharacteristic *pFitnessMachineStatus          = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pSrv) override {
    ble_connected = HIGH;
    motorMode     = KINOMAP; // BLE connected → ESP32 takes control
    rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // blue
    if (serial_debug) Serial.println("BLE connected → KINOMAP mode");
  }
  void onDisconnect(BLEServer *pSrv) override {
    ble_connected = LOW;
    motorMode     = MANUAL;  // BLE disconnected → EM78P510 buttons active
    motorStop();
    rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0); // yellow
    BLEDevice::getAdvertising()->start();
    if (serial_debug) Serial.println("BLE disconnected → MANUAL mode");
  }
};

class ControlPointCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    fmcpValueLength = pChar->getLength();
    memset(fmcpData.bytes, 0, sizeof(fmcpData.bytes));
    memcpy(fmcpData.bytes, pChar->getData(), fmcpValueLength);
    lastControlPointEvent = millis();
  }
};

// Training Status
enum status_t { STOPPED, RUNNING, PAUSED };
unsigned short training_status = STOPPED;

const short   NOTIFICATION_INTERVAL = 1000;
long          previous_notification  = 0;
long          current_millis         = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  if (serial_debug) {
    Serial.begin(115200);
    delay(1000);
    Serial.println("SmartErgoMeter starting...");
  }

  rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0); // yellow = startup

  // Motor pins: high-Z on startup (MANUAL mode, EM78P510 has priority)
  pinMode(PIN_MOTOR_IN_DOWN, INPUT);
  pinMode(PIN_MOTOR_IN_UP, INPUT);

  // EM78P510 activity detection (OR: motor-OUT1 + motor-OUT2 via 2× BAT43)
  pinMode(PIN_EM78_ACTIVE, INPUT);

  // Poti enable (PNP base, active LOW)
  pinMode(PIN_POTI_EN, OUTPUT);
  digitalWrite(PIN_POTI_EN, HIGH);  // PNP off → poti unpowered on startup

  // Reed contact (speed + cadence)
  pinMode(PIN_REED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), reedInterrupt, FALLING);

  // Heart rate
  pinMode(PIN_HEARTRATE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_HEARTRATE), heartRateInterrupt, RISING);

  // BLE Setup
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char deviceName[30];
  sprintf(deviceName, "FTMS_%02X%02X%02X", mac[3], mac[4], mac[5]);

  BLEDevice::init(deviceName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pFTMSService = pServer->createService(BLEUUID((uint16_t)0x1826));

  pFitnessMachineFeature         = pFTMSService->createCharacteristic(BLEUUID((uint16_t)0x2ACC), BLECharacteristic::PROPERTY_READ);
  pIndoorBikeData                = pFTMSService->createCharacteristic(BLEUUID((uint16_t)0x2AD2), BLECharacteristic::PROPERTY_NOTIFY);
  pIndoorBikeData->addDescriptor(new BLE2902());
  pTrainingStatus                = pFTMSService->createCharacteristic(BLEUUID((uint16_t)0x2AD3), BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTrainingStatus->addDescriptor(new BLE2902());
  pSupportedResistanceLevelRange = pFTMSService->createCharacteristic(BLEUUID((uint16_t)0x2AD6), BLECharacteristic::PROPERTY_READ);
  pFitnessMachineControlPoint    = pFTMSService->createCharacteristic(BLEUUID((uint16_t)0x2AD9), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE);
  pFitnessMachineControlPoint->addDescriptor(new BLE2902());
  pFitnessMachineStatus          = pFTMSService->createCharacteristic(BLEUUID((uint16_t)0x2ADA), BLECharacteristic::PROPERTY_NOTIFY);
  pFitnessMachineStatus->addDescriptor(new BLE2902());

  pFitnessMachineFeature->setValue(ftmfBuffer, 4);
  pIndoorBikeData->setValue(ibdBuffer, sizeof(ibdBuffer));
  pSupportedResistanceLevelRange->setValue(srlrBuffer, 4);
  pFitnessMachineStatus->setValue(ftmsBuffer, 2);
  pTrainingStatus->setValue(tsBuffer, 2);
  pFitnessMachineControlPoint->setCallbacks(new ControlPointCallbacks());

  pFTMSService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x1826));
  pAdvertising->start();

  if (serial_debug) Serial.printf("BLE started: %s\n", deviceName);
}

// ============================================================
// LOOP
// ============================================================
void loop() {

#if FAKE_DATA
  static unsigned long fake_tick;
  static unsigned long fake_duration = 900;
  static int           fake_increase = 50;
  static bool          led_toggle;
  static unsigned long led_tick;

  if (millis() - led_tick > 500) {
    led_tick = millis();
    if (ble_connected == LOW) {
      rgbLedWrite(RGB_BUILTIN,
        RGB_BRIGHTNESS, RGB_BRIGHTNESS,
        led_toggle ? 0 : RGB_BRIGHTNESS);
    }
    led_toggle = !led_toggle;
  }

  if (millis() - fake_tick >= fake_duration) {
    fake_tick = millis();
    speed_counter++;
    speed_timer            = millis();
    cadence_previous_timer = cadence_timer;
    cadence_timer          = millis();
    hr_previous_timer      = hr_timer;
    hr_timer               = millis() - 857; // ~70 BPM

    fake_duration += fake_increase;
    if (fake_duration >= 1100) fake_increase = -50;
    else if (fake_duration <= 600) fake_increase = 50;
  }
#endif

  // ---- Motor control ----
  if (motorMode == KINOMAP) {
    // ESP32 has priority → maintain target position
    // driveToPosition() is called from handleControlPoint()
  } else {
    // MANUAL: EM78P510 drives motor directly, keep pins high-Z
    handleManualMode();
  }

  // ---- BLE notifications ----
  current_millis = millis();
  if (current_millis > previous_notification + NOTIFICATION_INTERVAL) {

    // Calculate sensor values (always, to keep snapshots current)
    instantaneous_speed   = calculateSpeed();
    instantaneous_cadence = calculateCadence();
    HeartRate             = calculateHeartRate();
    instantaneous_power   = calculatePower(instantaneous_speed);

    // Always update counter snapshots (prevents accumulation)
    speed_counter_prev = speed_counter;
    speed_timer_prev   = speed_timer;
    previous_notification = millis();

    // Read current poti position (always, for serial output)
    int potiRaw = readPotiPosition();
    currentResistance = map(constrain(potiRaw, POTI_ADC_MIN, POTI_ADC_MAX), POTI_ADC_MIN, POTI_ADC_MAX, 0, 100);
    bool em78Active = digitalRead(PIN_EM78_ACTIVE);
    if (serial_debug) Serial.printf("Poti: %d%% (ADC=%d) | EM78: %s | Mode: %s\n",
      currentResistance, potiRaw,
      em78Active ? "ACTIVE" : "idle",
      motorMode == KINOMAP ? "KINOMAP" : "MANUAL");

    if (ble_connected == HIGH) {
      writeIndoorBikeDataCharacteristic();
    }
  }

  // ---- Control point handler ----
  if (previousControlPointEvent != lastControlPointEvent) {
    handleControlPoint();
    previousControlPointEvent = lastControlPointEvent;
  }
}

// ============================================================
// BLE: Write Indoor Bike Data characteristic
// ============================================================
void writeIndoorBikeDataCharacteristic() {
  // Flags: instantaneous cadence + power + heart rate
  uint16_t flags = flagInstantaneousCadence | flagIntantaneousPower | flagHeartRate;
  ibdBuffer[0] = flags & 0xFF;
  ibdBuffer[1] = (flags >> 8) & 0xFF;

  // Speed: km/h * 100 (uint16, resolution 0.01 km/h)
  int s = round(instantaneous_speed * 3.6 * 100.0);
  ibdBuffer[2] = s & 0xFF;
  ibdBuffer[3] = (s >> 8) & 0xFF;

  // Cadence: RPM * 2 (uint16, resolution 0.5 rpm)
  int c = instantaneous_cadence * 2;
  ibdBuffer[4] = c & 0xFF;
  ibdBuffer[5] = (c >> 8) & 0xFF;

  // Power: watts (sint16)
  int p = round(instantaneous_power);
  ibdBuffer[6] = p & 0xFF;
  ibdBuffer[7] = (p >> 8) & 0xFF;

  // Heart rate: BPM (uint8)
  ibdBuffer[8] = HeartRate;

  pIndoorBikeData->setValue(ibdBuffer, sizeof(ibdBuffer));
  pIndoorBikeData->notify();

  if (serial_debug) {
    Serial.printf("Speed: %.1f km/h | Cadence: %d RPM | Power: %.0f W | HR: %d BPM | Resistance: %d%%\n",
      instantaneous_speed * 3.6,
      instantaneous_cadence,
      instantaneous_power,
      HeartRate,
      currentResistance);
  }
}

// ============================================================
// BLE: Write Training Status characteristic
// ============================================================
void writeTrainingStatus() {
  switch (training_status) {
    case STOPPED:
      tsBuffer[0] = 0x02; tsBuffer[1] = 0x01;
      pTrainingStatus->setValue(tsBuffer, 2);
      pTrainingStatus->notify();
      break;
    case PAUSED:
      tsBuffer[0] = 0x02; tsBuffer[1] = 0x02;
      pTrainingStatus->setValue(tsBuffer, 2);
      pTrainingStatus->notify();
      break;
    case RUNNING:
      tsBuffer[0] = 0x04;
      pTrainingStatus->setValue(tsBuffer, 1);
      pTrainingStatus->notify();
      break;
  }
}

// ============================================================
// BLE: Control Point handler
// ============================================================
void handleControlPoint() {
  if (serial_debug) {
    Serial.printf("Control Point: OpCode=0x%02X\n", fmcpData.values.OPCODE);
  }

  switch (fmcpData.values.OPCODE) {

    case fmcpRequestControl: {
      ftmcpBuffer[0] = fmcpResponseCode;
      ftmcpBuffer[1] = fmcpData.values.OPCODE;
      ftmcpBuffer[2] = 0x01; // Success
      pFitnessMachineControlPoint->setValue(ftmcpBuffer, 3);
      pFitnessMachineControlPoint->indicate();
      break;
    }

    case fmcpStartOrResume: {
      training_status = RUNNING;
      writeTrainingStatus();
      break;
    }

    case fmcpStopOrPause: {
      training_status = STOPPED;
      motorStop();
      writeTrainingStatus();
      break;
    }

    case fmcpSetTargetResistanceLevel: {
      // KinoMap sends resistance 0-200 (unit 0.5%) → map to 0-100%
      int raw = fmcpData.values.OCTETS[0];
      targetResistance = constrain(map(raw, 0, 200, 0, 100), 0, 100);
      lastMotorCmd     = millis();

      if (serial_debug) Serial.printf("SetResistance: %d%% (raw=%d)\n", targetResistance, raw);

      driveToPosition(targetResistance);

      ftmcpBuffer[0] = fmcpResponseCode;
      ftmcpBuffer[1] = fmcpData.values.OPCODE;
      ftmcpBuffer[2] = 0x01; // Success
      pFitnessMachineControlPoint->setValue(ftmcpBuffer, 3);
      pFitnessMachineControlPoint->indicate();
      break;
    }

    case fmcpSetIndoorBikeSimulationParameters: {
      // Wind speed (int16, resolution 0.001 m/s)
      short ws  = (fmcpData.values.OCTETS[1] << 8) | fmcpData.values.OCTETS[0];
      wind_speed = ws / 1000.0;

      // Grade (int16, resolution 0.01%)
      short gr  = (fmcpData.values.OCTETS[3] << 8) | fmcpData.values.OCTETS[2];
      grade      = gr / 100.0;

      // Crr (uint8, resolution 0.0001)
      crr = fmcpData.values.OCTETS[4] / 10000.0;

      // Cw  (uint8, resolution 0.01)
      cw  = fmcpData.values.OCTETS[5] / 100.0;

      // Grade → map to resistance (-10% to +20% → 0-100%)
      targetResistance = constrain(map((int)grade, -10, 20, 0, 100), 0, 100);
      lastMotorCmd     = millis();

      if (serial_debug) {
        Serial.printf("Simulation: Wind=%.2f Grade=%.1f%% Crr=%.4f Cw=%.2f → Resistance=%d%%\n",
          wind_speed, grade, crr, cw, targetResistance);
      }

      driveToPosition(targetResistance);

      ftmcpBuffer[0] = fmcpResponseCode;
      ftmcpBuffer[1] = fmcpData.values.OPCODE;
      ftmcpBuffer[2] = 0x01; // Success
      pFitnessMachineControlPoint->setValue(ftmcpBuffer, 3);
      pFitnessMachineControlPoint->indicate();
      break;
    }

    default: {
      // All other opcodes: not supported
      ftmcpBuffer[0] = fmcpResponseCode;
      ftmcpBuffer[1] = fmcpData.values.OPCODE;
      ftmcpBuffer[2] = 0x02; // OpCode not supported
      pFitnessMachineControlPoint->setValue(ftmcpBuffer, 3);
      pFitnessMachineControlPoint->indicate();
      if (serial_debug) Serial.printf("Unsupported OpCode: 0x%02X\n", fmcpData.values.OPCODE);
      break;
    }
  }
}