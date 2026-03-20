#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "esp_mac.h"

// ============================================================
// KONFIGURATION
// ============================================================
#define FAKE_DATA 0                    // 1 = Testdaten, 0 = echte Sensoren

boolean serial_debug         = true;  // false wenn kein PC angeschlossen
boolean write_startup_message = true;

// Effektive Distanz pro Kurbel-Umdrehung (Reed sitzt an der Kurbel, 1 Impuls = 1 Umdrehung)
// Enthält implizit das Ergometer-Getriebe → kein separates GEAR_RATIO nötig
// Kalibrierung: WHEEL_CIRCUMFERENCE_MM = aktueller_Wert × (Display_km_h / App_km_h)
// Gemessen: Display=21.6 km/h, App=1.3 km/h → 377 × (21.6/1.3) ≈ 6264 mm
#define WHEEL_CIRCUMFERENCE_MM  6264.0

// Gewicht Fahrer in kg (für Leistungsberechnung)
#define RIDER_WEIGHT_KG  80.0

// ============================================================
// PIN DEFINITIONEN (ESP32-S3)
// ============================================================
#define PIN_REED        2    // Reed-Kontakt → Speed + Cadence
                             // Hardware: Reed-Leitung (4.7V) ──10kΩ──GPIO2
                             // 10kΩ Reihenwiderstand schützt ESP32 (max 3.6V)
                             // Kein Teiler nach GND → EM78P510 bleibt unbelastet
                             // GPIO6=JTAG/MOSI auf ESP32-C3 → spurious Interrupts, daher GPIO2
#define PIN_HEARTRATE   7    // Herzfrequenz Sensor (5V via 10k/18k Teiler) - GPIO7 funktioniert
#define PIN_POTI        4    // ADC Potentiometer Schleifer (ADC1_CH4, kein BLE-Konflikt)
#define PIN_POTI_EN     5    // MOSFET Gate: Poti-Versorgung einschalten
#define PIN_EM78_ACTIVE 1    // EM78P510 aktiv (OR: SENSE_A + SENSE_B via 2× BAT46 → GPIO1)
                             // GPIO8 = RGB-LED auf C3 Super Mini → nicht verwenden!
                             // Hardware: SENSE_A ──BAT46──┐
                             //           SENSE_B ──BAT46──┴── GPIO1 + 10k→GND
                             // HIGH = EM78P510 drückt Taste → DRV8833 sofort stoppen
#define PIN_MOTOR_IN1   10   // DRV8833 IN1
#define PIN_MOTOR_IN2   20   // DRV8833 IN2 (GPIO11 oft SPI-Flash intern auf C3!)

// ============================================================
// MOTORSTEUERUNG
// ============================================================
enum ControlMode { KINOMAP, MANUAL };
ControlMode motorMode        = MANUAL; // Beim Start ohne BLE: Manual
int targetResistance         = 0;      // 0-100% Zielwiderstand von KinoMap
int currentResistance        = 0;      // Aktuelle Position (aus Poti)
unsigned long lastMotorCmd   = 0;
#define MOTOR_TIMEOUT        5000      // ms ohne KinoMap-Befehl → zurück zu MANUAL
#define POTI_TOLERANCE       80        // ADC-Toleranzband (0-4095)
#define MOTOR_DRIVE_INTERVAL 50        // ms zwischen Poti-Lesungen beim Fahren

void motorUp() {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
}

void motorDown() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
}

void motorStop() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
}

// Poti kurz einschalten + ADC lesen
int readPotiPosition() {
  digitalWrite(PIN_POTI_EN, HIGH);
  delay(5);
  int val = analogRead(PIN_POTI);
  digitalWrite(PIN_POTI_EN, LOW);
  return val; // 0-4095
}

// Motor auf Zielposition fahren (blockierend, max 5 Sek.)
// Bricht sofort ab wenn EM78P510 eine Taste drückt (Kollisionsschutz)
void driveToPosition(int targetPct) {
  int targetAdc  = map(targetPct, 0, 100, 0, 4095);
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {

    // Kollisionsschutz: EM78P510 aktiv → sofort stoppen
    if (digitalRead(PIN_EM78_ACTIVE)) {
      motorStop();
      if (serial_debug) Serial.println("Motor: EM78P510 aktiv, DRV8833 gestoppt!");
      return;
    }

    int currentAdc = readPotiPosition();
    currentResistance = map(currentAdc, 0, 4095, 0, 100);

    if (abs(currentAdc - targetAdc) <= POTI_TOLERANCE) {
      motorStop();
      if (serial_debug) {
        Serial.printf("Motor: Ziel erreicht %d%% (ADC=%d)\n", targetPct, currentAdc);
      }
      return;
    }

    if (currentAdc < targetAdc) motorUp();
    else                        motorDown();

    delay(MOTOR_DRIVE_INTERVAL);
  }

  motorStop(); // Timeout
  if (serial_debug) Serial.println("Motor: Timeout!");
}

// MANUAL Modus: EM78P510 treibt Motor direkt über eigene H-Brücke
// DRV8833 bleibt gestoppt damit keine Kollision entsteht
void handleManualMode() {
  motorStop(); // DRV8833 aus dem Weg räumen
}

// ============================================================
// SENSOREN
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

// Reed-Kontakt Interrupt → Speed + Cadence
// Debounce 50ms: verhindert Mehrfachtrigger durch Kontaktprellen
void IRAM_ATTR reedInterrupt() {
  unsigned long now = millis();
  if (now - speed_timer < 50) return;  // Prellunterdrückung
  speed_counter++;
  speed_timer            = now;
  cadence_previous_timer = cadence_timer;
  cadence_timer          = now;
}

// Herzfrequenz Interrupt
void IRAM_ATTR heartRateInterrupt() {
  hr_previous_timer = hr_timer;
  hr_timer          = millis();
}

double calculateSpeed() {
  // Noch keine 2 gültigen Messungen vorhanden
  if (speed_timer == 0 || speed_timer_prev == 0) return 0.0;
  // Kein Signal seit >3 Sek. → Geschwindigkeit = 0
  if (millis() - speed_timer > 3000) return 0.0;
  if (speed_timer == speed_timer_prev) return 0.0;
  unsigned long delta_count = speed_counter - speed_counter_prev;
  unsigned long delta_time  = speed_timer   - speed_timer_prev;
  if (delta_count == 0 || delta_time == 0 || delta_time > 5000) return 0.0;
  return (delta_count * WHEEL_CIRCUMFERENCE_MM / 1000.0) / (delta_time / 1000.0); // m/s
}

unsigned int calculateCadence() {
  // Reed an Kurbel → 1 Impuls = 1 Kurbelumdrehung → direkt RPM
  unsigned long interval = cadence_timer - cadence_previous_timer;
  if (interval < 200 || interval > 5000) return 0;
  return (unsigned int)round(60000.0 / interval);
}

uint8_t calculateHeartRate() {
  // Kein Signal seit >3 Sek. → HR = 0 (Hände weg vom Sensor)
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
    motorMode     = KINOMAP; // BLE verbunden → ESP32 hat Vorrang
    rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Blau
    if (serial_debug) Serial.println("BLE verbunden → KINOMAP Modus");
  }
  void onDisconnect(BLEServer *pSrv) override {
    ble_connected = LOW;
    motorMode     = MANUAL;  // BLE getrennt → EM78P510 Tasten aktiv
    motorStop();
    rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0); // Gelb
    BLEDevice::getAdvertising()->start();
    if (serial_debug) Serial.println("BLE getrennt → MANUAL Modus");
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
    Serial.println("SmartErgoMeter startet...");
  }

  rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0); // Gelb = Start

  // Motor Pins
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  motorStop();

  // EM78P510 Aktivitätserkennung (OR: SENSE_A + SENSE_B via 2× BAT46)
  pinMode(PIN_EM78_ACTIVE, INPUT);

  // Poti Enable (MOSFET)
  pinMode(PIN_POTI_EN, OUTPUT);
  digitalWrite(PIN_POTI_EN, LOW);

  // Reed-Kontakt (Speed + Cadence)
  pinMode(PIN_REED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), reedInterrupt, FALLING);

  // Herzfrequenz
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

  if (serial_debug) Serial.printf("BLE gestartet: %s\n", deviceName);
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

  // ---- Motorsteuerung ----
  if (motorMode == KINOMAP) {
    // ESP32 hat Vorrang → Zielposition halten
    // driveToPosition() wird aus handleControlPoint() aufgerufen
  } else {
    // MANUAL: EM78P510 treibt Motor direkt, DRV8833 gestoppt halten
    handleManualMode();
  }

  // ---- BLE Notifications ----
  current_millis = millis();
  if (current_millis > previous_notification + NOTIFICATION_INTERVAL) {

    // Sensorwerte berechnen (immer, damit Snapshots aktuell bleiben)
    instantaneous_speed   = calculateSpeed();
    instantaneous_cadence = calculateCadence();
    HeartRate             = calculateHeartRate();
    instantaneous_power   = calculatePower(instantaneous_speed);

    // Counter-Snapshots immer aktualisieren (verhindert Akkumulation)
    speed_counter_prev = speed_counter;
    speed_timer_prev   = speed_timer;
    previous_notification = millis();

    if (ble_connected == HIGH) {
      // Aktuelle Poti-Position lesen
      currentResistance = map(readPotiPosition(), 0, 4095, 0, 100);
      writeIndoorBikeDataCharacteristic();
    }
  }

  // ---- Control Point Handler ----
  if (previousControlPointEvent != lastControlPointEvent) {
    handleControlPoint();
    previousControlPointEvent = lastControlPointEvent;
  }
}

// ============================================================
// BLE: Indoor Bike Data schreiben
// ============================================================
void writeIndoorBikeDataCharacteristic() {
  // Flags: Instantaneous Cadence + Power + Heart Rate
  uint16_t flags = flagInstantaneousCadence | flagIntantaneousPower | flagHeartRate;
  ibdBuffer[0] = flags & 0xFF;
  ibdBuffer[1] = (flags >> 8) & 0xFF;

  // Geschwindigkeit: km/h * 100 (uint16, resolution 0.01 km/h)
  int s = round(instantaneous_speed * 3.6 * 100.0);
  ibdBuffer[2] = s & 0xFF;
  ibdBuffer[3] = (s >> 8) & 0xFF;

  // Cadence: RPM * 2 (uint16, resolution 0.5 rpm)
  int c = instantaneous_cadence * 2;
  ibdBuffer[4] = c & 0xFF;
  ibdBuffer[5] = (c >> 8) & 0xFF;

  // Power: Watt (sint16)
  int p = round(instantaneous_power);
  ibdBuffer[6] = p & 0xFF;
  ibdBuffer[7] = (p >> 8) & 0xFF;

  // Heart Rate: BPM (uint8)
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
// BLE: Training Status schreiben
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
// BLE: Control Point Handler
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
      // KinoMap sendet Widerstand 0-200 (Einheit 0.5%) → wir mappen auf 0-100%
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

      // Grade → Widerstand mappen (-10% bis +20% → 0-100%)
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
      // Alle anderen Opcodes: nicht unterstützt
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