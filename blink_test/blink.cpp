// Minimaler Blink-Test für ESP32-C3 DevKitM-1
// RGB_BUILTIN = GPIO8 (WS2812), kein BLE, keine Libraries
#include <Arduino.h>

#define BRIGHTNESS 30  // 0-255, niedrig halten (WS2812 ist sehr hell)

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-C3 Blink-Test gestartet");
}

void loop() {
  Serial.println("ROT");
  rgbLedWrite(RGB_BUILTIN, BRIGHTNESS, 0, 0);
  delay(1000);

  Serial.println("GRUEN");
  rgbLedWrite(RGB_BUILTIN, 0, BRIGHTNESS, 0);
  delay(1000);

  Serial.println("BLAU");
  rgbLedWrite(RGB_BUILTIN, 0, 0, BRIGHTNESS);
  delay(1000);

  Serial.println("AUS");
  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
  delay(500);
}
