#include "Arduino.h"
// #include "pico.h"
#include "BlinkModule.h"

BlinkModule::BlinkModule() {
  LED_PIN = LED_BUILTIN;
  
  pinMode(LED_PIN, OUTPUT);
}

void BlinkModule::update() {
  if (millis() % 2000 == 0) {
    toggleLED();
  }
}

void BlinkModule::toggleLED() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

