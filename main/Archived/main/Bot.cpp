#include "Arduino.h"
#include "Bot.h"

// Constants
#define MENU 0xA1
#define CALIBRATION 0xA2
#define RUNNING 0xA3

Bot::Bot() {
  Serial.begin(115200);
  Serial.println("Started..");

  // Declare variables
  mode = CALIBRATION;
  calibrationDistance = 0.0;
  calibrationCooldown = 0;

  // Declare pins
  // pinMode(8, INPUT);

  // vision = new VisionModule();
}

void Bot::update() {
  Serial.println("Bot start");
  switch (mode) {
    case MENU:
      break;
    
    case CALIBRATION:
      calibrateLoop();
      break;

    case RUNNING:
      break;
  }
  Serial.println("Bot end");
}

void Bot::calibrate() {
  Serial.println("Calibrate start");
  mode = CALIBRATION;
  Serial.println("Calibrate end");
}

void Bot::calibrateLoop() {
  Serial.println("Calibrateloop start");
  vision->update();
  Serial.println("Calibrateloop end");

  // if (digitalRead(8)) {
  //   if (calibrationCooldown == 0) {
      
  //     calibrationDistance += 5.0;
  //     Serial.println(calibrationDistance);

  //     calibrationCooldown = 2000; // ticks
  //   }
  // }
  // else {
  //   if (calibrationCooldown > 0)
  //     calibrationCooldown--;
  // }
}