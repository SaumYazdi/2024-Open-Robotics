#include "Arduino.h"
#include "Bot.h"

// Constants
#define CALIBRATION 0xA2
#define NEUTRAL 0xA3
#define RUNNING 0xB6

Bot::Bot() {
  mode = NEUTRAL;
}

void Bot::update() {
  logic.update();
  // web.update();

  // switch (mode) {
  //   case CALIBRATION:
  //     break;
      
  //   case NEUTRAL:
  //     break;

  //   case RUNNING:
  //     break;
  // }
}

int Bot::getMode() {
  return mode;
}

void Bot::setMode(int botMode) {
  mode = botMode;
}