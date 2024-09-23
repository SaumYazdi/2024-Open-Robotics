#include "Arduino.h"
#include "Bot.h"

void Bot::update() {
  mode = logic.update();

  switch (mode) {
    case CALIBRATION:
      logic.calibrate();
      break;
      
    case NEUTRAL:
      logic.stop();
      break;

    case RUNNING:
      logic.logic(direction, speed);
      break;
  }
}

float Bot::heading() {
  return logic.correctedHeading();
}

int* Bot::tofs() {
  logic.readTOFs();
  for (int i = 0; i < 5; i++) {
    distances[i] = logic.distances[i];
  }

  return distances;
}

int Bot::getMode() {
  return mode;
}

void Bot::setMode(int botMode) {
  mode = botMode;
}