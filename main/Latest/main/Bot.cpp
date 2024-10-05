#include "Arduino.h"
#include "Bot.h"

void Bot::update() {
  mode = logic.update();

  switch (mode) {
    case CALIBRATION:
      logic.calibrate();
      break;
      
    case NEUTRAL:
      // logic.readBall();
      // logic.updateEstimatedPosition();
      logic.stop();
      logic.reachedPosition = false;
      break;

    case RUNNING:
      // REMOTE CONTROL STUFF
      if (speed != 0) {
        logic.motor5.setSpeed(80000000);
        logic.manual(direction, speed);
      } else {
        logic.logic();
      }
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

String Bot::getMode() {
  switch (mode) {
    case CALIBRATION:
      return "Calibration";
      break;
    case NEUTRAL:
      return "Neutral";
      break;
    case RUNNING:
      return "Running";
      break;
    default:
      return "None";
      break;
  }
}

void Bot::setMode(int botMode) {
  mode = botMode;
}