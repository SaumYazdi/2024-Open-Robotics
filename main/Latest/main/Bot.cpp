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
      logic.readBall();
      float correction = logic.correctedHeading();

      float deg = logic.ballAngle - 45; //  + correction
      float rad = deg * DEG_TO_RAD;

      speed = 45000000;
      float speedY1 = -cosf(rad) * speed;
      float speedX2 = -sinf(rad) * speed;
      float speedY3 = cosf(rad) * speed;
      float speedX4 = sinf(rad) * speed;
      
      logic.events.setSpeed(1, speedY1);
      logic.events.setSpeed(2, speedX2);
      logic.events.setSpeed(3, speedY3);
      logic.events.setSpeed(4, speedX4);

      bool hasBall = (logic.ballDistance < 20) && (-15 <= logic.ballAngle && logic.ballAngle <= 15);
      if (hasBall) {
        logic.events.setSpeed(5, 90000000);
      } else {
        logic.events.stop(5);
      }

      // if (turnSpeed != 0) {
      //   logic.motor1.setSpeed(turnSpeed);
      //   logic.motor2.setSpeed(turnSpeed);
      //   logic.motor3.setSpeed(turnSpeed);
      //   logic.motor4.setSpeed(turnSpeed);
      // } else {
      //   if (speed != 0) {
      //     logic.motor5.setSpeed(80000000);
      //     logic.manual(direction, speed);
      //   } else {
      //     logic.logic();
      //   }
      // }
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