#include "EventHandler.h"

// DEFINE MOTOR COMMANDS
#define SETSPEED "STSP"
#define STOP "STOP"

void EventHandler::stop(int motorNumber) {
  String command = STOP;
  commands[motorNumber - 1] = command;
}

void EventHandler::setSpeed(int motorNumber, int speed) {
  String command = SETSPEED + String(speed);
  commands[motorNumber - 1] = command;
}

void EventHandler::update() {
  deltaTime = millis() - prevTime;
  if (deltaTime >= commandInterval) {
    prevTime = millis();

    for (int motorNumber = 0; motorNumber < 5; motorNumber++) {

      PowerfulBLDCdriver* motor;
      switch (motorNumber) {
        case 0:
          motor = motor1;
          break;
        case 1:
          motor = motor2;
          break;
        case 2:
          motor = motor3;
          break;
        case 3:
          motor = motor4;
          break;
        case 4:
          motor = motor5;
          break;
      }

      String command = commands[motorNumber].substring(0, 4);
      if (command == STOP) {
        motor->setSpeed(0);
        commands[motorNumber] = "";
        continue;
      }

      int value = commands[motorNumber].substring(4, commands[motorNumber].length()).toInt();
      
      if (command == SETSPEED) {
        motor->setSpeed(value);
        commands[motorNumber] = "";
        continue;
      }
    }
  }
}