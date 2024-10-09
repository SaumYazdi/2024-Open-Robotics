#ifndef EventHandler_h
#define EventHandler_h
  
#include "PowerfulBLDCdriver.h"

class EventHandler {
  public:
    void stop(int motorNumber);
    void setSpeed(int motorNumber, float speed);
    void update();

    PowerfulBLDCdriver *motor1;
    PowerfulBLDCdriver *motor2;
    PowerfulBLDCdriver *motor3;
    PowerfulBLDCdriver *motor4;
    PowerfulBLDCdriver *motor5;

  private:
    const int maxMotorSpeed = 90000000;

    const int commandInterval = 400; // in milliseconds; delay to send i2c commands to update motor speed
    unsigned long prevTime = millis();
    unsigned long deltaTime = 0;
    int motorNumber;
    uint8_t command;
    int32_t value;
    String commands[5];
};

#endif