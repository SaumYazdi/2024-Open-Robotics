#ifndef Bot_h
#define Bot_h

#include "Arduino.h"
#include "LogicModule.h"

class Bot {
  public:
    Bot() {
      mode = NEUTRAL;
    }
    void update();
    String getMode();
    void setMode(int botMode);
    float heading();
    int* tofs();
    
    LogicModule logic;
    float speed = 0;
    float direction = 0;
    float turnSpeed = 0;
  private:
    int mode;
    int distances[8];
};

#endif