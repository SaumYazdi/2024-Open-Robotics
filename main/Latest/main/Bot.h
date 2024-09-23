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
    int getMode();
    void setMode(int botMode);
    float heading();
    int* tofs();
    
    LogicModule logic;
    float speed = 0;
    float direction = 0;
  private:
    int mode;
    int distances[8];
};

#endif