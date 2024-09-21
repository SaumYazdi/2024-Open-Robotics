#ifndef Bot_h
#define Bot_h

#include "Arduino.h"
#include "LogicModule.h"
// #include "WebserverModule.h"

class Bot {
  public:
    Bot();
    void update();
    int getMode();
    void setMode(int botMode);
  private:
    int mode;
    LogicModule logic;
    // WebserverModule web;
};

#endif