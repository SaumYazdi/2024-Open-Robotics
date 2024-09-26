#ifndef Bot_h
#define Bot_h

#include "Arduino.h"
#include "VisionModule.h"

#define CALIBRATION 0xef
#define NEUTRAL 0x3a

class Bot {
  public:
    Bot();
    void update();
  private:
    int mode;
};

#endif