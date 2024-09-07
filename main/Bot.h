#ifndef Bot_h
#define Bot_h

#include "Arduino.h"
#include "VisionModule.h"

class Bot {
  public:
    Bot();
    void update();
    void calibrate();
    void calibrateLoop();
  private:
    VisionModule* vision;
    int mode;
    float calibrationDistance;
    int calibrationCooldown;
};

#endif