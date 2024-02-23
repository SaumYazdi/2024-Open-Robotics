#ifndef VisionModule_h
#define VisionModule_h

#include "Arduino.h"
// #include "pico.h"
#include <Pixy2.h>

class VisionModule {
  public:
    VisionModule();
    void update();
  private:
    Pixy2 pixy;
};

#endif