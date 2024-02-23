#ifndef BlinkModule_h
#define BlinkModule_h

#include "Arduino.h"
// #include "pico.h"
#include <Pixy2.h>

class BlinkModule {
  public:
    BlinkModule();
    void update();
    void toggleLED();
  private:
    int LED_PIN;
};

#endif