#include "VisionModule.h"

VisionModule* vision;

void setup() {
  vision = new VisionModule();
}

void loop() {
  vision->update();
}



