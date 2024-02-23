#include "Arduino.h"
// #include "pico.h"
#include "VisionModule.h"

VisionModule::VisionModule() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void VisionModule::update() { 
  int i; 
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }
}