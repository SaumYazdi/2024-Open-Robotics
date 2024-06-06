#include "Bot.h"

Bot* bot;

void setup() {
  bot = new Bot();
}

void loop() {
  Serial.println("Loop start");
  bot->update();
  Serial.println("Loop end");
}



