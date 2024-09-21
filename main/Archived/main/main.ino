#include "Bot.h"

Bot* bot;

void setup() {
  bot = new Bot();
}

void loop() {
  bot->update();
}