#include <Arduino.h>
#include "LineFollower.h"


// Create LineFollower instance with base speed 200 and max speed 250
LineFollower lineFollower(200, 250);

void setup() {
  lineFollower.init();
  lineFollower.calibrate();
}

void loop() {
  lineFollower.update();
}