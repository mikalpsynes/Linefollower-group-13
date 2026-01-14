#include <Arduino.h>
#include "LineFollower.h"


// Create LineFollower instance with base speed 200 and max speed 250
LineFollower lineFollower(200, 250);

static void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("SETPID")) {
      // Expected format: SETPID Kp Ki Kd
      float kp, ki, kd;
      // Extract tokens
      int firstSpace = cmd.indexOf(' ');
      if (firstSpace > 0) {
        String rest = cmd.substring(firstSpace + 1);
        rest.trim();
        int s1 = rest.indexOf(' ');
        int s2 = (s1 > 0) ? rest.indexOf(' ', s1 + 1) : -1;
        if (s1 > 0 && s2 > 0) {
          kp = rest.substring(0, s1).toFloat();
          ki = rest.substring(s1 + 1, s2).toFloat();
          kd = rest.substring(s2 + 1).toFloat();
          lineFollower.setPIDGains(kp, ki, kd);
          Serial.print("PID updated: Kp="); Serial.print(kp);
          Serial.print(" Ki="); Serial.print(ki);
          Serial.print(" Kd="); Serial.println(kd);
        } else {
          Serial.println("Usage: SETPID <Kp> <Ki> <Kd>");
        }
      } else {
        Serial.println("Usage: SETPID <Kp> <Ki> <Kd>");
      }
    } else if (cmd == "HELP") {
      Serial.println("Commands:");
      Serial.println("  SETPID <Kp> <Ki> <Kd>  - update PID gains at runtime");
    }
  }
}

void setup() {
  Serial.begin(115200);
  lineFollower.init();
  lineFollower.calibrate();
  Serial.println("Type HELP for commands. Current default PID: Kp=1.0 Ki=0.04 Kd=7.0");
}

void loop() {
  handleSerialCommands();
  lineFollower.update();
}