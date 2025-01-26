#include "EEPROM.h"
#include "Arduino.h"
#include "robot.h"

// Define servo pins for the robot
const int YL = 10, YR = 11, RL = 12, RR = 13;

robot robot;

void setup() {
  // put your setup code here, to run once:
  // Initialise the robot
  robot.init(YL, YR, RL, RR);
  robot.setTrims(0, 0, 0, 0);
  robot.attachServos();
}

void loop() {
  // put your main code here, to run repeatedly:
  robot.turn(2, 1500, LEFT); // Turn left
  delay(100);
  robot.walk(4, 1500, FORWARD); // Walk forward
  delay(100);
  robot.crusaito(2, 1000, 20, FORWARD);
  delay(500);
}
