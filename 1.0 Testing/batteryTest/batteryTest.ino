#include "Arduino.h"
#include "battery.h"

battery battery;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  Serial.print("battery Voltage is: ");
  Serial.print(battery.Get_Battery_Voltage());
  Serial.printf("V \n");
  delay(1000);

}
