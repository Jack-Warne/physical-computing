#include "Arduino.h"
#include "buzzer.h"

buzzer buzzer;

void setup() {
  // put your setup code here, to run once:
  buzzer.Buzzer_Setup();
}

void loop() {
    // put your main code here, to run repeatedly:
    buzzer.Buzzer_Alert(3,4);
    delay(1000);
}
