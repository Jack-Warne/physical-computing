#include "UltrasonicSensor.h"

// Define pins for the ultrasonic sensor
const int trigPin = 8;
const int echoPin = 9;

ultrasonicSensor ultrasonicSensor(trigPin, echoPin);

void setup() {
  // Initialise the ultrasonic sensor
  ultrasonicSensor.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Get the distance in cm
  long distanceCM = ultrasonicSensor.getDistanceCM();
  Serial.print("the distance is: ");
  Serial.print(distanceCM);
  Serial.printf("cm \n");
  delay(100);
}
