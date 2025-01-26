#include "matrixLED.h"

const int trigPin = 8; // Trig Pin of the Ultrasonic Sensor
const int echoPin = 9; // Echo Pin of the Ultrasonic Sensor
matrixLED matrix;

void setup() {
  matrix.init(0x71);
  matrix.flipVertical();
  matrix.flipHorizontal();
  matrix.setBrightness(5);
  matrix.setBlink(VK16K33_BLINK_OFF);
  Serial.begin(9600); // Initialise serial communication
  pinMode(trigPin, OUTPUT); // Set the trigPin as OUTPUT
  pinMode(echoPin, INPUT);  // Set the echoPin as INPUT
}

void loop() {
  long duration, distance_cm;

  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance_cm = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  // Display based on distance
  if (distance_cm <= 10) {
    matrix.showArray(100, 3); // Startled
  } 
  if (distance_cm > 10 && distance_cm <= 40) {
    matrix.showArray(0, 2); // Happy
  } 
  if (distance_cm > 40){
    matrix.showArray(0, 1); // Sad
  }

  delay(99);
}
