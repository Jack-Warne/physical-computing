#include "SmileSensor.h"

const int trigPin = 8; // Trig Pin of the Ultrasonic Sensor
const int echoPin = 9; // Echo Pin of the Ultrasonic Sensor
SmileSensor matrix;

void setup() {
  Serial.begin(9600); // Initialise serial communication
  pinMode(trigPin, OUTPUT); // Set the trigPin as OUTPUT
  pinMode(echoPin, INPUT);  // Set the echoPin as INPUT

  matrix.init(0x71);
  matrix.setBrightness(5);
  matrix.setBlink(VK16K33_BLINK_OFF);
}

void loop() {
  long duration, distance_cm;

  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance_cm = duration * 0.034 / 2; // Speed of sound = 0.034 cm/μs

  // Decide the emotion based on distance
  if (distance_cm < 10) {
    matrix.showEmotion("STARTLED");
  } else if (distance_cm < 30) {
    matrix.showEmotion("SMILE");
  } else {
    matrix.showEmotion("SAD");
  }

  // Print the distance
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(100); // Wait for 500 ms
}
