#include "ultrasonicSensor.h"

// Constructor to initialise trig and echo pin
ultrasonicSensor::ultrasonicSensor(int trigPin, int echoPin) 
    : _trigPin(trigPin), _echoPin(echoPin) {}

void ultrasonicSensor::begin() {
    pinMode(_trigPin, OUTPUT); // Set trigPin as OUTPUT
    pinMode(_echoPin, INPUT);  // Set echoPin as INPUT
}

void ultrasonicSensor::triggerPulse() {
    // Clear the trigPin
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);

    // Set the trigPin on HIGH state for 10 microseconds
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
}

long ultrasonicSensor::getDistanceCM() {
    triggerPulse();

    // Read the echoPin and calculate the sound wave travel time
    long duration = pulseIn(_echoPin, HIGH);

    // Calculate the distance in cm
    return duration * 0.034 / 2; // Speed of sound is 0.034 cm/μs
}
