#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class ultrasonicSensor {
public:
    ultrasonicSensor(int trigPin, int echoPin);
    void begin();
    long getDistanceCM();

private:
    int _trigPin;
    int _echoPin;

    void triggerPulse();
};

#endif // ULTRASONIC_SENSOR_H
