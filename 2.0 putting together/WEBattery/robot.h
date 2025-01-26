#ifndef robot_h
#define robot_h

#include <Servo.h>
#include "Oscillator.h"
#include <EEPROM.h>  // For EEPROM functionality

//-- Constants
#define FORWARD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1

class robot {
public:
    // Bipedal_Robot initialisation
    void init(int YL, int YR, int RL, int RR);

    // Attach & detach functions
    void attachServos();
    void detachServos();

    // Oscillator Trims
    void setTrims(int YL, int YR, int RL, int RR);

    // Helper functions
    void _moveServos(int time, int* target);
    void oscillateServos(int* A, int* O, int T, double* phase_diff, float cycle);

    // Movement Functions
    void walk(float steps = 4, int T = 1500, int dir = FORWARD);
    void turn(float steps = 4, int T = 1500, int dir = LEFT);
    void crusaito(float steps = 1, int T = 1000, int h = 20, int dir = FORWARD);

    // EEPROM Functions
    void saveTrimsToEEPROM();
    void loadTrimsFromEEPROM();

private:
    // Servos and related variables
    Oscillator servo[4];
    int servo_pins[4];
    int servo_trim[4];

    // Internal time tracking
    unsigned long final_time;
    unsigned long partial_time;
    float increment[4];
};

#endif
