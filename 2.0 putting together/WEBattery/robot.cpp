#include "robot.h"

// Initialise the robot by setting up servos
void robot::init(int YL, int YR, int RL, int RR) {
    servo_pins[0] = YL;
    servo_pins[1] = YR;
    servo_pins[2] = RL;
    servo_pins[3] = RR;

    for (int i = 0; i < 4; i++) {
        servo[i].attach(servo_pins[i]);
        servo_trim[i] = 0; // Default trims to 0
    }
}

// Attach servos
void robot::attachServos() {
    for (int i = 0; i < 4; i++) {
        servo[i].attach(servo_pins[i]);
    }
}

// Detach servos
void robot::detachServos() {
    for (int i = 0; i < 4; i++) {
        servo[i].detach();
    }
}

// Function to move servos to a target position over a given duration
void robot::_moveServos(int time, int* target) {
    int partial_time = time / 50; // Dividing time into 50 increments
    int increment[4];
    
    for (int i = 0; i < 4; i++) {
        increment[i] = (target[i] - servo[i].getPosition()) / partial_time;
    }
    
    for (int t = 0; t < partial_time; t++) {
        for (int i = 0; i < 4; i++) {
            servo[i].SetPosition(servo[i].getPosition() + increment[i]);
        }
        delay(50);
    }
}

// Function to oscillate servos with given parameters
void robot::oscillateServos(int* A, int* O, int T, double* phase_diff, float cycle) {
    for (int i = 0; i < 4; i++) {
        servo[i].SetA(A[i]);         // Set amplitude
        servo[i].SetO(O[i]);         // Set offset
        servo[i].SetT(T);            // Set period
        servo[i].SetPh(phase_diff[i]); // Set phase difference
        servo[i].Play();             // Start oscillation
    }

    unsigned long start_time = millis();
    while ((millis() - start_time) < (unsigned long)(T * cycle)) {
        for (int i = 0; i < 4; i++) {
            servo[i].refresh(); // Refresh the servos
        }
    }

    for (int i = 0; i < 4; i++) {
        servo[i].Stop(); // Stop the servos
    }
}

// Walking motion
void robot::walk(float steps, int T, int dir) {
    int A[4] = {30, 30, 20, 20};        // Amplitudes for servos
    int O[4] = {0, 0, 4 * dir, -4 * dir}; // Offsets for servos
    double phase_diff[4] = {0, 0, 0.5, 0.5};

    oscillateServos(A, O, T, phase_diff, steps);
}

// Turning motion
void robot::turn(float steps, int T, int dir) {
    int A[4] = {30, 30, 20, 20};         // Amplitudes for servos
    int O[4] = {0, 0, dir * 4, -dir * 4}; // Offsets for servos
    double phase_diff[4] = {0, 0, 0.5, 0.5};

    oscillateServos(A, O, T, phase_diff, steps);
}

// Crusaito (crossing legs) motion
void robot::crusaito(float steps, int T, int h, int dir) {
    int A[4] = {h, h, 20, 20};          // Amplitudes for servos
    int O[4] = {0, 0, dir * 4, -dir * 4}; // Offsets for servos
    double phase_diff[4] = {0, 0, 0.25, 0.75};

    oscillateServos(A, O, T, phase_diff, steps);
}

// Function to apply trims to the servos
void robot::setTrims(int trim1, int trim2, int trim3, int trim4) {
    servo[0].SetTrim(trim1); // Apply the first trim to the first servo
    servo[1].SetTrim(trim2); // Apply the second trim to the second servo
    servo[2].SetTrim(trim3); // Apply the third trim to the third servo
    servo[3].SetTrim(trim4); // Apply the fourth trim to the fourth servo
}

// Save servo trims to EEPROM
void robot::saveTrimsToEEPROM() {
    for (int i = 0; i < 4; i++) {
        EEPROM.write(i, servo[i].getTrim());  // Save each servo's trim value
    }
    EEPROM.commit();  // Commit the changes to EEPROM
}

// Load servo trims from EEPROM
void robot::loadTrimsFromEEPROM() {
    for (int i = 0; i < 4; i++) {
        servo_trim[i] = EEPROM.read(i);  // Load servo trims from EEPROM
        servo[i].SetTrim(servo_trim[i]);  // Apply the loaded trim values
    }
}
