#include "Buzzer.h"
#include <Arduino.h> // Include Arduino header for pinMode, delay, digitalWrite, etc.

void buzzer::Buzzer_Setup(void) {
    pinMode(PIN_BUZZER, OUTPUT); // Set buzzer pin as output
}

// Buzzer alarm function
void buzzer::Buzzer_Alert(int beat, int rebeat) {
    beat = constrain(beat, 1, 9);   // Limit beat value between 1 and 9
    rebeat = constrain(rebeat, 1, 255); // Limit rebeat value between 1 and 255

    for (int j = 0; j < rebeat; j++) {
        for (int i = 0; i < beat; i++) {
            freq(PIN_BUZZER, BUZZER_FREQUENCY, 30); // Generate buzzer sound
        }
        delay(500); // Pause between sequences
    }
    freq(PIN_BUZZER, 0, 30); // Turn off the buzzer
}

// Function to generate a frequency on the buzzer
void buzzer::freq(int PIN, int freqs, int times) {
    if (freqs == 0) {
        digitalWrite(PIN, LOW); // Turn off the buzzer
    } else {
        for (int i = 0; i < times * freqs / 500; i++) {
            digitalWrite(PIN, HIGH); // Generate HIGH signal
            delayMicroseconds(500000 / freqs);
            digitalWrite(PIN, LOW);  // Generate LOW signal
            delayMicroseconds(500000 / freqs);
        }
    }
}
