#include "battery.h"
#include <Arduino.h> // Include Arduino header for pinMode, analogRead, etc.

// Gets the battery ADC value
int battery::Get_Battery_Voltage_ADC(void) {
    pinMode(PIN_BATTERY, INPUT); // Set the pin as input
    int batteryADC = 0;
    for (int i = 0; i < 5; i++) {
        batteryADC += analogRead(PIN_BATTERY); // Read battery voltage
    }
    return batteryADC / 5; // Return the average ADC value
}

// Get the battery voltage value
float battery::Get_Battery_Voltage(void) {
    int batteryADC = Get_Battery_Voltage_ADC();
    batteryVoltage = (batteryADC / 1023.0 * 3.3) * batteryCoefficient;
    return batteryVoltage;
}

// Set the battery coefficient
void battery::Set_Battery_Coefficient(float coefficient) {
    batteryCoefficient = coefficient;
}
