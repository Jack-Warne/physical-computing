#ifndef BATTERY_H
#define BATTERY_H


#define PIN_BATTERY        28        //Set the battery detection voltage pin
#define LOW_VOLTAGE_VALUE  525       //Set the minimum battery voltage

class battery{
  public:
    int Get_Battery_Voltage_ADC(void);   //Gets the battery ADC value
    float Get_Battery_Voltage(void);     //Get the battery voltage value
    void Set_Battery_Coefficient(float coefficient);//Set the partial pressure coefficient

  private:
    float batteryVoltage = 0;         //Battery voltage variable
    float batteryCoefficient = 3.95;  //Set the proportional coefficient
};


#endif