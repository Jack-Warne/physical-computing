#ifndef BUZZER_H
#define BUZZER_H

#define PIN_BUZZER 2                    //Define the pins for the Pico W control buzzer
#define BUZZER_FREQUENCY 2000
class buzzer{
  public:
    //Define the resonant frequency of the buzzer 
    void Buzzer_Setup(void);                //Buzzer initialization
    void Buzzer_Alert(int beat, int rebeat);//Buzzer alarm function
    void freq(int PIN, int freqs, int times);
};          
#endif