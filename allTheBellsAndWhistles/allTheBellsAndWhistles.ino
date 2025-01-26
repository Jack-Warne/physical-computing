#include <LittleFS.h>

#include "MatrixLED.h"
#include "UltrasonicSensor.h"
#include "EEPROM.h"
#include "Arduino.h"
#include "robot.h"
#include "battery.h"
#include "Buzzer.h"
#include "AudioFileSourceLittleFS.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"
#include "AudioFileSourceID3.h"

AudioGeneratorMP3 *mp3;
AudioFileSourceLittleFS *file;
AudioOutputI2SNoDAC *out;


// Define pins for the ultrasonic sensor
const int trigPin = 8;
const int echoPin = 9;

// Define servo pins for the robot
const int YL = 10, YR = 11, RL = 12, RR = 13;

int a = 0;

// Create objects for the LED matrix, ultrasonic sensor, and robot
matrixLED matrix;
ultrasonicSensor ultrasonicSensor(trigPin, echoPin);
robot robot;
buzzer buzzer;
battery battery;

void setup() {
    // Initialise the LED matrix
    matrix.init(0x71);
    matrix.flipVertical();
    matrix.flipHorizontal();
    matrix.setBrightness(5);
    matrix.setBlink(VK16K33_BLINK_OFF);

    // initialise the buzzer
    buzzer.Buzzer_Setup();

    // Initialise the ultrasonic sensor
    ultrasonicSensor.begin();

    // Initialise the robot
    robot.init(YL, YR, RL, RR);
    robot.setTrims(0, 0, 0, 0);
    robot.attachServos();

    //audio stuff
    file = new AudioFileSourceLittleFS("number_song.mp3");
    out = new AudioOutputI2SNoDAC(6);
    out->SetGain(3);  //Volume Setup
    mp3 = new AudioGeneratorMP3();
    mp3->begin(file, out);

    // Start Serial communication
    Serial.begin(9600);
}

void loop() {
    // Get the distance in cm
    long distanceCM = ultrasonicSensor.getDistanceCM();

    if (battery.Get_Battery_Voltage() < 0){
      buzzer.Buzzer_Alert(3,4);
    };

    // Respond based on distance
    if (distanceCM <= 10) {
        // Startled: Show startled face and turn around
        matrix.showArray(100, 3); // Startled face
        robot.turn(2, 1500, LEFT); // Turn left
        
    }  
    if (distanceCM > 10 && distanceCM <= 40) {
        // Happy: Show happy face and dance
        matrix.showArray(0, 2); // Happy face
        robot.crusaito(2, 1000, 20, FORWARD); // Crusaito dance
        Serial.printf("loop1..\n");
        if (mp3->isRunning()) {
          if (!mp3->loop()) {
            mp3->stop();
            delete file;
            delete mp3;
            mp3 = new AudioGeneratorMP3();

          }
        } else {
          Serial.printf("MP3 done\n");
          pinMode(6, OUTPUT);
          digitalWrite(6, LOW);
        }
    } 
    if (distanceCM > 40) {
        // Sad: Show sad face and walk forward
        matrix.showArray(0, 1); // Sad face
        robot.walk(4, 1500, FORWARD); // Walk forward
       
    }

    // Small delay before the next loop
    //delay(99);
}
