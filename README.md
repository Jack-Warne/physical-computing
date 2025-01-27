Physical Computing
Introduction
The purpose of this document is to monitor the progress of the creation of a children’s toy exoskeleton. Each stage of development is in loose order including experiments and videos to test the electronics used in development. 
For this project I will be using a Raspberry Pi Pico W, the reason for this is that it is compact and has extra capabilities that an Arduino doesn’t have such as Bluetooth and wifi capabilities.
I want to build a walking robot for the purpose of creating an enrichment activity for early years children. The Robot will have stimulating features to keep children engaged as well as having bonus features to make the product more robust and durable. 
Contents
Introduction	1
Individual Components Testing	2
Introduction	2
Servo Motors	2
Buzzer	4
Ultrasonic Sensor	7
Battery	11
Built in Speaker	13
LED Matrices	13
Final Product	14
Smile and Sensor	14
Sources	20
References	20

 
Individual Components Testing
Introduction
This will cover the individual electronic parts of the robot along with video and code evidence as to how they work and will be integrated into the final product. 
Servo Motors
A servo motor is a specialised motor designed to provide precise control of position, velocity, and acceleration. Unlike standard motors, servo motors have built-in feedback systems (usually a rotary encoder or potentiometer) that allow them to adjust their motion based on real-time input, ensuring high accuracy and repeatability. They consist of three main parts:
	Motor: Typically, a DC or AC motor.
	Controller: to manage the motor's position, speed, and direction based on input signals.
	Feedback Mechanism: to provide real-time position data to the controller.
In the computing and electronics industry they are used for:
	Robotics:
	Servo motors are widely used in robotic arms, legs, and grippers to provide precise movements.
	They enable humanoid robots and industrial robots to perform tasks with a high degree of accuracy.
	Automation and Manufacturing:
	Used in CNC machines, 3D printers, and other automated machinery for precision machining and positioning.
	Key components in conveyor systems for item sorting and placement.
	Aerospace and Defence:
	Employed in drones and unmanned aerial vehicles (UAVs) for stabilising controls, such as wing flaps and camera gimbals.
	Used in missile guidance systems and military-grade robotic systems.
	Consumer Electronics:
	Found in optical disc drives (CD/DVD players) for precise laser positioning.
	Used in camera autofocus systems to adjust lens positions smoothly.
	Automotive Applications:
	Used in electronic throttle control, power windows, and adaptive cruise control systems.
	Servo motors are essential in advanced driver-assistance systems (ADAS) for steering and braking automation.
	Medical Equipment:
	Found in surgical robots and prosthetics for precise, controlled movements.
	Used in diagnostic machines, such as MRI scanners and automated sample handlers.

Servo motors are often controlled using Pulse Width Modulation (PWM) signals from microcontrollers. The PWM signal determines the angle or position of the motor shaft. They are very good at precision movements which benefits my project as it is a walking robot and to be able to walk it needs to use precise movements to prevent toppling over.
Servo Motors consist of a DC motor, gears to provide torque, a sensor and a control board. The servos that I am using only have a 180 degree range. They have three wires, 2-VCC (red wire) for positive and the GND for negative and the 1-signal (orange wire) as the signal line. 
An initial experiment using a breadboard and a potentiometer rather than a control panel (below (remember to include video!)) 
The servos receive a 50hz PWM signal with a duty cycle to determine its rotation level.

For the purpose of, my project the servos are connect to GPIOS10-13 then through the control board they are grounded and connected to 5V for power.
Due to servo oscillation, there is an extra header and cpp file used for fluid movement as it will stop the program from crashing. It is important to note that I did not code the Oscillator Library I found it and use it in my code, the credit goes to Juan Gonzalez-Gomez (Obijuan), Dec 2011.

 
Buzzer
A buzzer is a small audio signalling device commonly used in electronics to produce sound. It works by converting electrical energy into mechanical energy, creating vibrations that produce audible tones. Buzzers are typically classified into two main types:
	Piezoelectric Buzzers: These use a piezoelectric element that vibrates when a voltage is applied, producing sound.
	Electromagnetic Buzzers: These use an electromagnet to move a diaphragm to generate sound.
Buzzers are compact, energy-efficient, and available in a range of sound levels and tones, making them versatile for different applications.
Commercial Uses of Buzzers in Computing/Electronics
	Alert Systems:
	Used in alarms (e.g., fire alarms, smoke detectors, intruder alarms) to emit a loud alert tone when triggered.
	Often integrated into household or industrial security systems.
	User Interface Feedback:
	Found in devices like microwave ovens, washing machines, and ATMs to provide audio confirmation for user inputs.
	Commonly used in computing devices (e.g., laptops or motherboards) for system error codes or boot diagnostics (POST beeps).
	Timers and Notifications:
	Used in consumer products such as egg timers, digital clocks, or countdown timers to indicate when time is up.
	Integrated into commercial electronics to notify workers in automated processes or assembly lines.
	Embedded Systems:
	Utilised in microcontroller-based projects to alert users to system statuses, such as successful or failed operations.
	In industrial applications, they provide alerts in machines or control panels.
	Medical Devices:
	Found in hospital equipment, such as heart monitors and infusion pumps, to alert medical professionals of vital changes.
	Automotive Applications:
	Used in cars for seatbelt warnings, reverse parking sensors, and indicator sounds.
For the purpose of my product I will use it to signal the battery is low. 
 
Code:
Buzzer.ino:
#include "Arduino.h"
#include "buzzer.h"

buzzer buzzer;

void setup() {
  // put your setup code here, to run once:
  buzzer.Buzzer_Setup();
}

void loop() {
    // put your main code here, to run repeatedly:
    buzzer.Buzzer_Alert(3,4);
    delay(1000);
}


Buzzer.h:
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
 
Buzzer.cpp:
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

 
Ultrasonic Sensor
An ultrasonic sensor is an electronic component used to measure distance or detect objects by emitting ultrasonic sound waves and interpreting the reflected waves. These sound waves are typically beyond the range of human hearing (above 20 kHz). Ultrasonic sensors consist of two main components:
	Transmitter: Emits ultrasonic sound waves.
	Receiver: Detects the reflected waves (echo) and measures the time it takes for the waves to return.
An ultrasonic sensor works by sending out ultrasonic soundwaves and then measuring the time it takes for them to return to the sensor. Speed (S) = Distance (D) divided by Time(T)
S=D÷T
If you know the Speed of the wave and the Time it took for the wave to travel the distance, divided by two to account for there and back then you can rearrange it to:
S×T/2=D
These parameters can be set up to measure the robot’s distance from objects surrounding it and tell the robot that it needs to turn around to avoid a collision.
A raspberry pi pico can have the sensor wired straight in however with some other raspberry pi you need to use a potential divider to get the voltage down to 3.3V instead of 5V
The diagrams below use a raspberry pi model B and python to use the ultrasonic sensor.

The ultrasonic sensor is used in my project to measure distances of the robot from other objects, this helps prevent it walking into objects and it is also used to see if something is near it. 
 
Code:
ultrasonicSensor.ino
#include "UltrasonicSensor.h"

// Define pins for the ultrasonic sensor
const int trigPin = 8;
const int echoPin = 9;

ultrasonicSensor ultrasonicSensor(trigPin, echoPin);

void setup() {
  // Initialise the ultrasonic sensor
  ultrasonicSensor.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Get the distance in cm
  long distanceCM = ultrasonicSensor.getDistanceCM();
  Serial.print("the distance is: ");
  Serial.print(distanceCM);
  Serial.printf("cm \n");
  delay(100);
}


 
UltrasonicSensor.h
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


UltrasonicSensor.cpp
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

 
Battery
The battery is wired in to the board that the Pico is attached to but there is code that can be used to check the battery output to make sure there is sufficient charge. To do this we take the voltage reading that the battery is output. This can be used to make a noise when the battery is low so that users know when to change it or to use the built in LED to give an indication of the battery level. 
This is commercially viable as it gives users a warning when their product needs new batteries, this can be important for some users that depend on the product as it will give them some lead time to get a new battery or recharge the current battery.
 
Code
Battery.ino:
#include "Arduino.h"
#include "battery.h"

battery battery;

void setup() {
  // put your setup code here, to run once:
}
void loop() {
  Serial.print("battery Voltage is: ");
  Serial.print(battery.Get_Battery_Voltage());
  Serial.printf("V \n");
  delay(1000);
}

Battery.h:
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
Battery.cpp:
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

Built in Speaker
A speaker is a compact audio output device designed to convert electrical signals into sound waves. The speaker I am using is lightweight, portable, and has a lower power consumption compared to larger, full-range speakers. 
Small speakers use principles of electromagnetism, where a voice coil interacts with a magnet to vibrate a diaphragm, producing sound.
For the purpose of my project the speaker is to play a nice tune to have the children play along to and to increase the interactivity of the product.
LED Matrices
An LED matrix is a grid of Light Emitting Diodes (LEDs) arranged in rows and columns, used to display text, images, patterns, or animations. It works by controlling individual LEDs within the matrix through electrical signals. The LEDs can either be monochromatic or multi-coloured, allowing for vibrant and dynamic displays. Typically, LED matrices are controlled using microcontrollers, driver ICs, or multiplexing techniques to address specific rows and columns, thereby lighting up selected LEDs at a time.
There are two 8 by 8 matrices used in the bipedal robot, creating a 16 by 8 grid of LEDs. To reduce the number of ports a common anode is used, this is where the positive poles and the negative poles of each column are respectively connected together inside the LED matrix module.
For these matrices the anode of LED matrix is connected to ROWx of HT16K33 chip, and the cathode is connected to COMx. The address of HT16K33 chip is (0x70+[A2:A0]), hexadecimal is used to determine which lights to turn on and off and these can be controlled by creating a list of x and y coordinates as to which light needs to turn on.
 
(Newbiely tutorials, ND)
 
Final Product
Matrix LED patterns
The first step was to get the ultrasonic sensor and the LED matrix to work in unison, which is what the smileSensor code does. It takes code from my ultrasonic sensor library and my LED matrix library and combines together, I later decide to keep them separate as it makes the code easier to digest and they don’t need to be in one file. However SmileSensor was not working well for me so I refactored it to use a switch statement which worked much better than trying to parse a string through the statement. The smileWorking fixes this and runs much better. I also moved the declaration of the matrix patterns to matrixLED.h file rather than trying to assign them later on.
Code
matrixLED.cpp
#include "matrixLED.h"

unsigned int _flip_uint16(unsigned int in) {
  unsigned int out = 0;
  for (unsigned char i = 0; i < 16; i++) {
    out <<= 1;
    out |= in & 1;
    in >>= 1;
  }
  return out;
}

void matrixLED::init(unsigned char addr) {
  _vFlipped = false;
  _hFlipped = false;
  _i2c_addr = addr;
  _buffer = (unsigned int*)calloc(8, sizeof(unsigned int));
  Wire.begin();
  Wire.beginTransmission(_i2c_addr);
  Wire.write(0x21); // turn it on
  Wire.endTransmission();
  setBlink(VK16K33_BLINK_OFF);
  setBrightness(15);
  show();
}

void matrixLED::setBrightness(unsigned char brightness) {
  brightness = brightness & 0x0F;
  Wire.beginTransmission(_i2c_addr);
  Wire.write(VK16K33_CMD_DIMMING | brightness);
  Wire.endTransmission();
}

void matrixLED::setBlink(unsigned char blink) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(VK16K33_CMD_SETUP | VK16K33_DISPLAY_ON | blink);
  Wire.endTransmission();
}

void matrixLED::flipVertical(void) {
  _vFlipped = !_vFlipped;
}

void matrixLED::flipHorizontal(void) {
  _hFlipped = !_hFlipped;
}

void matrixLED::clear(void) {
  for (unsigned char i = 0; i < 8; i++) {
    _buffer[i] = 0;
  }
}

void matrixLED::setPixel(unsigned char row, unsigned char col, unsigned char val, bool rowDirection) {
  col = col & 0x07;
  row = row & 0x0F;
  val = val & 0x01;
  if (rowDirection == 0) {
    if (val == 1) {
      _buffer[7 - col] |= 1 << (row);
    } else {
      _buffer[7 - col] &= ~(1 << (row));
    }
  } else {
    if (val == 1) {
      _buffer[col] |= 1 << (row);
    } else {
      _buffer[col] &= ~(1 << (row));
    }
  }
}

void matrixLED::setRow(unsigned char row, unsigned char value, bool rowDirection) {
  for (unsigned char col = 0; col < 8; col++) {
    setPixel(row, col, (value & (1 << col)) > 0, rowDirection);
  }
}

void matrixLED::writeRow(unsigned char row) {
  if (_hFlipped) {
    row = 7 - row;
  }
  unsigned int out = _buffer[row];
  if (_vFlipped) {
    out = _flip_uint16(out);
  }
  Wire.write(out & 0xFF);
  Wire.write(out >> 8);
}

void matrixLED::show(void) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(VK16K33_CMD_RAM);
  for (unsigned char row = 0; row < 8; row++) {
    writeRow(row);
  }
  Wire.endTransmission();
}

void matrixLED::showStaticArray(byte *array1, byte *array2) {
  for (int i = 0; i < 8; i++) {
    setRow(i, array1[i] & 0xff);
  }
  for (int i = 8; i < 16; i++) {
    setRow(i, array2[i - 8] & 0xff);
  }
  show();
}

void matrixLED::showLedMatrix(byte array[8][8], int x_offset, int y_offset) {
  byte array_buffer[8] = {0};
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      if (array[i][j] != 0) {
        array_buffer[i] |= 1 << (7 - j);
        _brightness = array[i][j];
      }
    }
  }
  _brightness = constrain(_brightness, 1, 15);
  setBrightness(_brightness);
}

void matrixLED::showArray(int delay_ms, int arrayType) {
  switch (arrayType){
      case 1: 
        count = sizeof(sad_x) / sizeof(sad_x[0]);
        for (int i = 0; i < count; i++) {
          showStaticArray(sad_x[i], sad_y[i]);
          delay(delay_ms);
        };
        for (int i = 0; i < count; i++) {
          showStaticArray(sad_x[i], sad_y[i]);
          delay(delay_ms);
        };
        break;
      case 2: 
        count = sizeof(happy_x) / sizeof(happy_x[0]);
        for (int i = 0; i < count; i++) {
          showStaticArray(happy_x[i], happy_y[i]);
          delay(delay_ms);
        };
        break;
      case 3: 
        count = sizeof(startled_x) / sizeof(startled_x[0]);
        for (int i = 0; i < count; i++) {
          showStaticArray(startled_x[i], startled_y[i]);
          delay(delay_ms);
        };
        break;
  }
}
}

 
matrixLED.h
#ifndef MATRIXLED_H
#define MATRIXLED_H

#include <Wire.h>

#define VK16K33_CMD_RAM     0x00
#define VK16K33_CMD_SETUP   0x80
#define VK16K33_CMD_DIMMING 0xE0

#define VK16K33_DISPLAY_OFF 0x00
#define VK16K33_DISPLAY_ON  0x01
#define VK16K33_BLINK_OFF   0x00
#define VK16K33_BLINK_1HZ   0x02
#define VK16K33_BLINK_2HZ   0x04
#define VK16K33_BLINK_0HZ5  0x06

class matrixLED {
  public:
    void init(unsigned char addr = 0x71);
    void setBrightness(unsigned char brightness);
    void setBlink(unsigned char blink);
    void flipVertical(void);
    void flipHorizontal(void);
    void clear(void);
    void setRow(unsigned char row, unsigned char value, bool rowDirection = 0);
    void show(void);
    void showStaticArray(byte *array1, byte *array2);
    void showLedMatrix(byte array[8][8], int x_offset = 4, int y_offset = 4);
    void showArray(int delay_ms, int arrayType);

  private:
    unsigned int *_buffer;
    unsigned char  _i2c_addr;
    bool     _vFlipped;
    bool     _hFlipped;
    int      _brightness;
    int count;

    void writeRow(unsigned char row);
    void setPixel(unsigned char row, unsigned char col, unsigned char val, bool rowDirection);
    
    // Predefined patterns
    byte startled_x[8][8] = {
      {0x08,0x14,0x14,0x22,0x22,0x41,0x41,0x80},
      {0x04,0xA,0xA,0x11,0x11,0xA0,0xA0,0x40},
      {0x02,0x05,0x05,0x88,0x88,0x50,0x50,0x20},
      {0x01,0x82,0x82,0x44,0x44,0x28,0x28,0x10},
      {0x80,0x41,0x41,0x22,0x22,0x14,0x14,0x08},
      {0x40,0xA0,0xA0,0x11,0x11,0xA,0xA,0x04},
      {0x20,0x50,0x50,0x88,0x88,0x05,0x05,0x02},
      {0x10,0x28,0x28,0x44,0x44,0x82,0x82,0x01}
    };
    byte startled_y[8][8] = {
      {0x08,0x14,0x14,0x22,0x22,0x41,0x41,0x80},
      {0x04,0xA,0xA,0x11,0x11,0xA0,0xA0,0x40},
      {0x02,0x05,0x05,0x88,0x88,0x50,0x50,0x20},
      {0x01,0x82,0x82,0x44,0x44,0x28,0x28,0x10},
      {0x80,0x41,0x41,0x22,0x22,0x14,0x14,0x08},
      {0x40,0xA0,0xA0,0x11,0x11,0xA,0xA,0x04},
      {0x20,0x50,0x50,0x88,0x88,0x05,0x05,0x02},
      {0x10,0x28,0x28,0x44,0x44,0x82,0x82,0x01}
    };

    // Flexible arrays moved to the end
    byte happy_x[1][8] = {
      {0x00,0x60,0x60,0x30,0x30,0x18,0x1F,0x00}
    };
    byte happy_y[1][8] = {
      {0x00,0x06,0x06,0xC,0xC,0x18,0xF8,0x00}
    };
    byte sad_x[1][8] = {
      {0x00,0x07,0x1F,0x18,0x30,0x30,0x60,0x60}
    };
    byte sad_y[1][8] = {
      {0x00,0xE0,0xF8,0x18,0xC,0xC,0x06,0x06}
    };
};

#endif

 
smileSensor.ino
#include "matrixLED.h"

const int trigPin = 8; // Trig Pin of the Ultrasonic Sensor
const int echoPin = 9; // Echo Pin of the Ultrasonic Sensor
matrixLED matrix;

void setup() {
  matrix.init(0x71);
  matrix.flipVertical();
  matrix.flipHorizontal();
  matrix.setBrightness(5);
  matrix.setBlink(VK16K33_BLINK_OFF);
  Serial.begin(9600); // Initialise serial communication
  pinMode(trigPin, OUTPUT); // Set the trigPin as OUTPUT
  pinMode(echoPin, INPUT);  // Set the echoPin as INPUT
}

void loop() {
  long duration, distance_cm;

  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the echoPin
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance_cm = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  // Display based on distance
  if (distance_cm <= 10) {
    matrix.showArray(100, 3); // Startled
  } 
  if (distance_cm > 10 && distance_cm <= 40) {
    matrix.showArray(0, 2); // Happy
  } 
  if (distance_cm > 40){
    matrix.showArray(0, 1); // Sad
  }
  delay(99);
}
Integrating walking
The next stage was to introduce the walking patterns which can be added to the if statements to have the robot’s behaviour reflect the distance. This was completed in the walkingEmotion folder, where my robot library and the Oscillator library were integrated to use the three different walking methods, forwards left and crusiato. This was done to further exaggerate the emotions of the robot to make it easier for a child to understand how the robot was feeling. 
Code:
walkingEmotion.ino
#include "MatrixLED.h"
#include "UltrasonicSensor.h"
#include "EEPROM.h"
#include "Arduino.h"
#include "robot.h"

// Define pins for the ultrasonic sensor
const int trigPin = 8;
const int echoPin = 9;

// Define servo pins for the robot
const int YL = 10, YR = 11, RL = 12, RR = 13;

// Create objects for the LED matrix, ultrasonic sensor, and robot
matrixLED matrix;
ultrasonicSensor ultrasonicSensor(trigPin, echoPin);
robot robot;

void setup() {
    // Initialise the LED matrix
    matrix.init(0x71);
    matrix.flipVertical();
    matrix.flipHorizontal();
    matrix.setBrightness(5);
    matrix.setBlink(VK16K33_BLINK_OFF);

    // Initialise the ultrasonic sensor
    ultrasonicSensor.begin();

    // Initialise the robot
    robot.init(YL, YR, RL, RR);
    robot.setTrims(0, 0, 0, 0);
    robot.attachServos();

    // Start Serial communication
    Serial.begin(9600);
}

void loop() {
    // Get the distance in cm
    long distanceCM = ultrasonicSensor.getDistanceCM();

    // Respond based on distance
    if (distanceCM <= 10) {
        // Startled: Show startled face and turn around
        matrix.showArray(100, 3); // Startled face
        robot.turn(2, 1500, LEFT); // Turn left
        delay(1000);
    }  
    else if (distanceCM > 10 && distanceCM <= 40) {
        // Happy: Show happy face and dance
        matrix.showArray(0, 2); // Happy face
        robot.crusaito(2, 1000, 20, FORWARD); // Crusaito dance
        delay(1000);
    } 
    else if (distanceCM > 40) {
        // Sad: Show sad face and walk forward
        matrix.showArray(0, 1); // Sad face
        robot.walk(4, 1500, FORWARD); // Walk forward
        delay(1000);
    }

    // Small delay before the next loop
    delay(99);
}


robot.h:
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


 
Robot.cpp
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


 
Battery functionality
It is important to be able to check the voltage of the batteries used in the robot and have an early warning system for when they are running low so users know when to switch them, the code takes a battery reading and then sounds the buzzer when the battery gets too low. It integrates the previously mentioned code plus the battery library I created and the buzzer library I created. 
Code:
WEBattery.ino:
#include "MatrixLED.h"
#include "UltrasonicSensor.h"
#include "EEPROM.h"
#include "Arduino.h"
#include "robot.h"
#include "battery.h"
#include "Buzzer.h"

// Define pins for the ultrasonic sensor
const int trigPin = 8;
const int echoPin = 9;

// Define servo pins for the robot
const int YL = 10, YR = 11, RL = 12, RR = 13;

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
    else if (distanceCM > 10 && distanceCM <= 40) {
        // Happy: Show happy face and dance
        matrix.showArray(0, 2); // Happy face
        robot.crusaito(2, 1000, 20, FORWARD); // Crusaito dance
        
    } 
    else if (distanceCM > 40) {
        // Sad: Show sad face and walk forward
        matrix.showArray(0, 1); // Sad face
        robot.walk(4, 1500, FORWARD); // Walk forward
       
    }

    // Small delay before the next loop
    delay(99);
}


The Final Product
The final product integrates all the components together to create an emotive walking robot that sings and dances, with the purpose of entertaining small children and teaching them about making friends and personal space, this is achieved using an ultrasonic sensor which uses distances to determine whether it has found a friend or whether its personal space has been invaded and show how that is wrong and that personal space needs to be respected. The final addition is the speaker which plays a nice tune. I did not create a library for this as it seemed redundant as it only takes a few lines of code to get a song to play through the speaker.
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

 
Sources
GitHub Repository:
A copy of this document will also be in the REPO as the readme.md
 https://github.com/Jack-Warne/physical-computing.git
Oscillator Library: https://github.com/bq/robopad/blob/master/Arduino/robopad_plusplus_crab_droid_arduino/Oscillator_Lib/Oscillator/Oscillator.h 
Number song:
https://pixabay.com/music/happy-childrens-tunes-number-song-233460/ 
References

