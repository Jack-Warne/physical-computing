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

const int trigPin = 8; // Trig Pin of the Ultrasonic Sensor
const int echoPin = 9; // Echo Pin of the Ultrasonic Sensor

unsigned int _flip_uint16(unsigned int in) {
  unsigned int out = 0;
  for (unsigned char i = 0; i < 16; i++) {
    out <<= 1;
    out |= in & 1;
    in >>= 1;
  }
  return out;
}

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
  x_offset = constrain(x_offset, 0, 8);
  byte left_array[8];
  byte right_array[8];
  for (int i = 0; i < 8; i++) {
    left_array[i] = (array_buffer[i] >> x_offset) & 0xff;
  }
  for (int i = 0; i < 8; i++) {
    right_array[i] = (array_buffer[i] << (8 - x_offset)) & 0xff;
  }
  for (int i = 0; i < 16; i++) {
    setRow(i, 0x00);
  }
  show();
  if (y_offset > 0) {
    y_offset = 8 - abs(y_offset);
    y_offset = constrain(y_offset, 0, 9);
    for (int i = 0; i <= y_offset; i++) {
      setRow(i, left_array[7 - y_offset + i] & 0xff);
      setRow(i + 8, right_array[7 - y_offset + i] & 0xff);
    }
  } else {
    y_offset = 8 - abs(y_offset);
    y_offset = constrain(y_offset, 0, 9);
    for (int i = 0; i < y_offset; i++) {
      setRow(8 - y_offset + i, left_array[i] & 0xff);
      setRow(16 - y_offset + i, right_array[i] & 0xff);
    }
  }
  show();
}

void matrixLED::showArray(int delay_ms, int arrayType) {
  switch (arrayType){
      case 1: 
        count = sizeof(sad_x) / sizeof(sad_x[0]);
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
       
  };
  
}

matrixLED matrix;

void setup() {
  matrix.init(0x71);
  matrix.flipVertical();
  matrix.flipHorizontal();
  matrix.setBrightness(5);
  matrix.setBlink(VK16K33_BLINK_OFF);
  Serial.begin(9600); // Initialize serial communication
  pinMode(trigPin, OUTPUT); // Set the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Set the echoPin as an INPUT
}

void loop() {
  long duration, distance_cm;
  
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  distance_cm = duration * 0.034 / 2; // Speed of sound is 0.034 cm/μs, divide by 2 because signal travels back and forth
  
  // Print the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  if (distance_cm <= 10){
      matrix.showArray(100, 3); // startled
  }
  if (distance_cm > 10 && distance_cm <= 40){
    matrix.showArray(0, 2); // happy
  }
  if (distance > 40){
    matrix.showArray(0, 1); // sad
  }
  delay(1000);  
}
