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
    void showArray(int delay_ms);

  private:
    unsigned int *_buffer;
    unsigned char  _i2c_addr;
    bool     _vFlipped;
    bool     _hFlipped;
    int      _brightness;

    void writeRow(unsigned char row);
    void setPixel(unsigned char row, unsigned char col, unsigned char val, bool rowDirection);

    byte x_array[2][8] = {
      {0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00},
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    };

    byte y_array[2][8] = {
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x78, 0x84, 0x84, 0x84, 0x78, 0x00, 0x00}
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

void matrixLED::showArray(int delay_ms) {
  int count = sizeof(x_array) / sizeof(x_array[0]);
  for (int i = 0; i < count; i++) {
    showStaticArray(x_array[i], y_array[i]);
    delay(delay_ms);
  }
}

matrixLED matrix;

void setup() {
  matrix.init(0x71);
  matrix.flipVertical();
  matrix.flipHorizontal();
  matrix.setBrightness(5);
  matrix.setBlink(VK16K33_BLINK_OFF);
}

void loop() {
  matrix.showArray(500);
}
