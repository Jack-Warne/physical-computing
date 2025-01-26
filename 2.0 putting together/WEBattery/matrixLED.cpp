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
