#include "SmileSensor.h"

unsigned int _flip_uint16(unsigned int in) {
  unsigned int out = 0;
  for (unsigned char i = 0; i < 16; i++) {
    out <<= 1;
    out |= in & 1;
    in >>= 1;
  }
  return out;
}

void SmileSensor::init(unsigned char addr) {
  _i2c_addr = addr;
  _buffer = (unsigned int*)calloc(8, sizeof(unsigned int));
  Wire.begin();
  Wire.beginTransmission(_i2c_addr);
  Wire.write(0x21); // turn it on
  Wire.endTransmission();
  setBlink(VK16K33_BLINK_OFF);
  setBrightness(15);
  clear();
}

void SmileSensor::setBrightness(unsigned char brightness) {
  brightness &= 0x0F;
  Wire.beginTransmission(_i2c_addr);
  Wire.write(VK16K33_CMD_DIMMING | brightness);
  Wire.endTransmission();
}

void SmileSensor::setBlink(unsigned char blink) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(VK16K33_CMD_SETUP | VK16K33_DISPLAY_ON | blink);
  Wire.endTransmission();
}

void SmileSensor::clear(void) {
  for (unsigned char i = 0; i < 8; i++) {
    _buffer[i] = 0;
  }
  show();
}

void SmileSensor::setRow(unsigned char row, unsigned char value) {
  _buffer[row] = value;
}

void SmileSensor::show(void) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(VK16K33_CMD_RAM);
  for (unsigned char row = 0; row < 8; row++) {
    unsigned int out = _buffer[row];
    Wire.write(out & 0xFF);
    Wire.write(out >> 8);
  }
  Wire.endTransmission();
}

void SmileSensor::showStaticArray(byte *array1, byte *array2) {
  // Display for the LEFT matrix
  Wire.beginTransmission(_i2c_addr); // LEFT matrix I2C address
  Wire.write(VK16K33_CMD_RAM);
  for (int i = 0; i < 8; i++) {
    Wire.write(array1[i] & 0xFF);      // Send row data for LEFT matrix
    Wire.write((array1[i] >> 8) & 0xFF); // Ensure all 16 bits are written
  }
  Wire.endTransmission();

  // Display for the RIGHT matrix
  Wire.beginTransmission(_i2c_addr + 1); // RIGHT matrix I2C address
  Wire.write(VK16K33_CMD_RAM);
  for (int i = 0; i < 8; i++) {
    Wire.write(array2[i] & 0xFF);      // Send row data for RIGHT matrix
    Wire.write((array2[i] >> 8) & 0xFF); // Ensure all 16 bits are written
  }
  Wire.endTransmission();
}

void SmileSensor::showEmotion(const char* emotion) {
  // Define hexadecimal arrays for emotions
  static const byte SMILE_LEFT[8]  = {0x3C, 0x42, 0xA5, 0x81, 0x81, 0xA5, 0x42, 0x3C};
  static const byte SMILE_RIGHT[8] = {0x3C, 0x42, 0xA5, 0x81, 0x81, 0xA5, 0x42, 0x3C};
  
  static const byte SAD_LEFT[8]  = {0x3C, 0x42, 0x81, 0xA5, 0xA5, 0x81, 0x42, 0x3C};
  static const byte SAD_RIGHT[8] = {0x3C, 0x42, 0x81, 0xA5, 0xA5, 0x81, 0x42, 0x3C};

  static const byte STARTLED_LEFT[9][8]  = {
      {0x08,0x14,0x14,0x22,0x22,0x41,0x41,0x80},
      {0x04,0xA,0xA,0x11,0x11,0xA0,0xA0,0x40},
      {0x02,0x05,0x05,0x88,0x88,0x50,0x50,0x20},
      {0x01,0x82,0x82,0x44,0x44,0x28,0x28,0x10},
      {0x80,0x41,0x41,0x22,0x22,0x14,0x14,0x08},
      {0x40,0xA0,0xA0,0x11,0x11,0xA,0xA,0x04},
      {0x20,0x50,0x50,0x88,0x88,0x05,0x05,0x02},
      {0x10,0x28,0x28,0x44,0x44,0x82,0x82,0x01}
  };
  static const byte STARTLED_RIGHT[9][8] = {
      {0x08,0x14,0x14,0x22,0x22,0x41,0x41,0x80},
      {0x04,0xA,0xA,0x11,0x11,0xA0,0xA0,0x40},
      {0x02,0x05,0x05,0x88,0x88,0x50,0x50,0x20},
      {0x01,0x82,0x82,0x44,0x44,0x28,0x28,0x10},
      {0x80,0x41,0x41,0x22,0x22,0x14,0x14,0x08},
      {0x40,0xA0,0xA0,0x11,0x11,0xA,0xA,0x04},
      {0x20,0x50,0x50,0x88,0x88,0x05,0x05,0x02},
      {0x10,0x28,0x28,0x44,0x44,0x82,0x82,0x01}
  };

  if (strcmp(emotion, "SMILE") == 0) {
    showStaticArray((byte*)SMILE_LEFT, (byte*)SMILE_RIGHT);
  } else if (strcmp(emotion, "SAD") == 0) {
    showStaticArray((byte*)SAD_LEFT, (byte*)SAD_RIGHT);
  } else if (strcmp(emotion, "STARTLED") == 0) {
    showStaticArray((byte*)STARTLED_LEFT, (byte*)STARTLED_RIGHT);
  }
}


