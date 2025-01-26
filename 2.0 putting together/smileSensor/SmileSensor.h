#ifndef SMILESENSOR_H
#define SMILESENSOR_H

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

class SmileSensor {
  public:
    void init(unsigned char addr = 0x71);

    void setBrightness(unsigned char brightness);
    void setBlink(unsigned char blink);
    void clear(void);
    void showStaticArray(byte *array1, byte *array2);
    void showEmotion(const char* emotion);

  private:
    unsigned int *_buffer;
    unsigned char  _i2c_addr;

    void setRow(unsigned char row, unsigned char value);
    void show(void);
};

#endif
