#ifndef AS5601_H
#define AS5601_H
#include "Arduino.h"

#define AS5600_AS5601_DEV_ADDRESS       0x36
#define AS5601_REG_ABN                  0x09

#define AS5600_AS5601_REG_RAW_ANGLE    0x0C

#define AS5601_RESOLUTION_AB            2048
#define AS5601_RESOLUTION_I2C           4096

enum class EncoderDirection
{
  CW = 1,
  CCW = -1,
};

class AS5601
{
  public:
    AS5601();
    float getEncoderDegree();
    void setPosition(long newPosition);
    void updateEncoderCount(void);

    // set CW or CCW
    void setDirection(EncoderDirection dir);
    
    virtual long getEncoderCount();
    
  protected:
    int _pin1, _pin2;
    int _dir;
    int8_t _oldState;
    long _count;
    float encoder_resolution;
};

class AS5601_AB : public AS5601
{
  public:
  void Encoder_GPIO_init(int pin1, int pin2, void (*fptr)());
  void Encoder_I2C_init(void);
  void init(int pin1, int pin2, void (*fptr)());
  void updateEncoderCount(void);
  long getEncoderCount();
};

class AS5601_I2C : public AS5601
{
  public:
  void Encoder_I2C_init(void);
  void init();
  long getEncoderCount();
};

#endif
