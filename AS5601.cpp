#include "Arduino.h"
#include "AS5601.h"

#include <stdint.h>
#include <Wire.h>

const int8_t EncoderIndexTable[] = {0, -1,  1,  0, 1,  0,  0, -1, -1,  0,  0,  1, 0,  1, -1,  0  };

AS5601::AS5601()
{
  _oldState = 3;
  _count = 0;
  _dir = (int)EncoderDirection::CW;
}

float  AS5601::getEncoderDegree() {
  return (float)getEncoderCount() * 360.0/encoder_resolution;
}

void AS5601::setDirection(EncoderDirection dir)
{
  _dir = (int)dir;
}

/*using AB***************************************************************************************/
void AS5601_AB::init(int pin1, int pin2, void (*fptr)())
{
  encoder_resolution = AS5601_RESOLUTION_AB;
  // I2C init
  Wire.begin();
  Wire.setClock(400000);

  // Peripheral init
  Encoder_I2C_init();
  Encoder_GPIO_init(pin1, pin2, fptr);
}

void AS5601_AB::Encoder_GPIO_init(int pin1, int pin2, void (*fptr)()) {
  _pin1 = pin1;
  _pin2 = pin2;
  pinMode(_pin1, INPUT);
  pinMode(_pin2, INPUT);
  digitalWrite(_pin1, INPUT_PULLUP);
  digitalWrite(_pin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_pin1), fptr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pin2), fptr, CHANGE);
}

void AS5601_AB::Encoder_I2C_init(void) {
  // Set AS5601 resolution 2048ppr
  Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
  Wire.write(AS5601_REG_ABN);
  Wire.write(0b00001000);   // ABN(3:0)
  Wire.endTransmission();
  delay(1);
}

void AS5601_AB::updateEncoderCount(void)
{
  int sig1 = digitalRead(_pin1);
  int sig2 = digitalRead(_pin2);
  int8_t thisState = sig1 | (sig2 << 1);

  if (_oldState != thisState) {
    _count += EncoderIndexTable[thisState | (_oldState<<2)] * _dir;
    _oldState = thisState;
  }
}

long  AS5601_AB::getEncoderCount() {
  return _count;
}

/*using i2c*******************************************************************************************/
void AS5601_I2C::init()
{
  encoder_resolution = AS5601_RESOLUTION_I2C;
  // I2C init
  Wire.begin();
  Wire.setClock(400000);

  // Peripheral init
  Encoder_I2C_init();
}

void AS5601_I2C::Encoder_I2C_init(void) {
  Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
  Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
  Wire.endTransmission(false);
}

long  AS5601_I2C::getEncoderCount() {
  Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
  
  _count  = ((uint16_t)Wire.read() << 8) & 0x0F00;
  _count |= (uint16_t)Wire.read();

  return _count * _dir;
}
