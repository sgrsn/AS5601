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

  current_time = 0;
  prev_time = 0;
  measureTimeInterval();

  prev_degree = 0;
  current_degree = 0;
}

void AS5601::update()
{
  updateEncoderCount();
  float dt = measureTimeInterval();
  prev_degree = current_degree;
  current_degree = getDegree();
  deg_per_sec_ = (current_degree - prev_degree) / dt;
}

long AS5601::getEncoderCount()
{
  return _count;
}

float AS5601::getDegree() {
  return (float)getEncoderCount() * 360.0/encoder_resolution;
}

float AS5601::measureTimeInterval()
{
  prev_time = current_time;
  current_time = micros();
  float dt = float(current_time - prev_time) / 1000000.0;
  return dt;
}

float AS5601::getDegreePerSeconds()
{
  return deg_per_sec_;
}

void AS5601::setCount(long newCount)
{
  _count = newCount;
}

int AS5601::getEncoderResolution()
{
  return (int)encoder_resolution;
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

void AS5601_AB::Encoder_I2C_init_64(void) {
  // Set AS5601 resolution 2048ppr
  Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
  Wire.write(AS5601_REG_ABN);
  Wire.write(0b00000011);   // ABN(3:0)
  Wire.endTransmission();
  delay(1);
  encoder_resolution = 64;
}

void AS5601_AB::Encoder_I2C_init_8(void) {
  // Set AS5601 resolution 2048ppr
  Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
  Wire.write(AS5601_REG_ABN);
  Wire.write(0b00000000);   // ABN(3:0)
  Wire.endTransmission();
  delay(1);
  encoder_resolution = 8;
}

void AS5601_AB::updateEncoderCount(void)
{  
}

void AS5601_AB::interruptEncoderCount(void)
{
  int sig1 = digitalRead(_pin1);
  int sig2 = digitalRead(_pin2);
  int8_t thisState = sig1 | (sig2 << 1);

  if (_oldState != thisState) {
    _count += EncoderIndexTable[thisState | (_oldState<<2)] * _dir;
    _oldState = thisState;
  }
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

void AS5601_I2C::updateEncoderCount(void)
{
  Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
  _count  = ((uint16_t)Wire.read() << 8) & 0x0F00;
  _count |= (uint16_t)Wire.read();
  _count = _count * _dir;
}
