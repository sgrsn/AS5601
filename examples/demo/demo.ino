#include <AS5601.h>

AS5601_AB encoder;
// AS5601_I2C encoder;

void encoderInterrupt()
{
  encoder.updateEncoderCount();
}

void setup() {
  Serial.begin(9600);
  encoder.init(2, 3, &encoderInterrupt);    // using AS5601_AB class
  // encoder.init();                        // using AS5601_I2C class
}

void loop() {
  
  int count = encoder.getEncoderCount();
  float deg = encoder.getEncoderDegree();
  Serial,print(count);
  Serial.print(", ");
  Serial.println(deg);
}
