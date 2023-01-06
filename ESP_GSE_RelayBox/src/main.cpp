#include <Arduino.h>

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1, 4, 5);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println('a');
  // Serial1.println('a');
  delay(1000);
}