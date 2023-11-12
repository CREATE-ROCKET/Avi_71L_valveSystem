#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
}

unsigned long lastSendTime = 0;

void loop()
{
  if (millis() - lastSendTime > 1000)
  {
    Serial.write(0xE5);
    lastSendTime = millis();
  }

  if (Serial.available() > 0)
  {
    Serial.write(Serial.read());
  }
}