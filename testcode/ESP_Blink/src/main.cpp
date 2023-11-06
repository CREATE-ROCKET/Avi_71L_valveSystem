#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN 14
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  pixels.begin();
  pixels.clear();
  Serial.begin(115200);
}

void changeColor(int r, int g, int b)
{
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

void loop()
{
  for (int i = 0; i < 3; i++)
  {
    uint32_t time_start = micros();
    changeColor(i % 3, (i + 1) % 3, (i + 2) % 3);
    uint32_t time_end = micros();
    Serial.printf("Time: %d\r\n", time_end - time_start); // 197us
    delay(1000);
  }
}