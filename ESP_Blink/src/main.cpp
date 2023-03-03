#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN 32
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  pixels.begin();
  pixels.clear();
  Serial.begin(115200);
}

void loop()
{
  Serial.print("r");
  pixels.setPixelColor(0, pixels.Color(20, 0, 0));
  pixels.show();
  delay(100);
  Serial.print("g");
  pixels.setPixelColor(0, pixels.Color(0, 20, 0));
  pixels.show();
  delay(100);
  Serial.print("b");
  pixels.setPixelColor(0, pixels.Color(0, 0, 20));
  pixels.show();
  delay(100);
}
