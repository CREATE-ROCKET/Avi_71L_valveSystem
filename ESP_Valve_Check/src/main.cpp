#include <Arduino.h>
#include "../../communication/gseCom.hpp"

constexpr uint8_t LED_R = 19;
constexpr uint8_t LED_B = 18;
constexpr uint8_t LED_Y = 17;

/**受信バッファ configuration*/
constexpr uint8_t RXPACKETBFFMAX = 128;

/**受信バッファ*/
class rxBff
{
public:
  uint8_t index = 0;
  uint8_t data[RXPACKETBFFMAX];
};
rxBff valveRxBff;

/** 受信用関数，パケット受信完了したらtrueを返す*/
IRAM_ATTR bool recieve(HardwareSerial &SER, rxBff &rx)
{
  while (SER.available())
  {
    uint8_t tmp = SER.read();

    if (rx.index == 0) /**ヘッダ受信*/
    {
      if (tmp == 0x43)
      {
        rx.data[rx.index++] = tmp;
      }
    }
    else if ((rx.index == 1) || (rx.index == 2)) /**cmdidおよびlengthの受信*/
    {
      rx.data[rx.index++] = tmp;
    }
    else if (rx.index < (rx.data[2] - 1)) /**受信完了1個前までの処理*/
    {
      rx.data[rx.index++] = tmp;
    }
    else if (rx.index == rx.data[2] - 1) /**受信完了*/
    {
      rx.data[rx.index] = tmp;
      rx.index = 0;
      if (GseCom::checkPacket(rx.data) == 0)
      {
        return true;
      }
    }
  }
  return false;
}

IRAM_ATTR void send(void *parameters)
{
  for (;;)
  {
    portTickType xLastWakeTime = xTaskGetTickCount();
    /**パラメータの生成*/
    uint8_t swPayload[1];
    swPayload[0] = 0x01;
    /**パラメータからパケットの生成*/
    uint8_t swPacket[5];
    GseCom::makePacket(swPacket, 0x61, swPayload, 1);
    Serial1.write(swPacket, swPacket[2]);

    digitalWrite(LED_B, LOW);
    delay(100);
    digitalWrite(LED_B, HIGH);

    vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_B, HIGH);
  digitalWrite(LED_Y, HIGH);
  Serial1.begin(9600, SERIAL_8N1, 33, 32);

  // digitalWrite(LED_R, LOW);
  // delay(1000);

  // uint8_t swPayload[1];
  // swPayload[0] = 0x00;
  // /**パラメータからパケットの生成*/
  // uint8_t swPacket[5];
  // GseCom::makePacket(swPacket, 0x61, swPayload, 1);
  // Serial1.write(swPacket, swPacket[2]);

  // digitalWrite(LED_B, LOW);
  // delay(1000);

  // // uint8_t swPayload[1];
  // swPayload[0] = 0x5A;
  // /**パラメータからパケットの生成*/
  // // uint8_t swPacket[5];
  // GseCom::makePacket(swPacket, 0x71, swPayload, 1);
  // Serial1.write(swPacket, swPacket[2]);

  // digitalWrite(LED_Y, LOW);

  // delay(10000);

  xTaskCreatePinnedToCore(send, "send", 4096, NULL, 1, NULL, 1);
}
void loop()
{
  if (recieve(Serial1, valveRxBff))
  {
    if (valveRxBff.data[3] == 0x81)
    {
      digitalWrite(LED_R, LOW);
      delay(100);
      digitalWrite(LED_R, HIGH);
    }
    else if (valveRxBff.data[3] == 0x41)
    {
      digitalWrite(LED_Y, LOW);
      delay(100);
      digitalWrite(LED_Y, HIGH);
    }
  }
}