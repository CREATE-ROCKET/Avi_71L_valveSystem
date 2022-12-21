#include <Arduino.h>
#include "driver/pcnt.h"

#define PULSE_INPUT_PIN 34 // パルスの入力ピン 今回はエンコーダのA相を接続
#define PULSE_CTRL_PIN 35  // 制御ピン 今回はエンコーダのB相を接続

#define MA_N 26
#define MA_P 25
#define MB_N 33
#define MB_P 32

#define DEADTIME 1

void setup()
{
  pcnt_config_t pcnt_config; // 設定用の構造体の宣言
  pcnt_config.pulse_gpio_num = PULSE_INPUT_PIN;
  pcnt_config.ctrl_gpio_num = PULSE_CTRL_PIN;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;
  pcnt_unit_config(&pcnt_config);  // ユニット初期化
  pcnt_counter_pause(PCNT_UNIT_0); // カウンタ一時停止
  pcnt_counter_clear(PCNT_UNIT_0); // カウンタ初期化

  Serial.begin(115200);
  pinMode(MA_N, OUTPUT);
  pinMode(MA_P, OUTPUT);
  pinMode(MB_N, OUTPUT);
  pinMode(MB_P, OUTPUT);

  delay(5000);
  pcnt_counter_resume(PCNT_UNIT_0); // カウント開始
  Serial.println("breaking");
  digitalWrite(MA_N, HIGH);
  digitalWrite(MB_N, HIGH);

  delay(1000);
}

void loop()
{
  int16_t count;

  Serial.println("powering");
  digitalWrite(MB_N, LOW);
  delay(DEADTIME);
  digitalWrite(MB_P, HIGH);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count < 1000);
  digitalWrite(MB_P, LOW);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count < 1200);
  digitalWrite(MB_N, HIGH);

  delay(1000);

  Serial.println("reverse powering");
  digitalWrite(MA_N, LOW);
  delay(DEADTIME);
  digitalWrite(MA_P, HIGH);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count > 200);
  digitalWrite(MA_P, LOW);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count > 0);
  digitalWrite(MA_N, HIGH);

  delay(1000);
}