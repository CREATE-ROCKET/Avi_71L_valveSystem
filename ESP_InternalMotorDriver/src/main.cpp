#include <Arduino.h>
#include "driver/pcnt.h"

#define ENC_A 34
#define ENC_B 35

#define MA_N 26
#define MA_P 25
#define MA_P_PWM_CH 0
#define MB_N 33
#define MB_P 32
#define MB_P_PWM_CH 2
#define PWMFREQ 1000
#define PWM_RES_BIT 8

#define DEADTIME 1

void setup()
{
  // pcnt init
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = ENC_A;
  pcnt_config.ctrl_gpio_num = ENC_B;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);

  Serial.begin(115200);
  pinMode(MA_N, OUTPUT);
  pinMode(MA_P, OUTPUT);
  pinMode(MB_N, OUTPUT);
  pinMode(MB_P, OUTPUT);

  // pwm init (Pch)
  ledcSetup(MA_P_PWM_CH, PWMFREQ, PWM_RES_BIT);
  ledcSetup(MB_P_PWM_CH, PWMFREQ, PWM_RES_BIT);
  ledcWrite(MA_P_PWM_CH, 0);
  ledcWrite(MB_P_PWM_CH, 0);

  // Pch PWM attach
  ledcAttachPin(MA_P, MA_P_PWM_CH);
  ledcAttachPin(MB_P, MB_P_PWM_CH);

  // Nch breaking
  digitalWrite(MA_N, HIGH);
  digitalWrite(MB_N, HIGH);

  // pcnt on
  pcnt_counter_resume(PCNT_UNIT_0);

  delay(1000);
}

void loop()
{
  int16_t count;

  Serial.println("powering");
  digitalWrite(MB_N, LOW);
  delay(DEADTIME);
  ledcWrite(MB_P_PWM_CH, 255);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count < 1000);
  ledcWrite(MB_P_PWM_CH, 0);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count < 1200);
  digitalWrite(MB_N, HIGH);

  delay(1000);

  Serial.println("reverse powering");
  digitalWrite(MA_N, LOW);
  delay(DEADTIME);
  ledcWrite(MA_P_PWM_CH, 255);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count > 200);
  ledcWrite(MA_P_PWM_CH, 0);
  do
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
  } while (count > 0);
  digitalWrite(MA_N, HIGH);

  delay(1000);
}