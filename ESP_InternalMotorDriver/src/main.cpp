#include <Arduino.h>
#include "driver/pcnt.h"
#include <Adafruit_NeoPixel.h>

#define ENC_A 34
#define ENC_B 35

#define MA_N 21
#define MA_P 19
#define MA_P_PWM_CH 0
#define MB_N 23
#define MB_P 22
#define MB_P_PWM_CH 2
#define PWMFREQ 5000
#define PWM_RES_BIT 10

#define DEADTIME_MS 1

#define GEAR_RATIO 150.
#define ENC_CPR 64.

#define CONTROLINTERVAL_MS 1

#define PIN 32
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

double motor_angle = 0., old_motor_angle = 0., d_motor_angle = 0.; // 角度, 一つ前の時刻の角度, 角速度
const double dt = 0.001;                                           // サンプリング間隔
const double F = 200.;                                             // モータの角速度を求めるときのカットオフ周波数 [rad/s]
int16_t pwm_ratio = 0;                                             // PWM比
int old_pwm_ratio = 0;
double target_angle = 45. / 180. * M_PI; // 目標角度 [rad]
double Kp = 40., Kd = 0.9;               // コントローラのゲイン
double MAX_V = 12.;                      // 電源電圧
double Voltage = 0.;                     // 指令電圧
int16_t fric_up_border = 300, fric_down_border = 5;
uint8_t isMdFinished = 0;

// memory variables
int MDLogMemIndex = 0;
struct MDLogData
{
  unsigned long _time;
  int16_t _enc_pcnt;
  int16_t _pwm_ratio;
  double _motor_angle;
  double _d_motor_angle;
};
struct MDLogData MDLogDataMem[4096];

TaskHandle_t controlHandle;

pcnt_config_t pcnt_config;

IRAM_ATTR void controlDCM(void *parameters)
{
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // 読み取り値をpulseに代入
    int16_t enc_pcnt; /**エンコーダ読み取り用変数*/
    MDLogDataMem[MDLogMemIndex]._time = micros();
    pcnt_get_counter_value(PCNT_UNIT_0, &enc_pcnt);

    // old_motor_angleに1つ前の時刻の角度を代入
    old_motor_angle = motor_angle;

    // 現在の角度 [rad]を計算
    motor_angle = (double)(enc_pcnt) / ENC_CPR / GEAR_RATIO * (2. * M_PI);

    // 疑似微分を用いて角速度 [rad/s]を計算
    d_motor_angle = (1 - dt * F) * d_motor_angle + F * (motor_angle - old_motor_angle);

    // 指令電圧をP-D制御を用いて計算 目標角度は一定の場合
    Voltage = (target_angle - motor_angle) * Kp - d_motor_angle * Kd;

    if (Voltage > 12.)
    {
      Voltage = 12.;
    }
    else if (Voltage < -12.)
    {
      Voltage = -12.;
    }

    old_pwm_ratio = pwm_ratio;

    pwm_ratio = (int)(Voltage / MAX_V * 1024.); // 指令電圧をPWMの指令duty比に変換　duty比の範囲は-1024～1024

    // 摩擦補償
    if (pwm_ratio > fric_down_border)
    {
      if (pwm_ratio < fric_up_border)
      {
        pwm_ratio = fric_up_border;
      }
    }
    else if (pwm_ratio < (-1 * fric_down_border))
    {
      if (pwm_ratio > (-1 * fric_up_border))
      {
        pwm_ratio = (-1 * fric_up_border);
      }
    }

    // モーター制御が終了していた場合255回モーターのコントローラーをスキップし、タスクを終了
    if (isMdFinished == 255)
    {
      Serial.println("time,enc_pcnt,pwm_ratio,motor_angle,d_motor_angle");
      for (int i = 0; i < MDLogMemIndex; i++)
      {
        Serial.printf("%d,%d,%d,%f5.5,%f5.5\n", MDLogDataMem[i]._time, MDLogDataMem[i]._enc_pcnt, MDLogDataMem[i]._pwm_ratio, MDLogDataMem[i]._motor_angle, MDLogDataMem[i]._d_motor_angle);
      }
      MDLogMemIndex = 0;
      isMdFinished = 0;
      pixels.setPixelColor(0, pixels.Color(00, 20, 0));
      pixels.show();
      vTaskDelete(controlHandle);
    }
    else if (isMdFinished > 0)
    {
      isMdFinished++;
      digitalWrite(MA_N, HIGH);
      digitalWrite(MB_N, HIGH);
    }
    else if ((fabsf(motor_angle - target_angle) < 1e-2) && (fabsf(old_motor_angle - target_angle) < 1e-2) && (fabsf(d_motor_angle) < 5e-2))
    {
      // モーター制御
      // 目標角に到達したら、PWMを停止する
      ledcWrite(MA_P_PWM_CH, 0);
      ledcWrite(MB_P_PWM_CH, 0);
      digitalWrite(MA_N, LOW);
      digitalWrite(MB_N, LOW);

      isMdFinished = 1;
    }
    else if ((pwm_ratio * old_pwm_ratio) <= 0)
    {
      // 指令値が正負逆転した場合はfetをidle状態にする
      digitalWrite(MA_N, LOW);
      digitalWrite(MB_N, LOW);
      ledcWrite(MA_P_PWM_CH, 0);
      ledcWrite(MB_P_PWM_CH, 0);
    }
    else if (pwm_ratio > 0)
    {
      // 指令duty比が正のとき
      digitalWrite(MA_N, HIGH);
      ledcWrite(MB_P_PWM_CH, pwm_ratio);
    }
    else
    {
      // 指令duty比が負のとき
      digitalWrite(MB_N, HIGH);
      ledcWrite(MA_P_PWM_CH, (-1) * pwm_ratio);
    }

    // write log data
    MDLogDataMem[MDLogMemIndex]._enc_pcnt = enc_pcnt;
    MDLogDataMem[MDLogMemIndex]._pwm_ratio = pwm_ratio;
    MDLogDataMem[MDLogMemIndex]._motor_angle = motor_angle;
    MDLogDataMem[MDLogMemIndex++]._d_motor_angle = d_motor_angle;

    vTaskDelayUntil(&xLastWakeTime, CONTROLINTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup()
{
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(20, 0, 0));
  pixels.show();
  // pcnt init
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

  pixels.setPixelColor(0, pixels.Color(0, 20, 0));
  pixels.show();

  delay(1000);
}

void loop()
{
  if (Serial.available())
  {
    char chr = Serial.read();
    if (chr == 'a')
    {
      pixels.setPixelColor(0, pixels.Color(12, 8, 0));
      pixels.show();
      target_angle = 90. / 360. * M_PI;
      // Nch idle
      digitalWrite(MA_N, LOW);
      digitalWrite(MB_N, LOW);
      delay(1);
      xTaskCreate(controlDCM, "DCM", 8192, NULL, 1, &controlHandle);
    }
    if (chr == 's')
    {
      pixels.setPixelColor(0, pixels.Color(12, 8, 0));
      pixels.show();
      target_angle = 0.;
      // Nch idle
      digitalWrite(MA_N, LOW);
      digitalWrite(MB_N, LOW);
      delay(1);
      xTaskCreate(controlDCM, "DCM", 8192, NULL, 1, &controlHandle);
    }
  }
  // Serial.print("u\n");
  int16_t enc_pcnt;
  pcnt_get_counter_value(PCNT_UNIT_0, &enc_pcnt);
  // Serial.printf("%d,,,\n", enc_pcnt);
  delay(1000);
}