#include <Arduino.h>
#include <stdio.h>
#include "driver/pcnt.h"
#include <Adafruit_NeoPixel.h>
#include "../../communication/gseCom.hpp"

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
#define LOGDATASIZE 4096

#define SER_RELAY Serial1
#define SER_RELAY_RX 36
#define SER_RELAY_TX 25

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
struct MDLogData MDLogDataMem[LOGDATASIZE];

TaskHandle_t controlHandle;
bool isControlRunning = false;

pcnt_config_t pcnt_config;

/**ack configuration*/
#define ACKWAITTIME 0 /**ackの待ち時間*/
#define OWNNODEID 0b00000100

/**受信バッファ configuration*/
#define RXPACKETBFFMAX 128

/**受信バッファ*/
class rxBff
{
public:
  uint8_t index = 0;
  uint8_t data[RXPACKETBFFMAX];
};
rxBff RelayRxBff;

/** ack返答用パラメータ*/
namespace ackRecieveClass
{
  bool isAckRecieved = true; /**trueならpcからackを受信済み，応答したらfalseに変更*/
  uint32_t ackRecieveTime;   /**pcからackを受け取った時刻(us)，ACKWAITTIME後までに受信がなければ*/
  uint8_t ackNodesIds;       /**ackのノードIDのOR*/
};

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
      pixels.setPixelColor(0, pixels.Color(00, 20, 0));
      pixels.show();
      MDLogMemIndex = 0;
      isMdFinished = 0;
      isControlRunning = false;
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
    if (MDLogMemIndex < LOGDATASIZE)
    {
      MDLogDataMem[MDLogMemIndex]._enc_pcnt = enc_pcnt;
      MDLogDataMem[MDLogMemIndex]._pwm_ratio = pwm_ratio;
      MDLogDataMem[MDLogMemIndex]._motor_angle = motor_angle;
      MDLogDataMem[MDLogMemIndex]._d_motor_angle = d_motor_angle;
      // プリスケールしデータをシリアル通信に流すコードを追加
      if (MDLogMemIndex % 100 == 0)
      {
        // シリアル通信に流す
        uint8_t payload[30];
        memcpy(payload, &MDLogDataMem[MDLogMemIndex]._time, sizeof(unsigned long));
        memcpy(payload + 4, &MDLogDataMem[MDLogMemIndex]._enc_pcnt, sizeof(int16_t));
        memcpy(payload + 6, &target_angle, sizeof(double));
        memcpy(payload + 14, &MDLogDataMem[MDLogMemIndex]._motor_angle, sizeof(double));
        memcpy(payload + 22, &MDLogDataMem[MDLogMemIndex]._d_motor_angle, sizeof(double));
        uint8_t packet[34];
        GseCom::makePacket(packet, 0x61, payload, 30);
        SER_RELAY.write(packet, packet[2]);
      }

      // ログの配列のインデックスを加算
      MDLogMemIndex++;
    }

    vTaskDelayUntil(&xLastWakeTime, CONTROLINTERVAL_MS / portTICK_PERIOD_MS);
  }
}

/** 受信用関数，パケット受信完了したらtrueを返す*/
IRAM_ATTR bool
recieve(HardwareSerial &SER, rxBff &rx)
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
  pcnt_config.channel = PCNT_CHANNEL_1;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;
  pcnt_set_filter_value(PCNT_UNIT_0,1000);
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);

  Serial.begin(115200);
  SER_RELAY.begin(9600, SERIAL_8N1, SER_RELAY_RX, SER_RELAY_TX);
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
  if (recieve(SER_RELAY, RelayRxBff))
  {
    Serial.println("cmd rec");
    uint8_t tmpCmdId = GseCom::getCmdId(RelayRxBff.data);
    if (tmpCmdId == 0x00) /**ack受信*/
    {
      Serial.println("ack");
      /**ackのIDの上書き*/
      RelayRxBff.data[3] |= OWNNODEID;
      /**CRCの再生成*/
      GseCom::regenPacketCRC(RelayRxBff.data);
      /**ack返答関係の起動*/
      ackRecieveClass::isAckRecieved = 1;
      ackRecieveClass::ackNodesIds = RelayRxBff.data[3];
      ackRecieveClass::ackRecieveTime = micros();

      /**上位ノードへackの返信*/
      SER_RELAY.write(RelayRxBff.data, RelayRxBff.data[2]);
    }
    if (tmpCmdId == 0x71) /**バルブ制御コマンド*/
    {
      Serial.println("valve");
      uint8_t valveTarget = RelayRxBff.data[3];
      if (valveTarget == 0x01)
      {
        pixels.setPixelColor(0, pixels.Color(12, 8, 0));
        pixels.show();
        target_angle = 90. / 360. * M_PI;
        // Nch idle
        digitalWrite(MA_N, LOW);
        digitalWrite(MB_N, LOW);
        delay(1);
        if (!isControlRunning)
        {
          isControlRunning = true;
          xTaskCreate(controlDCM, "DCM", 8192, NULL, 1, &controlHandle);
        }
      }
      else if (valveTarget == 0x00)
      {
        pixels.setPixelColor(0, pixels.Color(12, 8, 0));
        pixels.show();
        target_angle = 0. / 360. * M_PI;
        // Nch idle
        digitalWrite(MA_N, LOW);
        digitalWrite(MB_N, LOW);
        delay(1);
        if (!isControlRunning)
        {
          isControlRunning = true;
          xTaskCreate(controlDCM, "DCM", 8192, NULL, 1, &controlHandle);
        }
      }
    }
    /**その他コマンドはここへ*/
  }
}