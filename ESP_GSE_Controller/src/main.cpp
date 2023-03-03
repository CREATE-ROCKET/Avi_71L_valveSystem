#include <Arduino.h>

/**pin configuration*/
#define EXT_D_SW 5
#define INT_D_SW 4
#define FILL_SW 3
#define IGN_SW 2

/**serial configuration*/
#define SER_PC Serial
#define SER_RELAY Serial1
#define SER_RELAY_TX 20
#define SER_RELAY_RX 21

/**comTask configuration*/
#define TASKINTERVAL_MS 10

/**com consts.*/

/**comTask parameters*/
TaskHandle_t comTaskHandle;
bool isSequenceStarted = 0;           /**点火シーケンスに入ったことを示す*/
uint8_t prescaleWhileNonSequence = 0; /**点火シーケンスに入ってない場合インクリメントされ，99のときにコマンド処理が実行*/
bool isCmd2RelayBRDSended = 0;        /**1なら中継基板に対し送信済み，応答が一定時間ない場合PCに対してコマンドを発行し送信*/
uint32_t timeCmd2RelayBRDSended;      /**中継基板に対し送信した時刻*/

typedef struct
{
  uint8_t data[12];
  uint8_t index = 0;
} recieve;

recieve recieveDatafromRelayBRD;
recieve recieveDatafromPC;

/**
 * @brief コマンドを作成する
 *
 * @param cmd コマンドを格納するポインタ
 * @param cmdid　発行するコマンドのコマンド番号
 * @param parameters 発行するコマンドのパラメーターの配列のポインタ
 * @param parameterlength 発行するコマンドのパラメータ
 */
IRAM_ATTR void
makeCmd(uint8_t *cmd, uint8_t cmdid, uint8_t *parameters, uint8_t parameterlength)
{
  cmd[0] = 0x71;
  cmd[1] = cmdid;
  cmd[2] = parameterlength + 4;
  memcpy(parameters, cmd + 3, parameterlength);
  uint8_t sum = 0;
  for (int i = 0; i < parameterlength; i++)
  {
    sum += parameters[i] * 63;
  }
  cmd[parameterlength + 3] = sum; /**parity*/
}

/**
 * @brief parityを確認し，正規のコマンドか確認
 *
 * @param cmd　コマンドの配列のポインタ
 * @return 正常なら0,異常なら1
 */
IRAM_ATTR uint8_t checkCmd(uint8_t *cmd)
{
  uint8_t sum = 0;
  for (int i = 3; i < cmd[3] - 1; i++)
  {
    sum += cmd[i] * 63;
  }
  if (sum == cmd[cmd[3] - 1])
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

/** 中継基板に対し通信を発行するスレッド*/
IRAM_ATTR void comTask(void *parameters)
{
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (isSequenceStarted || (prescaleWhileNonSequence++ == 99))
    {
      prescaleWhileNonSequence = 0;

      /**スイッチの状態確認*/
      int extdstatus = digitalRead(EXT_D_SW);
      int intdstatus = digitalRead(INT_D_SW);
      int fillstatus = digitalRead(FILL_SW);
      int ignstatus = digitalRead(IGN_SW);

      /**中継基板へのコマンド送信*/
      uint8_t cmd2RelayBRDParameter[1] = {(extdstatus << 7) | (intdstatus << 6) | (fillstatus << 5) | (ignstatus << 4)};
      uint8_t cmd2RelayBRD[5];
      makeCmd(cmd2RelayBRD, 0x21, cmd2RelayBRDParameter, 1);
      SER_RELAY.write(cmd2RelayBRD, 5);

      /**送信したことを中継基板からの受信スレッドに伝える*/
      isCmd2RelayBRDSended = 1;
      timeCmd2RelayBRDSended = micros();
    }
    vTaskDelayUntil(&xLastWakeTime, TASKINTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup()
{
  /**serial init*/
  SER_PC.begin(115200);
  SER_RELAY.begin(115200, SERIAL_8N1, SER_RELAY_RX, SER_RELAY_TX);

  /**pin init*/
  pinMode(EXT_D_SW, INPUT);
  pinMode(INT_D_SW, INPUT);
  pinMode(FILL_SW, INPUT);
  pinMode(IGN_SW, INPUT);

  xTaskCreateUniversal(comTask, "comTask", 8192, NULL, 1, &comTaskHandle, PRO_CPU_NUM);
}

void loop()
{
  while (1)
  {
    /**PCからコマンドを受信した場合*/
    if ((recieveDatafromRelayBRD.index == 0) && (SER_PC.available()))
    {
      recieveDatafromRelayBRD.data[0] = SER_PC.read();
      if (recieveDatafromRelayBRD.data[0] == 0x71)
      {
        recieveDatafromRelayBRD.index++;
      }
    }
    if ((recieveDatafromRelayBRD.index > 0) && (SER_PC.available() > 3))
    {
      SER_PC.read(recieveDatafromRelayBRD.data + 1, 4);
      SER_RELAY.write(recieveDatafromRelayBRD.data, 5);
      recieveDatafromRelayBRD.index = 0;
    }

    /**中継基板からコマンドを受信した場合*/
    if ((recieveDatafromPC.index == 0) && (SER_RELAY.available()))
    {
      recieveDatafromPC.data[0] = SER_RELAY.read();
      if (recieveDatafromPC.data[0] == 0x71)
      {
        recieveDatafromPC.index++;
      }
    }
    if ((recieveDatafromPC.index > 0) && (SER_RELAY.available() > 3))
    {
      SER_RELAY.read(recieveDatafromPC.data + 1, 4);
      SER_PC.write(recieveDatafromPC.data, 5);
      recieveDatafromPC.index = 0;
      isCmd2RelayBRDSended = 0;
    }

    /**中継基板から応答が帰ってこない場合*/
    if ((isCmd2RelayBRDSended == 1) && (micros() - timeCmd2RelayBRDSended > 20000))
    {
      uint8_t relayErrCmdParams[8];
      uint8_t relayErrCmd[12];
      relayErrCmdParams[7] = 0b10;
      makeCmd(relayErrCmd, 0x61, relayErrCmdParams, 8);
      isCmd2RelayBRDSended = 0;
    }
  }