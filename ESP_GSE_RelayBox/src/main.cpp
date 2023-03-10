#include <Arduino.h>

/**pin configuration*/
#define EXT_D_SW 5
#define INT_D_SW 4
#define FILL_SW 3
#define IGN_SW 2

/**serial configuration*/
#define SER_RELAY Serial1
#define SER_RELAY_TX 20
#define SER_RELAY_RX 21
#define SER_ROCKET Serial
#define SER_ROCKET_TX 7
#define SER_ROCKET_RX 6

/**comTask configuration*/
#define TASKINTERVAL_MS 10

/**com consts.*/

/**comTask parameters*/
bool isCmd2RocketSended = 0;   /**1なら中継基板に対し送信済み，応答が一定時間ない場合PCに対してコマンドを発行し送信*/
uint32_t timeCmd2RocketSended; /**中継基板に対し送信した時刻*/

typedef struct
{
  uint8_t data[12];
  uint8_t index = 0;
} recieve;

recieve recieveDatafromRelayBRD;
recieve recieveDatafromRocket;

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

void setup()
{
  /**serial init*/
  SER_ROCKET.begin(115200);
  SER_RELAY.begin(115200, SERIAL_8N1, SER_ROCKET_RX, SER_ROCKET_TX);

  /**pin init*/
  pinMode(EXT_D_SW, INPUT);
  pinMode(INT_D_SW, INPUT);
  pinMode(FILL_SW, INPUT);
  pinMode(IGN_SW, INPUT);
}

void loop()
{
  while (1)
  {
    /**PCからコマンドを受信した場合*/
    if ((recieveDatafromRelayBRD.index == 0) && (SER_RELAY.available()))
    {
      recieveDatafromRelayBRD.data[0] = SER_RELAY.read();
      if (recieveDatafromRelayBRD.data[0] == 0x71)
      {
        recieveDatafromRelayBRD.index++;
      }
    }
    if ((recieveDatafromRelayBRD.index > 0) && (SER_RELAY.available() > 3))
    {
      SER_RELAY.read(recieveDatafromRelayBRD.data + 1, 4);
      SER_ROCKET.write(recieveDatafromRelayBRD.data, 5);
      recieveDatafromRelayBRD.index = 0;
    }

    /**中継基板からコマンドを受信した場合*/
    if ((recieveDatafromRocket.index == 0) && (SER_ROCKET.available()))
    {
      recieveDatafromRocket.data[0] = SER_ROCKET.read();
      if (recieveDatafromRocket.data[0] == 0x71)
      {
        recieveDatafromRocket.index++;
      }
    }
    if ((recieveDatafromRocket.index > 0) && (SER_ROCKET.available() > 3))
    {
      SER_ROCKET.read(recieveDatafromRocket.data + 1, 4);
      SER_RELAY.write(recieveDatafromRocket.data, 5);
      recieveDatafromRocket.index = 0;
      isCmd2RocketSended = 0;
    }

    /**中継基板から応答が帰ってこない場合*/
    if ((isCmd2RocketSended == 1) && (micros() - timeCmd2RocketSended > 20000))
    {
      uint8_t relayErrCmdParams[8];
      uint8_t relayErrCmd[12];
      relayErrCmdParams[7] = 0b10;
      makeCmd(relayErrCmd, 0x61, relayErrCmdParams, 8);
      isCmd2RocketSended = 0;
    }
  }