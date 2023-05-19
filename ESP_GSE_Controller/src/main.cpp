#include <Arduino.h>
#include "../../communication/gseCom.hpp"

/**pin configuration*/
#define SIG_OUT_INDICATOR 14
#define SIG_OUT_GSEBATERY_LED 26
#define SIG_OUT_GSEPOWER_LED 27
#define SIG_IN_KEYSW 25

#define SW1 16
#define SW2 4
#define SW3 19
#define SW4 18
#define SW5 17

/**solenoids allocation*/
#define EXT_D_SW SW1
#define INT_D_SW SW2
#define FILL_SW SW3
#define IGN_SW SW4

/**serial configuration*/
#define SER_PC Serial1
#define SER_PC_TX 23
#define SER_PC_RX 22
#define SER_RELAY Serial0
#define SER_RELAY_TX 32
#define SER_RELAY_RX 33

/**swComTask configuration*/
#define TASKINTERVAL_MS 100

/**ack configuration*/
#define ACKWAITTIME 200 /**ackの待ち時間*/
#define OWNNODEID 0b00000001

/**受信バッファ configuration*/
#define RXPACKETBFFMAX 128

/**受信バッファ*/
class rxBff
{
public:
  uint8_t index = 0;
  uint8_t data[RXPACKETBFFMAX];
};
rxBff PCRxBff;
rxBff RelayRxBff;

/**com consts.*/
TaskHandle_t swComTaskHandle; /**スイッチのデータ送信用の処理系*/

/** ack返答用パラメータ*/
namespace ackRecieveClass
{
  bool isAckRecieved;      /**trueならpcからackを受信済み，応答したらfalseに変更*/
  uint32_t ackRecieveTime; /**pcからackを受け取った時刻(us)，ACKWAITTIME後までに受信がなければ*/
  uint8_t ackNodesIds;     /**ackのノードIDのOR*/
};

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

/** 中継基板に対し通信を発行するスレッド*/
IRAM_ATTR void swComTask(void *parameters)
{
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    /**スイッチの状態確認*/
    uint8_t extdstatus = digitalRead(EXT_D_SW);
    uint8_t intdstatus = digitalRead(INT_D_SW);
    uint8_t fillstatus = digitalRead(FILL_SW);
    uint8_t ignstatus = digitalRead(IGN_SW);

    /**パラメータの生成*/
    uint8_t swPayload[1];
    swPayload[0] = (extdstatus << 7) | (intdstatus << 6) | (fillstatus << 5) | (ignstatus << 4);

    /**パラメータからパケットの生成*/
    uint8_t swPacket[5];
    GseCom::makePacket(swPacket, 0x21, swPayload, 1);
    SER_RELAY.write(swPacket, swPacket[2]);

    vTaskDelayUntil(&xLastWakeTime, TASKINTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup()
{
  /**serial init*/
  SER_PC.begin(9600, SERIAL_8N1, SER_PC_RX, SER_PC_TX);
  SER_RELAY.begin(9600, SERIAL_8N1, SER_RELAY_RX, SER_RELAY_TX);

  /**pin init*/
  pinMode(EXT_D_SW, INPUT);
  pinMode(INT_D_SW, INPUT);
  pinMode(FILL_SW, INPUT);
  pinMode(IGN_SW, INPUT);

  /**ack返答用パラメータの初期化*/
  ackRecieveClass::isAckRecieved = false;
  /**スイッチ状態確認&送信タスクの起動*/
  xTaskCreateUniversal(swComTask, "comTask", 8192, NULL, 1, &swComTaskHandle, PRO_CPU_NUM);
}

void loop()
{
  /**PCからの受信に対する処理*/
  if (recieve(SER_PC, PCRxBff))
  {
    uint8_t tmpCmdId = GseCom::getCmdId(PCRxBff.data);
    if (tmpCmdId == 0x00) /**ack受信*/
    {
      /**ackのIDの上書き*/
      PCRxBff.data[3] |= OWNNODEID;
      /**CRCの再生成*/
      GseCom::regenPacketCRC(PCRxBff.data);
      /**ack返答関係の起動*/
      ackRecieveClass::isAckRecieved = true;
      ackRecieveClass::ackNodesIds = PCRxBff.data[3];
      ackRecieveClass::ackRecieveTime = micros();

      /**下位ノードへackの送信*/
      SER_RELAY.write(PCRxBff.data, PCRxBff.data[2]);
    }

    if (tmpCmdId == 0x71) /**PCからの転送処理*/
    {
      SER_RELAY.write(PCRxBff.data, PCRxBff.data[2]);
    }
  }

  /**中継基板からの受信に対する処理*/
  if (recieve(SER_RELAY, RelayRxBff))
  {
    uint8_t tmpCmdId = GseCom::getCmdId(RelayRxBff.data);
    if (tmpCmdId == 0x00) /**ack受信*/
    {
      /**ackのIDの上書き*/
      ackRecieveClass::ackNodesIds |= RelayRxBff.data[3];
    }
    if (tmpCmdId == 0x61) /**PCからの転送処理*/
    {
      SER_PC.write(RelayRxBff.data, RelayRxBff.data[2]);
    }
  }

  /**ackに関する処理*/
  if (ackRecieveClass::isAckRecieved)
  {
    if ((micros() - ackRecieveClass::ackRecieveTime) > ACKWAITTIME * 1000)
    {
      /**ackを受信し，上位ノードに応答していない状態
       * もし下位ノードから応答があった場合
       * ackRecieveClass::ackNodesIdsが更新されているため問題なし
       */
      uint8_t ackPayLoad[1];
      ackPayLoad[0] = ackRecieveClass::ackNodesIds;
      uint8_t ackPacket[5];
      GseCom::makePacket(ackPacket, 0x00, ackPayLoad, 1);
      SER_PC.write(ackPacket, ackPacket[2]);

      /**送信後処理，ackRecieveClass::isAckRecievedの解放*/
      ackRecieveClass::isAckRecieved = false;
    }
  }
}