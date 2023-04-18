#include <Arduino.h>
#include "../../communication/gseCom.hpp"

/**pin configuration*/
#define EXT_D_FET 19
#define INT_D_FET 21
#define FILL_FET 18
#define O2_FET 17
#define IGN_FET 16

/**serial configuration*/
#define SER_CON Serial1
#define SER_CON_TX 27
#define SER_CON_RX 13
#define SER_VALVE Serial2
#define SER_VALVE_TX 26
#define SER_VALVE_RX 25
#define SER_SEP Serial
#define SER_SEP_TX 32
#define SER_SEP_RX 33

/**comTask configuration*/
#define TASKINTERVAL_MS 10

/**ack configuration*/
#define ACKWAITTIME 5 /**ackの待ち時間*/
#define OWNNODEID 0b00000010

/**受信バッファ configuration*/
#define RXPACKETBFFMAX 128

/**受信バッファ*/
class rxBff
{
public:
  uint8_t index = 0;
  uint8_t data[RXPACKETBFFMAX];
};
rxBff ConRxBff;
rxBff ValveRxBff;
rxBff SepRxBff;

/** ack返答用パラメータ*/
namespace ackRecieveClass
{
  bool isAckRecieved;      /**trueならpcからackを受信済み，応答したらfalseに変更*/
  uint32_t ackRecieveTime; /**pcからackを受け取った時刻(us)，ACKWAITTIME後までに受信がなければ*/
  uint8_t ackNodesIds;     /**ackのノードIDのOR*/
};

namespace ignitionControlClass
{
  bool isFireing; /**点火スイッチが押されたらtrue，解除されたらfalse*/
};

/** 受信用関数，パケット受信完了したらtrueを返す*/
IRAM_ATTR bool
recieve(HardwareSerial SER, rxBff rx)
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

IRAM_ATTR void send(HardwareSerial SER, uint8_t packet[RXPACKETBFFMAX])
{
  SER.write(packet, packet[2]);
}

void setup()
{
  /**pin init*/
  pinMode(EXT_D_FET, OUTPUT);
  pinMode(INT_D_FET, OUTPUT);
  pinMode(FILL_FET, OUTPUT);
  pinMode(IGN_FET, OUTPUT);
  pinMode(O2_FET, OUTPUT);

  /**pin io init*/
  digitalWrite(EXT_D_FET, LOW);
  digitalWrite(INT_D_FET, LOW);
  digitalWrite(FILL_FET, LOW);
  digitalWrite(IGN_FET, LOW);
  digitalWrite(O2_FET, LOW);

  /**serial init*/
  pinMode(SER_VALVE_TX, OUTPUT);
  pinMode(SER_CON_RX, INPUT);
  pinMode(SER_CON_TX, OUTPUT);
  pinMode(SER_CON_RX, INPUT);
  SER_VALVE.begin(115200, SERIAL_8N1, SER_VALVE_RX, SER_VALVE_TX);
  SER_CON.begin(115200, SERIAL_8N1, SER_CON_RX, SER_CON_TX);

  /**ack返答用パラメータの初期化*/
  ackRecieveClass::isAckRecieved = false;
}

void loop()
{
  while (1)
  {
    /**CONTROLLERからの受信に対する処理*/
    if (recieve(SER_CON, ConRxBff))
    {
      uint8_t tmpCmdId = GseCom::getCmdId(ConRxBff.data);
      if (tmpCmdId == 0x00) /**ack受信*/
      {
        /**ackのIDの上書き*/
        ConRxBff.data[3] |= OWNNODEID;
        /**ack返答関係の起動*/
        ackRecieveClass::isAckRecieved = 1;
        ackRecieveClass::ackNodesIds = ConRxBff.data[3];
        ackRecieveClass::ackRecieveTime = micros();

        /**下位ノードへackの送信*/
        send(SER_VALVE, ConRxBff.data);
        send(SER_SEP, ConRxBff.data);
      }

      if (tmpCmdId == 0x43) /**スイッチ状態受信*/
      {
        uint8_t payload = ConRxBff.data[3];
        digitalWrite(EXT_D_FET, (payload & 0b10000000));
        digitalWrite(INT_D_FET, (payload & 0b01000000));
        digitalWrite(FILL_FET, (payload & 0b00100000));
        if (payload & 0b00010000) /**点火スイッチオン*/
        {
          if (!ignitionControlClass::isFireing)
          {
            ignitionControlClass::isFireing = true;
            /**点火コマンド送信~~~~~~~~~~~~~~~~~~~~~~~~~~
             * 書け
             */
          }
        }
        else
        {
          ignitionControlClass::isFireing = false;
        }
      }

      if (tmpCmdId == 0x71) /**Relayからの転送処理*/
      {
        send(SER_VALVE, ConRxBff.data);
        send(SER_SEP, ConRxBff.data);
      }
    }

    /**バルブからの受信に対する処理*/
    if (recieve(SER_VALVE, ValveRxBff))
    {
      uint8_t tmpCmdId = GseCom::getCmdId(ValveRxBff.data);
      if (tmpCmdId == 0x00) /**ack受信*/
      {
        /**ackのIDの上書き*/
        ackRecieveClass::ackNodesIds |= ValveRxBff.data[3];
      }
      if (tmpCmdId == 0x61) /**中継からの転送処理*/
      {
        send(SER_VALVE, ValveRxBff.data);
      }
    }

    /**切り離しからの受信に対する処理*/
    if (recieve(SER_SEP, SepRxBff))
    {
      uint8_t tmpCmdId = GseCom::getCmdId(SepRxBff.data);
      if (tmpCmdId == 0x00) /**ack受信*/
      {
        /**ackのIDの上書き*/
        ackRecieveClass::ackNodesIds |= SepRxBff.data[3];
      }
      if (tmpCmdId == 0x61) /**中継からの転送処理*/
      {
        send(SER_SEP, SepRxBff.data);
      }
    }

    /**ackに関する処理*/
    if (ackRecieveClass::isAckRecieved)
    {
      if ((micros() - ackRecieveClass::ackRecieveTime) > ACKWAITTIME)
      {
        /**ackを受信し，上位ノードに応答していない状態
         * もし下位ノードから応答があった場合
         * ackRecieveClass::ackNodesIdsが更新されているため問題なし
         */
        uint8_t ackPayLoad[1];
        ackPayLoad[0] = ackRecieveClass::ackNodesIds;
        uint8_t ackPacket[5];
        GseCom::makePacket(ackPacket, 0x00, ackPayLoad, 1);

        /**送信後処理，ackRecieveClass::isAckRecievedの解放*/
        ackRecieveClass::isAckRecieved = false;
      }
    }

    /**見やすくするための行*/
  }
}