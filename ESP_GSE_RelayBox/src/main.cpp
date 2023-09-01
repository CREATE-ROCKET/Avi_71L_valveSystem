#include <Arduino.h>
#include "../../communication/gseCom.hpp"

/**pin configuration*/
#define SIG_OUT_INDICATOR 5
#define SIG_IN_BAT 34
#define SIG_IN_POW 35
#define SIG_OUT_RELAY 4
#define IGN_FET 16

/**allocatable pins configuration*/
#define S6 17
#define S5 18
#define S4 19
#define S3 21
#define S2 22
#define S1 23

/**pins allocatoion*/
#define EXT_D_FET S1
#define INT_D_FET S2
#define FILL_FET S3
#define O2_FET S4

/**serial configuration*/
#define SER_CON Serial1
#define SER_CON_TX 13
#define SER_CON_RX 27
#define SER_VALVE Serial2
#define SER_VALVE_TX 26
#define SER_VALVE_RX 25
#define SER_SEP Serial
#define SER_SEP_TX 33
#define SER_SEP_RX 32

/**comTask configuration*/
#define TASKINTERVAL_MS 10

/**ack configuration*/
#define ACKWAITTIME 100 /**ackの待ち時間*/
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
  /**pin init*/
  pinMode(EXT_D_FET, OUTPUT);
  pinMode(INT_D_FET, OUTPUT);
  pinMode(FILL_FET, OUTPUT);
  pinMode(IGN_FET, OUTPUT);
  pinMode(O2_FET, OUTPUT);
  pinMode(S5, OUTPUT);
  pinMode(S6, OUTPUT);

  /**pin io init*/
  digitalWrite(EXT_D_FET, LOW);
  digitalWrite(INT_D_FET, LOW);
  digitalWrite(FILL_FET, LOW);
  digitalWrite(IGN_FET, LOW);
  digitalWrite(O2_FET, LOW);
  digitalWrite(S5, LOW);
  digitalWrite(S6, LOW);

  /**serial init*/
  pinMode(SER_VALVE_TX, OUTPUT);
  pinMode(SER_CON_RX, INPUT);
  pinMode(SER_CON_TX, OUTPUT);
  pinMode(SER_CON_RX, INPUT);
  SER_VALVE.begin(9600, SERIAL_8N1, SER_VALVE_RX, SER_VALVE_TX);
  SER_CON.begin(9600, SERIAL_8N1, SER_CON_RX, SER_CON_TX);
  SER_SEP.begin(9600, SERIAL_8N1, SER_SEP_RX, SER_SEP_TX);

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
        /**CRCの再生成*/
        GseCom::regenPacketCRC(ConRxBff.data);
        /**ack返答関係の起動*/
        ackRecieveClass::isAckRecieved = 1;
        ackRecieveClass::ackNodesIds = ConRxBff.data[3];
        ackRecieveClass::ackRecieveTime = micros();

        /**下位ノードへackの送信*/
        SER_VALVE.write(ConRxBff.data, ConRxBff.data[2]);
        SER_SEP.write(ConRxBff.data, ConRxBff.data[2]);
      }

      if (tmpCmdId == 0x21) /**スイッチ状態受信*/
      {
        uint8_t payload = ConRxBff.data[3];
        digitalWrite(EXT_D_FET, (payload & 0b10000000) >> 7);
        digitalWrite(INT_D_FET, (payload & 0b01000000) >> 6);
        digitalWrite(FILL_FET, (payload & 0b00100000) >> 5);
        digitalWrite(O2_FET, (payload & 0b00010000) >> 4);
        digitalWrite(IGN_FET, (payload & 0b00010000) >> 4);
        digitalWrite(S5, (payload & 0b10000000) >> 3);
        digitalWrite(S6, (payload & 0b10000000) >> 2);
        if ((payload & 0b00010000) >> 4) /**点火スイッチオン*/
        {
          if (!ignitionControlClass::isFireing)
          {
            ignitionControlClass::isFireing = true;
            /**点火コマンド送信 */
            uint8_t ignPayload[1] = {0x5A}; // 90 deg
            uint8_t ignPacket[5];
            GseCom::makePacket(ignPacket, 0x71, ignPayload, 1);
            SER_VALVE.write(ignPacket, ignPacket[2]);
          }
        }
        else
        {
          ignitionControlClass::isFireing = false;
        }
      }

      if (tmpCmdId == 0x71) /**CONTROLLERからの転送処理*/
      {
        SER_VALVE.write(ConRxBff.data, ConRxBff.data[2]);
        SER_SEP.write(ConRxBff.data, ConRxBff.data[2]);
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
      if (tmpCmdId == 0x61) /**バルブからの転送処理 コマンドid一覧を追加すること*/
      {
        SER_CON.write(ValveRxBff.data, ValveRxBff.data[2]);
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
      if (tmpCmdId == 0x61) /**中継からの転送処理 コマンドid一覧を追加すること*/
      {
        SER_CON.write(SepRxBff.data, SepRxBff.data[2]);
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
        SER_CON.write(ackPacket, ackPacket[2]);

        /**送信後処理，ackRecieveClass::isAckRecievedの解放*/
        ackRecieveClass::isAckRecieved = false;
      }
    }

    /**見やすくするための行*/
  }
}