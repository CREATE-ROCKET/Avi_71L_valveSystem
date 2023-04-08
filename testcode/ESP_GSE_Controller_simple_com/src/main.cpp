#include <Arduino.h>

/**pin configuration*/
#define EXT_D_SW 5
#define INT_D_SW 4
#define FILL_SW 3
#define IGN_SW 2

/**serial configuration*/
#define SER_PC Serial
#define SER_RELAY Serial1
#define SER_RELAY_TX 21
#define SER_RELAY_RX 20

/**comTask configuration*/
#define TASKINTERVAL_MS 10

/**comTask parameters*/
TaskHandle_t comTaskHandle;

/** 中継基板に対し通信を発行するスレッド*/
IRAM_ATTR void comTask(void *parameters)
{
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    /**中継基板のスイッチのステータスに応じコマンドを発行
     * 外部ダンプON：E
     * 外部ダンプOFF：e
     * 内部ダンプON：I
     * 内部ダンプOFF：i
     * 充填ON：F
     * 充填OFF：f
     * 点火操作ON：L
     * 点火操作OFF：l
     */
    if (digitalRead(EXT_D_SW))
    {
      SER_RELAY.print('E');
    }
    else
    {
      SER_RELAY.print('e');
    }

    if (digitalRead(IGN_SW))
    {
      SER_RELAY.print('L');
    }
    else
    {
      SER_RELAY.print('l');
    }

    if (digitalRead(INT_D_SW))
    {
      SER_RELAY.print('I');
    }
    else
    {
      SER_RELAY.print('i');
    }

    if (digitalRead(FILL_SW))
    {
      SER_RELAY.print('F');
    }
    else
    {
      SER_RELAY.print('f');
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
  pinMode(EXT_D_SW, INPUT_PULLDOWN);
  pinMode(INT_D_SW, INPUT_PULLDOWN);
  pinMode(FILL_SW, INPUT_PULLDOWN);
  pinMode(IGN_SW, INPUT_PULLDOWN);

  xTaskCreateUniversal(comTask, "comTask", 8192, NULL, 1, &comTaskHandle, PRO_CPU_NUM);
}

void loop()
{
  /**PCからコマンドを受信した場合，そのまま転送
   * コマンド内容
   * 'a' バルブの解放
   * 's' バルブの閉塞
   */
  if (SER_PC.available())
  {
    uint8_t tmp = SER_PC.read();
    SER_RELAY.write(tmp);
  }
}