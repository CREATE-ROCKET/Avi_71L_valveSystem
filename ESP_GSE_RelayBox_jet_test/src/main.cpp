#include <Arduino.h>

/**pin configuration*/
#define EXT_D_FET 8
#define INT_D_FET 10
#define FILL_FET 9
#define O2_FET 5
#define IGN_FET 4

/**serial configuration*/
#define SER_CON Serial1
#define SER_CON_TX 21
#define SER_CON_RX 20
#define SER_ROCKET Serial0
#define SER_ROCKET_TX 7
#define SER_ROCKET_RX 6

/**comTask configuration*/
#define TASKINTERVAL_MS 10

uint32_t igntime;
bool isValveWaitMoving = false;
bool igniting = false;

void setup()
{
  /**serial init*/
  SER_ROCKET.begin(115200, SERIAL_8N1, SER_ROCKET_RX, SER_ROCKET_TX);
  SER_CON.begin(115200, SERIAL_8N1, SER_CON_RX, SER_CON_TX);

  /**pin init*/
  pinMode(EXT_D_FET, OUTPUT);
  pinMode(INT_D_FET, OUTPUT);
  pinMode(FILL_FET, OUTPUT);
  pinMode(IGN_FET, OUTPUT);
  pinMode(O2_FET, OUTPUT);

  digitalWrite(EXT_D_FET, LOW);
  digitalWrite(INT_D_FET, LOW);
  digitalWrite(FILL_FET, LOW);
  digitalWrite(IGN_FET, LOW);
  digitalWrite(O2_FET, LOW);
}

void loop()
{
  /**コントローラ基板から受信した場合*/
  if (SER_CON.available())
  {
    uint8_t cmd = SER_CON.read();
    if (cmd == 'E')
    {
      digitalWrite(EXT_D_FET, HIGH);
    }
    else if (cmd == 'e')
    {
      digitalWrite(EXT_D_FET, LOW);
    }

    else if (cmd == 'I')
    {
      digitalWrite(INT_D_FET, HIGH);
    }
    else if (cmd == 'i')
    {
      digitalWrite(INT_D_FET, LOW);
    }

    else if (cmd == 'F')
    {
      digitalWrite(FILL_FET, HIGH);
    }
    else if (cmd == 'f')
    {
      digitalWrite(FILL_FET, LOW);
    }

    else if (cmd == 'L')
    {
      digitalWrite(O2_FET, HIGH);
      digitalWrite(IGN_FET, HIGH);
      /** 1秒後にバルブを動作させる*/
      if (!igniting)
      {
        igntime = micros();
        isValveWaitMoving = true;
      }
      igniting = true;
    }
    else if (cmd == 'l')
    {
      igniting = false;
      digitalWrite(O2_FET, LOW);
      digitalWrite(IGN_FET, LOW);
    }

    else if (cmd == 'a')
    {
      SER_ROCKET.write(cmd);
    }
    else if (cmd == 's')
    {
      SER_ROCKET.write(cmd);
    }
  }

  if (igniting)
  {
    if (isValveWaitMoving)
    {
      if (micros() - igntime > 1000000)
      {
        isValveWaitMoving = false;
        SER_ROCKET.write('a');
      }
    }
  }
}