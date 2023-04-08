#include <Arduino.h>

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
#define SER_ROCKET Serial2
#define SER_ROCKET_TX 26
#define SER_ROCKET_RX 25

/**comTask configuration*/
#define TASKINTERVAL_MS 10

uint32_t igntime;
bool isValveWaitMoving = false;
bool igniting = false;

void setup()
{
  /**pin init*/
  pinMode(EXT_D_FET, OUTPUT);
  pinMode(INT_D_FET, OUTPUT);
  pinMode(FILL_FET, OUTPUT);
  pinMode(IGN_FET, OUTPUT);
  pinMode(O2_FET, OUTPUT);

  /**serial init*/
  pinMode(SER_ROCKET_TX, OUTPUT);
  pinMode(SER_CON_RX, INPUT);
  pinMode(SER_CON_TX, OUTPUT);
  pinMode(SER_CON_RX, INPUT);
  SER_ROCKET.begin(115200, SERIAL_8N1, SER_ROCKET_RX, SER_ROCKET_TX);
  SER_CON.begin(115200, SERIAL_8N1, SER_CON_RX, SER_CON_TX);

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