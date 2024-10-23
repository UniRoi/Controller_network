#include <encoder.h>

#include <digital_out.h>
#include <analog_out.h>
#include <speed_control.h>
#include <Arduino.h>

#include <avr/interrupt.h>

// #define DEBUG_PRINT // enable debug print messages

enum eStates
{
  NO_STATE = 0,
  INIT = 1,
  PRE_OPERATIONAL = 3,
  OPERATIONAL = 6,
  STOPPED = 10
};

static uint32_t u32LastTime;

volatile bool bUpdateSpeed;
static int command = 0;
uint16_t targetRpm = 10000;

encoder Encoder(2, 3);
Analog_out ana_out(1);
Digital_out led(5);
Digital_out EncSlp(2);
Digital_in EncFlt(4);

// Digital_out TestPin(3); // output only for testing

static float m_fKp = 0.27;
static float m_fTi = 0.41;

Controller *P_speed = nullptr;



eStates controllerState = eStates::INIT;

void setup()
{
  // put your setup code here, to run once:

  bUpdateSpeed = false;

  ana_out.init(10);
  led.init();

  // TestPin.init();

  // here interrupt registers are set
  Encoder.init();
  EncFlt.init();
  EncSlp.init();
  EncSlp.set_lo();

  sei();

  // Add serial for part 2
  Serial.begin(115200);

  u32LastTime = millis();
}

#ifdef DEBUG_PRINT
#include <stdio.h>
#define dbg_print Serial.print
#else
#define dbg_print(...) /**/
#endif

enum eStates fn_checkForTransition(int cmd)
{
  enum eStates Transition = NO_STATE;

  if (cmd == 'o')
  {
    Transition = eStates::OPERATIONAL;
  }
  else if (cmd == 'p')
  {
    Transition = eStates::PRE_OPERATIONAL;
  }
  else if (cmd == 'r')
  {
    Transition = eStates::INIT;
  }

  return Transition;
}

void fn_PrintDbgMsg(const char *msg, uint32_t TimeNow)
{
  static uint32_t LastTime = 0;

  if ((TimeNow - LastTime) > 500)
  {
    dbg_print(msg);
    LastTime = TimeNow;
  }
}

bool fn_IsEncInFault(uint32_t TimeNow, bool bFltLogic)
{
  static uint32_t LastTime = 0;
  bool bRet = false;

  if (bFltLogic == true)
  {
    if ((TimeNow - LastTime) > 5)
    {
      bRet = true;
    }
  }
  else
  {
    // bLastFltLogic = bFltLogic;
    LastTime = TimeNow;
  }

  return bRet;
}

void loop()
{
  uint32_t u32TimeNow = 0;
  eStates eStateTransition;
  bool bFltState = false;
  bool bResume = false;

  int16_t i16Rps = 0;

  int new_duty = 0;
  double speed_new = 0;

  u32TimeNow = millis();

  // send data only when you receive data:
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    command = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(command, DEC);
  }

  bFltState = fn_IsEncInFault(u32TimeNow, EncFlt.is_lo());
  eStateTransition = fn_checkForTransition(command);

  switch (controllerState)
  {
  case eStates::INIT:
    /* Init code */
    dbg_print("Init\n");
    command = 0;
    EncSlp.set_lo();

    led.set_lo();
    controllerState = eStates::PRE_OPERATIONAL;
    Serial.print("Boot-up\n");
    break;

  case eStates::PRE_OPERATIONAL:
    fn_PrintDbgMsg("pre_op\n", u32TimeNow);
    /* led blinks with 1 Hz */
    if ((u32TimeNow - u32LastTime) > 500)
    {
      led.toggle();
      u32LastTime = u32TimeNow;
    }
    EncSlp.set_lo();

    if (command == 'k')
    {
      Serial.println("***");
      Serial.print("Current value for Kp is: ");
      Serial.println(m_fKp);
      Serial.println("Now, send the new value from serial monitor, i.e. 0.06");
      float newKp;
      while (bResume == false)
      {
        if (Serial.available() > 0)
        {
          newKp = Serial.parseFloat();
          if (newKp != 0)
          {
            Serial.print("New Kp value is: ");
            Serial.println(newKp);
            m_fKp = newKp;
            bResume = true;
            command = 0;
          }
        }
      }
    }
    if (command == 't')
    {
      Serial.println("***");
      Serial.print("Current value for Ti is: ");
      Serial.println(m_fTi);
      Serial.println("Now, send the new value from serial monitor, i.e. 0.06");
      float newTi;
      while (bResume == false)
      {
        if (Serial.available() > 0)
        {
          newTi = Serial.parseFloat();
          if (newTi != 0)
          {
            Serial.print("New Ti value is: ");
            Serial.println(newTi);
            m_fTi = newTi;
            bResume = true;
            command = 0;
          }
        }
      }
    }

    if(command == 'c')
    {
      Serial.print("Using P-control, press v to change to PI-control\n");
      P_speed = new P_control(m_fKp);
      command = 0;
    }
    if(command == 'v')
    {
      Serial.print("Using PI-control, press c to change to P-control\n");
      P_speed = new PI_control(m_fKp, m_fTi, 0.1, 12500, 1);
      command = 0;
    }
    

    // eStateTransition = fn_checkForTransition(command);
    if (bFltState == true)
    {
      controllerState = eStates::STOPPED;
    }
    else if ((eStateTransition == eStates::INIT) ||
             (eStateTransition == eStates::OPERATIONAL))
    {
      controllerState = eStateTransition;
      command = 0;
    }

    break;

  case eStates::OPERATIONAL:

    // fn_PrintDbgMsg("op\n", u32TimeNow);
    /* led is on */
    led.set_hi();
    EncSlp.set_hi();

    if (command == 's')
    {
      Serial.println("***");
      Serial.print("Current value for TargetRpm is: ");
      Serial.println(targetRpm);
      Serial.println("Now, send the new value from serial monitor, i.e. 9050");
      uint16_t newTargetRpm;
      while (bResume == false)
      {
        if (Serial.available() > 0)
        {
          newTargetRpm = Serial.parseInt();
          if (newTargetRpm != 0)
          {
            Serial.print("New TargetRpm value is: ");
            Serial.println(newTargetRpm);
            targetRpm = newTargetRpm;
            bResume = true;
            command = 0;
          }
        }
      }
    }

    if (bUpdateSpeed == true)
    {
      // TestPin.toggle();
      i16Rps = Encoder.GetRpm();

      // Serial.print(">Rpm: ");
      Serial.print(i16Rps);
      Serial.print(" ");

      speed_new = P_speed->update(targetRpm, static_cast<double>(i16Rps));

      new_duty = (constrain(speed_new / targetRpm, 0.01, 0.99) * 100);

      // Serial.print("duty:");
      Serial.print(new_duty);
      Serial.println();

      ana_out.set(new_duty);
      bUpdateSpeed = false;
    }

    if (bFltState == true)
    {
      controllerState = eStates::STOPPED;
    }
    else if ((eStateTransition == eStates::INIT) ||
             (eStateTransition == eStates::PRE_OPERATIONAL))
    {
      controllerState = eStateTransition;
      command = 0;
    }

    break;

  default:
  case eStates::STOPPED:
    fn_PrintDbgMsg("stopped\n", u32TimeNow);
    EncSlp.set_lo();
    /* default / stopped code */
    /* led blinks with 2 Hz (250 ms for 2 Hz) */
    if ((u32TimeNow - u32LastTime) > 250)
    {
      led.toggle();
      u32LastTime = u32TimeNow;
    }

    if ((eStateTransition == eStates::INIT) ||
        (eStateTransition == eStates::PRE_OPERATIONAL) ||
        (eStateTransition == eStates::OPERATIONAL))
    {
      controllerState = eStateTransition;
      command = 0;
    }

    break;
  }
}

// interupt service routine of external int0
ISR(INT0_vect)
{
  Encoder.updatePos();
}

volatile uint8_t ui8PpsCnt = 0;
volatile uint8_t ui8SpeedCtrlCnt = 0;
ISR(TIMER2_COMPA_vect) // timer0 overflow interrupt
{
  // event to be exicuted every 2ms here
  ui8PpsCnt++;
  ui8SpeedCtrlCnt++;
  if (ui8PpsCnt >= 10)
  {
    /* code to be executed every 20 ms */
    ui8PpsCnt = 0;
    Encoder.updatePps();
  }
    
  if (ui8SpeedCtrlCnt >= 50)
  {
    /* code to be exectued every 100 ms */
    ui8SpeedCtrlCnt = 0;

    bUpdateSpeed = true;
  }
}

ISR(TIMER1_COMPA_vect)
{
  // action to be taken at the start of the on-cycle
  ana_out.pin_out.set_hi();
}

ISR(TIMER1_COMPB_vect)
{
  // action to be taken at the start of the off-cycle
  ana_out.pin_out.set_lo();
}

// ISR(TIMER2_COMPA_vect) // timer2 overflow interrupt
// {
//   // SysTime.Inc_SysTimeMs();
// }
