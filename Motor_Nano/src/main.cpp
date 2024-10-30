#include <encoder.h>

#include <digital_out.h>
#include <analog_out.h>
#include <speed_control.h>
#include <Arduino.h>

#include <avr/interrupt.h>


// #define DEBUG_PRINT // enable debug print messages
#define NODE_ID 0x01

#define MAX_SPEED 12500
#define MIN_SPEED 20

static const uint8_t ill_func = 0x01;
static const uint8_t ill_addr = 0x02;
static const uint8_t ill_data = 0x03;
static const uint8_t dev_fail = 0x04;

struct stMessage
{
  uint8_t u8ID;
  uint8_t u8Task;
  uint16_t u16Addr;
  uint16_t u16Msg;
  uint16_t u16Crc;
};

enum eFunction
{
  E_READ = 3,
  E_WRITE = 6,
  E_RDWR = 9
};

struct stFunctions
{
  enum eFunction eFunc;
  uint16_t u16Addr;
  uint16_t (*rd_fnc)(void);
  int (*wr_fnc)(uint16_t);
};

static uint16_t fn_rd_state();
static uint16_t fn_rd_rpm();
static int fn_wr_state(uint16_t wr_val);
static int fn_wr_speed(uint16_t wr_val);

static struct stFunctions ProtocolFunktions[] =
    {
        {E_RDWR, 1, fn_rd_state, fn_wr_state},
        {E_READ, 5, fn_rd_rpm, nullptr},
        {E_WRITE, 9, nullptr, fn_wr_speed}};

enum eStates
{
  NO_STATE = 0x00,
  OPERATIONAL = 0x01,
  STOPPED = 0x02,
  PRE_OPERATIONAL = 0x80,
  INIT = 0x81,
  E_NODE_RESET_COM = 0x82
};

// enum eCANOpenStates
// {
//   E_NODE_OPERATIONAL = 0x01,
//   E_NODE_STOP = 0x02,
//   E_NODE_PRE_OPERATIONAL = 0x80,
//   E_NODE_RESET = 0x81,
//   E_NODE_RESET_COM = 0x82
// };

static uint32_t u32LastTime;

volatile bool bUpdateSpeed;
static int command = 0;
static uint16_t targetRpm = 10000;

encoder Encoder(2, 3);
Analog_out ana_out(1);
Digital_out led(5); // Pin D13
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
  Serial.begin(115200, SERIAL_8N1);

  u32LastTime = millis();
  P_speed = new PI_control(m_fKp, m_fTi, 0.1, 12500, 1);
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

// ---------------------------------------------------------------------------------------
// Modbus RTU functions
// ---------------------------------------------------------------------------------------

/// @brief Calculate a ModRTU Crc value for a buffer of a given length
/// @param buf [uint8_t] pointer to buffer
/// @param len [int] length of buffer
/// @return [uint16_t] calculated crc value
uint16_t ModRTU_CRC(uint8_t buf[], int len)
{
   uint16_t crc = 0xFFFF;
   for (int pos = 0; pos < len; pos++) 
   {
   crc ^= (uint16_t)buf[pos];
      // XOR byte into least sig. byte of crc
      for (int i = 8; i != 0; i--) 
      {
         if ((crc & 0x0001) != 0) 
         {
            crc >>= 1;
            crc ^= 0xA001;
         }
         else
         {
            crc >>= 1;
         }
      }
      // Loop over each bit
      // If the LSB is set
      // Shift right and XOR 0xA001
      // Else LSB is not set
      // Just shift right
   }
   // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
   return crc;
}

// ---------------------------------------------------------------------------------------
// receiving and sending functions
// ---------------------------------------------------------------------------------------

/// @brief check if a message was received
/// @param tMsg [struct stMessage] pointer to message struct
/// @return [int] 0 if no msg was received, 1 if a msg was received, -1 for false msg len, -2 for crc error
int fn_checkForMsg(struct stMessage *tMsg)
{
  int ret = 0;
  size_t msglen = 0;
  uint8_t u8DataFrame[8] = {0};
  uint16_t rec_crc = 0;

  if (Serial.available() > 0)
  {
    msglen = Serial.readBytes(u8DataFrame, 8);

    rec_crc = ModRTU_CRC(u8DataFrame, 6);

    if(rec_crc == (uint16_t) ((((uint16_t)u8DataFrame[6])  << 8) | (uint16_t)u8DataFrame[7]))
    {
      if (u8DataFrame[0] == NODE_ID)
      { // Requested id is motor id
        if (msglen == 8)
        {
          tMsg->u8ID = u8DataFrame[0];
          tMsg->u8Task = u8DataFrame[1];
          tMsg->u16Addr = (uint16_t) ((((uint16_t)u8DataFrame[2]) << 8) | (uint16_t)u8DataFrame[3]);
          tMsg->u16Msg = (uint16_t) ((((uint16_t)u8DataFrame[4])  << 8) | (uint16_t)u8DataFrame[5]);
          // tMsg->u16Crc = (uint16_t) ((((uint16_t)u8DataFrame[6])  << 8) | (uint16_t)u8DataFrame[7]);
          tMsg->u16Crc = rec_crc;

          ret = 1;
        }
        else
        {
          // received a msg but not 8 bytes
          ret = -1;
        }
      }
    }
    else
    {
      // crc is not valid
      ret = -2;
    }
  }

  return ret;
}

/// @brief Transmit a message, currently only fixed length messages are transmitted
/// @param tMsg [struct stMessage] pointer to message
/// @return -2 if tMsg was nullpointer, -1 if not 8 bytes were transmitted, 0 on success
int fn_TransmitResponse(struct stMessage *tMsg)
{
  int iRet = -2;
  size_t len = 0;
  uint8_t u8DataFrame[8] = {0};
  uint16_t snd_crc = 0;

  if (tMsg != nullptr)
  {

    u8DataFrame[0] = tMsg->u8ID;
    u8DataFrame[1] = tMsg->u8Task;
    u8DataFrame[2] = (uint8_t)((tMsg->u16Addr & 0xFF00) >> 8);
    u8DataFrame[3] = (uint8_t)((tMsg->u16Addr & 0x00FF) >> 0);
    u8DataFrame[4] = (uint8_t)((tMsg->u16Msg & 0xFF00) >> 8);
    u8DataFrame[5] = (uint8_t)((tMsg->u16Msg & 0x00FF) >> 0);

    snd_crc = ModRTU_CRC(u8DataFrame, 6);
    // snd_crc += 1; // simulate a sending crc error
    u8DataFrame[6] = (uint8_t)((snd_crc & 0xFF00) >> 8);
    u8DataFrame[7] = (uint8_t)((snd_crc & 0x00FF) >> 0);

    len = Serial.write(u8DataFrame, 8);
    if (len == 8)
    {
      iRet = 0;
    }
    else
    {
      iRet = -1;
    }
  }

  return iRet;
}

/// @brief Handle the received message and return new message with the same pointer
/// @param tMsg [struct stMessage] pointer to message 
void fn_HandleMsg(struct stMessage *tMsg)
{
  uint8_t i = 0;
  bool bFound = false;
  bool bIllAddr = true;
  bool bIllFunc = true;
  bool bIllData = true;
  // check for crc error here

  for (i = 0; i < sizeof(ProtocolFunktions) / sizeof(ProtocolFunktions[0]); i++)
  {
    if (tMsg->u16Addr == ProtocolFunktions[i].u16Addr)
    {
      bIllAddr = false;
      
      if ((tMsg->u8Task == (uint8_t)E_READ) && (ProtocolFunktions[i].rd_fnc != nullptr))
      {
        bIllFunc = false;
        bIllData = false;
        bFound = true;
        tMsg->u16Msg = ProtocolFunktions[i].rd_fnc();
      }
      else if ((tMsg->u8Task == (uint8_t)E_WRITE) && (ProtocolFunktions[i].wr_fnc != nullptr))
      {
        bIllFunc = false;
        bFound = true;
        bIllData = (bool)ProtocolFunktions[i].wr_fnc(tMsg->u16Msg);
      }
    }
  }

  // set errors if not found
  if (bIllAddr == true)
  {
    tMsg->u8Task |= (1 << 7);
    tMsg->u16Msg = (uint16_t)ill_addr;
  }
  else if (bIllFunc == true)
  {
    tMsg->u8Task |= (1 << 7);
    tMsg->u16Msg = (uint16_t)ill_func;
  }
  else if (bIllData == true)
  {
    tMsg->u8Task |= (1 << 7);
    tMsg->u16Msg = (uint16_t)ill_data;
  }
}



void fn_updateMotor(void)
// void loop()
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

  // bFltState = fn_IsEncInFault(u32TimeNow, EncFlt.is_lo());
  // eStateTransition = fn_checkForTransition(command);

  switch (controllerState)
  {
  case eStates::INIT:
    /* Init code */
    // dbg_print("Init\n");
    command = 0;
    EncSlp.set_lo();

    led.set_lo();
    controllerState = eStates::PRE_OPERATIONAL;
    // Serial.print("Boot-up\n");
    break;

  case eStates::PRE_OPERATIONAL:
    // fn_PrintDbgMsg("pre_op\n", u32TimeNow);
    /* led blinks with 1 Hz */
    if ((u32TimeNow - u32LastTime) > 500)
    {
      led.toggle();
      u32LastTime = u32TimeNow;
    }
    EncSlp.set_lo();

    // if (command == 'k')
    // {
    //   Serial.println("***");
    //   Serial.print("Current value for Kp is: ");
    //   Serial.println(m_fKp);
    //   Serial.println("Now, send the new value from serial monitor, i.e. 0.06");
    //   float newKp;
    //   while (bResume == false)
    //   {
    //     if (Serial.available() > 0)
    //     {
    //       newKp = Serial.parseFloat();
    //       if (newKp != 0)
    //       {
    //         Serial.print("New Kp value is: ");
    //         Serial.println(newKp);
    //         m_fKp = newKp;
    //         bResume = true;
    //         command = 0;
    //       }
    //     }
    //   }
    // }
    // if (command == 't')
    // {
    //   Serial.println("***");
    //   Serial.print("Current value for Ti is: ");
    //   Serial.println(m_fTi);
    //   Serial.println("Now, send the new value from serial monitor, i.e. 0.06");
    //   float newTi;
    //   while (bResume == false)
    //   {
    //     if (Serial.available() > 0)
    //     {
    //       newTi = Serial.parseFloat();
    //       if (newTi != 0)
    //       {
    //         Serial.print("New Ti value is: ");
    //         Serial.println(newTi);
    //         m_fTi = newTi;
    //         bResume = true;
    //         command = 0;
    //       }
    //     }
    //   }
    // }

    // P_speed = new PI_control(m_fKp, m_fTi, 0.1, 12500, 1);

    // if(command == 'c')
    // {
    //   Serial.print("Using P-control, press v to change to PI-control\n");
    //   P_speed = new P_control(m_fKp);
    //   command = 0;
    // }
    // if(command == 'v')
    // {
    //   Serial.print("Using PI-control, press c to change to P-control\n");
    //   command = 0;
    // }

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

    // if (command == 's')
    // {
    //   Serial.println("***");
    //   Serial.print("Current value for TargetRpm is: ");
    //   Serial.println(targetRpm);
    //   Serial.println("Now, send the new value from serial monitor, i.e. 9050");
    //   uint16_t newTargetRpm;
    //   while (bResume == false)
    //   {
    //     if (Serial.available() > 0)
    //     {
    //       newTargetRpm = Serial.parseInt();
    //       if (newTargetRpm != 0)
    //       {
    //         Serial.print("New TargetRpm value is: ");
    //         Serial.println(newTargetRpm);
    //         targetRpm = newTargetRpm;
    //         bResume = true;
    //         command = 0;
    //       }
    //     }
    //   }
    // }

    if (bUpdateSpeed == true)
    {
      // TestPin.toggle();
      i16Rps = Encoder.GetRpm();

      // Serial.print(">Rpm: ");
      // Serial.print(i16Rps);
      // Serial.print(" ");

      speed_new = P_speed->update(targetRpm, static_cast<double>(i16Rps));

      new_duty = (constrain(speed_new / targetRpm, 0.01, 0.99) * 100);

      // Serial.print("duty:");
      // Serial.print(new_duty);
      // Serial.println();

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
    // fn_PrintDbgMsg("stopped\n", u32TimeNow);
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

void loop()
{
  // check for msg
  // handle incomming messages
  // do update motor
  int iResult = 0;
  struct stMessage tMsg;

  iResult = fn_checkForMsg(&tMsg);

  if (iResult > 0)
  {
    fn_HandleMsg(&tMsg);
    fn_TransmitResponse(&tMsg);
  }

  fn_updateMotor();
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

// ---------------------------------------------------------------------------------------
// Supported Protocol functions
// ---------------------------------------------------------------------------------------


static uint16_t fn_rd_state()
{
  return (uint16_t)controllerState;
}

static uint16_t fn_rd_rpm()
{
  return Encoder.GetRpm();
}

static int fn_wr_state(uint16_t wr_val)
{
  int iRet = 0;

  if(wr_val == (uint16_t)INIT)
  {
    controllerState = INIT;
  }
  else if(wr_val == (uint16_t)STOPPED)
  {
    controllerState = STOPPED;
  }
  else if(wr_val == (uint16_t)OPERATIONAL)
  {
    controllerState = OPERATIONAL;
  }
  else if(wr_val == (uint16_t)PRE_OPERATIONAL)
  {
    controllerState = PRE_OPERATIONAL;
  }
  else if(wr_val == (uint16_t)E_NODE_RESET_COM)
  {
    controllerState = E_NODE_RESET_COM;
  }
  else
  {
    iRet = 1;
  }

  return iRet;
}

static int fn_wr_speed(uint16_t wr_val)
{
  int iRet = 1;
  if ((wr_val < MAX_SPEED) && (wr_val > MIN_SPEED))
  {
    targetRpm = wr_val;
    iRet = 0;
  }

  return iRet;
}