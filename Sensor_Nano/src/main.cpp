
#include <Arduino.h>

#include <avr/interrupt.h>
#include <SoftwareSerial.h>

#define pin_tx 9
#define pin_rx 8

SoftwareSerial mySerial(pin_rx, pin_tx);

// #define DEBUG_PRINT // enable debug print messages
#define NODE_ID 0x02


const int analogPin = A0;           // Analog pin to read the voltage
const float referenceVoltage = 5.0; // Reference voltage
const float thresholdVoltage = 3.0; // Threshold for light/dark detection
const int sampleInterval = 20;      // Sample interval in milliseconds
const unsigned long timeout = 5000; // 5 seconds timeout in milliseconds

const int numSamples = 5;      // Number of samples for moving average
float rpmSamples[numSamples];  // Array to store past RPM readings
int sampleIndex = 0;           // Index to keep track of the current sample
bool samplesFilled = false;    // Flag to check if all samples are filled
float alpha = 0.3;    // Smoothing factor (adjust between 0.1 and 0.3 for best results)
float smoothedRPM = 0;

int analogValue = 0;
float voltage = 0.0;
bool isDark = false;                // Tracks if the sensor is in a dark state
unsigned long lastRoundTime = 0;    // Time of the last dark-to-light transition
unsigned long firstRoundTime = 0;   // Time of the first detected round after timeout
float rpm = 0.0;                    // Calculated RPM value
bool timeoutOccurred = false;       // Tracks if a timeout has reset the RPM

unsigned long previousMillis = 0;   // Stores the last time the sensor was updated

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


static uint16_t fn_rd_rpm();


static struct stFunctions ProtocolFunktions[] =
    {
        {E_READ, 6, fn_rd_rpm, nullptr}
};

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




void setup()
{
  // put your setup code here, to run once:

  pinMode(pin_tx, OUTPUT);
  pinMode(pin_rx, INPUT);





  // Add serial for part 2
  Serial.begin(115200, SERIAL_8N1);
  mySerial.begin(115200);

  u32LastTime = millis();
}

#ifdef DEBUG_PRINT
#include <stdio.h>
#define dbg_print Serial.print
#else
#define dbg_print(...) /**/
#endif



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

void updateSensor() {
  // Read the analog value and convert it to voltage
  analogValue = analogRead(analogPin);
  voltage = (analogValue * referenceVoltage) / 1023.0;

  // Check if the sensor detects "dark" or "light"
  if (voltage < thresholdVoltage) { // Dark state detected
    if (!isDark) {                  // If previously light, detect a round
      isDark = true;                // Set state to dark

      // Check if we are restarting after a timeout
      if (timeoutOccurred) {        
        firstRoundTime = millis();  // Start new timing interval
        timeoutOccurred = false;    // Clear the timeout flag
        lastRoundTime = firstRoundTime; // Reset last round time for next interval
      } else {
        // Calculate RPM based on time between two consecutive dark states
        unsigned long currentTime = millis();
        unsigned long timeDifference = currentTime - lastRoundTime;
        rpm = (60000.0 / timeDifference); // Calculate RPM
        lastRoundTime = currentTime;      // Update last round time
      }
    }
  } else {                     // Light state detected
    isDark = false;            // Reset state to light
  }
   // Add the current RPM to the samples array
  rpmSamples[sampleIndex] = rpm;
  sampleIndex = (sampleIndex + 1) % numSamples;

  // Check if all samples have been filled
  if (sampleIndex == 0) {
    samplesFilled = true;
  }

  // Calculate the moving average only if samples are filled
  if (samplesFilled) {
    float smoothedRPM = 0;
    for (int i = 0; i < numSamples; i++) {
      smoothedRPM += rpmSamples[i];
    }
    smoothedRPM = smoothedRPM / numSamples;  // Use the averaged RPM value
    smoothedRPM = (alpha * rpm) + ((1 - alpha) * smoothedRPM);
    rpm = smoothedRPM;
  }
  // Timeout: Check if more than 5 seconds have passed since the last round
  if (millis() - lastRoundTime >= timeout) {
    rpm = 0.0;                 // Set RPM to zero if timeout occurs
    timeoutOccurred = true;    // Set timeout flag to reset calculation
  }


}

void loop()
{
  // check for msg
  // handle incomming messages
  // do update motor
  unsigned long currentMillis = millis(); // Get the current time
  int iResult = 0;
  struct stMessage tMsg;

  iResult = fn_checkForMsg(&tMsg);

  if (iResult > 0)
  {
    fn_HandleMsg(&tMsg);
    fn_TransmitResponse(&tMsg);
  }

  // fn_updateSensor();
  

  // Check if the sample interval has passed
  if (currentMillis - previousMillis >= sampleInterval) {
    previousMillis = currentMillis; // Save the last time the sensor was updated
    updateSensor();                 // Update sensor reading and calculate RPM
    
  }

}


// ---------------------------------------------------------------------------------------
// Supported Protocol functions
// ---------------------------------------------------------------------------------------




static uint16_t fn_rd_rpm()
{
  return (uint16_t)(rpm*100);
}



