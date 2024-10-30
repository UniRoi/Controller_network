#include <iostream>
#include <thread>
#include <stdint.h>
#include <mutex>
#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>


// What I have to do:
/**
 * 
 * Request for Sensor data cyclic
 * Wait for Sensor data answer
 * Interprete Sensor answer
 * 
 * Write Motor data
 * Wait for response from Motor
 * Interprete Motor data
 * 
 */

struct stMessage
{
  uint8_t u8ID;
  uint8_t u8Task;
  uint16_t u16Addr;
  uint16_t u16Msg;
  uint16_t u16Crc;
};

static uint8_t ill_func = 0x01;
static uint8_t ill_addr = 0x02;
static uint8_t ill_data = 0x03;
static uint8_t dev_fail = 0x04;


static uint16_t set_op = 0x0001;
static uint16_t stp_node = 0x0002;
static uint16_t set_preop = 0x0080;
static uint16_t rst_node = 0x0081;
static uint16_t rst_comm = 0x0082;

static uint8_t motor_id = 0x01;
static uint8_t sensor_id = 0x02;



/** MODBUS FUNCTION  */
void fn_swap16(uint16_t *val)
{
    uint16_t tmp;
    tmp = (*val & 0x00FF);
    *val = (tmp << 8 | *val >> 8);
}

// Compute the MODBUS RTU CRC
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

int sendMessage(stMessage send_msg)
{
   int file, count;

//    if(len(send_msg)!=5){
//        printf("Invalid number of arguments, exiting [%d]!\n", argc);
//        return -2;
//    }
   if ((file = open("/dev/ttyS0", O_RDWR | O_NOCTTY))<0) // remove O_NDELAY
   {
      perror("UART: Failed to open the file.\n");
      return -1;
   }

   struct termios options;
   tcgetattr(file, &options);
   cfsetospeed(&options, B115200); // Set up the communication options: 115200 baud
   cfmakeraw(&options); // set raw mode
   options.c_cc[VMIN]=1; // min byte number of characters for raw mode
   options.c_cc[VTIME] = 0;
   tcflush(file, TCIFLUSH); // discard file information
   tcsetattr(file, TCSANOW, &options);

   struct stMessage tMsg;

   // populate the message with integer values in binary format
//    tMsg.u8ID = (uint8_t)atoi(argv[1]);
//    tMsg.u8Task = (uint8_t)atoi(argv[2]);
//    tMsg.u16Addr = (uint16_t)atol(argv[3]);
//    tMsg.u16Msg = (uint16_t)atol(argv[4]);

   tMsg.u8ID = send_msg.u8ID;
   tMsg.u8Task = send_msg.u8Task;
   tMsg.u16Addr = send_msg.u16Addr;
   tMsg.u16Msg = send_msg.u16Msg;

   uint8_t msg[6];

   msg[0] = tMsg.u8ID;
   msg[1] = tMsg.u8Task;
   msg[2] = (uint8_t)((tMsg.u16Addr & 0xFF00) >> 8);
   msg[3] = (uint8_t)((tMsg.u16Addr & 0x00FF) >> 0);
   msg[4] = (uint8_t)((tMsg.u16Msg & 0xFF00) >> 8);
   msg[5] = (uint8_t)((tMsg.u16Msg & 0x00FF) >> 0);
   
   tMsg.u16Crc = ModRTU_CRC(msg, 6);   
   
   // from his notes: this number has low and high bytes swapped
   // hopefully this doesn't explode in the future ^^ 
//    tMsg.u16Crc = ModRTU_CRC(msg, 6);
   // tMsg.u16Crc = 0x55aa;




   printf("Sent request: %02x %02x %04x %04x %04x\n", tMsg.u8ID, tMsg.u8Task, tMsg.u16Addr, tMsg.u16Msg, tMsg.u16Crc);

   fn_swap16(&tMsg.u16Addr);
   fn_swap16(&tMsg.u16Msg);
   fn_swap16(&tMsg.u16Crc);

   // send the string plus the null character
   if (count = write(file, (void*)&tMsg, 8)<0)
   {
      perror("Failed to write to the output\n");
      return -1;
   }
   close(file);

   return 0;
}

int recMessage(uint8_t *rec_msg)
{
   int file, count;

   if ((file = open("/dev/ttyS0", O_RDWR | O_NOCTTY))<0) // remove O_NDELAY
   {
      perror("UART: Failed to open the file.\n");
      return -1;
   }

   struct termios options;
   tcgetattr(file, &options);
   cfsetospeed(&options, B115200); // Set up the communication options: 115200 baud
   cfmakeraw(&options); // set raw mode
   options.c_cc[VMIN]=1; // min byte number of characters for raw mode
   options.c_cc[VTIME] = 0;
   tcflush(file, TCIFLUSH); // discard file information
   tcsetattr(file, TCSANOW, &options);

   const size_t MSG_LEN = 8;
   uint8_t rd_msg[MSG_LEN];

   if ((count = read(file, rd_msg, sizeof(rd_msg)))<0)
   {
      perror("Failed to read from the input\n");
      return -1;
   }

   // CRC check
   uint16_t rd_crc = ModRTU_CRC(rd_msg, 6);

   if (count==0) printf("There was no data available to read!\n");
   else if(rd_crc != (uint16_t) ((((uint16_t)rd_msg[6])  << 8) | (uint16_t)rd_msg[7]))
   {
      printf("CRC Error, will not use message. Received: 0x%02x%02x Calculated: 0x%04x\n", rd_msg[6], rd_msg[7], rd_crc);
   }
   else {
      printf("The following was read in [%d]: %02x %02x %02x%02x %02x%02x %02x%02x\n",count, rd_msg[0], rd_msg[1],rd_msg[2], rd_msg[3],rd_msg[4], rd_msg[5],rd_msg[6], rd_msg[7]);
      rec_msg = rd_msg;
   }

   close(file);
   
   return 0;
}


void handleError(stMessage rec_mes)
{
    if(rec_mes.u8Task == ill_func) printf("Illegal Function\n");
    else if(rec_mes.u8Task == ill_addr) printf("Illegal Data Address\n");
    else if(rec_mes.u8Task == ill_data) printf("Illegal Data Value\n");
    else if(rec_mes.u8Task == dev_fail) printf("Server Device Failure\n");
    else{
        printf("Unknown error value. Message: %d\n", rec_mes);
    }
}

int encodeMessage(uint8_t *rec_msg, uint8_t *response)
{ 
    stMessage msg;
    msg.u8ID = rec_msg[0];
    msg.u8Task = rec_msg[1];
    msg.u16Addr = (uint16_t) ((((uint16_t)rec_msg[2])  << 8) | (uint16_t)rec_msg[3]);
    msg.u16Msg = (uint16_t) ((((uint16_t)rec_msg[4])  << 8) | (uint16_t)rec_msg[5]);
  
    if(msg.u8ID == motor_id) //Motor
    {
        if((msg.u8Task & 0xF0) == 0x80)
            {
                printf("Error rec motor values. Rec msg: %d\n", rec_msg);
                handleError(msg);
                return -1;
            } // Error

        if((msg.u8Task & 0x0F) == 0x06 && msg.u16Addr == 0x0001)
        {
            printf("Response Motor Value: %d\n", msg.u16Msg);
            *response = (uint16_t) ((((uint16_t)rec_msg[4])  << 8) | (uint16_t)rec_msg[5]);
        } // Writing Task
        else if ((msg.u8Task & 0x0F) == 0x03 && msg.u16Addr == 0x0000)
        {
            printf("Response Motor State: %d\n", msg.u16Msg);
            *response = (uint16_t) ((((uint16_t)rec_msg[4])  << 8) | (uint16_t)rec_msg[5]);
        } // Reading Task
        else {
            printf("Wrong Task code. Rec msg: %d\n", rec_msg);
            return -1;
        }
        return 1;
    } 
    else if (msg.u8ID == sensor_id) //Sensor
    {
        if((msg.u8Task & 0xF0) == 0x80)
        {
            printf("Error rec sensor values. Rec msg: %d\n", rec_msg);
            handleError(msg);
            return -1;
        } // Error

        if((msg.u8Task & 0x0F) == 0x03 && msg.u16Addr == 0x0001)
        {
            printf("Response Sensor Value: %d\n", msg.u16Msg);
            *response = (uint16_t) ((((uint16_t)rec_msg[4])  << 8) | (uint16_t)rec_msg[5]);
        } // Reading Task
        else if((msg.u8Task & 0x0F) == 0x03 && msg.u16Addr == 0x0001)
        {
            printf("Response Sensor State: %d\n", msg.u16Msg);
            *response = (uint16_t) ((((uint16_t)rec_msg[4])  << 8) | (uint16_t)rec_msg[5]);
        } 
        else {
            printf("Wrong Task code. Rec msg: %d\n", rec_msg);
            return -1;
        }
        return 1;
    }
    return 0;
}


// // write Motor data
stMessage buildMessage(uint8_t id, uint8_t task, uint16_t addr, uint16_t data)
{
    stMessage m_message;
    m_message.u8ID = id; 
    m_message.u8Task = task; 
    m_message.u16Addr = addr; 
    m_message.u16Msg = data;


    uint8_t msg[6];
    msg[0] = id; 
    msg[1] = task; 
    msg[2] = (addr >> 8) & 0xFF; // High byte
    msg[3] = addr & 0xFF;
    msg[4] = (data >> 8) & 0xFF; // High byte
    msg[5] = data & 0xFF;
    m_message.u16Crc = ModRTU_CRC(msg, 6);

    return m_message;
}

int main(void)
{
    uint8_t motorValue[2] = {0};
    uint16_t set_motorValue = 0x0000;
    uint8_t received_message[8] = {0};
    uint8_t sensor_value[2] = {0};
    uint8_t motor_state[2] = {0};


    while(true)
    {
        // Send Sensor Request   
        sendMessage(buildMessage(sensor_id, 0x03, 0x0001, 0x0001));
        // Wait for Sensor Response
        recMessage(received_message);
        int ret = encodeMessage(received_message, sensor_value);

        if(ret >= 0)
        {
            // Interprete Sensor Data and then
            // sensor_value can be used
            // calculating new motor value
        }

        // Send Motor Request for state
        sendMessage(buildMessage(motor_id, 0x03, 0x0000, 0x0000));
        recMessage(received_message);
        ret = encodeMessage(received_message, motor_state);

        if(ret >= 0)
        {
            // Interprete Motor state
            // change motor state ........
        }

        // Send Motor Data
        sendMessage(buildMessage(motor_id, 0x06, 0x0001, set_motorValue));
        // Wait for Motor Response
        recMessage(received_message);
        ret = encodeMessage(received_message, motorValue);

        if(ret > 0)
        {
            continue;
        }
    }
    
    return 0;
}

// g++ -o controller main.cpp 
// ./controller


