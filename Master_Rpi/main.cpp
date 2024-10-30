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

//    msg[0] = tMsg.u8ID;
//    msg[1] = tMsg.u8Task;
//    msg[2] = (uint8_t)((tMsg.u16Addr & 0xFF00) >> 8);
//    msg[3] = (uint8_t)((tMsg.u16Addr & 0x00FF) >> 0);
//    msg[4] = (uint8_t)((tMsg.u16Msg & 0xFF00) >> 8);
//    msg[5] = (uint8_t)((tMsg.u16Msg & 0x00FF) >> 0);
   
   // from his notes: this number has low and high bytes swapped
   // hopefully this doesn't explode in the future ^^ 
//    tMsg.u16Crc = ModRTU_CRC(msg, 6);
   // tMsg.u16Crc = 0x55aa;

   tMsg.u8ID = send_msg.u8ID;
   tMsg.u8Task = send_msg.u8Task;
   tMsg.u16Addr = send_msg.u16Addr;
   tMsg.u16Msg = send_msg.u16Msg;
   tMsg.u16Crc = ModRTU_CRC(send_msg, 6);

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

int recMessage(stMessage *rec_msg)
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
      *rec_msg = rd_msg;
   }

   close(file);
   
   return 0;
}


void handleError(stMessage rec_mes)
{
    if(rec_mes[2] == ill_func) printf("Illegal Function\n");
    else if(rec_mes[2] == ill_addr) printf("Illegal Data Address\n");
    else if(rec_mes[2] == ill_data) printf("Illegal Data Value\n");
    else if(rec_mes[2] == dev_fail) printf("Server Device Failure\n");
    else{
        printf("Unknown error value. Message: %d\n", rec_mes);
    }
}

int encodeMessage(stMessage *rec_msg, uint16_t *sensor_value)
{ 
    if(rec_msg.u8ID == 0x01) //Motor
    {
        if((rec_msg.u8Task & 0xF0) == 0x80) 
        {
            // TODO: Handle Error
            printf("Error writing motor values. Rec msg: %d\n", rec_msg);
            handleError(rec_msg);
            return -1;
        } // Error
        else{
            printf("Data successfully written\n");
        }
        return 1;

    } else if (rec_msg.u8ID == 0x02) //Sensor
    {
        if((rec_msg.u8Task & 0xF0) == 0x80)
        {
            printf("Error rec sensor values. Rec msg: %d\n", rec_msg);
            handleError(rec_msg);
            return -1;
        } // Error

        if((rec_msg.u8Task & 0x0F) == 0x03)
        {
            *sensor_value = (uint16_t) ((((uint16_t)rd_msg[4])  << 8) | (uint16_t)rd_msg[5]);
        } // Reading Task
        else {
            printf("Wrong Task code. Rec msg: %d\n", rec_msg);
            return -1;
        }
        return 1;
    }
}


// // write Motor data
// stMessage motorMessage(uint16_t reg, uint16_t data)
// {
//     stMessage m_motor_message;
//     m_motor_message.u8ID = 0x01; // 01 for Motor
//     m_motor_message.u8Task = 0x06; // 03 for write
//     m_motor_message.u16Addr = reg; // register
//     m_motor_message.u16Msg = data;
//     m_motor_message.u16Crc = detChecksum();
//     return m_motor_message;
// }


int main(void)
{
    stMessage req_sensor = 0x02 0x03 0x0001 0x0001 0xFFFF; // initial sensor Message
    stMessage write_motor = 0x01 0x06 0x0001 0x0001 0xFFFF; // initial motor Message

    stMessage *received_message;
    uint16_t motorValue;
    uint16_t *sensorValue;

    while(true)
    {
        // Send Sensor Request   
        sendMessage(req_sensor);
        // Wait for Sensor Response
        recMessage(received_message);
        int ret = encodeMessage(received_message, sensorValue);

        if(ret > 0)
        {
            // Interprete Sensor Data and then
            motorValue = received_message->u16Msg;
        }

        // Send Motor Data
        write_motor.u16Msg = motorValue;
        sendMessage(write_motor);
        // Wait for Motor Response
        recMessage(received_message);
        int ret = encodeMessage(received_message, sensorValue);

        if(ret > 0)
        {
            // Message from Motor was okay
            continue;
        }
    }
    
    
    return 0;
}


