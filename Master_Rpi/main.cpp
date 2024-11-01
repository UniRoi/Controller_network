#include <iostream>
#include <thread>
#include <stdint.h>
#include <mutex>
#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include <cstring>

struct stMessage
{
  uint8_t u8ID;
  uint8_t u8Task;
  uint16_t u16Addr;
  uint16_t u16Msg;
  uint16_t u16Crc;
};

// Error codes
static uint8_t ill_func = 0x01;
static uint8_t ill_addr = 0x02;
static uint8_t ill_data = 0x03;
static uint8_t dev_fail = 0x04;

// OP Set Modes of nodes
static uint16_t set_op = 0x0001;
static uint16_t stp_node = 0x0002;
static uint16_t set_preop = 0x0080;
static uint16_t rst_node = 0x0081;
static uint16_t rst_comm = 0x0082;


// OP Modes of nodes
static uint8_t no_state = 0;//0x00;
static uint8_t operational = 0x01;
static uint8_t stopped = 0x02;
static uint8_t pre_operational = 0x80;
static uint8_t init = 0x81;
static uint8_t e_node_reset_com = 0x82;


// different node ids
static uint8_t motor_id = 0x01;
static uint8_t sensor_id = 0x02;

// Address
static uint16_t motor_state_addr = 0x0001;
static uint16_t motor_rpm_addr = 0x0005;
static uint16_t sensor_rpm_addr = 0x0006;

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



int send_and_rec(uint8_t *rec_msg, stMessage send_msg)
{
    int file, count;

   if ((file = open("/dev/ttyS0", O_RDWR | O_NOCTTY))<0) // remove O_NDELAY
   {
      perror("UART: Failed to open the file.\n");
      return -1;
   }

   struct termios options;

   tcgetattr(file, &options);

   // Set up the communication options: 115200 baud
   cfsetospeed(&options, B115200);
   // set raw mode
   cfmakeraw(&options);
   options.c_cc[VMIN]=1; // min byte number of characters for raw mode
   options.c_cc[VTIME] = 0;
          
   tcflush(file, TCIFLUSH); // discard file information
   tcsetattr(file, TCSANOW, &options);

   const size_t MSG_LEN = 8;
   uint8_t msg[MSG_LEN];
   uint8_t rd_msg[MSG_LEN];
   struct stMessage tMsg;

   tMsg.u8ID = send_msg.u8ID;
   tMsg.u8Task = send_msg.u8Task;
   tMsg.u16Addr = send_msg.u16Addr;
   tMsg.u16Msg = send_msg.u16Msg;

   msg[0] = tMsg.u8ID;
   msg[1] = tMsg.u8Task;
   msg[2] = (uint8_t)((tMsg.u16Addr & 0xFF00) >> 8);
   msg[3] = (uint8_t)((tMsg.u16Addr & 0x00FF) >> 0);
   msg[4] = (uint8_t)((tMsg.u16Msg & 0xFF00) >> 8);
   msg[5] = (uint8_t)((tMsg.u16Msg & 0x00FF) >> 0);
   
   tMsg.u16Crc = ModRTU_CRC(msg, 6);   

   printf("Sent request: %02x %02x %04x %04x %04x\n", tMsg.u8ID, tMsg.u8Task, tMsg.u16Addr, tMsg.u16Msg, tMsg.u16Crc);

   fn_swap16(&tMsg.u16Addr);
   fn_swap16(&tMsg.u16Msg);
   fn_swap16(&tMsg.u16Crc);

   // send the string plus the null character
   if (count = write(file, (void*)&tMsg, 8)<0)
   {
      perror("Failed to write to the output\n");
      close(file);
      return -1;
   }

   usleep(100000);

   unsigned char receive[100];

   if ((count = read(file, rd_msg, sizeof(rd_msg)))<0)
   {
      perror("Failed to read from the input\n");
      return -1;
   }
   uint16_t rd_crc = ModRTU_CRC(rd_msg, 6);

   if (count==0) printf("There was no data available to read!\n");
   else if(rd_crc != (uint16_t) ((((uint16_t)rd_msg[6])  << 8) | (uint16_t)rd_msg[7]))
   {
      printf("CRC Error, will not use message. Received: 0x%02x%02x Calculated: 0x%04x\n", rd_msg[6], rd_msg[7], rd_crc);
   }
   else {
      // receive[count]=0;  //There is no null character sent by the Arduino
      printf("The following was read in [%d]: %02x %02x %02x%02x %02x%02x %02x%02x\n",count, rd_msg[0], rd_msg[1],rd_msg[2], rd_msg[3],rd_msg[4], rd_msg[5],rd_msg[6], rd_msg[7]);
      memcpy(rec_msg, rd_msg, sizeof(rd_msg));
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

        if((msg.u8Task & 0x0F) == 0x06)
        {
            printf("Response Motor Value: %d\n", msg.u16Msg);
            *response = (uint16_t) ((((uint16_t)rec_msg[4])  << 8) | (uint16_t)rec_msg[5]);
        } // Writing Task
        else if ((msg.u8Task & 0x0F) == 0x03)
        {
            printf("Response Motor: %d\n", msg.u16Msg);
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
    msg[2] = (addr >> 8) & 0xFF;
    msg[3] = addr & 0xFF;
    msg[4] = (data >> 8) & 0xFF;
    msg[5] = data & 0xFF;
    m_message.u16Crc = ModRTU_CRC(msg, 6);

    return m_message;
}



int main(void)
{
    uint8_t motor_rpm[2] = {0};
    uint16_t set_motorValue = 10000;
    uint8_t received_message[8] = {0};
    uint8_t sensor_value[2] = {0};
    uint8_t motor_state[2] = {0};
    uint8_t response[2] = {0};
    //motor_state = no_state;

    while(*motor_state != operational)
    {
        send_and_rec(received_message, buildMessage(motor_id, 0x03, motor_state_addr, 0x0000));

        if(encodeMessage(received_message, response) > 0)
        {
            motor_state[0]  = response[0];
            motor_state[1]  = response[1];
            printf("Motor State: %04x\n", *motor_state);
        }   

        if(motor_state == pre_operational)
        {
            send_and_rec(received_message, buildMessage(motor_id, 0x06, motor_state_addr, set_op));
            if(encodeMessage(received_message, response) > 0)
            {
                motor_state[0]  = response[0];
                motor_state[1]  = response[1];
                printf("Motor State: %04x\n", *motor_state);
            }   
        }     
    }


    // while(true)
    // {
    //     Motor in OP Mode
    //     if(motor_state == operational)
    //     {
    //         ask for motor RPM
    //         sendMessage(buildMessage(motor_id, 0x03, motor_rpm_addr, 0x0001));
    //         recMessage(received_message);

    //         if(encodeMessage(received_message, response) > 0)
    //         {
    //             motor_rpm[0]  = response[0];
    //             motor_rpm[1]  = response[1];
    //             printf("Motor RMP is: %04x\n", motor_rpm);
    //         }

    //         Send Motor Target RPM
    //         sendMessage(buildMessage(motor_id, 0x06, motor_rpm_addr, set_motorValue));
    //         recMessage(received_message);

    //         if(encodeMessage(received_message, response) > 0)
    //         {
    //             printf("Motor data is set:%04x\n", response);
    //         }
    //     }  
        
    //     Send Sensor Request   
    //     sendMessage(buildMessage(sensor_id, 0x03, sensor_rpm_addr, 0x0001));
    //     recMessage(received_message);

    //     if(encodeMessage(received_message, response) >= 0)
    //     {
    //         sensor_value[0]  = response[0];
    //         sensor_value[1]  = response[1];
    //         printf("Sensor RMP: %04x\n", sensor_value);
    //     }

        

    // }
    
    return 0;
}

// g++ -o controller main.cpp 
// ./controller


