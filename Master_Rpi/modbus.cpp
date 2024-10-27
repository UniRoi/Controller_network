/* Simple send and receive C example (line-mode terminal program with local echo) 
* for communicating with the Arduino using /dev/ttyS0. */

#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>
#include<stdint.h>
#include<stdlib.h>

struct stMessage
{
  uint8_t u8ID;
  uint8_t u8Task;
  uint16_t u16Addr;
  uint16_t u16Msg;
  uint16_t u16Crc;
};

void fn_swap16(uint16_t *val)
{
    uint16_t tmp;
    tmp = (*val & 0x00FF);
    *val = (tmp << 8 | *val >> 8);
}

int main(int argc, char *argv[]){
   int file, count;

   if(argc!=5){
       printf("Invalid number of arguments, exiting [%d]!\n", argc);
       return -2;
   }

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

   // populate the message with integer values in binary format
   tMsg.u8ID = (uint8_t)atoi(argv[1]);
   tMsg.u8Task = (uint8_t)atoi(argv[2]);
   tMsg.u16Addr = (uint16_t)atoi(argv[3]);
   tMsg.u16Msg = (uint16_t)atoi(argv[4]);
   tMsg.u16Crc = 0x55AA;

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

   usleep(100000);

   unsigned char receive[100];

   if ((count = read(file, rd_msg, sizeof(rd_msg)))<0)
   {
      perror("Failed to read from the input\n");
      return -1;
   }

   if (count==0) printf("There was no data available to read!\n");
   else {
      receive[count]=0;  //There is no null character sent by the Arduino
      printf("The following was read in [%d]: %02x %02x %02x %02x %02x %02x %02x %02x\n",count, rd_msg[0], rd_msg[1],rd_msg[2], rd_msg[3],rd_msg[4], rd_msg[5],rd_msg[6], rd_msg[7]);
   }

   close(file);
   return 0;
}