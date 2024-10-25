#include <iostream>
#include <thread>
#include <stdint.h>
#include <mutex>
#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>   // using the termios.h library


struct stMessage
{
  uint8_t u8ID;
  uint8_t u8Task;
  uint16_t u16Addr;
  uint16_t u16Msg;
  uint16_t u16Crc;
};

uint16_t motorvalue;
stMessage msg_to_motor;
stMessage msg_from_sensor;


uint32_t detChecksum()
{
    return 0x23;
}

int encodeMessage(stMessage rec_msg)
{
    uint32_t crc = detChecksum();

    if(rec_msg.u8ID == 0x01) //Motor
    {
        // No full error handling
        if((rec_msg.u8Task & 0xF0) == 0x80) 
        {
            printf("Error writing motor values. Rec msg: %d\n", rec_msg);
            return -1;
        } // Error

        // send and receive have to be the same
        if(crc == rec_msg.u16Crc) 
        {
            printf("Motor values succ written: %d\n", rec_msg);
            return 1;
        } // Right Message

    } else if (rec_msg.u8ID == 0x02) //Sensor
    {
        // No full error handling
        if((rec_msg.u8Task & 0xF0) == 0x80)
        {
            printf("Error rec sensor values. Rec msg: %d\n", rec_msg);
            return -1;
        } // Error

        // easy cheat: we define the message directly, not dependent on any other bit
        uint16_t rec_data = 0x0000;

    }
}


// Ask Sensor for data
stMessage sensorMessage(uint16_t reg)
{
    stMessage m_sensor_message;
    m_sensor_message.u8ID = 0x02; // 02 for Sensor
    m_sensor_message.u8Task = 0x03; // 03 for read
    m_sensor_message.u16Addr = reg; // register
    m_sensor_message.u16Msg = 0x0001;
    m_sensor_message.u16Crc = detChecksum();
    return m_sensor_message;
}

// write Motor data
stMessage motorMessage(uint16_t reg, uint16_t data)
{
    stMessage m_motor_message;
    m_motor_message.u8ID = 0x01; // 01 for Motor
    m_motor_message.u8Task = 0x06; // 03 for write
    m_motor_message.u16Addr = reg; // register
    m_motor_message.u16Msg = data;
    m_motor_message.u16Crc = detChecksum();
    return m_motor_message;
}


int sendTask(stMessage send_message)
{
    int fd, count;
    // open device for writing not as controlling tty because we don't want to get killed if line noise sends CTRL-C.
    if ((fd= open("/dev/ttyS0", O_WRONLY | O_NOCTTY))<0)
    {
        perror("UART: Failed to open the file.\n");
        return -1;
    }

    struct termios options;       // the termios structure is vital
    tcgetattr(fd, &options);    // sets the parameters for the file
    // Set up the communication options:
    cfsetospeed(&options, B19200);
    // set raw mode
    cfmakeraw(&options);
    options.c_cc[VMIN]=16; // min byte number of characters for raw mode
    options.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);            // discard file information
    tcsetattr(fd, TCSANOW, &options); // changes occur immmediately



    count = write(fd, &send_message, sizeof(send_message)); // transmit



    if(count < 0)
    {
        perror("Failed to write to the output\n");
        return -1;
    }
    printf("The following counter will be message [%d]: %d\n", count, send_message);

    close(fd);

    return 0;
}

int receiveTask()
{
    static int fd, count;
    // Remove O_NDELAY to *wait* on serial read (blocking read) and not as controlling tty because we don't want to get killed if line noise sends CTRL-C.
    if ((fd= open("/dev/ttyS0", O_RDONLY | O_NOCTTY))<0)
    {
        perror("UART: Failed to open the file.\n");
        return -1;
    }

    struct termios options;       // the termios structure is vital
    tcgetattr(fd, &options);    // sets the parameters for the file
    // Set up the communication options:
    cfsetispeed(&options, B57600);
    // set raw mode
    cfmakeraw(&options);
    //options.c_cc[VMIN]=10; // min byte number of characters for raw mode -- min 10 but max 16
    options.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);            // discard file information
    tcsetattr(fd, TCSANOW, &options); // changes occur immmediately

    

    printf("Start reading counter\n");
    while (1)
    {
        stMessage received_message;

        if ((count = read(fd, &received_message, sizeof(received_message)))<0)
        {   //receive data
            perror("Failed to read from the input\n");
            return -1;
        }
        printf("Received bytes: [%d] counter value: %d\n",count, received_message);
        // TODO: check for count -> is it a shorter message then wait for the rest

        encodeMessage(received_message);
    }

    close(fd);
    return 0;
}




int main(void)
{
    // /* Threads for reading from Controller 2 and writing to Controller 1 */
    // std::thread sendThread (sendTask);
    // std::thread receiveThread (receiveTask);
    
    // /* Allow the tasks to run. */
    // sendThread.join();
    // receiveThread.join();
    
    return 0;
}
