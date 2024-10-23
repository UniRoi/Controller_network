#include <iostream>
#include <thread>
#include <stdint.h>
#include <mutex>
#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>   // using the termios.h library


int incrementWriteTask()
{
    int fd, count;
    static uint32_t counter = 0;

    // open device for writing not as controlling tty because we don't want to get killed if line noise sends CTRL-C.
    if ((fd= open("/dev/ttyS0", O_WRONLY | O_NOCTTY))<0)
    {
        perror("UART: Failed to open the file.\n");
        return -1;
    }

    struct termios options;       // the termios structure is vital
    tcgetattr(fd, &options);    // sets the parameters for the file

    // Set up the communication options:
    cfsetospeed(&options, B57600);
    // set raw mode
    cfmakeraw(&options);
    options.c_cc[VMIN]=4; // min byte number of characters for raw mode
    options.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);            // discard file information
    tcsetattr(fd, TCSANOW, &options); // changes occur immmediately

    printf("Start incrementing counter\n");
    while (1)
    {
        counter++;
        // write all (four) bytes of the counter
        count = write(fd, &counter, sizeof(counter)); // transmit
        if(count < 0)
        {
            perror("Failed to write to the output\n");
            return -1;
        }

        printf("The following counter will be transmitted [%d]: %d\n", count, counter);
    

        /* increment counter every second. */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    }

    close(fd);

    return 0;
}

int decrementReadTask()
{
    static int fd, count;
    static uint32_t rd_cnt;

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
    options.c_cc[VMIN]=4; // min byte number of characters for raw mode
    options.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);            // discard file information
    tcsetattr(fd, TCSANOW, &options); // changes occur immmediately

    printf("Start reading counter\n");
    while (1)
    {

        if ((count = read(fd, &rd_cnt, sizeof(rd_cnt)))<0)
        {   //receive data
            perror("Failed to read from the input\n");
            return -1;
        }
        // printf("read was success\n");

        printf("Received bytes: [%d] counter value: %d\n",count, rd_cnt);
    }

    close(fd);
    return 0;
}

int main(void)
{
    /* Create the increment and decrement tasks using the default task
    * attributes. Do not pass in any parameters to the tasks. */
    std::thread incrementTaskObj (incrementWriteTask);
    std::thread decrementTaskObj (decrementReadTask);
    
    /* Allow the tasks to run. */
    incrementTaskObj.join();
    decrementTaskObj.join();
    
    return 0;
}


