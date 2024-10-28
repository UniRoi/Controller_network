# Controller Network
## Part 1
For the calculation of the minimal response time we use the Baudrate from the lecture of Modbus RTU which is 19200 bits per second. For the message we use 8 Bytes per message. Well need a request and a response until we can update so the formula is:


$$
\frac{1}{19200\frac{bits}{s}}\cdot (8 \cdot 8 \cdot 2) bits = 0.006666 s \approx 0.007 s
$$
We round to 7 ms for having time for the bus idle.

### Messages
Messages are built up as follows: 
 - 1 Byte: ID of the Responder: 01 (Motor) or 02 (Sensor)
 - 2 Byte: ID of the Task: 03 (read) or 06 (write)
 - 3 & 4 Byte: Address 
 - 5 & 6 Byte: Message
 - 7 & 8 Byte: CRC

 Address:
 - Motor: Control state:  0001, RPM 0005, Set Speed 0009
 - Sensor: Read Value: 0006 
    
 Message: Data  
 CRC: Calculated

 Error codes:
static uint8_t ill_func = 0x01;
static uint8_t ill_addr = 0x02;
static uint8_t ill_data = 0x03;
static uint8_t dev_fail = 0x04;


## Part 2

### simulating crc error

#### RPi is sending wrong crc value

Message is ignored from Arduino

Picture here

#### Arduino is sending a wrong crc in the answer

Print from linux terminal:

    pi@rpi-matthiasr24:~/Controller_network/Master_Rpi $ ./modbus 1 6 1 128
    Sent request: 01 06 0001 0080 aad9
    CRC Error, will not use message. Received: 0xaada Calculated: 0xaad9


Picture here