#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <digital_in.h>

class encoder
{
private:
    uint8_t encoder_pin;
    Digital_in EncoderAPin;
    Digital_in EncoderBPin;
    volatile int16_t i16EncoderPos = 0;
    int16_t i16MotorPps = 0;
public:

    void init(void);
    void updatePos(void);
    int16_t GetPosition(void);
    void ResetPosition(void);

    void updatePps(void);
    int16_t GetPps(void);
    int16_t GetRpm(void);


    encoder(int8_t pin_A_number, int8_t pin_B_number);
    ~encoder();
};

#endif