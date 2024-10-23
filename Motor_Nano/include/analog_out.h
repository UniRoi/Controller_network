
#ifndef ANALOG_OUT_H
#define ANALOG_OUT_H

#include <timer_msec.h>
#include <digital_out.h>

class Analog_out
{
public:
    /// @brief constructor for analog out driver
    /// @param pin_no analog out pin
    Analog_out(int pin_no);

    /// @brief init function with default duty cycle of 50%
    /// @param period_ms period for the analog out value in ms
    /// @return 1 if period is to long or short, 0 on success
    int init(int period_ms = 1000);

    /// @brief set duty cycle on the fly
    /// @param duty_cycle in percent from 1 to 99
    /// @return 1 if duty cycle is to long or short, 0 on success
    int set(int duty_cycle);
    ~Analog_out();

private:
    Timer_msec timer;
public:    
    Digital_out pin_out;
};

#endif // ANALOG_OUT_H


