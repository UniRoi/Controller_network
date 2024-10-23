// #ifndef TIMER_MSEC
// #define TIMER_MSEC
#pragma once

class Timer_msec
{
private:
    /* data */
public:
    Timer_msec();
    ~Timer_msec();

    /// @brief init the timer with period and duty cycle
    /// @param period_ms time in ms, max 4,1s min 1ms
    /// @param duty_cycle in percent
    /// @return -1 if period is to long or short same for duty cycle, 0 on success
    int init(int period_ms, int duty_cycle);

    /// @brief set duty cycle on the fly
    /// @param duty_cycle in percent from 1 to 99
    /// @return 1 if duty cycle is to long or short, 0 on success
    int set_duty_cycle(int duty_cycle);
};

// #endif
