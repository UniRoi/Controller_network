#include <analog_out.h>


Analog_out::Analog_out(int pin_no): pin_out(pin_no)
{
    
}

int Analog_out::init(int period_ms)
{
    int result = -1;
    pin_out.init();
    // default duty cycle 0%
    result = timer.init(period_ms, 10);

    return result;
}

int Analog_out::set(int duty_cycle)
{
    return timer.set_duty_cycle(duty_cycle);
}

Analog_out::~Analog_out()
{

}