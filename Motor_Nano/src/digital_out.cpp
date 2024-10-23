#include <digital_out.h>
#include <avr/io.h>


void Digital_out::init()
{
    DDRB |= pinMask;
}

void Digital_out::set_hi()
{
    PORTB |= pinMask;
}

void Digital_out::set_lo()
{
    PORTB &= ~pinMask;
}

void Digital_out::toggle()
{
    PORTB ^= pinMask;
}

/// @brief Set a pin of port b as output
/// @param pin pin number at portb
Digital_out::Digital_out(uint8_t pin)
{
    pinMask = 1 << pin;
}

Digital_out::~Digital_out()
{
}