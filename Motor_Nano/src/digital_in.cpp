#include <digital_in.h>
#include <avr/io.h>

/// @brief Set a pin of port c as input
/// @param pin pin number at portc
Digital_in::Digital_in(uint8_t pin)
{
    pinMask = 1 << pin;
}

Digital_in::~Digital_in()
{
}

/// @brief init a pin at portd as input, because of the external interrupts
void Digital_in::init()
{
    DDRD &= ~pinMask;
}

/// @brief reads defined pin value
/// @return True if pin is high
bool Digital_in::is_hi()
{
    bool bIsHi = false;
    
    if ( (PIND & pinMask) == pinMask)
    {
        bIsHi = true;
    }

    return bIsHi;
}

/// @brief reads defined pin value
/// @return True if pin is low
bool Digital_in::is_lo()
{
    bool bIsLo = false;
    
    if ( (PIND & pinMask) == 0x00)
    {
        bIsLo = true;
    }

    return bIsLo;
}