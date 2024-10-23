#ifndef DIGITAL_OUT_H
#define DIGITAL_OUT_H

#include <stdint.h>

class Digital_out
{
private:
    uint8_t pinMask;
public:
    void init();
    void set_hi();
    void set_lo();
    void toggle();

    /// @brief Set a pin of port b as output
    /// @param pin pin number at portb
    Digital_out(uint8_t pin);
    ~Digital_out();
};

#endif
