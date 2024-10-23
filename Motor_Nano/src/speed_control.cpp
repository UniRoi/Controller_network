#include "speed_control.h"

// P_control class methods
P_control::P_control(float fKp) : fKp_(fKp)
{
}

P_control::~P_control()
{
}

double P_control::update(double ref, double actual)
{
    return (ref - actual) * fKp_;
}

// PI_control class methods
PI_control::PI_control(float fKp_t, float fTi_t, float dt_t, double maxOutput_t, double minOutput_t)
    : fKp(fKp_t), fTi(fTi_t), integral(0.0), dt(dt_t), minOutput(minOutput_t), maxOutput(maxOutput_t)
{
}

PI_control::~PI_control()
{
}

double PI_control::update(double ref, double actual)
{
    // Error calculation
    double error = ref - actual;

    // Update integral
    integral += (error * dt) / fTi;

    // Proportional term
    double P_out = fKp * error;

    // Integral term
    double I_out = fKp * integral;

    // Compute raw output (without saturation)
    double output = P_out + I_out;

    // Apply saturation (windup protection)
    if (output > maxOutput) {
        output = maxOutput;
        integral -= (error * dt) / fTi; // Undo integral update
    } else if (output < minOutput) {
        output = minOutput;
        integral -= (error * dt) / fTi; // Undo integral update
    }

    return output;
}