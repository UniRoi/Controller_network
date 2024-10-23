#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller
{
public:
    Controller() {}
    virtual ~Controller() {}
    
    // Pure virtual function to enforce derived classes to implement this method
    virtual double update(double ref, double actual) = 0;
};

class P_control : public Controller
{
private:
    float fKp_;
public:
    P_control(float fKp);
    ~P_control();
    
    // Override update method
    double update(double ref, double actual) override;
};

class PI_control : public Controller
{
private:
    float fKp;    // Proportional gain
    float fTi;    // Integral time
    double integral; // To accumulate integral error over time
    float dt;
    double minOutput;
    double maxOutput;

public:
    PI_control(float fKp_t, float fTi_t, float dt_t, double maxOutput_t, double minOutput_t);
    ~PI_control();

    // Override update method
    double update(double ref, double actual) override;
};

#endif // CONTROLLER_H