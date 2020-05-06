#pragma once

#include <atomic>
#include <iostream>
#include <cmath>

class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID(double min, double max, double Kp, double Kd, double Ki);

    void update(double dt, double feedback_value);
    void clear();

    std::atomic<double> output;
    std::atomic<double> setpoint;

private:
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;

    double _pre_error;
    double _integral;
};
