#include "pid.h"

using namespace std;

PID::PID(double min, double max, double Kp, double Kd, double Ki) :
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    output(0),
    setpoint(0)
{
}

void PID::update(double dt, double feedback_value)
{
    // Calculate error
    auto error = setpoint - feedback_value;

    // Proportional term
    auto Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    auto Iout = _Ki * _integral;

    // Derivative term
    auto derivative = (error - _pre_error) / dt;
    auto Dout = _Kd * derivative;

    // Calculate total output
    auto _output = Pout + Iout + Dout;

    // Restrict to max/min
    if (_output > _max)
        _output = _max;
    else if (_output < _min)
        _output = _min;

    // Save error to previous error
    _pre_error = error;

    output = _output;
}

void PID::clear()
{
    output = 0;
    setpoint = 0;
    _Kp = 0;
    _Kd = 0;
    _Ki = 0;
    _pre_error = 0;
    _integral = 0;
}
