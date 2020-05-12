#include "pid.h"

using namespace std;

PID::PID(double min, double max, PIDConfig cfg) :
    _max(max),
    _min(min),
    _pre_error(0),
    _integral(0),
    cfg(cfg),
    output(0),
    setpoint(0),
    feedback_value(0)
{
}

void PID::update(double feedback_value)
{
    auto current_time = high_resolution_clock::now();
    auto dt = duration_cast<milliseconds>(current_time - last_time).count();
    last_time = current_time;

    this->feedback_value = feedback_value;

    // Calculate error
    auto error = setpoint - feedback_value;

    // Proportional term
    auto Pout = cfg.P * error;

    // Integral term
    _integral += (_pre_error + error / 2) * dt;
    // auto Iout = cfg.I * _integral;

    if (_integral > 30000) _integral = 30000;
    if (_integral < -30000) _integral = -30000;

    // Derivative term
    auto derivative = (error - _pre_error) / dt;
    // auto Dout = cfg.D * derivative;

    // Calculate total output
    // auto _output = Pout + Iout + Dout;

    output = (error * cfg.P) + (_integral * cfg.I) + (derivative * cfg.D);

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;
}

void PID::reset(double setpoint)
{
    this->output = 0;
    this->setpoint = setpoint;
    this->feedback_value = 0;
    this->_pre_error = 0;
    this->_integral = 0;

    this->last_time = high_resolution_clock::now();
}

void PID::reset(PIDConfig config)
{
    reset(setpoint);

    cfg = config;
}

void to_json(json& j, const PIDConfig& d)
{
    j = json
    {
        {"P", d.P},
        {"I", d.I},
        {"D", d.D}
    };
}

void from_json(const json& j, PIDConfig& d)
{
    j.at("P").get_to(d.P);
    j.at("I").get_to(d.I);
    j.at("D").get_to(d.D);
}
