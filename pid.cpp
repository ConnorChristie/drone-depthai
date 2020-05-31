#include "pid.h"

using namespace std;

PID::PID(float min, float max, PIDConfig cfg) :
    _min(min),
    _max(max)
{
    reset(cfg);
}

void PID::update(float measurement)
{
    auto now = high_resolution_clock::now();
    std::chrono::duration<float> duration = now - last_time;
    float dt = duration.count();;
    last_time = now;

    float error = setpoint - measurement;

    float proportional = cfg.P * error;

    _integral = _integral + 0.5f * cfg.I * dt * (error + _prev_error);

    float limMinInt, limMaxInt;

    if (proportional < _max)
    {
        limMaxInt = _max - proportional;
    }
    else
    {
        limMaxInt = 0;
    }

    if (proportional > _min)
    {
        limMinInt = _min - proportional;
    }
    else
    {
        limMinInt = 0;
    }

    if (_integral > limMaxInt)
    {
        _integral = limMaxInt;
    }
    else if (_integral < limMinInt)
    {
        _integral = limMinInt;
    }

    _derivative = (2.0f * cfg.D * (measurement - _prev_measurement) + (2.0f * cfg.tau - dt) * _derivative)
                / (2.0f * cfg.tau + dt);

    float out = proportional + _integral + _derivative;

    if (out > _max)
    {
        output = _max;
    }
    else if (out < _min)
    {
        out = _min;
    }

    _prev_error = error;
    _prev_measurement = measurement;

    output = out;
}

void PID::reset(float setpoint)
{
    _prev_error = 0;
    _prev_measurement = 0;
    _integral = 0;
    _derivative = 0;
    output = 0;

    this->setpoint = setpoint;
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
        {"D", d.D},
        {"tau", d.tau}
    };
}

void from_json(const json& j, PIDConfig& d)
{
    j.at("P").get_to(d.P);
    j.at("I").get_to(d.I);
    j.at("D").get_to(d.D);
    j.at("tau").get_to(d.tau);
}
