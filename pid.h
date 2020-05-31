#pragma once

#include <atomic>
#include <iostream>
#include <cmath>
#include <chrono>

#include <nlohmann/json.hpp>

using namespace std::chrono;
using json = nlohmann::json;

struct PIDConfig
{
    float P;
    float I;
    float D;
    float tau;
};

class PID
{
public:
    PID(float min, float max, PIDConfig cfg);

    void update(float feedback_value);
    void reset(float setpoint);
    void reset(PIDConfig config);

    float output;
    float setpoint;
    float _prev_measurement;

    PIDConfig cfg;

private:
    float _max;
    float _min;

    float _prev_error;

    float _integral;
    float _derivative;

    high_resolution_clock::time_point last_time;
};

void to_json(json& j, const PIDConfig& d);
void from_json(const json& j, PIDConfig& d);
