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
    double P;
    double I;
    double D;
};

class PID
{
public:
    PID(double min, double max, PIDConfig cfg);

    void update(double feedback_value);
    void reset(double setpoint);
    void reset(PIDConfig config);

    double output;
    double setpoint;
    double feedback_value;

    PIDConfig cfg;

private:
    double _max;
    double _min;

    double _pre_error;
    double _integral;

    high_resolution_clock::time_point last_time;
};

void to_json(json& j, const PIDConfig& d);
void from_json(const json& j, PIDConfig& d);
