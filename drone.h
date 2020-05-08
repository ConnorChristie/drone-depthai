#pragma once

#include <exception>
#include <list>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <experimental/optional>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <boost/format.hpp>

#include "../core/host_json_helper.hpp"
#include "depthai_core.hpp"
#include "msp.h"
#include "pid.h"
#include "web_server.h"

using namespace boost;
using namespace std::chrono;
using json = nlohmann::json;

extern volatile sig_atomic_t is_running;

void run_drone();
void run_detection();

struct Detection
{
    int label;
    float confidence;
    float left;
    float top;
    float right;
    float bottom;
    float center_x;
    float center_y;
    float distance_z;
    int size;
};
void to_json(json& j, const Detection& d);
