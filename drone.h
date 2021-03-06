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

namespace Drone
{

#define DroneFlightMode_Macro \
    X(PENDING_INIT,        0) \
    X(RECOVERY_MODE,       1) \
    X(FLY_UP_MODE,         2) \
    X(HOVER_MODE,          3) \
    X(FOLLOW_MODE,         4) \
    X(FLY_UP_AND_DETECTED, 5)

#define X(flag, value) flag = value,
enum DroneFlightMode
{
    DroneFlightMode_Macro
};
#undef X

#define X(flag, value) #flag,
static char const* DroneFlightMode_Name[] =
{
    DroneFlightMode_Macro
};
#undef X

constexpr uint16_t LOOP_SLEEP_TIME = 2;
constexpr uint16_t HOVER_TIMEOUT   = 10000;

constexpr uint16_t MIDDLE_VALUE  = 1500;
constexpr uint16_t DISABLE_VALUE = MIDDLE_VALUE - 512;
constexpr uint16_t ENABLE_VALUE  = MIDDLE_VALUE + 512;

struct DroneReceiver
{
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;

    uint16_t flight_mode;
    uint16_t aux_2;
    uint16_t arm_mode;
    uint16_t aux_4;
    uint16_t aux_5;
    uint16_t aux_6;
    uint16_t aux_7;
    uint16_t aux_8;
    uint16_t aux_9;
    uint16_t aux_10;
    uint16_t aux_11;
    uint16_t aux_12;
    uint16_t aux_13;
    uint16_t aux_14;
};
void to_json(json& j, const DroneReceiver& d);
void from_json(const json& j, DroneReceiver& d);

};

void init_drone();
void deinit_drone();
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
