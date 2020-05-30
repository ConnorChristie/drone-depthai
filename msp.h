#pragma once

#include <errno.h>
#include <cstring>
#include <limits>
#include <iostream>
#include <chrono>
#include <ceSerial.h>
#include <nlohmann/json.hpp>

using namespace ce;
using namespace std;
using namespace std::chrono;
using nlohmann::json;

namespace Msp
{

enum MspCommand : uint8_t
{
    // Out messages
    API_VERSION          = 1,
    FC_VARIANT           = 2,
    FC_VERSION           = 3,
    BOARD_INFO           = 4,
    BUILD_INFO           = 5,
    CALIBRATION_DATA     = 14,
    FEATURE              = 36,
    BOARD_ALIGNMENT      = 38,
    CURRENT_METER_CONFIG = 40,
    RX_CONFIG            = 44,
    SONAR_ALTITUDE       = 58,
    ARMING_CONFIG        = 61,
    RX_MAP               = 64,     // get channel map (also returns number of channels total)
    LOOP_TIME            = 73,     // FC cycle time i.e looptime parameter
    FAILSAFE_CONFIG      = 75,     //out message         Returns FC Fail-Safe settings
    STATUS               = 101,    //out message         cycletime & errors_count & sensor present & box activation & current setting number
    RAW_IMU              = 102,    //out message         9 DOF
    SERVO                = 103,    //out message         servos
    MOTOR                = 104,    //out message         motors
    RC                   = 105,    //out message         rc channels and more
    RAW_GPS              = 106,    //out message         fix, numsat, lat, lon, alt, speed, ground course
    COMP_GPS             = 107,    //out message         distance home, direction home
    ATTITUDE             = 108,    //out message         2 angles 1 heading
    ALTITUDE             = 109,    //out message         altitude, variometer
    ANALOG               = 110,    //out message         vbat, powermetersum, rssi if available on RX
    RC_TUNING            = 111,    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
    PID                  = 112,    //out message         P I D coeff (9 are used currently)
    BOXNAMES             = 116,    //out message         the aux switch names
    PIDNAMES             = 117,    //out message         the PID names
    WP                   = 118,    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
    BOXIDS               = 119,    //out message         get the permanent IDs associated to BOXes
    SERVO_CONFIGURATIONS = 120,    //out message         All servo configurations.
    NAV_STATUS           = 121,    //out message         Returns navigation status
    NAV_CONFIG           = 122,    //out message         Returns navigation parameters
    MOTOR_3D_CONFIG      = 124,    //out message         Settings needed for reversible ESCs
    RC_DEADBAND          = 125,    //out message         deadbands for yaw alt pitch roll
    SENSOR_ALIGNMENT     = 126,    //out message         orientation of acc,gyro,mag
    LED_STRIP_MODECOLOR  = 127,    //out message         Get LED strip mode_color settings
    VOLTAGE_METERS       = 128,    //out message         Voltage (per meter)
    CURRENT_METERS       = 129,    //out message         Amperage (per meter)
    BATTERY_STATE        = 130,    //out message         Connected/Disconnected, Voltage, Current Used
    MOTOR_CONFIG         = 131,    //out message         Motor configuration (min/max throttle, etc)
    GPS_CONFIG           = 132,    //out message         GPS configuration
    COMPASS_CONFIG       = 133,    //out message         Compass configuration
    ESC_SENSOR_DATA      = 134,    //out message         Extra ESC data from 32-Bit ESCs (Temperature, RPM)
    GPS_RESCUE           = 135,    //out message         GPS Rescues's angle, initialAltitude, descentDistance, rescueGroundSpeed, sanityChecks and minSats
    GPS_RESCUE_PIDS      = 136,    //out message         GPS Rescues's throttleP and velocity PIDS + yaw P
    STATUS_EX            = 150,    //out message         cycletime, errors_count, CPU load, sensor present etc

    SET_FAILSAFE_CONFIG     = 76,     //in message          Sets FC Fail-Safe settings
    SET_RAW_RC              = 200,    //in message          8 rc chan
    SET_RAW_GPS             = 201,    //in message          fix, numsat, lat, lon, alt, speed
    SET_PID                 = 202,    //in message          P I D coeff (9 are used currently)
    SET_RC_TUNING           = 204,    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
    ACC_CALIBRATION         = 205,    //in message          no param
    MAG_CALIBRATION         = 206,    //in message          no param
    RESET_CONF              = 208,    //in message          no param
    SET_WP                  = 209,    //in message          sets a given WP (WP#,lat, lon, alt, flags)
    SELECT_SETTING          = 210,    //in message          Select Setting Number (0-2)
    SET_HEADING             = 211,    //in message          define a new heading hold direction
    SET_SERVO_CONFIGURATION = 212,    //in message          Servo settings
    SET_MOTOR               = 214,    //in message          PropBalance function
    SET_NAV_CONFIG          = 215,    //in message          Sets nav config parameters - write to the eeprom
    SET_MOTOR_3D_CONFIG     = 217,    //in message          Settings needed for reversible ESCs
    SET_RC_DEADBAND         = 218,    //in message          deadbands for yaw alt pitch roll
    SET_RESET_CURR_PID      = 219,    //in message          resetting the current pid profile to defaults
    SET_SENSOR_ALIGNMENT    = 220,    //in message          set the orientation of the acc,gyro,mag
    SET_LED_STRIP_MODECOLOR = 221,    //in  message         Set LED strip mode_color settings
    SET_MOTOR_CONFIG        = 222,    //out message         Motor configuration (min/max throttle, etc)
    SET_GPS_CONFIG          = 223,    //out message         GPS configuration
    SET_COMPASS_CONFIG      = 224,    //out message         Compass configuration
    SET_GPS_RESCUE          = 225,    //in message          GPS Rescues's angle, initialAltitude, descentDistance, rescueGroundSpeed, sanityChecks and minSats
    SET_GPS_RESCUE_PIDS     = 226     //in message          GPS Rescues's throttleP and velocity PIDS + yaw P
};

#define MspArmingDisableFlags_Macro \
    X(ARMING_DISABLED_NO_GYRO,         0) \
    X(ARMING_DISABLED_FAILSAFE,        1) \
    X(ARMING_DISABLED_RX_FAILSAFE,     2) \
    X(ARMING_DISABLED_BAD_RX_RECOVERY, 3) \
    X(ARMING_DISABLED_BOXFAILSAFE,     4) \
    X(ARMING_DISABLED_RUNAWAY_TAKEOFF, 5) \
    X(ARMING_DISABLED_CRASH_DETECTED,  6) \
    X(ARMING_DISABLED_THROTTLE,        7) \
    X(ARMING_DISABLED_ANGLE,           8) \
    X(ARMING_DISABLED_BOOT_GRACE_TIME, 9) \
    X(ARMING_DISABLED_NOPREARM,        10) \
    X(ARMING_DISABLED_LOAD,            11) \
    X(ARMING_DISABLED_CALIBRATING,     12) \
    X(ARMING_DISABLED_CLI,             13) \
    X(ARMING_DISABLED_CMS_MENU,        14) \
    X(ARMING_DISABLED_BST,             15) \
    X(ARMING_DISABLED_MSP,             16) \
    X(ARMING_DISABLED_PARALYZE,        17) \
    X(ARMING_DISABLED_GPS,             18) \
    X(ARMING_DISABLED_RESC,            19) \
    X(ARMING_DISABLED_RPMFILTER,       20) \
    X(ARMING_DISABLED_ARM_SWITCH,      21)

#define X(flag, value) flag = (1 << value),
enum MspArmingDisableFlags : uint32_t
{
    MspArmingDisableFlags_Macro
};
#undef X

#define X(flag, value) #flag,
static char const* MspArmingDisableFlags_Name[] =
{
    MspArmingDisableFlags_Macro
};
#undef X

enum MspFlightModeFlags : uint32_t
{
    ANGLE_MODE      = (1 << 0),
    HORIZON_MODE    = (1 << 1),
    MAG_MODE        = (1 << 2),
    HEADFREE_MODE   = (1 << 6),
    PASSTHRU_MODE   = (1 << 8),
    FAILSAFE_MODE   = (1 << 10),
    GPS_RESCUE_MODE = (1 << 11)
};

struct MspRequest
{
    char preamble[2];
    char direction;

    uint8_t size;
    MspCommand command;
};

char* send_raw_command(ceSerial* serial, MspCommand command, char* param_data, uint8_t param_size);

template<typename T>
void send_command(ceSerial* serial, MspCommand command, T* params)
{
    send_raw_command(serial, command, (char*)params, (uint8_t)sizeof(T));
}

template<typename T>
T* receive_parameters(ceSerial* serial, MspCommand command)
{
    return (T*)send_raw_command(serial, command, NULL, (uint8_t)sizeof(T));
}

#pragma pack(push, 1)
struct MspApiVersion
{
    uint8_t protocol_version;
    uint8_t api_major;
    uint8_t api_minor;
};

struct MspFcVariant
{
    char flight_control_identifier[4];
};

struct MspFcVersion
{
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch_level;
};

struct MspBoardInfo
{
    char     board_identifier[4];
    uint16_t hardware_revision;
};

struct MspBuildInfo
{
    char build_date[11];
    char build_time[8];
    char short_git_revision[7];
};

struct MspFailsafeConfig
{
    uint8_t  failsafe_delay;
    uint8_t  failsafe_off_delay;
    uint16_t failsafe_throttle;
    uint8_t  failsafe_switch_mode;
    uint16_t failsafe_throttle_low_delay;
    uint8_t  failsafe_procedure;
};

struct MspImu
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
};

struct MspStatusEx
{
    uint16_t cycle_time;
    uint16_t i2c_error_counter;
    uint16_t sensor;
    uint32_t initial_flight_mode_flags;
    uint8_t  current_pid_profile_index;

    uint16_t average_system_load_percent;
    uint8_t  pid_profile_count;
    uint8_t  current_control_rate_profile_index;
    uint8_t  additional_flight_mode_flags_count;
    // uint32_t additional_flight_mode_flags; // Only here when flight mode flags count > 0
    uint8_t  arming_flags_count;
    uint32_t arming_flags;
    uint8_t  reboot_required;
};
void to_json(json& j, const MspStatusEx& d);

struct MspStatus
{
    uint16_t cycle_time;
    uint16_t i2c_error_counter;
    uint16_t sensor;
    uint32_t flight_mode_flags;
    uint8_t  config_profile_index;
};

struct MspSensorStatus
{
    uint8_t is_hardware_healthy;
    uint8_t hw_gyro_status;
    uint8_t hw_accelerometer_status;
    uint8_t hw_compass_status;
    uint8_t hw_barometer_status;
    uint8_t hw_GPS_status;
    uint8_t hw_rangefinder_status;
    uint8_t hw_pitotmeter_status;
    uint8_t hw_optical_flow_status;
};

#define MSP_MAX_SUPPORTED_SERVOS 8

struct MspServo
{
    uint16_t value[MSP_MAX_SUPPORTED_SERVOS];
};

struct MspServoConfiguration
{
    uint16_t min;
    uint16_t max;
    uint16_t middle;
    uint8_t  rate;
    uint8_t  angle_at_min;
    uint8_t  angle_at_max;
    uint8_t  forward_from_channel;
    uint32_t reversed_sources;
};

struct MspServoConfigurations
{
    MspServoConfiguration config[MSP_MAX_SUPPORTED_SERVOS];
};

#define MSP_MAX_SERVO_RULES (2 * MSP_MAX_SUPPORTED_SERVOS)

struct MspServoMixRule
{
    uint8_t target_channel;
    uint8_t input_source;
    uint8_t rate;
    uint8_t speed;
    uint8_t min;
    uint8_t max;
};

struct MspServoMixRules
{
    MspServoMixRule mix_rule[MSP_MAX_SERVO_RULES];
};

#define MSP_MAX_SUPPORTED_MOTORS 8

struct MspMotor
{
    uint16_t motor[MSP_MAX_SUPPORTED_MOTORS];
};
void to_json(json& j, const MspMotor& d);

#define MSP_MAX_SUPPORTED_CHANNELS 18

struct MspReceiver
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
void to_json(json& j, const MspReceiver& d);

union MspRc
{
    uint16_t value[MSP_MAX_SUPPORTED_CHANNELS];
    MspReceiver receiver;
};

struct MspAttitude
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};

struct MspAltitude
{
    int32_t estimated_altitude;  // cm
    int16_t estimated_velocity;  // cm/s
    int32_t baro_latest_altitude;
};

struct MspAnalog
{
    uint8_t  vbat;
    uint16_t int_power_meter_sum;
    uint16_t rssi;
    uint16_t amperage;
};

struct MspRcTuning
{
    uint8_t rate;
    uint8_t expo;
    uint8_t roll_pitch_rate;
    uint8_t yaw_rate;
    uint8_t dyn_thr_pid;
    uint8_t throttle_mid;
    uint8_t throttle_expo;
};

struct MspBoardAlignment
{
    int16_t roll_deci_degrees;
    int16_t pitch_deci_degrees;
    int16_t yaw_deci_degrees;
};
#pragma pack(pop)

}
