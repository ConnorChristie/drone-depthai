#include "drone.h"

#define SHOW_PREVIEW false

PID* pid_x;
PID* pid_y;
PID* pid_z;

std::atomic<bool> is_tracking;
std::experimental::optional<high_resolution_clock::time_point> started_tracking;
std::experimental::optional<high_resolution_clock::time_point> lost_tracking;

void init_drone()
{
    json drone_config;
    if (!getJSONFromFile("drone.json", drone_config))
    {
        std::cout << "error: getJSONFromFile (drone.json)\n";
        return;
    }

    pid_x = new PID(
        -350,
        350,
        drone_config["pids"]["x"].get<PIDConfig>()
    );

    pid_y = new PID(
        -350,
        350,
        drone_config["pids"]["y"].get<PIDConfig>()
    );

    pid_z = new PID(
        -350,
        350,
        drone_config["pids"]["z"].get<PIDConfig>()
    );
}

const double ACCEL_FACTOR    = 320;
const double DIVISION_FACTOR = 60;

// https://www.desmos.com/calculator/u0sumemvto
inline double curve_fn(double x)
{
    return ACCEL_FACTOR * log10((x + DIVISION_FACTOR) / DIVISION_FACTOR);
}

void set_receiver_values(ceSerial *serial, bool armed, int throttle, int pitch, int roll)
{
    Drone::DroneReceiver params =
    {
        .roll        = (uint16_t)(Drone::MIDDLE_VALUE + roll),
        .pitch       = (uint16_t)(Drone::MIDDLE_VALUE + pitch),
        .throttle    = (uint16_t)(Drone::DISABLE_VALUE + throttle),
        .yaw         = Drone::MIDDLE_VALUE,
        .flight_mode = Drone::DISABLE_VALUE, // DISABLE = Horizon mode
        .aux_2       = Drone::MIDDLE_VALUE,
        .arm_mode    = (armed ? Drone::ENABLE_VALUE : Drone::DISABLE_VALUE)
    };

    Msp::send_command<Drone::DroneReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &params);
}

bool web_armed = false;
auto flight_mode = Drone::DroneFlightMode::RECOVERY_MODE;
auto task_start = high_resolution_clock::now();

void to_mode(Drone::DroneFlightMode mode)
{
    std::cout << "State transition: " << flight_mode << " -> " << mode << "\n";

    flight_mode = mode;
    task_start = high_resolution_clock::now();

    if (mode == Drone::DroneFlightMode::FOLLOW_MODE)
    {
        pid_x->reset(0);
        pid_y->reset(0);
        pid_z->reset(100); // centimeters
    }
}

void run_drone()
{
    ceSerial *serial = new ceSerial("/dev/ttySAC3", 115200, 8, 'N', 1);
    serial->Open();

    auto t0 = high_resolution_clock::now();
    auto telem_start = high_resolution_clock::now();

    bool armed = false;
    int throttle = 0;

    Drone::DroneReceiver remote_controls;

    while (is_running)
    {
        auto now = high_resolution_clock::now();
        auto dt = duration_cast<milliseconds>(now - t0).count();
        t0 = now;

        // std::cout << "Drone Elapsed: " << dt << "\n";

        auto commands = ws_get_messages();

        for (auto i = 0; i < commands.size(); i++)
        {
            auto json_obj = json::parse(commands[i]);

            if (json_obj.find("command") != json_obj.end())
            {
                auto command = json_obj["command"].get<std::string>();

                if (command == "getPids")
                {
                    json o;
                    o["pid_config"]["x"] = pid_x->cfg;
                    o["pid_config"]["y"] = pid_y->cfg;
                    o["pid_config"]["z"] = pid_z->cfg;

                    queue_ws_broadcast(o.dump());
                }
                else if (command == "setPids")
                {
                    pid_x->reset(json_obj["pid_config"]["x"].get<PIDConfig>());
                    pid_y->reset(json_obj["pid_config"]["y"].get<PIDConfig>());
                    pid_z->reset(json_obj["pid_config"]["z"].get<PIDConfig>());
                }
                else if (command == "setArmed")
                {
                    web_armed = json_obj["armed"].get<bool>();
                }
                else if (command == "setChannels")
                {
                    json_obj["channels"].get_to(remote_controls);

                    std::cout << "A: " << remote_controls.roll << "\n";
                    std::cout << "B: " << remote_controls.pitch << "\n";
                    std::cout << "C: " << remote_controls.throttle << "\n";
                    std::cout << "D: " << remote_controls.yaw << "\n";
                }
            }
        }

        int pitch = 0;
        int roll = 0;

        Msp::MspStatusEx *status = Msp::receive_parameters<Msp::MspStatusEx>(serial, Msp::MspCommand::STATUS_EX);

        if (status == NULL)
        {
            continue;
        }

        if (!web_armed)
        {
            flight_mode = Drone::DroneFlightMode::RECOVERY_MODE;

            set_receiver_values(serial, web_armed, 0, 0, 0);

            continue;
        }

        if ((status->arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE) == Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE
            || (status->arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE) == Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE)
        {
            flight_mode = Drone::DroneFlightMode::RECOVERY_MODE;
        }
        else if (flight_mode == Drone::DroneFlightMode::RECOVERY_MODE)
        {
            to_mode(Drone::DroneFlightMode::FLY_UP_MODE);
        }

        if (flight_mode == Drone::DroneFlightMode::RECOVERY_MODE)
        {
            armed = false;
            throttle = 0;
        }
        else if (flight_mode == Drone::DroneFlightMode::FLY_UP_MODE)
        {
            armed = true;

            if (is_tracking)
            {
                to_mode(Drone::DroneFlightMode::FLY_UP_AND_DETECTED);

                continue;
            }
            else
            {
                auto task_elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - task_start).count();

                if (task_elapsed >= 5000)
                {
                    to_mode(Drone::DroneFlightMode::THROTTLE_DOWN);

                    continue;
                }
                else
                {
                    throttle = static_cast<int>(round(curve_fn(task_elapsed)));
                }
            }
        }
        else if (flight_mode == Drone::DroneFlightMode::FLY_UP_AND_DETECTED)
        {
            armed = true;

            if (is_tracking && started_tracking)
            {
                auto tracking_elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - started_tracking.value()).count();

                if (tracking_elapsed >= 10)
                {
                    to_mode(Drone::DroneFlightMode::FOLLOW_MODE);

                    continue;
                }
            }
            else if (!is_tracking && lost_tracking)
            {
                auto lost_tracking_elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - lost_tracking.value()).count();

                if (lost_tracking_elapsed >= 50)
                {
                    to_mode(Drone::DroneFlightMode::FLY_UP_MODE);

                    continue;
                }
            }
        }
        else if (flight_mode == Drone::DroneFlightMode::THROTTLE_DOWN)
        {
            armed = true;
            throttle -= 10;

            if (throttle <= 0)
            {
                to_mode(Drone::DroneFlightMode::RECOVERY_MODE);

                continue;
            }
        }
        else if (flight_mode == Drone::DroneFlightMode::FOLLOW_MODE)
        {
            armed = true;

            if (is_tracking)
            {
                roll = pid_x->output;
                throttle = 350 + pid_y->output;
                pitch = pid_z->output;
            }
            else if (!is_tracking && lost_tracking)
            {
                auto lost_tracking_elapsed = duration_cast<seconds>(high_resolution_clock::now() - lost_tracking.value()).count();

                if (lost_tracking_elapsed >= 10)
                {
                    to_mode(Drone::DroneFlightMode::HOVER_MODE);

                    continue;
                }
            }
        }

        if (flight_mode == Drone::DroneFlightMode::HOVER_MODE)
        {
            // Use last throttle
            armed = true;
        }

        set_receiver_values(serial, armed, throttle, pitch, roll);

        if (duration_cast<milliseconds>(high_resolution_clock::now() - telem_start).count() >= 200)
        {
            telem_start = high_resolution_clock::now();

            auto rc = Msp::receive_parameters<Msp::MspReceiver>(serial, Msp::MspCommand::RC);
            auto motor = Msp::receive_parameters<Msp::MspMotor>(serial, Msp::MspCommand::MOTOR);

            json o;
            o["receiver"] = *rc;
            o["motors"] = *motor;
            o["pids"]["x"] = std::map<std::string, double>{ {"set_point", pid_x->feedback_value}, {"curr_value", pid_x->output} };
            o["pids"]["y"] = std::map<std::string, double>{ {"set_point", pid_y->feedback_value}, {"curr_value", pid_y->output} };
            o["pids"]["z"] = std::map<std::string, double>{ {"set_point", pid_z->feedback_value}, {"curr_value", pid_z->output} };

            queue_ws_broadcast(o.dump());

            delete[] rc;
            delete[] motor;
        }

        delete[] status;

        std::this_thread::sleep_for(milliseconds(10));
    }

    serial->Close();
}

void run_detection()
{
    json config_json;
    if (!getJSONFromFile("config.json", config_json))
    {
        std::cout << "error: getJSONFromFile (config.json)\n";
        return;
    }

    bool did_init = init_device("/home/pi/depthai/depthai.cmd", "");

    if (!did_init)
    {
        std::cout << "Failed to init device\n";
    }

    auto pipeline = create_pipeline(config_json.dump());
    auto t0 = high_resolution_clock::now();

    std::vector<Detection> detections;

    int height = -1;
    int width = -1;

    while (is_running)
    {
        auto now = high_resolution_clock::now();
        auto dt = duration_cast<milliseconds>(now - t0).count();
        t0 = now;

        auto nets_and_data = pipeline->getAvailableNNetAndDataPackets();

        auto nnet_packets = std::get<0>(nets_and_data);
        auto data_packets = std::get<1>(nets_and_data);

        if (nnet_packets.size() != 0)
        {
            detections.clear();
        }

        if (width != -1 && height != -1)
        {
            for (auto it = nnet_packets.begin(); it != nnet_packets.end(); ++it)
            {
                auto entries = (*it)->getTensorEntryContainer();

                for (auto i = 0; i < entries->size(); i++)
                {
                    auto e = entries->getByIndex(i);

                    auto left = e[0].getFloat("left") * width;
                    auto top = e[0].getFloat("top") * height;
                    auto right = e[0].getFloat("right") * width;
                    auto bottom = e[0].getFloat("bottom") * height;

                    auto center_x = (left + right) / 2.0f;
                    auto center_y = (top + bottom) / 2.0f;
                    auto size = static_cast<int>((right - left) * (bottom - top));

                    Detection detection = {
                        .label      = static_cast<int>(e[0].getFloat("label")),
                        .confidence = e[0].getFloat("confidence"),
                        .left       = left,
                        .top        = top,
                        .right      = right,
                        .bottom     = bottom,
                        .center_x   = center_x,
                        .center_y   = center_y,
                        .distance_z = e[0].getFloat("distance_z"),
                        .size       = size
                    };

                    if (detection.confidence >= 0.8 && detection.label == 1)
                    {
                        detections.push_back(detection);
                    }
                }
            }
        }

        for (auto it = data_packets.begin(); it != data_packets.end(); ++it)
        {
            if ((*it)->stream_name == "previewout")
            {
                auto data = (*it)->getData();
                auto dims = (*it)->dimensions;
                height = dims.at(1);
                width = dims.at(2);

                if (SHOW_PREVIEW)
                {
                    void* img_data_ptr = (void*) data;
                    int channel_size = height * width;

                    cv::Mat R(height, width, CV_8UC1, img_data_ptr);
                    cv::Mat G(height, width, CV_8UC1, img_data_ptr + channel_size);
                    cv::Mat B(height, width, CV_8UC1, img_data_ptr + channel_size * 2);

                    std::vector<cv::Mat> channels;
                    channels.push_back(R);
                    channels.push_back(G);
                    channels.push_back(B);

                    cv::Mat frame;
                    cv::merge(channels, frame);

                    for (auto i = 0; i < detections.size(); i++)
                    {
                        auto d = &detections[i];

                        cv::rectangle(frame, cv::Point2f(d->left, d->top), cv::Point2f(d->right, d->bottom), cv::Scalar(23, 23, 255));

                        cv::putText(frame, str(format("label: %1%") % d->label), cv::Point2f(d->left, d->top + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                        cv::putText(frame, str(format("%1% %%") % (d->confidence * 100)), cv::Point2f(d->left, d->top + 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                        cv::putText(frame, str(format("z: %1%") % d->distance_z), cv::Point2f(d->left, d->top + 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                    }

                    cv::imshow("previewout", frame);
                }
            }
        }

        bool has_detections = !detections.empty();

        if (has_detections)
        {
            std::sort(detections.begin(), detections.end(),
                [](const Detection &a, const Detection &b) -> bool
            {
                return a.size > b.size;
            });
        }

        // Just started tracking
        if (!is_tracking && has_detections)
        {
            is_tracking = true;
            started_tracking = high_resolution_clock::now();
            lost_tracking = std::experimental::nullopt;
        }
        // Lost tracking
        else if (is_tracking && !has_detections)
        {
            std::cout << "No objects detected, did we lose track of it?\n";

            is_tracking = false;
            started_tracking = std::experimental::nullopt;
            lost_tracking = high_resolution_clock::now();
        }

        if (is_tracking)
        {
            auto best_detection = &detections[0];

            pid_x->update(best_detection->center_x - (width / 2.0));
            pid_y->update(best_detection->center_y - (height / 2.0));
            pid_z->update(best_detection->distance_z * 100);
        }

        const int key = cv::waitKey(1);
    }
}

void to_json(json& j, const Detection& d)
{
    j = json
    {
        {"label", d.label},
        {"confidence", d.confidence},
        {"left", d.left},
        {"top", d.top},
        {"right", d.right},
        {"bottom", d.bottom},
        {"center_x", d.center_x},
        {"center_y", d.center_y},
        {"distance_z", d.distance_z},
        {"size", d.size},
    };
}

namespace Drone
{

void to_json(json& j, const DroneReceiver& d)
{
    j = json
    {
        {"roll", d.roll},
        {"pitch", d.pitch},
        {"throttle", d.throttle},
        {"yaw", d.yaw},
        {"flight_mode", d.flight_mode},
        {"aux_2", d.aux_2},
        {"arm_mode", d.arm_mode},
        {"aux_4", d.aux_4},
        {"aux_5", d.aux_5},
        {"aux_6", d.aux_6},
        {"aux_7", d.aux_7},
        {"aux_8", d.aux_8},
        {"aux_9", d.aux_9},
        {"aux_10", d.aux_10},
        {"aux_11", d.aux_11},
        {"aux_12", d.aux_12},
        {"aux_13", d.aux_13},
        {"aux_14", d.aux_14},
    };
}

void from_json(const json& j, DroneReceiver& d)
{
    j.at("roll").get_to(d.roll);
    j.at("pitch").get_to(d.pitch);
    j.at("throttle").get_to(d.throttle);
    j.at("yaw").get_to(d.yaw);
    j.at("flight_mode").get_to(d.flight_mode);
    j.at("aux_2").get_to(d.aux_2);
    j.at("arm_mode").get_to(d.arm_mode);
    j.at("aux_4").get_to(d.aux_4);
    j.at("aux_5").get_to(d.aux_5);
    j.at("aux_6").get_to(d.aux_6);
    j.at("aux_7").get_to(d.aux_7);
    j.at("aux_8").get_to(d.aux_8);
    j.at("aux_9").get_to(d.aux_9);
    j.at("aux_10").get_to(d.aux_10);
    j.at("aux_11").get_to(d.aux_11);
    j.at("aux_12").get_to(d.aux_12);
    j.at("aux_13").get_to(d.aux_13);
    j.at("aux_14").get_to(d.aux_14);
}

};
