#include "drone.h"

PID pid_x(-100, 100, 0.1, 0.01, 0.5);
PID pid_y(-100, 100, 0.1, 0.01, 0.5);
PID pid_z(-100, 100, 0.1, 0.01, 0.5);

std::atomic<bool> is_tracking;
std::experimental::optional<high_resolution_clock::time_point> started_tracking;
std::experimental::optional<high_resolution_clock::time_point> lost_tracking;

// Logarithmic curve coefs
// Gets to 350 at t = 1.9s
const double ACCEL_FACTOR    = 210;
const double DIVISION_FACTOR = 30;
const double Y_OFFSET        = -30;

// Determine the x offset by solving for x-intercept
const double X_OFFSET = pow(10, -Y_OFFSET / ACCEL_FACTOR) * DIVISION_FACTOR;

inline double curve_fn(double x)
{
    return ACCEL_FACTOR * log10((x + X_OFFSET) / DIVISION_FACTOR) + Y_OFFSET;
}

void send_throttle_command(ceSerial *serial, int throttle, bool armed)
{
    Drone::DroneReceiver params =
    {
        .roll        = Drone::MIDDLE_VALUE,
        .pitch       = Drone::MIDDLE_VALUE,
        .throttle    = (uint16_t)(Drone::DISABLE_VALUE + throttle),
        .yaw         = Drone::MIDDLE_VALUE,
        .flight_mode = Drone::DISABLE_VALUE, // DISABLE = Horizon mode
        .aux_2       = Drone::MIDDLE_VALUE,
        .arm_mode    = (armed ? Drone::ENABLE_VALUE : Drone::DISABLE_VALUE)
    };

    Msp::send_command<Drone::DroneReceiver>(serial, Msp::MspCommand::SET_RAW_RC, &params);
}

void run_drone()
{
    ceSerial *serial = new ceSerial("/dev/ttySAC3", 115200, 8, 'N', 1);
    serial->Open();

    auto flight_mode = Drone::DroneFlightMode::RECOVERY_MODE;
    auto task_start = high_resolution_clock::now();
    auto telem_start = high_resolution_clock::now();

    while (is_running)
    {
        Msp::MspStatusEx *status = Msp::receive_parameters<Msp::MspStatusEx>(serial, Msp::MspCommand::STATUS_EX);

        if (status == NULL)
        {
            continue;
        }

        if ((status->arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE) == Msp::MspArmingDisableFlags::ARMING_DISABLED_RX_FAILSAFE
            || (status->arming_flags & Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE) == Msp::MspArmingDisableFlags::ARMING_DISABLED_FAILSAFE)
        {
            flight_mode = Drone::DroneFlightMode::RECOVERY_MODE;
        }
        else if (flight_mode == Drone::DroneFlightMode::RECOVERY_MODE)
        {
            flight_mode = Drone::DroneFlightMode::FLY_UP_MODE;
            task_start = high_resolution_clock::now();
        }

        bool armed;
        int throttle;

        if (flight_mode == Drone::DroneFlightMode::RECOVERY_MODE)
        {
            armed = false;
            throttle = 0;
        }
        else if (flight_mode == Drone::DroneFlightMode::FLY_UP_MODE)
        {
            auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - task_start).count();

            // switch to 10ms of constant detection
            if (elapsed >= 10000)
            {
                flight_mode = Drone::DroneFlightMode::HOVER_MODE;
                task_start = high_resolution_clock::now();
            }
            else
            {
                armed = true;
                throttle = static_cast<int>(round(curve_fn(elapsed)));
            }
        }
        else if (flight_mode == Drone::DroneFlightMode::HOVER_MODE)
        {
            armed = true;
            throttle = 350;
        }

        send_throttle_command(serial, throttle, armed);

        if (duration_cast<milliseconds>(high_resolution_clock::now() - telem_start).count() >= 200)
        {
            telem_start = high_resolution_clock::now();

            auto rc = Msp::receive_parameters<Msp::MspReceiver>(serial, Msp::MspCommand::RC);
            auto motor = Msp::receive_parameters<Msp::MspMotor>(serial, Msp::MspCommand::MOTOR);

            json o;
            o["receiver"] = *rc;
            o["motors"] = *motor;
            o["pids"]["z"] = std::map<std::string, double>{ {"set_point", pid_z.setpoint}, {"curr_value", pid_z.output} };

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
    auto json_fpath = "config.json";

    json json_obj;
    if (!getJSONFromFile(json_fpath, json_obj))
    {
        std::cout << "error: getJSONFromFile (configuration)\n";
        return;
    }

    bool did_init = init_device("/home/pi/depthai/depthai.cmd", "");

    if (!did_init)
    {
        std::cout << "Failed to init device\n";
    }

    auto pipeline = create_pipeline(json_obj.dump());
    auto t0 = high_resolution_clock::now();

    std::vector<Detection> detections;

    while (is_running)
    {
        auto now = high_resolution_clock::now();
        auto dt = duration_cast<milliseconds>(now - t0).count();
        auto dt_ms = dt / 1000.0;
        t0 = now;

        auto nets_and_data = pipeline->getAvailableNNetAndDataPackets();

        auto nnet_packets = std::get<0>(nets_and_data);
        auto data_packets = std::get<1>(nets_and_data);

        if (nnet_packets.size() != 0)
        {
            detections.clear();
        }

        for (auto it = nnet_packets.begin(); it != nnet_packets.end(); ++it)
        {
            auto entries = (*it)->getTensorEntryContainer();

            for (auto i = 0; i < entries->size(); i++)
            {
                auto e = entries->getByIndex(i);

                Detection detection = {
                    .label      = static_cast<int>(e[0].getFloat("label")),
                    .confidence = e[0].getFloat("confidence"),
                    .left       = e[0].getFloat("left"),
                    .top        = e[0].getFloat("top"),
                    .right      = e[0].getFloat("right"),
                    .bottom     = e[0].getFloat("bottom"),
                    .center_x   = 0,
                    .center_y   = 0,
                    .distance_z = e[0].getFloat("distance_z"),
                    .size       = 0
                };

                if (detection.confidence >= 0.8 && detection.label == 1)
                {
                    detections.push_back(detection);
                }
            }
        }

        int height;
        int width;

        for (auto it = data_packets.begin(); it != data_packets.end(); ++it)
        {
            if ((*it)->stream_name == "previewout")
            {
                auto data = (*it)->getData();
                auto dims = (*it)->dimensions;
                height = dims.at(1);
                width = dims.at(2);

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

                    if (d->size == 0)
                    {
                        d->left = d->left * width;
                        d->top = d->top * height;
                        d->right = d->right * width;
                        d->bottom = d->bottom * height;

                        d->center_x = (d->left + d->right) / 2.0;
                        d->center_y = (d->top + d->bottom) / 2.0;
                        d->size = static_cast<int>((d->right - d->left) * (d->bottom - d->top));
                    }

                    cv::rectangle(frame, cv::Point2f(d->left, d->top), cv::Point2f(d->right, d->bottom), cv::Scalar(23, 23, 255));

                    cv::putText(frame, str(format("label: %1%") % d->label), cv::Point2f(d->left, d->top + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                    cv::putText(frame, str(format("%1% %%") % (d->confidence * 100)), cv::Point2f(d->left, d->top + 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                    cv::putText(frame, str(format("z: %1%") % d->distance_z), cv::Point2f(d->left, d->top + 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                }

                cv::imshow("previewout", frame);
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

            pid_x.reset(width / 2.0);
            pid_y.reset(height / 2.0);
            pid_z.reset(1.0); // meters
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
            auto best_detection = detections[0];

            pid_x.update(dt_ms, best_detection.center_x);
            pid_y.update(dt_ms, best_detection.center_y);
            pid_z.update(dt_ms, best_detection.distance_z);
        }

        const int key = cv::waitKey(1);

        std::this_thread::sleep_for(milliseconds(1));
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
