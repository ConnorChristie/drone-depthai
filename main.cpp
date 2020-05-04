#include <exception>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "../core/host_json_helper.hpp"
#include "depthai_core.hpp"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

using namespace std::chrono;
using json = nlohmann::json;

typedef duration<double, std::ratio<1, 1000>> ms;

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

typedef websocketpp::connection_hdl connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;
typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

server ws_server;
con_list ws_connections;

void run_ws()
{
    ws_server.listen(8080);
    ws_server.start_accept();

    try
    {
        ws_server.run();
    }
    catch (const std::exception & e)
    {
        std::cout << e.what() << std::endl;
    }
}

void on_open(connection_hdl hdl)
{
    ws_connections.insert(hdl);
}

void on_close(connection_hdl hdl)
{
    ws_connections.erase(hdl);
}

int main(int argc, char** argv)
{
    ws_server.clear_access_channels(websocketpp::log::alevel::all);
    ws_server.set_access_channels(websocketpp::log::alevel::access_core);
    ws_server.set_access_channels(websocketpp::log::alevel::app);
    ws_server.init_asio();

    ws_server.set_open_handler(&on_open);
    ws_server.set_close_handler(&on_close);

    std::thread ws_thread(&run_ws);

    auto json_fpath = "config.json";

    json json_obj;
    if (!getJSONFromFile(json_fpath, json_obj))
    {
        std::cout << "error: getJSONFromFile (configuration)\n";
        return 0;
    }

    bool did_init = init_device("/home/pi/depthai/depthai.cmd", "");

    if (!did_init)
    {
        std::cout << "Failed to init device\n";
    }

    auto pipeline = create_pipeline(json_obj.dump());
    auto t0 = high_resolution_clock::now();

    std::vector<Detection> detections;

    while (true)
    {
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
                    .distance_z = 0,
                    .size       = 0
                };

                if (detection.confidence >= 0.8 && detection.label == 1)
                {
                    detections.push_back(detection);
                }
            }
        }

        for (auto it = data_packets.begin(); it != data_packets.end(); ++it)
        {
            if ((*it)->stream_name == "previewout")
            {
                auto data = (*it)->getData();
                auto dims = (*it)->dimensions;
                auto height = dims.at(1);
                auto width = dims.at(2);

                void* img_data_ptr = (void*) data;
                int channel_size = height * width;

                cv::Mat R(height, width, CV_8UC1, img_data_ptr);
                cv::Mat G(height, width, CV_8UC1, img_data_ptr + channel_size);
                cv::Mat B(height, width, CV_8UC1, img_data_ptr + channel_size * 2);

                std::vector<cv::Mat> channels;
                channels.push_back(R);
                channels.push_back(G);
                channels.push_back(B);

                cv::Mat color;
                cv::merge(channels, color);

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

                    cv::rectangle(color, cv::Point2f(d->left, d->top), cv::Point2f(d->right, d->bottom), cv::Scalar(23, 23, 255));
                }

                cv::imshow("previewout", color);
            }
        }

        for (auto i = 0; i < detections.size(); i++)
        {
            auto d = detections[i];
            json js = d;

            for (con_list::iterator it = ws_connections.begin(); it != ws_connections.end(); ++it)
            {
                ws_server.send(*it, js.dump(), websocketpp::frame::opcode::text);
            }
        }

        auto now = high_resolution_clock::now();
        auto dt = duration_cast<ms>(now - t0).count();
        t0 = now;

        // std::cout << "Elapsed: " << dt << "\n";

        const int key = cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    deinit_device();

    ws_thread.join();
}
