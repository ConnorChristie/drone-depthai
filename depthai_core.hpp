#pragma once

#include <string>

#include "../core/pipeline/cnn_host_pipeline.hpp"

bool soft_deinit_device();
bool init_device(
    const std::string &device_cmd_file,
    const std::string &usb_device
);
std::shared_ptr<CNNHostPipeline> create_pipeline(
    const std::string &config_json_str
);
bool deinit_device();