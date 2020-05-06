#pragma once

#include <set>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <nlohmann/json.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/common/thread.hpp>

extern volatile sig_atomic_t is_running;

void run_ws();
void stop_ws();
void run_ws_sender();
void broadcast_ws(std::string msg);
