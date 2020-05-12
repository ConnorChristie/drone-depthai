#pragma once

#include <set>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/common/thread.hpp>

extern volatile sig_atomic_t is_running;

void run_ws();
void stop_ws();
void run_ws_sender();
void queue_ws_broadcast(std::string msg);
std::vector<std::string> ws_get_messages();
