#include "web_server.h"

typedef websocketpp::connection_hdl connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;
typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;

server ws_server;
con_list ws_connections;

std::queue<std::string> ws_queue;

std::mutex ws_action_lock;
std::mutex ws_connection_lock;
std::condition_variable ws_action_cond;

void on_open(connection_hdl hdl)
{
    std::lock_guard<std::mutex> guard(ws_connection_lock);
    ws_connections.insert(hdl);
}

void on_close(connection_hdl hdl)
{
    std::lock_guard<std::mutex> guard(ws_connection_lock);
    ws_connections.erase(hdl);
}

void run_ws()
{
    ws_server.clear_access_channels(websocketpp::log::alevel::all);
    ws_server.set_access_channels(websocketpp::log::alevel::access_core);
    ws_server.set_access_channels(websocketpp::log::alevel::app);

    ws_server.set_reuse_addr(true);
    ws_server.init_asio();

    ws_server.set_open_handler(&on_open);
    ws_server.set_close_handler(&on_close);

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

void stop_ws()
{
    ws_action_cond.notify_one();
    ws_server.stop_listening();

    for (con_list::iterator it = ws_connections.begin(); it != ws_connections.end(); ++it)
    {
        ws_server.close(*it, websocketpp::close::status::normal, "");
    }

    ws_server.stop();
}

void run_ws_sender()
{
    while (is_running)
    {
        std::unique_lock<std::mutex> lock(ws_action_lock);
        ws_action_cond.wait(lock);

        if (!is_running) break;

        auto msg = ws_queue.front();
        ws_queue.pop();

        {
            std::lock_guard<std::mutex> guard(ws_connection_lock);

            for (con_list::iterator it = ws_connections.begin(); it != ws_connections.end(); ++it)
            {
                ws_server.send(*it, msg, websocketpp::frame::opcode::text);
            }
        }

        lock.unlock();
    }
}

void queue_ws_broadcast(std::string msg)
{
    {
        std::lock_guard<std::mutex> guard(ws_action_lock);
        ws_queue.push(msg);
    }

    ws_action_cond.notify_one();
}