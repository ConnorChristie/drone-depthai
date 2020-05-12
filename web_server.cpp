#include "web_server.h"

typedef websocketpp::connection_hdl connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;
typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;
typedef server::message_ptr message_ptr;

server ws_server;
con_list ws_connections;

std::queue<std::string> ws_send_queue;
std::queue<std::string> ws_recv_queue;

std::mutex ws_send_lock;
std::timed_mutex ws_recv_lock;
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

void on_message(connection_hdl hdl, message_ptr msg)
{
    std::lock_guard<std::timed_mutex> guard(ws_recv_lock);
    ws_recv_queue.push(msg->get_payload());
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
    ws_server.set_message_handler(&on_message);

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
        std::unique_lock<std::mutex> lock(ws_send_lock);
        ws_action_cond.wait(lock);

        if (!is_running) break;

        auto msg = ws_send_queue.front();
        ws_send_queue.pop();

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
        std::lock_guard<std::mutex> guard(ws_send_lock);
        ws_send_queue.push(msg);
    }

    ws_action_cond.notify_one();
}

std::chrono::milliseconds one_ms(1);

std::vector<std::string> ws_get_messages()
{
    std::vector<std::string> ret;
    std::unique_lock<std::timed_mutex> lock(ws_recv_lock, one_ms);

    while (!ws_recv_queue.empty())
    {
        ret.push_back(ws_recv_queue.front());
        ws_recv_queue.pop();
    }

    lock.unlock();

    return ret;
}
