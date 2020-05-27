#include <pthread.h>
#include <sched.h>

#include "drone.h"
#include "web_server.h"

volatile sig_atomic_t is_running = 1;

void my_handler(int s)
{
    if (!is_running)
    {
        exit(1);
    }
    else
    {
        is_running = 0;
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, my_handler);

    init_drone();

    std::thread thread_ws(&run_ws);
    std::thread thread_ws_sender(&run_ws_sender);
    // std::thread thread_drone(&run_drone);

    int policy = SCHED_OTHER;
    struct sched_param priority;
    priority.sched_priority = 5;

    pthread_setschedparam(thread_ws.native_handle(), policy, &priority);
    pthread_setschedparam(thread_ws_sender.native_handle(), policy, &priority);

    try
    {
        run_detection();
    }
    catch (const std::exception &e)
    {
        std::cout << "Unhandled exception:\n";
        std::cout << e.what() << std::endl;
    }

    deinit_drone();
    deinit_device();
    stop_ws();

    thread_ws.join();
    thread_ws_sender.join();
    // thread_drone.join();
}
