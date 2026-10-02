#pragma message "Compiling Thread/Test"

#include "thread.hpp"
#include "Init_entry.hpp"
#include <zephyr/kernel.h>
#include "zephyr/zbus/zbus.h"
#include "vofa.hpp"

namespace thread::test {

static Thread<2048> thread_{};

static void Task(void*, void*, void*)
{
    float tx[] = {0.1, 0.2, 0.3};
    for (;;)
    {
        vofa::Send(tx, sizeof(tx) / sizeof(tx[0]));
        k_msleep(100);
    }
}

bool thread_init()
{
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::Low);
    return true;
}

REGISTER_INIT  (thread_init,  PreInit,   Low, "test_init");
REGISTER_THREAD(thread_start, PreThread, Low, "test_start");

}
