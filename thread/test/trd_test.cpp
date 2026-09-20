#pragma message "Compiling Thread/Test"

#include "thread.hpp"
#include "Init_entry.hpp"
#include <zephyr/kernel.h>
#include "zephyr/zbus/zbus.h"
#include "from_head.hpp"

namespace thread::test {

static Thread<2048> thread_{};

static void Task(void*, void*, void*)
{
    topic::from_head::Message g;
    for (;;)
    {
        if (zbus_chan_read(&pub_from_head, &g, K_NO_WAIT) == 0)
        {
            printk("yaw_angle:%f\r\n", g.comm.yaw_angle);
            printk("pitch_angle:%f\r\n", g.comm.pitch_angle);
            printk("vx:%f\r\n", g.comm.chassis_vx);
            printk("vy:%f\r\n", g.comm.chassis_vy);
            printk("yaw:%f\r\n", g.imu.total_yaw_angle);
            printk("pitch:%f\r\n", g.imu.pitch_angle);
        }
        
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
