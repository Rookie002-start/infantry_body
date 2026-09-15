#include "gimbal_to.hpp"

#pragma message "Compiling Topic/gimbal_to"

// 状态通道：无订阅者（消费者用 zbus_chan_read 轮询，避免多线程共用 subscriber 抢通知）
ZBUS_CHAN_DEFINE(pub_gimbal_to,
                 topic::gimbal_to::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT({}));