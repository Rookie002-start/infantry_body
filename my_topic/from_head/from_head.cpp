/**
 * @file from_head.cpp
 * @author qingyu
 * @brief 上板→下板接收数据契约定义
 * @version 0.1
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#include "from_head.hpp"

#pragma message "Compiling Topic/From_Head"

// 状态通道：无订阅者（消费者用 zbus_chan_read 轮询，避免多线程共用 subscriber 抢通知）
ZBUS_CHAN_DEFINE(pub_from_head,
                 topic::from_head::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT({}));
