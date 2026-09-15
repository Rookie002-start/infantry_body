/**
 * @file trd_mcu_inter.cpp
 * @author qingyu
 * @brief 下板 → 上板发送线程
 *        - 入队的状态分片存进【本线程私有】缓存，按 Dir::Down 布局聚合成一帧定长发出（0x200）
 *
 * ⚠️ 本线程是 user-can2 总线的所有者：初始化与收帧分发入口都在这里。
 *    接收侧（thread/inter_rx）只注册 CAN_RX_HANDLER，**不要**再次 Can::Init() /
 *    SetRxCallback()，否则会重复注册过滤器并覆盖分发入口。
 *
 * @version 0.5
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/trd_mcu_inter"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_mcu_tx.hpp"
#include "inter_cmd.hpp"
#include "can.hpp"
#include "Irq_handlers.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcu_inter, LOG_LEVEL_INF);

namespace thread::mcu_inter
{
    static Thread<2048> thread_{};
    static Can mcu_inter_can{};

    static constexpr inter_cmd::Dir kDir      = inter_cmd::Dir::Down;   // 本线程发送方向
    static constexpr uint8_t        kFrameSize = 64;                    // 聚合帧载荷上限
    static constexpr uint16_t       kTxId      = inter_cmd::StateTxId(kDir);          // 0x200
    static constexpr uint8_t        kStateLen  = inter_cmd::FrameLen(kDir);           // 12

    // ---- 私有分片缓存：按契约布局表下标索引（本线程独有，不共享 → 无需锁）----
    struct Slot
    {
        uint8_t data[kFrameSize];
        uint8_t len;
        bool    valid;
    };
    static Slot slots[inter_cmd::kStateFragCount] {};

    /// 更新本方向的分片缓存
    static void UpdateSlot(inter_cmd::FrameType type, const uint8_t *data, uint8_t len)
    {
        for (uint8_t i = 0; i < inter_cmd::kStateFragCount; ++i)
        {
            const inter_cmd::StateFrag &f = inter_cmd::kStateFrags[i];
            if (f.dir != kDir || f.type != type) {
                continue;
            }
            if (len > f.len) {
                len = f.len;                       // 按契约长度截断
            }
            memcpy(slots[i].data, data, len);
            slots[i].len   = len;
            slots[i].valid = true;
            return;
        }
        LOG_WRN("not my direction/type: 0x%02x", static_cast<uint8_t>(type));
    }

    /// 按契约表把本方向分片拼成一帧（返回 0 = 还没有任何分片）
    static uint8_t PackState(uint8_t *buf)
    {
        bool any = false;
        memset(buf, 0, kFrameSize);

        for (uint8_t i = 0; i < inter_cmd::kStateFragCount; ++i)
        {
            if (!slots[i].valid) {
                continue;
            }
            const inter_cmd::StateFrag &f = inter_cmd::kStateFrags[i];
            if (f.dir != kDir) {
                continue;
            }
            if (static_cast<uint16_t>(f.offset) + slots[i].len > kFrameSize) {
                LOG_WRN("state slot %u overflow", i);
                continue;
            }
            memcpy(buf + f.offset, slots[i].data, slots[i].len);
            any = true;
        }
        // 定长发送：接收端按固定偏移解析，短帧会让尾部区域读到未定义字节
        return any ? kStateLen : 0;
    }

    static void SendFrame(uint16_t id, const uint8_t *data, uint8_t len)
    {
        if (len > CAN_MAX_DLEN) {
            return;
        }
        can_frame tx{};
        tx.id    = id;
        tx.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;   // FD 帧 + 数据段 2Mbps（BRS）
        tx.dlc   = can_bytes_to_dlc(len);
        memcpy(tx.data, data, len);
        mcu_inter_can.Send(&tx);
    }

    static void Task(void*, void*, void*)
    {
        static constexpr uint32_t kPeriodMs = 1;

        for (;;)
        {
            const int64_t tick_start = k_uptime_get();

            // 1) 清空队列：本方向各分片更新缓存
            topic::to_mcu_tx::Message ev{};
            while (k_msgq_get(&user_can2_msgq, &ev, K_NO_WAIT) == 0)
            {
                UpdateSlot(static_cast<inter_cmd::FrameType>(ev.tag), ev.data, ev.len);
            }

            // 2) 聚合本方向状态帧：一帧装齐所有分片最新值
            uint8_t buf[kFrameSize];
            const uint8_t len = PackState(buf);
            if (len > 0) {
                SendFrame(kTxId, buf, len);
            }

            const int64_t elapsed = k_uptime_get() - tick_start;
            const int64_t remain  = static_cast<int64_t>(kPeriodMs) - elapsed;
            if (remain > 0) {
                k_msleep(remain);
            }
        }
    }

    bool thread_init()
    {
        const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can2));
        if (!device_is_ready(dev)) {
            LOG_ERR("user_can2 not ready");
            return false;
        }

        const can_filter filter{.id = 0, .mask = 0, .flags = 0};
        // 必须进 FD 模式（FDOE/BRSE），否则 FD 帧收发都会失败
        if (!mcu_inter_can.Init(dev, filter, CAN_MODE_FD)) {
            LOG_ERR("mcu_inter_can init fail");
            return false;
        }
        // 收帧分发入口：接收侧（thread/inter_rx）的 CAN_RX_HANDLER 依赖它
        mcu_inter_can.SetRxCallback(user_can2_rx_callback);
        LOG_INF("inter tx ready (rx 0x%03x, tx 0x%03x, len %u)",
                static_cast<unsigned>(inter_cmd::StateTxId(inter_cmd::Dir::Up)),
                static_cast<unsigned>(kTxId), static_cast<unsigned>(kStateLen));
        return true;
    }

    bool thread_start()
    {
        thread_.Start(Task, ThreadPrio::High);
        return true;
    }

    REGISTER_INIT  (thread_init,  MidInit,    High, "mcu_inter_init");
    REGISTER_THREAD(thread_start, MidThread,  High, "mcu_inter_start");
}
