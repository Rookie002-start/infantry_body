/**
 * @file trd_mcu_inter.cpp
 * @author qingyu
 * @brief 下板 → 上板发送线程（经典 CAN 拆帧）
 *        - 状态分片有两个来源：
 *          ① 拉取（pull）：本板已有消费者、只发一次通道的量 → RefreshFromChannels()
 *          ② 入队（push）：只给对端用、本地没人消费的量 → PostFrame 走 k_msgq
 *        - 两个来源都写进同一份【本线程私有】分片缓存，聚合成 12B 聚合帧后整帧发出（0x200）
 *
 * ⚠️ 本线程是 user-can2 总线的所有者：初始化与收帧分发入口都在这里。
 *    接收侧（thread/inter_rx）只注册 CAN_RX_HANDLER，**不要**再次 Can::Init() /
 *    SetRxCallback()，否则会重复注册过滤器并覆盖分发入口。
 *
 * ⚠️ 总线带宽（CAN FD：仲裁段 1Mbps / 数据段 2Mbps，FDF + BRS）：
 *    本方向 12B 一帧 ≈ 0.12ms，对端 62B 一帧 ≈ 0.34ms，一轮合计 ≈ 0.46ms
 *    → 1ms 周期约占 46%，可行。总线忙时靠 Can::Send 的超时等待空邮箱，不直接丢帧。
 *
 * @version 0.6
 * @date 2026-09-15
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/trd_mcu_inter"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_mcu_tx.hpp"
#include "inter_cmd.hpp"
#include "gimbal_to.hpp"      // pull 来源：云台通道
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

    static constexpr inter_cmd::Dir kDir       = inter_cmd::Dir::Down;   // 本线程发送方向
    static constexpr uint8_t        kFrameSize = 64;                     // 聚合帧缓冲上限
    static constexpr uint8_t        kStateLen  = inter_cmd::FrameLen(kDir);    // 12
    static constexpr uint32_t       kPeriodMs  = 1;                      // FD 整帧，见文件头带宽说明
    static constexpr uint32_t       kTxTimeoutMs = 2;                    // 等空 TX 邮箱，别直接丢帧

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

    /// 按契约表把本方向分片拼成聚合帧（返回 0 = 还没有任何分片）
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
        // 定长：接收端按固定偏移解析，短帧会让尾部区域读到未定义字节
        return any ? kStateLen : 0;
    }

    /// 发一帧 CAN FD：整帧一次发完（FDF + BRS，DLC 用 can_bytes_to_dlc 编码）
    static void SendFrame(const uint8_t *buf, uint8_t len)
    {
        if (len == 0 || len > CAN_MAX_DLEN) {
            return;
        }
        can_frame tx{};
        tx.id    = inter_cmd::StateTxId(kDir);
        tx.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;   // FD 帧 + 数据段 2Mbps
        tx.dlc   = can_bytes_to_dlc(len);
        memcpy(tx.data, buf, len);
        if (!mcu_inter_can.Send(&tx, K_MSEC(kTxTimeoutMs))) {
            // 等不到空 TX 缓冲区（总线被占满 / 控制器没起来）：限流记录，别刷屏
            static int64_t last_warn_ms = 0;
            const int64_t now = k_uptime_get();
            if (now - last_warn_ms >= 1000) {
                last_warn_ms = now;
                LOG_WRN("can tx busy (TX 缓冲区一直满)");
            }
        }
    }

    /// ① 拉取来源：本板通道里的量（云台线程只发通道，这里拉过来打包）
    /// 读失败（生产者持锁）时沿用上一轮缓存值，不清零。
    static void RefreshFromChannels()
    {
        // GimbalState ← pub_gimbal_to（云台电机反馈 yaw + pitch）
        static topic::gimbal_to::Message g {};
        if (zbus_chan_read(&pub_gimbal_to, &g, K_NO_WAIT) == 0) {
            const inter_cmd::GimbalState st { .yaw   = g.yaw_fb_rad,
                                              .pitch = g.pitch_fb_rad };
            UpdateSlot(inter_cmd::FrameType::StateGimbal,
                       reinterpret_cast<const uint8_t *>(&st), sizeof(st));
        }
    }

    static void Task(void*, void*, void*)
    {
        for (;;)
        {
            const int64_t tick_start = k_uptime_get();

            // 1) 拉取通道来源的分片
            RefreshFromChannels();

            // 2) 清空队列：只给对端用的分片（push 路径）
            topic::to_mcu_tx::Message ev{};
            while (k_msgq_get(&user_can2_msgq, &ev, K_NO_WAIT) == 0)
            {
                UpdateSlot(static_cast<inter_cmd::FrameType>(ev.tag), ev.data, ev.len);
            }

            // 3) 聚合本方向状态帧 → 拆帧发出
            uint8_t buf[kFrameSize];
            const uint8_t len = PackState(buf);
            if (len > 0) {
                SendFrame(buf, len);
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

        // CAN FD：必须进 FD 模式（FDOE/BRSE），否则发不出 FD 帧
        const can_filter filter{.id = 0, .mask = 0, .flags = 0};
        if (!mcu_inter_can.Init(dev, filter, CAN_MODE_FD)) {
            LOG_ERR("mcu_inter_can init fail");
            return false;
        }
        // 收帧分发入口：接收侧（thread/inter_rx）的 CAN_RX_HANDLER 依赖它
        mcu_inter_can.SetRxCallback(user_can2_rx_callback);
        LOG_INF("inter tx ready (dir=%s, FD frame %uB, period=%ums)",
                (kDir == inter_cmd::Dir::Up) ? "up" : "down",
                static_cast<unsigned>(kStateLen), static_cast<unsigned>(kPeriodMs));
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
