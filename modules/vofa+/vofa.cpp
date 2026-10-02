/**
 * @file vofa.cpp
 * @author qingyu
 * @brief VOFA+ JustFloat 输出组件实现 —— 组帧进环形缓冲，TX 中断逐字节外发
 * @version 0.1
 * @date 2026-09-26
 *
 * 串口取设备树别名 `vofa-uart`；协议与用法见 vofa.hpp。
 *
 * 初始化归属：vofa::Init() 只由所有者组件 thread/vofa/trd_vofa.cpp（CONFIG_TRD_VOFA）
 * 在启动阶段调一次；业务线程只调 Send()，Send() 自己不做懒初始化。
 *
 * 为什么不用 uart_poll_out / uart_tx：
 *   · uart_poll_out 每个字节都等硬件 FIFO，调用线程按波特率被拖住
 *     （921600 bps 下发 16 字节 ≈ 0.2ms，1ms 控制环里不可接受）；
 *   · uart_tx（中断模式）会阻塞到整包进了 FIFO，长包同样会占住控制环。
 *   所以这里自己做"环形缓冲 + TX 中断"：组帧只 memcpy 一份就返回，发送在中断里推进，
 *   缓冲放不下就丢整帧（丢数据好过丢实时性）。
 *
 * @copyright Copyright (c) 2026
 */

#include "vofa.hpp"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>

#include <string.h>

LOG_MODULE_REGISTER(vofa, LOG_LEVEL_INF);

static_assert(sizeof(float) == 4, "JustFloat 要求 4 字节 IEEE-754 float");

namespace vofa
{

#if !IS_ENABLED(CONFIG_VOFA)

// 组件没打开（CONFIG_VOFA=n，通常是因为 TRD_VOFA=n）：保留空实现，
// 业务代码里的 vofa::Send(...) 照常能写、能链接，只是不发数据。
bool Init()
{
    LOG_WRN("vofa disabled (CONFIG_VOFA=n)");
    return false;
}

void Send(const float *ch, uint8_t n)
{
    ARG_UNUSED(ch);
    ARG_UNUSED(n);
}

void Send(std::initializer_list<float> ch)
{
    ARG_UNUSED(ch);
}

void Send(float v)
{
    ARG_UNUSED(v);
}

uint32_t DropCount()
{
    return 0;
}

#elif !DT_HAS_ALIAS(vofa_uart)

// 板卡没定义 vofa-uart 别名（例如 HPM 上板）：同样空实现，业务代码不用写 #if。
bool Init()
{
    LOG_WRN("no `vofa-uart` alias on this board: VOFA+ output disabled");
    return false;
}

void Send(const float *ch, uint8_t n)
{
    ARG_UNUSED(ch);
    ARG_UNUSED(n);
}

void Send(std::initializer_list<float> ch)
{
    ARG_UNUSED(ch);
}

void Send(float v)
{
    ARG_UNUSED(v);
}

uint32_t DropCount()
{
    return 0;
}

#else

namespace
{
    /// TX 环形缓冲大小：921600 bps ≈ 92 B/ms，这里能兜住 ~11ms 的堆积
    constexpr uint32_t kTxRingSize = 1024;

    /// 一次 TX 中断最多搬运的字节数（= 单帧上限，够一帧走完一次中断）
    constexpr uint32_t kTxChunkMax = kMaxFrameBytes;

    /// 丢帧计数每累计这么多上报一次（限流，避免刷屏）
    constexpr uint32_t kDropLogPeriod = 200;

    /// JustFloat 帧尾：0x7F800000（+Inf 位型），VOFA+ 靠它切帧
    constexpr uint8_t kTail[4] = { 0x00, 0x00, 0x80, 0x7F };

    uint8_t tx_mem[kTxRingSize];
    struct ring_buf tx_ring {};
    const device *uart_dev = nullptr;   // Init() 成功后非空，也是“已初始化”标志
    uint32_t drop_cnt = 0;
    bool warned_uninit = false;         // "没初始化就发"只报一次，避免刷屏

    /**
     * 多生产者串行化锁。
     *
     * Zephyr 的 ring_buf 只保证"单生产者"安全：两个线程各自 put_claim/put_finish
     * 时会互相踩 —— 后 finish 的那个看到 claimed_size 已经被对方清零，返回 -EINVAL，
     * 于是它 claim 掉的那段空间（put.head 已经前进）再也不会被归还，
     * 几十秒内就把环形缓冲漏满 → 之后所有 Send() 全部丢帧（整个 VOFA 输出都没了）。
     *
     * k_spin_lock 在单核上会同时关中断，所以 TX 中断也不会插进 claim/finish 中间。
     * 中断侧只碰 get 侧，不需要拿这把锁。
     */
    struct k_spinlock tx_lock {};

    /**
     * @brief UART TX 中断：把环形缓冲里的字节灌进硬件 FIFO
     *
     * FIFO 吃不下（返回 < 请求长度）就只 finish 已写入的部分，剩下的下次中断再来，
     * 不能在这里死等。缓冲空了就关 TX 中断，避免空转中断占 CPU。
     */
    void TxIsr(const device *dev, void *)
    {
        uart_irq_update(dev);
        if (!uart_irq_tx_ready(dev)) {
            return;
        }

        uint8_t *chunk = nullptr;
        uint32_t len;

        while ((len = ring_buf_get_claim(&tx_ring, &chunk, kTxChunkMax)) > 0u)
        {
            const int sent = uart_fifo_fill(dev, chunk, static_cast<int>(len));
            const uint32_t done = (sent > 0) ? static_cast<uint32_t>(sent) : 0u;

            // 只释放真正写进 FIFO 的字节；finish(0) 合法，剩余字节留到下次中断
            (void)ring_buf_get_finish(&tx_ring, done);

            if (done < len) {
                return;      // FIFO 满：等 TX 中断再来，别在这里自旋
            }
        }

        if (ring_buf_is_empty(&tx_ring)) {
            uart_irq_tx_disable(dev);
        }
    }

} // namespace

bool Init()
{
    if (uart_dev != nullptr) {
        return true;                                  // 幂等：重复调不会重复挂中断
    }

    const device *dev = DEVICE_DT_GET(DT_ALIAS(vofa_uart));
    if (!device_is_ready(dev)) {
        LOG_ERR("vofa_uart not ready");
        return false;
    }

    ring_buf_init(&tx_ring, sizeof(tx_mem), tx_mem);

    const int ret = uart_irq_callback_user_data_set(dev, TxIsr, nullptr);
    if (ret != 0) {
        // 驱动不支持中断收发的典型原因：CONFIG_UART_INTERRUPT_DRIVEN 没开
        LOG_ERR("vofa_uart irq callback fail (%d), check CONFIG_UART_INTERRUPT_DRIVEN", ret);
        return false;
    }

    uart_dev = dev;
    LOG_INF("vofa ready: %s @ %u baud (JustFloat)",
            dev->name,
            static_cast<unsigned>(DT_PROP_OR(DT_ALIAS(vofa_uart), current_speed, 0)));
    return true;
}

void Send(const float *ch, uint8_t n)
{
    if (ch == nullptr) {
        return;
    }
    if (uart_dev == nullptr) {
        // 初始化归所有者组件（thread/vofa/trd_vofa.cpp）在启动阶段做一次，
        // 这里不做懒初始化 —— 多线程同时首调会重复初始化同一路串口。
        if (!warned_uninit) {
            warned_uninit = true;
            LOG_WRN("vofa not initialized (CONFIG_TRD_VOFA?) - frame dropped");
        }
        return;
    }
    if (n > kMaxChannels) {
        n = kMaxChannels;
    }

    uint8_t frame[kMaxFrameBytes];
    for (uint8_t i = 0; i < n; ++i)
    {
        uint32_t bits = 0;
        memcpy(&bits, &ch[i], sizeof(bits));          // 取 float 的 IEEE-754 位型
        sys_put_le32(bits, &frame[i * 4]);            // JustFloat 线上就是小端
    }
    memcpy(&frame[n * 4], kTail, sizeof(kTail));

    const uint32_t len = static_cast<uint32_t>(n) * 4u + sizeof(kTail);

    // 临界区：查空间 + 整帧入队 + 开中断，必须原子（可以被多个线程调用）
    bool full = false;

    k_spinlock_key_t key = k_spin_lock(&tx_lock);

    if (ring_buf_space_get(&tx_ring) >= len) {
        const uint32_t put = ring_buf_put(&tx_ring, frame, len);
        __ASSERT_NO_MSG(put == len);                  // 临界区内先查空间 → 必定整帧写入
        ARG_UNUSED(put);

        // 无条件开 TX 中断：中断里"缓冲空了就关"，如果先判状态再开会和它抢跑
        // （中断在判定之后、开启之前把缓冲抽干并关中断 → 数据卡在缓冲里没人推）
        uart_irq_tx_enable(uart_dev);
    } else {
        full = true;                                  // 放不下就整帧丢，绝不让调用者等
        ++drop_cnt;
    }

    k_spin_unlock(&tx_lock, key);

    // 日志放到临界区外（关中断时不做 RTT 输出）
    if (full && (drop_cnt % kDropLogPeriod) == 1u) {
        LOG_WRN("vofa tx buffer full: frame dropped (total %u)",
                static_cast<unsigned>(drop_cnt));
    }
}

void Send(std::initializer_list<float> ch)
{
    Send(ch.begin(), static_cast<uint8_t>(ch.size()));
}

void Send(float v)
{
    Send(&v, 1);
}

uint32_t DropCount()
{
    return drop_cnt;
}

#endif // CONFIG_VOFA / DT_HAS_ALIAS(vofa_uart)

} // namespace vofa
