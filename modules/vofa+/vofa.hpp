/**
 * @file vofa.hpp
 * @author Rookie002-start
 * @brief VOFA+ 上位机波形输出组件 —— JustFloat 协议，串口取设备树别名 `vofa-uart`
 * @version 0.1
 * @date 2026-09-26
 *
 * ## 协议（VOFA+ "JustFloat"）
 *
 *     一帧 = n 个通道 × 4 字节 float（IEEE-754，小端） + 4 字节帧尾
 *     帧尾 = { 0x00, 0x00, 0x80, 0x7F }（0x7F800000，VOFA+ 用它切帧）
 *
 * 上位机设置：数据格式 = JustFloat，通道数 = 你实际发的个数，
 * 串口波特率 = 设备树里 `vofa-uart` 节点的 current-speed（dm_mc02：uart7 @921600）。
 *
 * ## 发送路径（为什么不直接 uart_poll_out）
 *
 *     业务线程 ──组帧──→ 环形缓冲 ──TX 中断──→ UART FIFO ──→ 总线
 *
 * 组帧只往环形缓冲拷一份（几十字节的 memcpy），随后立刻返回；真正的逐字节发送
 * 在 TX 中断里推进。所以可以在 1ms 控制环里直接调用，不会因为波特率低被拖住。
 * 缓冲放不下整帧（上位机没在收 / 波特率不够）时**整帧丢弃**并计数，绝不阻塞调用者。
 *
 * ## 用法
 *
 * ```cpp
 * #include "vofa.hpp"
 *
 * vofa::Send({ g_vx, g_vy, g_vw });        // 3 通道一帧（通道 0/1/2）
 * vofa::Send(omega);                       // 单通道一帧
 * vofa::Send(arr, 4);                      // 数组形式：arr[0..3] → 通道 0..3
 * ```
 *
 * 初始化由所有者组件在启动阶段做一次（`CONFIG_TRD_VOFA` → thread/vofa/trd_vofa.cpp，
 * 只 REGISTER_INIT、不建线程，和 thread/inter_bus 一个路子），业务线程只调 Send()。
 * Send() 不做懒初始化：没初始化就直接丢帧（只报一次警告），免得哪个线程顺手初始化、
 * 多线程首调时重复初始化同一路串口。
 *
 * 板卡没定义 `vofa-uart` 别名时组件照常编译链接，只是 Init() 失败、Send() 空转。
 *
 * ⚠️ 环形缓冲按"单生产者"设计：只用同一个线程调用 Send()；多线程同时发请自行加锁，
 *    或者把数据攒到一个线程里统一发。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <cstdint>
#include <initializer_list>

namespace vofa
{
    /// 单帧通道数上限（JustFloat 本身不限，这里只受内部缓冲约束）
    constexpr uint8_t kMaxChannels = 16;

    /// 单帧字节数上限：kMaxChannels × 4 字节 float + 4 字节帧尾
    constexpr uint8_t kMaxFrameBytes = kMaxChannels * 4 + 4;

    /**
     * @brief 初始化 vofa-uart（幂等）
     *
     * 做三件事：取设备树别名 `vofa-uart` 的设备 → 检查就绪 → 挂 TX 中断回调。
     *
     * ⚠️ 只应由所有者组件（thread/vofa/trd_vofa.cpp）在启动阶段调用一次；
     *    业务线程不要调，直接 Send() 即可。
     *
     * @return true=串口可用；false=没别名 / 设备没就绪（此后 Send() 都是空操作）
     */
    bool Init();

    /**
     * @brief 发一帧：ch[0..n-1] 依次作为通道 0..n-1，自动补帧尾
     * @param ch 通道数据（不合法指针直接返回）
     * @param n  通道数，超过 kMaxChannels 的部分截掉
     */
    void Send(const float *ch, uint8_t n);

    /// 同上，通道数由列表长度决定：vofa::Send({vx, vy, vw})
    void Send(std::initializer_list<float> ch);

    /// 单通道一帧
    void Send(float v);

    /// 累计丢帧数（环形缓冲放不下整帧的次数，诊断用）
    uint32_t DropCount();

} // namespace vofa
