/**
 * @file trd_vofa.cpp
 * @author qingyu
 * @brief VOFA+ 输出组件所有者实现（无 REGISTER_THREAD，只有 REGISTER_INIT）
 * @version 0.1
 * @date 2026-09-26
 *
 * 为什么单独拎一个"没有线程的文件"：
 *   vofa::Init() 会挂 UART 的 TX 中断回调、初始化发送环形缓冲，全局只能做一次。
 *   如果让各个业务线程在第一次 Send() 时懒初始化，多线程同时首调就会重复初始化
 *   （模块内部没有锁，vofa::Init() 自身的判重也是"检查后设置"的竞态）。
 *   所以把初始化固定到启动阶段（REGISTER_INIT）由这里做一次，
 *   业务线程只调 vofa::Send()，谁都不碰初始化。
 *
 * 与 thread/inter_bus 的约定一致：只做初始化，不建线程、不做周期性收发。
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Vofa"

#include "Init_entry.hpp"
#include "vofa.hpp"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trd_vofa, LOG_LEVEL_INF);

namespace thread::vofa
{
    /// 只做一件事：把 vofa-uart 初始化一次。
    ///
    /// 时机：阶段 MidInit 早于所有 MidThread 启动，所以最早的业务线程调 Send() 时
    /// 串口已经就绪（Zephyr 的 UART 设备本身在 main() 之前就 init 完了）。
    ///
    /// 失败处理：返回 false 但等级用 Mid —— 框架对 Mid/Low 只打一行警告、不停机。
    /// VOFA+ 是调试输出，板卡没这路串口（没有 vofa-uart 别名）或上位机没接，
    /// 都不该把整车拦在启动阶段；此后 Send() 就是空操作（丢帧计数不涨）。
    static bool thread_init()
    {
        // 注意：本文件在 thread::vofa 里，会遮住组件命名空间 vofa，所以写全限定 ::vofa
        return ::vofa::Init();
    }

    REGISTER_INIT(thread_init, MidInit, Mid, "vofa_init");

} // namespace thread::vofa
