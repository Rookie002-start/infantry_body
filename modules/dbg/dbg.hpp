/**
 * @file dbg.hpp
 * @author Rookie002-start
 * @brief 公共调参接口 —— 用双环 PID 的业务对象把自己的参数登记给调参命令行
 * @version 1.0
 * @date 2026-09-27
 *
 * ## 依赖方向（低耦合 / 业务线程互不包含）
 *
 *     业务（gimbal / ...） ------------只 include 本头文件--> dbg 命令行线程
 *     dbg.cpp ------------------------不认识任何业务类型
 *
 * 业务侧在初始化时把"一台电机 / 一个轴"的内外环两个 Pid 地址交出来：
 *
 *     dbg::RegisterPid({ "gimbal", "yaw", &axis.position, &axis.omega, YawLive, ctx });
 *
 * 之后就能在 RTT 终端里用通用命令读写它们的 kp/ki、看实时值。业务线程之间、
 * 业务与 dbg 实现之间都不互相 include，只共享这一个头文件。
 *
 * ## CONFIG_DBG=n 时
 *
 * 本头文件里的函数全部退化成空操作（`return false` / 什么都不做），所以业务代码
 * **不用写任何 #if**：线程离开 dbg 也能独立、高内聚地跑，PID 初值就是业务自己的常量。
 *
 * ## 命令（dbg.cpp 提供）
 *
 *     help                                这条
 *     pid                                 列出所有已登记的轴（group/axis/内外环 kp ki）
 *     pid <group> [<axis>]                看某组 / 某条轴（有 live 回调就带实时值）
 *     pid <group> <axis> out <kp> <ki>    改外环增益
 *     pid <group> <axis> in  <kp> <ki>    改内环增益
 *     pid <group> <axis> mon <ms>         周期打印该轴实时值（0=关）
 *
 * 写进去的只是 RAM 里的 kp/ki（单次 32 位写，跨线程安全；业务线程下一拍就用到新值）。
 * 掉电/复位会回到业务初值 —— 想长期生效，把整定好的数字抄回业务文件的初值常量再烧录。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "pid.hpp"       // alg::pid::Pid（双环 PID 对象就是两个 Pid）
#include <cstdint>

namespace dbg
{
    /// 最多登记多少条双环 PID（当前只有云台 2 轴，留足余量）
    constexpr uint8_t kMaxPids = 16;

    /// 一条双环 PID 的实时值（没有实时值的对象把 live 留空即可）
    struct PidLive
    {
        float ref_outer  = 0.0f;   ///< 外环给定（位置环给角度 / 速度环给 ω）
        float meas_outer = 0.0f;   ///< 外环实测
        float ref_inner  = 0.0f;   ///< 内环给定（通常是外环输出）
        float meas_inner = 0.0f;   ///< 内环实测（力矩 / 电流）
    };

    /// 实时值读取回调：返回 false 表示这一拍没有有效数据
    using PidLiveFn = bool (*)(void *ctx, PidLive &out);

    /// 登记项：一条"轴"（一台电机）的两个环
    struct PidKnobs
    {
        const char    *group;             ///< 组名："gimbal"
        const char    *axis;              ///< 轴名："yaw" / "pitch"
        alg::pid::Pid *outer = nullptr;   ///< 外环（速度 / 位置环），可空
        alg::pid::Pid *inner = nullptr;   ///< 内环（力矩环），可空
        PidLiveFn      live  = nullptr;   ///< 可选：实时值（给 `pid` / `mon` 用）
        void          *ctx   = nullptr;   ///< 原样传给 live
    };

#if defined(CONFIG_DBG)
    /// 登记一条双环 PID（业务初始化时调一次；表满或参数非法返回 false）
    bool RegisterPid(const PidKnobs &knobs);
    /// 回一行给调参终端（走日志，串口和 RTT 都能看到）
    void Reply(const char *fmt, ...);
    /// 业务命令解析参数用的小工具（失败会自己回一行提示并返回 false）
    bool ParseFloat(const char *tok, const char *what, float &out);
    bool ParseInt(const char *tok, const char *what, long min, long max, long &out);
#else
    // 没开调参终端：空操作 —— 业务代码照常调用，编译后不占任何资源
    inline bool RegisterPid(const PidKnobs &) { return false; }
    inline void Reply(const char *fmt, ...) { (void)fmt; }
    inline bool ParseFloat(const char *, const char *, float &) { return false; }
    inline bool ParseInt(const char *, const char *, long, long, long &) { return false; }
#endif

} // namespace dbg
