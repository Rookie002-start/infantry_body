/**
 * @file trd_chassis.hpp
 * @author qingyu
 * @brief 底盘实例集合：电机对象 / 速度环 PID / CAN 数据槽位表
 * @version 0.2
 * @date 2026-09-15
 *
 * 只被 thread/chassis/trd_chassis.cpp 使用：这里只放"实例与常量"，
 * 不放控制逻辑、不放 CAN_RX_HANDLER 注册（注册放在 .cpp，避免头文件
 * 被多处包含时在 .can_rx1 段里重复登记同一个 ID）。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "Irq_handlers.h"
#include "to_can_tx.hpp"
#include "dji_c6xx.hpp"
#include "pid.hpp"
#include <cstdint>

/// 底盘电机反馈所在 bus（user-can1）；总线所有者是 thread/can
#define CHASSIS_RX_CAN USER_RX_CAN1

namespace instance::chassis
{
    /// 控制帧去向：CAN 发送 topic（消费者 = thread/can 的发送线程）
    constexpr auto *chassis_tx = &user_can1_msgq;
    constexpr auto *to_head = &user_can2_msgq;

    constexpr uint8_t  kMotorCount = 4;                                        ///< 麦轮数量
    constexpr uint16_t kMotorRxId[kMotorCount] = {0x201, 0x202, 0x203, 0x204}; ///< C620 反馈 ID

    inline motor::dji::DjiC620 chassis_motor[kMotorCount] {};                  ///< C620 + M3508
    inline alg::pid::Pid    chassis_motor_omega_pid[kMotorCount] {};           ///< 外环：轮速 ω → 力矩给定
    inline alg::pid::Pid    chassis_motor_torque_pid[kMotorCount] {};          ///< 内环：力矩 → 电流
}
