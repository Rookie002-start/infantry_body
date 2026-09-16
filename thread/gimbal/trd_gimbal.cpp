/**
 * @file trd_gimbal.cpp
 * @author qingyu
 * @brief 云台控制线程 — 1ms 固定周期：上板指令 → 双环 PID → DM 控制帧 → user-can3
 * @version 0.2
 * @date 2026-09-16
 *
 * ## 坐标与角度约定
 *
 *   yaw   ：绕 z 轴，逆时针为正，可无限旋转（虚拟角连续累加，不折叠）
 *   pitch ：抬头为正，软限位 [kPitchMin, kPitchMax]
 *
 * ## 数据来源与所有权
 *
 * | 数据 | 通道 / 段 | 生产者 | 本线程的角色 |
 * |------|-----------|--------|--------------|
 * | 目标角 | zbus `pub_from_head` | thread/inter_rx | 轮询读（offline → 冻结虚拟角） |
 * | IMU 角反馈 | zbus `pub_imu_to`（仅 IMU 模式） | thread/imu | 订阅取最新并排空队列 |
 * | 编码器角反馈 | `.can_rx3` 段 | 本线程（总线所有者） | 只注册 CAN_RX_HANDLER |
 * | 电机控制帧 | k_msgq `user_can3_msgq` | 本线程 | 入队后由本线程外发 |
 * | 云台相对底盘角 | zbus `pub_gimbal_to` | 本线程 | 发布（底盘做云台系→车体系变换） |
 * | 实测 yaw/pitch | inter_cmd `Dir::Down` | 本线程 | PostFrame 发给上板 |
 *
 * 总线所有权：user-can3 由本线程持有并初始化（Init + 收帧分发入口），
 * 与 thread/can（can1）、thread/mcu_inter（can2）的约定一致。
 *
 * ## 控制流（每轴一组双环）
 *
 *     目标角(虚拟角, 斜坡限速) ─→ ⊖ ─→ [位置环] ─→ ω_ref ─→ ⊖ ─→ [角速度环] ─→ τ ─→ DM MIT 帧
 *                                ↑                          ↑
 *                        实测角(IMU/编码器)            实测 ω(电机反馈)
 *
 *   DM 侧 kp/kd = 0，闭环全在板端；力矩由内环直接给出。
 *
 * ## 反馈源二选一（Kconfig TRD_GIMBAL_FB_SOURCE，互斥）
 *
 *   · TRD_GIMBAL_FB_ENCODER：角度反馈取电机编码器 —— 控制"云台相对底盘"的角，
 *     底盘自转时云台跟着转，不依赖 IMU（默认）。
 *   · TRD_GIMBAL_FB_IMU：角度反馈取 imu_to 的 yaw_total/pitch —— 控制世界系绝对角，
 *     底盘自转时云台可稳住指向。要求 IMU 随云台运动，且 IMU 掉线即不再驱动电机。
 *
 * ## 对齐沿（安全）
 *
 *   每轴记录上一拍的使能状态与反馈有效性；出现下面两种"沿"时，把虚拟角直接
 *   对齐到实测角（IMU 或编码器）并清 PID 积分，避免目标与实际差一大截导致
 *   电机猛抽 / 撞限位：
 *     ① 电机「失能 → 使能」；
 *     ② 角度反馈「无效 → 有效」（IMU 掉线恢复等，期间云台可能是自由状态）。
 *   对齐之后虚拟角按 kMaxYawRate / kMaxPitchRate 斜坡跟上上板指令。
 *
 * ## 失能 / 故障策略
 *
 *   · 未使能：每 kEnableResendTicks 重发一次使能帧（DM 上电默认失能）；
 *   · 非「失能」的其它状态（超压/过流/过温/通讯丢失…）：不驱动、不自动清故障，
 *     只上报一次，命令帧照发（避免电机再叠加一个"通讯丢失"故障）；
 *   · 角度反馈无效：力矩置 0 但保持发帧（不拿陈旧角度闭环）。
 *
 * ## 待实测参数
 *
 * | 参数 | 当前值 | 说明 |
 * |------|--------|------|
 * | `kPitchMin/Max` | -0.310 / +0.350 rad | pitch 软限位（关节侧物理角） |
 * | `kJointRatio` | 10.0 | 10:1 减速比（已确认） |
 * | `k*MotorSign` | +1 | 电机安装方向（单轴点动确认） |
 * | `k*ImuSign` | +1（IMU 模式） | IMU 安装方向 |
 * | 位置环 | kp=4.0 / ki=0 | 输出 = 角速度给定 rad/s，待整定 |
 * | 速度环 | kp=0.3 / ki=0 | 输出 = 力矩给定（电机侧 N·m），待整定 |
 * | `kTorqueLimit` | 2.0 N·m（电机侧） | 内环输出限幅；关节侧 ≈ ×10×0.85 ≈ 17 N·m |
 * | `kMotorTmax` | 10.0 N·m | 电机手册 TMAX，协议标定用，**不随限幅改** |
 *
 * ## 减速比 10:1 的单位约定（重要）
 *
 * DmMotor 模块的单位并不统一，业务侧按这张表换算：
 *
 * | 量 | 模块内的语义 | 本线程的处理 |
 * |----|--------------|--------------|
 * | `snap.radian` | **电机侧**角度（没除减速比） | ÷ kJointRatio → 关节角 |
 * | `snap.omega`  | 模块已 ÷ gearbox_ratio → 关节角速度 | 直接用 |
 * | `snap.torque` / `SetTargetTorque` | **电机侧** N·m（标定 = kMotorTmax） | 直接用（输出限幅 = kTorqueLimit） |
 *
 * 所以：角度反馈除 10，角速度不动，力矩不换算（减速比对力矩的影响
 * 折进速度环增益里整定；关节力矩 = 电机力矩 × 10 × 效率）。
 * 内环限幅同样按"电机侧"给：想按关节侧定值（例如机构只允许 10 N·m），
 * 就填 kTorqueLimit = 10 / (10 × 0.85) ≈ 1.2。
 *
 * ⚠️ 上电第一步先验证这条约定：电机使能但力矩很小，用手把关节转一圈，
 *    看 `snap.radian` 的变化量 ——
 *      ≈ 62.8 rad（10 × 2π，电机侧）→ 当前实现正确（除 10）；
 *      ≈ 6.28 rad （2π，协议本身就是关节侧）→ 把 kJointRatio 和
 *      cfg.gearbox_ratio 一起设回 1.0（两者都由 kJointRatio 驱动）。
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Gimbal"

#include "from_head.hpp"
#include "gimbal_to.hpp"
#include "inter_cmd.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "trd_gimbal.hpp"
#include "can.hpp"

#if CONFIG_TRD_GIMBAL_FB_IMU
#include "imu_to.hpp"          // 仅 IMU 模式需要（include 路径由 DUST_TPC_IMU_TO 提供）
#endif

#include <algorithm>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gimbal, LOG_LEVEL_INF);

namespace {

constexpr float kPi  = 3.14159265f;
constexpr float k2Pi = 2.0f * kPi;

/// 角度归一化到 [-π, π)
float NormalizeAngle(float a)
{
    a = fmodf(a, k2Pi);
    if (a >  kPi) a -= k2Pi;
    if (a < -kPi) a += k2Pi;
    return a;
}

} // namespace

namespace thread::gimbal {

using namespace instance::gimbal;

static Thread<2048> thread_ {};
static Can can3 {};                          // user-can3 总线对象（本线程私有）

// ==================== 周期 ====================
static constexpr uint32_t kPeriodMs = 1;
static constexpr float    kDt      = static_cast<float>(kPeriodMs) / 1000.0f;

// ==================== 虚拟角斜坡限速（rad/s）====================
// 使能对齐之后，虚拟角用这个速率平滑接管上板指令；同时也是指令突变保护
static constexpr float kMaxYawRate   = 3.0f;
static constexpr float kMaxPitchRate = 2.0f;

// ==================== 机械与限幅 ====================
static constexpr float kPitchMin    = -0.310f;                    // 关节侧 rad，抬头为正
static constexpr float kPitchMax    =  0.350f;
static constexpr float kJointRatio  = 10.0f;                      // 10:1 减速比（已确认）
static constexpr float kMotorVmax   = 30.0f;                      // 电机侧角速度上限 rad/s（DM VMAX）
// 电机/协议力矩标定：必须等于电机手册的 TMAX（反馈力矩解码与 MIT 帧编码都用它）
static constexpr float kMotorTmax   = 10.0f;
// 内环输出限幅（本线程实际发出的力矩上限，电机侧）——保护机构用，先给小值：
//   关节侧等效 ≈ kTorqueLimit × 减速比 × 效率(0.8~0.9) ≈ 2 × 10 × 0.85 ≈ 17 N·m
static constexpr float kTorqueLimit = 2.0f;
static_assert(kTorqueLimit <= kMotorTmax, "inner-loop torque limit exceeds motor TMAX");
// 位置环输出的是"关节侧"角速度给定，上限 = 电机侧上限 ÷ 减速比（否则会超出 VMAX）
static constexpr float kMaxOmegaRef = kMotorVmax / kJointRatio;   // = 3.0 rad/s（关节侧）

// 方向极性（实测后锁定：单轴点动确认正方向）
static constexpr int8_t kYawMotorSign   = +1;
static constexpr int8_t kPitchMotorSign = +1;
#if CONFIG_TRD_GIMBAL_FB_IMU
static constexpr int8_t kYawImuSign     = +1;
static constexpr int8_t kPitchImuSign   = +1;
#endif

// ==================== 周期计数与超时 ====================
static constexpr uint32_t kEnableResendTicks = 200;   // 失能时每 200ms 重发一次使能帧
static constexpr uint32_t kImuTimeoutMs      = 20;    // IMU 反馈超时（仅 IMU 模式）
static constexpr uint8_t  kTxPerTickMax      = 8;     // 每拍最多外发帧数
static constexpr uint32_t kDropLogPeriod     = 1000;  // 丢帧/失败上报周期

// ==================== 本线程私有状态 ====================
struct AxisState {
    bool     enabled_prev  = false;   // 上一拍电机是否使能（检测使能沿）
    bool     fb_valid_prev = false;   // 上一拍角度反馈是否有效（检测反馈恢复沿）
    bool     fault_logged  = false;   // 本次故障是否已上报（避免刷屏）
    uint32_t disable_ticks = 0;       // 失能期间的重发计数
};

static AxisState g_yaw_state   {};
static AxisState g_pitch_state {};

static float g_yaw_cmd      = 0.0f;   // 上板指令角（yaw 已折算到离虚拟角最近的等价值）
static float g_pitch_cmd    = 0.0f;   // 上板指令角（已限幅）
static float g_yaw_target   = 0.0f;   // 虚拟角：位置环的实际目标
static float g_pitch_target = 0.0f;
static uint32_t g_gimbal_version = 0; // pub_gimbal_to 版本号
static uint32_t g_drop = 0;           // 队列满丢帧计数
static uint32_t g_tx_fail = 0;        // CAN 发送失败计数

/**
 * @brief 一次控制周期用的角度反馈（物理系：逆时针 / 抬头为正）
 */
struct Feedback {
    float yaw   = 0.0f;
    float pitch = 0.0f;
    bool  valid = false;
};

// ==================== 上板指令 ====================

/**
 * @brief 读上板云台指令（角度）并更新指令角
 *
 * 状态通道 pub_from_head 由 thread/inter_rx 发布，定义为 ZBUS_OBSERVERS_EMPTY，
 * 因此用 zbus_chan_read 轮询（不占 subscriber）。
 *
 * 上板离线时不更新指令角 → 虚拟角冻结、位置环继续稳住当前姿态；
 * 云台不像底盘那样"离线即停"：突然失能会让 pitch 自由掉落，更危险。
 */
static void ReadCommand()
{
    static topic::from_head::Message msg {};
    static bool online_prev = false;

    if (zbus_chan_read(&pub_from_head, &msg, K_NO_WAIT) != 0) {
        return;                                    // 读忙：沿用上一轮指令
    }

    if (!msg.online) {
        if (online_prev) {
            LOG_INF("gimbal command OFFLINE -> hold angle");
        }
        online_prev = false;
        return;
    }

    if (!online_prev) {
        LOG_INF("gimbal command online");
        online_prev = true;
    }

    // yaw 指令可能被上板折叠到 ±π：折算成离当前虚拟角最近的等价值（走劣弧，不乱转圈）
    g_yaw_cmd   = g_yaw_target + NormalizeAngle(msg.comm.yaw_angle - g_yaw_target);
    g_pitch_cmd = std::clamp(msg.comm.pitch_angle, kPitchMin, kPitchMax);
}

/**
 * @brief 指令角 → 虚拟角（斜坡限速 + pitch 软限位）
 */
static void UpdateTarget()
{
    const float yaw_step   = std::clamp(g_yaw_cmd - g_yaw_target,
                                        -kMaxYawRate * kDt, kMaxYawRate * kDt);
    const float pitch_step = std::clamp(g_pitch_cmd - g_pitch_target,
                                        -kMaxPitchRate * kDt, kMaxPitchRate * kDt);

    g_yaw_target  += yaw_step;
    g_pitch_target = std::clamp(g_pitch_target + pitch_step, kPitchMin, kPitchMax);
}

// ==================== 角度反馈（二选一）====================

#if CONFIG_TRD_GIMBAL_FB_IMU
/**
 * @brief IMU 模式：取 imu_to 最新一帧的世界系角度
 *
 * 订阅者队列必须每拍排空：IMU 侧用 zbus_chan_pub(..., K_MSEC(1)) 发布，
 * 队列积压会让它返回 -EAGAIN（表现为 IMU 线程被拖慢）。
 * 超时（kImuTimeoutMs 内没有新帧）判为无效 → 上层不再驱动电机。
 */
static Feedback ReadImuFeedback()
{
    static topic::imu_to::Message msg {};
    static uint32_t last_ms = 0;

    const zbus_channel *chan = nullptr;
    bool got = false;

    while (zbus_sub_wait(&sub_imu_to, &chan, K_NO_WAIT) == 0 && chan != nullptr) {
        if (zbus_chan_read(chan, &msg, K_NO_WAIT) == 0) {
            got = true;
        }
    }

    if (got) {
        last_ms = k_uptime_get_32();
    }

    Feedback fb {};
    fb.yaw   = kYawImuSign   * msg.yaw_total;      // 累计偏航：可跨圈，不绕回
    fb.pitch = kPitchImuSign * msg.pitch;
    fb.valid = (last_ms != 0u) && ((k_uptime_get_32() - last_ms) <= kImuTimeoutMs);
    return fb;
}
#endif

/**
 * @brief 角度反馈入口（编译期二选一）
 *
 * 编码器模式用电机快照（电机侧角度 ÷ 减速比 = 关节角）；
 * 其有效性由每轴的使能状态（snap.err）单独判定，这里只给出角度。
 */
static Feedback ReadFeedback(const DmMotor::Snapshot &yaw_snap,
                             const DmMotor::Snapshot &pitch_snap)
{
    Feedback fb {};

#if CONFIG_TRD_GIMBAL_FB_IMU
    (void)yaw_snap;
    (void)pitch_snap;
    fb = ReadImuFeedback();
#else
    fb.yaw   = kYawMotorSign   * yaw_snap.radian   / kJointRatio;
    fb.pitch = kPitchMotorSign * pitch_snap.radian / kJointRatio;
    fb.valid = true;
#endif

    return fb;
}

// ==================== CAN 发送 ====================

static void QueueFrame(Axis &axis, bool cmd)
{
    topic::to_can_tx::Message msg {};
    msg.tx_id = axis.motor.GetTxId();

    if (cmd) {
        axis.motor.PackCmdFrame(msg.data, DmMotor::Cmd::Enable);
    } else {
        axis.motor.PackCtrlFrame(msg.data);
    }

    if (k_msgq_put(gimbal_tx, &msg, K_NO_WAIT) != 0) {
        if ((++g_drop % kDropLogPeriod) == 1u) {
            LOG_WRN("gimbal tx queue full: %u frames dropped",
                    static_cast<unsigned>(g_drop));
        }
    }
}

/**
 * @brief 外发本线程排在 user_can3_msgq 里的帧
 *
 * 本线程是 can3 总线所有者；队列同时对外开放，其它线程可以把帧投进来。
 */
static void FlushTx()
{
    topic::to_can_tx::Message msg {};
    uint8_t sent = 0;

    while (sent < kTxPerTickMax && k_msgq_get(gimbal_tx, &msg, K_NO_WAIT) == 0)
    {
        // 契约固定 8 字节载荷（不能用 sizeof(tx.data)：CAN FD 模式下是 64 字节）
        can_frame tx {};
        tx.id  = msg.tx_id;
        tx.dlc = 8;
        memcpy(tx.data, msg.data, sizeof(msg.data));

        if (!can3.Send(&tx)) {
            if ((++g_tx_fail % kDropLogPeriod) == 1u) {
                LOG_WRN("can3 send fail id=0x%02x (total %u)",
                        static_cast<unsigned>(msg.tx_id), static_cast<unsigned>(g_tx_fail));
            }
        }
        ++sent;
    }
}

// ==================== 单轴控制 ====================

/**
 * @brief 单轴：使能管理 + 双环 PID
 *
 * @param axis         轴实例
 * @param st           轴的运行状态（使能沿）
 * @param sign         电机极性：物理正方向 → 电机正方向
 * @param target_angle 虚拟角（使能沿对齐时会被改写）
 * @param fb_angle     实测角（物理系；IMU 或编码器）
 * @param fb_valid     实测角是否有效
 * @param snap         本拍电机快照（与反馈同帧，避免跨帧读数）
 */
static void ProcessAxis(Axis &axis, AxisState &st, int8_t sign,
                        float &target_angle, float fb_angle, bool fb_valid,
                        const DmMotor::Snapshot &snap)
{
    const bool enabled = (snap.err == DmErrorStatus::Enable);

    // ---- 对齐沿：失能→使能，或角度反馈由无效→有效 ----
    // 两种情况下虚拟角都可能和实际差很远，直接对齐 + 清积分，避免猛抽 / 撞限位
    const bool align_edge = (enabled && !st.enabled_prev) ||
                            (enabled && fb_valid && !st.fb_valid_prev);
    if (align_edge) {
        if (fb_valid) {
            target_angle = fb_angle;
            axis.position.SetIntegralError(0.0f);
            axis.omega.SetIntegralError(0.0f);
            LOG_INF("gimbal axis ready: target aligned to feedback %d mrad",
                    static_cast<int>(fb_angle * 1000.0f));
        } else {
            LOG_WRN("gimbal axis ready but feedback invalid -> hold");
        }
    }
    st.enabled_prev = enabled;
    st.fb_valid_prev = fb_valid;

    if (!enabled)
    {
        // 未使能（上电默认 / 故障后）：定期重发使能帧
        if ((st.disable_ticks++ % kEnableResendTicks) == 0u) {
            QueueFrame(axis, true);
        }

        // 非"失能"的其它状态（超压/欠压/过流/过温/通讯丢失/过载）只上报，
        // 不自动清故障；继续发命令帧是为了不让电机再叠加一个"通讯丢失"故障
        if (snap.err != DmErrorStatus::Disable) {
            if (!st.fault_logged) {
                st.fault_logged = true;
                LOG_WRN("gimbal axis fault: err=0x%x", static_cast<unsigned>(snap.err));
            }
        } else {
            st.fault_logged = false;
        }
        return;
    }

    st.disable_ticks = 0;               // 使能后重新计时，下次失能立即补发使能帧
    st.fault_logged  = false;

    if (!fb_valid)
    {
        // 反馈无效：给 0 力矩但保持发帧（不让电机带着陈旧角度闭环乱冲）
        axis.motor.SetTargetTorque(0.0f);
        QueueFrame(axis, false);
        return;
    }

    // 外环：角度 → 角速度给定（物理系）
    axis.position.SetTarget(target_angle);
    axis.position.SetNow(fb_angle);
    const float omega_ref = std::clamp(axis.position.Calc(), -kMaxOmegaRef, kMaxOmegaRef);

    // 内环：角速度 → 力矩（电机系：给定按极性折算后与电机反馈比较）
    axis.omega.SetTarget(static_cast<float>(sign) * omega_ref);
    axis.omega.SetNow(snap.omega);
    const float torque = axis.omega.Calc();

    axis.motor.SetTargetTorque(torque);
    QueueFrame(axis, false);
}

// ==================== 对外发布 ====================

/**
 * @brief 发布云台角度：底盘要的"相对角" + 上板要的"实测角"
 *
 * @param fb            本拍反馈（模式相关：IMU 或编码器）
 * @param yaw_relative  云台相对底盘 yaw（始终来自编码器机械角，与反馈模式无关）
 * @param online        yaw 电机是否在使能态
 */
static void PublishAngles(const Feedback &fb, float yaw_relative, bool online)
{
    topic::gimbal_to::Message g {};
    g.version      = ++g_gimbal_version;
    g.timestamp_ms = k_uptime_get_32();
    g.yaw_rad      = NormalizeAngle(yaw_relative);
    g.online       = online;
    (void)zbus_chan_pub(&pub_gimbal_to, &g, K_NO_WAIT);

    inter_cmd::GimbalState st {};
    st.yaw   = fb.yaw;
    st.pitch = fb.pitch;
    inter_cmd::PostFrame(inter_cmd::FrameType::StateGimbal, st);
}

/**
 * @brief 云台控制主循环（固定 1ms 周期，tick 计时）
 */
static void Task(void*, void*, void*)
{
    for (;;)
    {
        const int64_t tick_start = k_uptime_ticks();

        ReadCommand();
        UpdateTarget();

        // 每轴一次快照：位置/速度/使能状态都取自同一帧
        const auto yaw_snap   = yaw_.motor.ReadAll();
        const auto pitch_snap = pitch_.motor.ReadAll();
        const Feedback fb = ReadFeedback(yaw_snap, pitch_snap);

        ProcessAxis(yaw_,   g_yaw_state,   kYawMotorSign,
                    g_yaw_target,   fb.yaw,   fb.valid, yaw_snap);
        ProcessAxis(pitch_, g_pitch_state, kPitchMotorSign,
                    g_pitch_target, fb.pitch, fb.valid, pitch_snap);

        FlushTx();

        const float yaw_relative =
            kYawMotorSign * yaw_snap.radian / kJointRatio;
        PublishAngles(fb, yaw_relative, yaw_snap.err == DmErrorStatus::Enable);

        const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
        const int64_t used   = k_uptime_ticks() - tick_start;
        if (used < period) {
            k_sleep(K_TICKS(period - used));
        }
    }
}

/**
 * @brief 单轴初始化：电机（DM MIT，kp/kd=0）+ 双环 PID
 */
static void InitAxis(Axis &axis, uint16_t can_id, uint16_t master_id,
                     const alg::pid::Pid::Config &pos_cfg,
                     const alg::pid::Pid::Config &omega_cfg)
{
    DmMotor::Config cfg {};
    cfg.ctrl_met      = ControlMethon::Mit;
    cfg.can_id        = can_id;
    cfg.master_id     = master_id;
    cfg.gearbox_ratio = kJointRatio;
    cfg.wheel_r       = 1.0f;               // 云台不做线速度换算
    cfg.kp            = 0.0f;               // 电机内置位置环关闭：闭环在板端
    cfg.kd            = 0.0f;
    cfg.PMAX          = 12.5f;
    cfg.VMAX          = kMotorVmax;
    cfg.TMAX          = kMotorTmax;         // 协议标定，不随内环限幅改动

    axis.motor.Init(cfg);
    axis.position.Init(pos_cfg);
    axis.omega.Init(omega_cfg);
}

bool thread_init()
{
    // ---- user-can3：本线程是总线所有者 ----
    const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can3));
    if (!device_is_ready(dev)) {
        LOG_ERR("user_can3 not ready");
        return false;
    }

    const can_filter filter { .id = 0, .mask = 0, .flags = 0 };   // 全收，按 ID 分发
    if (!can3.Init(dev, filter)) {
        LOG_ERR("user_can3 init fail");
        return false;
    }
    can3.SetRxCallback(user_can3_rx_callback);

    // ---- 双环 PID（占位增益，上车前按实车整定）----
    alg::pid::Pid::Config pos_cfg {};
    pos_cfg.kp      = 4.0f;                    // rad/s per rad
    pos_cfg.ki      = 0.0f;
    pos_cfg.kd      = 0.0f;
    pos_cfg.iOutMax = kMaxOmegaRef;
    pos_cfg.outMax  = kMaxOmegaRef;            // 输出的角速度给定上限
    pos_cfg.dt      = kDt;
    pos_cfg.dFirst  = alg::pid::DFirst::Enable;

    alg::pid::Pid::Config omega_cfg {};
    omega_cfg.kp      = 0.3f;                  // N·m per rad/s
    omega_cfg.ki      = 0.0f;
    omega_cfg.kd      = 0.0f;
    omega_cfg.iOutMax = kTorqueLimit;          // 积分限幅与输出限幅一致，防止饱和蓄积分
    omega_cfg.outMax  = kTorqueLimit;          // 输出即力矩（电机侧），限幅保护机构
    omega_cfg.dt      = kDt;
    omega_cfg.dFirst  = alg::pid::DFirst::Enable;

    InitAxis(yaw_,   kYawCanId,   kYawMasterId,   pos_cfg, omega_cfg);
    InitAxis(pitch_, kPitchCanId, kPitchMasterId, pos_cfg, omega_cfg);

    LOG_INF("gimbal ready: yaw 0x%02x/0x%02x pitch 0x%02x/0x%02x, %s feedback, %u ms",
            static_cast<unsigned>(kYawCanId), static_cast<unsigned>(kYawMasterId),
            static_cast<unsigned>(kPitchCanId), static_cast<unsigned>(kPitchMasterId),
            IS_ENABLED(CONFIG_TRD_GIMBAL_FB_IMU) ? "IMU" : "encoder",
            static_cast<unsigned>(kPeriodMs));
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

// CAN 收帧注册：反馈帧按 master_id 分发，回调内再校验 payload 里的 can_id
CAN_RX_HANDLER(GIMBAL_RX, kYawMasterId,
               [](uint8_t *data) { yaw_.motor.CanCpltRxCallback(data); }, yaw);
CAN_RX_HANDLER(GIMBAL_RX, kPitchMasterId,
               [](uint8_t *data) { pitch_.motor.CanCpltRxCallback(data); }, pitch);

REGISTER_INIT  (thread_init,  MidInit,   Mid, "gimbal_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "gimbal_start");

} // namespace thread::gimbal
