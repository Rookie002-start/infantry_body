/**
 * @file dbg.cpp
 * @author qingyu
 * @brief 公共调参命令行线程（RTT 收命令，改任何登记过的双环 PID 的 kp/ki）
 * @version 1.0
 * @date 2026-09-27
 *
 * 本文件**不认识任何业务类型**（不 include 底盘/云台的头文件）：业务通过 dbg.hpp 的
 * RegisterPid() 把可调 PID 登记进来，这里只负责收行、切词、执行 `pid ...`、把回包
 * 打回终端。
 *
 * ## 通信链路
 *
 *   主机 nc localhost 9090 ──> OpenOCD RTT server ──> 目标 RAM 的 RTT down-buffer
 *        ↑                                                    │ 每 10ms SEGGER_RTT_Read()
 *        └──── RTT up-buffer ←── LOG_INF 回包 ←── 本线程解析/执行
 *
 *   可选第二条输入口：控制台串口 RX（CONFIG_DBG_UART_RX=y，默认关 —— console 是日志口，
 *   往它上面挂 RX 中断等于和日志子系统共用一路串口，RTT 那条路已经够用）。
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Modules/Dbg"

#include "dbg.hpp"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#if defined(CONFIG_USE_SEGGER_RTT)
#include <SEGGER_RTT.h>
#endif

#include <cstdarg>
#include <cstdlib>
#include <cstring>

LOG_MODULE_REGISTER(dbg_shell, LOG_LEVEL_INF);

namespace dbg
{
    // ==================== 业务登记表（dbg 侧只认这张表）====================
    namespace
    {
        PidKnobs g_pids[kMaxPids];
        uint8_t  g_pid_cnt = 0;
    }

    bool RegisterPid(const PidKnobs &knobs)
    {
        if (knobs.group == nullptr || knobs.axis == nullptr || g_pid_cnt >= kMaxPids) {
            return false;
        }
        for (uint8_t i = 0; i < g_pid_cnt; ++i) {          // 同一个 group/axis 只登记一次
            if (std::strcmp(g_pids[i].group, knobs.group) == 0 &&
                std::strcmp(g_pids[i].axis,  knobs.axis)  == 0) {
                return false;
            }
        }
        g_pids[g_pid_cnt++] = knobs;                        // 复制一份，调用方传临时量也安全
        return true;
    }

    void Reply(const char *fmt, ...)
    {
        char    buf[160];
        va_list ap;

        va_start(ap, fmt);
        vsnprintk(buf, sizeof(buf), fmt, ap);
        va_end(ap);

        LOG_INF("%s", buf);
    }

    bool ParseFloat(const char *tok, const char *what, float &out)
    {
        if (tok == nullptr) {
            Reply("%s 缺参数", what);
            return false;
        }
        char *end = nullptr;
        const float v = strtof(tok, &end);
        if (end == tok || *end != '\0') {
            Reply("%s 不是合法数字: '%s'", what, tok);
            return false;
        }
        out = v;
        return true;
    }

    bool ParseInt(const char *tok, const char *what, long min, long max, long &out)
    {
        if (tok == nullptr) {
            Reply("%s 缺参数（%ld~%ld）", what, min, max);
            return false;
        }
        char *end = nullptr;
        const long v = strtol(tok, &end, 10);
        if (end == tok || *end != '\0' || v < min || v > max) {
            Reply("%s 只接受 %ld~%ld，收到 '%s'", what, min, max, tok);
            return false;
        }
        out = v;
        return true;
    }

    // ==================== 命令行线程（内部实现）====================
    namespace
    {
        // 栈给大一点：这条路径上有 vsnprintk(picolibc 浮点格式化) + LOG_INF 就地格式化，
        // 都是本工程最吃栈的调用；而且任何线程栈溢出都会把整机 halt，调试线程宁可多占 1KB。
        constexpr size_t kStackSize = 3072;
        K_THREAD_STACK_DEFINE(stack_, kStackSize);
        struct k_thread       thread_ {};

        // 优先级：比控制线程（3）低，保证调参永远不抢控制环的时间
        constexpr int kPriority = 5;

        constexpr uint32_t kPollMs     = 10;    // 输入口轮询间隔
        constexpr size_t   kLineMax    = 96;    // 一行最长字符（超了截断并提示）
        constexpr uint8_t  kArgMax     = 8;     // 一行最多切出几个 token
        constexpr size_t   kUartRxSize = 128;   // 控制台串口 RX 环形缓冲（可选输入口）

        char     line_[kLineMax];
        uint32_t line_len_      = 0;
        bool     line_overflow_ = false;

#if defined(CONFIG_DBG_UART_RX)
        uint8_t         uart_rx_mem_[kUartRxSize];
        struct ring_buf uart_rx_ring_ {};
#endif
        const device   *console_ = nullptr;

        // 实时值监视（`pid <g> <a> mon <ms>`）
        const PidKnobs *g_mon      = nullptr;
        uint32_t        g_mon_ms   = 0;
        int64_t         g_mon_next = 0;

        /**
         * @brief 极简分词：按空格/Tab 切开，原地写 '\0'
         *
         * 没用 strtok_r（POSIX 扩展，换 libc 容易踩坑），这点逻辑自己写更省心。
         */
        char *NextTok(char **cursor)
        {
            char *p = *cursor;
            if (p == nullptr) {
                return nullptr;
            }
            while (*p == ' ' || *p == '\t') {
                ++p;
            }
            if (*p == '\0') {
                *cursor = nullptr;
                return nullptr;
            }

            char *tok = p;
            while (*p != '\0' && *p != ' ' && *p != '\t') {
                ++p;
            }
            if (*p != '\0') {
                *p      = '\0';
                *cursor = p + 1;
            } else {
                *cursor = nullptr;
            }
            return tok;
        }

        // ---------- 通用命令：pid ----------
        const PidKnobs *FindPid(const char *group, const char *axis)
        {
            for (uint8_t i = 0; i < g_pid_cnt; ++i) {
                if (strcmp(g_pids[i].group, group) != 0) {
                    continue;
                }
                if (axis != nullptr && strcmp(g_pids[i].axis, axis) != 0) {
                    continue;
                }
                return &g_pids[i];
            }
            return nullptr;
        }

        void PrintPid(const PidKnobs &k)
        {
            const alg::pid::Pid *o = k.outer;
            const alg::pid::Pid *n = k.inner;

            Reply("%-8s %-7s  outer kp=%.4f ki=%.4f | inner kp=%.4f ki=%.4f",
                  k.group, k.axis,
                  o ? static_cast<double>(o->GetKp()) : 0.0,
                  o ? static_cast<double>(o->GetKi()) : 0.0,
                  n ? static_cast<double>(n->GetKp()) : 0.0,
                  n ? static_cast<double>(n->GetKi()) : 0.0);

            PidLive v;
            if (k.live != nullptr && k.live(k.ctx, v)) {
                Reply("         live: ref=%8.3f meas=%8.3f | tau_ref=%7.3f tau=%7.3f",
                      static_cast<double>(v.ref_outer), static_cast<double>(v.meas_outer),
                      static_cast<double>(v.ref_inner), static_cast<double>(v.meas_inner));
            }
        }

        void CmdPid(int argc, char **argv)
        {
            if (argc == 0) {                                // pid：列出全部
                if (g_pid_cnt == 0) {
                    Reply("(没有登记任何 PID：业务侧是否调了 dbg::RegisterPid？)");
                    return;
                }
                for (uint8_t i = 0; i < g_pid_cnt; ++i) {
                    PrintPid(g_pids[i]);
                }
                return;
            }

            if (argc == 1) {                                // pid <group>
                if (FindPid(argv[0], nullptr) == nullptr) {
                    Reply("没有登记过组 '%s'（敲 pid 看列表）", argv[0]);
                    return;
                }
                for (uint8_t i = 0; i < g_pid_cnt; ++i) {
                    if (strcmp(g_pids[i].group, argv[0]) == 0) {
                        PrintPid(g_pids[i]);
                    }
                }
                return;
            }

            const PidKnobs *k = FindPid(argv[0], argv[1]);
            if (k == nullptr) {
                Reply("没有登记 %s/%s（敲 pid 看列表）", argv[0], argv[1]);
                return;
            }
            if (argc == 2) {                                // pid <group> <axis>：看这条轴
                PrintPid(*k);
                return;
            }

            if (strcmp(argv[2], "mon") == 0) {               // pid <g> <a> mon <ms>
                long ms = 0;
                if (!ParseInt(argc > 3 ? argv[3] : nullptr, "mon 周期", 0, 10000, ms)) {
                    return;
                }
                if (ms == 0 || k->live == nullptr) {
                    const bool had = (g_mon != nullptr);
                    g_mon    = nullptr;
                    g_mon_ms = 0;
                    Reply(had ? "实时值监视已关" : "这条轴没有实时值回调，不能 mon");
                } else {
                    g_mon      = k;
                    g_mon_ms   = static_cast<uint32_t>(ms);
                    g_mon_next = k_uptime_get();
                    Reply("监视 %s/%s 实时值 = %ld ms（0=关）", k->group, k->axis, ms);
                }
                return;
            }

            if (strcmp(argv[2], "in") == 0 || strcmp(argv[2], "out") == 0) {
                const bool outer = (argv[2][0] == 'o');
                alg::pid::Pid *pid = outer ? k->outer : k->inner;
                if (pid == nullptr) {
                    Reply("%s/%s 没有%s环", k->group, k->axis, outer ? "外" : "内");
                    return;
                }
                float kp = 0.0f;
                float ki = 0.0f;
                if (!ParseFloat(argc > 3 ? argv[3] : nullptr, "kp", kp)) return;
                if (!ParseFloat(argc > 4 ? argv[4] : nullptr, "ki", ki)) return;

                pid->SetKp(kp);                             // 单字写：控制线程下一拍用新值
                pid->SetKi(ki);
                Reply("%s/%s %s环 = kp %.4f / ki %.4f（下一拍生效，掉电回初值）",
                      k->group, k->axis, outer ? "外" : "内",
                      static_cast<double>(kp), static_cast<double>(ki));
                return;
            }

            Reply("用法：pid <group> <axis> in|out <kp> <ki>");
            Reply("      pid <group> <axis> mon <ms>（0=关）");
        }

        void Help()
        {
            Reply("help                              这条");
            Reply("pid                               列出已登记的 PID（group/axis/内外环 kp ki）");
            Reply("pid <g> [<a>]                     看某组 / 某条轴（含实时值）");
            Reply("pid <g> <a> out <kp> <ki>         改外环增益");
            Reply("pid <g> <a> in  <kp> <ki>         改内环增益");
            Reply("pid <g> <a> mon <ms>              周期打印该轴实时值（0=关）");
        }

        void RunLine(char *line)
        {
            char *argv[kArgMax];
            int   argc = 0;
            char *cur  = line;
            char *tok;

            while (argc < kArgMax && (tok = NextTok(&cur)) != nullptr) {
                argv[argc++] = tok;
            }
            if (argc == 0) {
                return;                                     // 空行
            }

            if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
                Help();
                return;
            }
            if (strcmp(argv[0], "pid") == 0) {
                CmdPid(argc - 1, argv + 1);
                return;
            }

            Reply("未知命令 '%s'，敲 help 看用法", argv[0]);
        }

        /// 逐字符收行：回车/换行执行，退格删除，其它控制字符忽略
        void Feed(char c)
        {
            if (c == '\r' || c == '\n') {
                line_[line_len_] = '\0';
                if (line_len_ > 0) {
                    RunLine(line_);
                }
                line_len_      = 0;
                line_overflow_ = false;
                return;
            }
            if (c == '\b' || c == 0x7F) {                   // 退格
                if (line_len_ > 0) {
                    --line_len_;
                }
                return;
            }
            if (c < 0x20) {
                return;
            }
            if (line_len_ >= kLineMax - 1) {
                if (!line_overflow_) {
                    line_overflow_ = true;
                    Reply("命令太长（> %u 字符），已截断", static_cast<unsigned>(kLineMax - 1));
                }
                return;
            }
            line_[line_len_++] = c;
        }

        // ---------- 输入口 ①：RTT 下行缓冲（推荐）----------
        void PollRtt()
        {
#if defined(CONFIG_USE_SEGGER_RTT)
            char buf[32];
            unsigned n;

            while ((n = SEGGER_RTT_Read(0, buf, sizeof(buf))) > 0) {  // 非阻塞：主机没写就返回 0
                for (unsigned i = 0; i < n; ++i) {
                    Feed(buf[i]);
                }
            }
#endif
        }

        // ---------- 输入口 ②：控制台串口 RX（可选）----------
#if defined(CONFIG_DBG_UART_RX)
        void UartRxIsr(const device *dev, void *)
        {
            uint8_t buf[16];

            uart_irq_update(dev);
            while (uart_irq_rx_ready(dev))
            {
                const int n = uart_fifo_read(dev, buf, sizeof(buf));
                if (n <= 0) {
                    break;
                }
                // 线程没来得及取就丢（调参输入，丢几个字符无所谓），绝不能在这里等
                (void)ring_buf_put(&uart_rx_ring_, buf, static_cast<uint32_t>(n));
            }
        }

        void PollUart()
        {
            uint8_t  buf[32];
            uint32_t n;

            while ((n = ring_buf_get(&uart_rx_ring_, buf, sizeof(buf))) > 0) {
                for (uint32_t i = 0; i < n; ++i) {
                    Feed(static_cast<char>(buf[i]));
                }
            }
        }
#endif

        // ---------- 实时值监视 ----------
        void PollMonitor()
        {
            if (g_mon == nullptr || g_mon_ms == 0) {
                return;
            }
            const int64_t now = k_uptime_get();
            if (now < g_mon_next) {
                return;
            }
            g_mon_next = now + g_mon_ms;

            PidLive v;
            if (g_mon->live != nullptr && g_mon->live(g_mon->ctx, v)) {
                Reply("%s/%s  ref=%8.3f meas=%8.3f | tau_ref=%7.3f tau=%7.3f",
                      g_mon->group, g_mon->axis,
                      static_cast<double>(v.ref_outer), static_cast<double>(v.meas_outer),
                      static_cast<double>(v.ref_inner), static_cast<double>(v.meas_inner));
            }
        }

        // ---------- 线程 ----------
        void Task(void *, void *, void *)
        {
            for (;;)
            {
                PollRtt();
#if defined(CONFIG_DBG_UART_RX)
                PollUart();
#endif
                PollMonitor();
                k_msleep(kPollMs);
            }
        }

        /// 模块自己的启动：建线程 + （可选）挂控制台串口 RX。
        /// 用 Zephyr 的 SYS_INIT 而不是项目的 init 表：本模块不依赖项目的任何启动框架。
        int init()
        {
#if defined(CONFIG_DBG_UART_RX)
            ring_buf_init(&uart_rx_ring_, sizeof(uart_rx_mem_), uart_rx_mem_);

            // 控制台串口收命令：只占 RX，日志（TX）照走 poll_out，互不影响
            const device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
            if (device_is_ready(dev) &&
                uart_irq_callback_user_data_set(dev, UartRxIsr, nullptr) == 0)
            {
                uart_irq_rx_enable(dev);
                console_ = dev;
            }
#endif

            k_thread_create(&thread_, stack_, K_THREAD_STACK_SIZEOF(stack_),
                            Task, nullptr, nullptr, nullptr,
                            kPriority, 0, K_NO_WAIT);

            LOG_INF("dbg shell ready: RTT%s / console=%s —— 敲 help 看命令",
                    IS_ENABLED(CONFIG_USE_SEGGER_RTT) ? " channel0" : "(未开)",
                    IS_ENABLED(CONFIG_DBG_UART_RX)
                        ? ((console_ != nullptr) ? console_->name : "(不可用)")
                        : "(已关, CONFIG_DBG_UART_RX=n)");
            return 0;
        }

        SYS_INIT(init, POST_KERNEL, 90);
    }

} // namespace dbg
