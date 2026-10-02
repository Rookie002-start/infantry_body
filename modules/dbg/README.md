# modules/dbg —— 公共调参终端（给双环 PID 在线改参数）

## 这个目录是什么

RTT 串口终端里敲一行命令，就能改**任何登记过的双环 PID** 的 kp/ki，不用重新烧录：

```
pid                                 列出所有登记过的轴
pid gimbal yaw                      看这条轴：内外环 kp/ki + 实时值
pid gimbal yaw out 6.0 0.1          改外环 kp/ki（下一拍生效）
pid gimbal yaw in  0.8 0.05         改内环 kp/ki
pid gimbal yaw mon 200              每 200ms 打一行实时值（0=关）
```

登记与否由业务侧决定：底盘（`thread/chassis`）是单速度环、不登记（整定看 VOFA+），
下面都以云台轴为例。

由根 `CMakeLists.txt` 的 `ZEPHYR_EXTRA_MODULES` 引入，是个普通 Zephyr 模块
（`zephyr/module.yml` + 自带 CMakeLists/Kconfig），只依赖 Zephyr 和框架的
`alg::pid::Pid`，**不依赖工程里的任何业务线程**。

## 依赖方向（低耦合的关键）

```
业务（thread/gimbal、...） --只 include dbg.hpp--> 本模块
本模块 -----------------------------------------> 不认识任何业务类型
```

* 业务线程之间、业务与 dbg 实现之间**互不 include**，只共享 `dbg.hpp` 这一个头；
* `CONFIG_DBG=n` 时 `dbg.hpp` 里的函数全是空操作（`return false` / 什么都不做），
  业务代码**不用写 #if**：线程照常独立、高内聚地跑，PID 初值取业务自己的常量；
* 想加自己的命令/参数只改业务侧（登记更多 PID 即可），本模块不用动。

## 业务侧怎么用（两行）

```cpp
#include "dbg.hpp"                    // include 路径由本模块提供

// 初始化时（PID Init 之后）把一条轴的两个环登记出去：
dbg::RegisterPid({ "gimbal", "yaw",
                   &yaw_.position,      // 外环（位置/速度环）
                   &yaw_.omega,         // 内环（速度/力矩环）
                   YawLive, &yaw_index });   // 可选：实时值回调（给 pid/mon 用）
```

实时值回调长这样（`ctx` 原样传回）：

```cpp
static bool YawLive(void *ctx, dbg::PidLive &out)
{
    out.ref_outer  = yawRef_;    // 外环给定
    out.meas_outer = yawMeas_;   // 外环实测
    out.ref_inner  = yawRef_;    // 内环给定（通常 = 外环输出）
    out.meas_inner = yawMeas_;   // 内环实测
    return true;
}
```

## 输入口

| 输入 | 配置 | 说明 |
|------|------|------|
| RTT 下行缓冲（默认） | `CONFIG_DEBUG_RTT=y` + `CONFIG_DBG=y` | `tools/rtt.sh` 起服务后 `nc localhost 9090`，和日志同一个窗口 |
| 控制台串口 RX（可选） | `CONFIG_DBG_UART_RX=y` | 在 `zephyr,console` 上挂 RX 中断收命令。默认关：console 是日志口，多挂一路 RX 中断台架上有噪声风险 |

## 开关与代价

* `CONFIG_DBG=y`：起一个优先级 5 的低优先级线程（3KB 栈），每 10ms 看一次输入口；
  控制线程（优先级 3）不受影响。
* 掉电/复位后参数回业务初值 —— 整定完把数字抄回业务文件的常量再烧录，这才算"写进固件"。
