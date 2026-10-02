# modules/vofa+ —— VOFA+ JustFloat 输出

## 这个目录是什么

把板子上的 float 通道发到 [VOFA+](https://www.vofa.plus/) 上位机看波形，
协议用 VOFA+ 的 **JustFloat**（不依赖任何上位机 SDK，纯字节流）。

不是从别处拉来的第三方模块，是本工程的本地组件：源码（`vofa.cpp` / `vofa.hpp`）
全在这里，根 `CMakeLists.txt` 用 `ZEPHYR_EXTRA_MODULES` 引入，
`zephyr/module.yml` 声明它是个普通 Zephyr 模块（自带 CMakeLists + Kconfig）。

## 协议

```
一帧 = 通道0(float, 小端 4B) | 通道1 | ... | 通道 n-1 | 帧尾
帧尾 = 00 00 80 7F        (0x7F800000，VOFA+ 用它切帧)
```

上位机设置：

* 数据格式：**JustFloat**
* 通道数：按实际发送个数设（多余通道显示为 0）
* 串口：与设备树 `vofa-uart` 节点一致（dm_mc02 = uart7，**921600** 8N1）

## 用法

```cpp
#include "vofa.hpp"

vofa::Send({ g_vx, g_vy, g_vw });   // 3 通道一帧 → 通道 0/1/2
vofa::Send(omega);                  // 单通道一帧
vofa::Send(arr, 4);                 // 数组：arr[0..3] → 通道 0..3
```

初始化不在业务线程里做：`CONFIG_TRD_VOFA`（`thread/vofa/trd_vofa.cpp`）是
**只做初始化、不建线程**的所有者组件，启动阶段调一次 `vofa::Init()`，
业务线程只管 `Send()`。这样串口中断回调和环形缓冲只被初始化一次，
不会出现"哪个线程先调谁就顺手初始化"的多线程重复初始化。
丢帧次数用 `vofa::DropCount()` 查。

## 为什么不用阻塞发送

业务线程是 1ms 控制环，`uart_poll_out()`（逐字节等 FIFO）和 `uart_tx()`
（阻塞到整包进 FIFO）都会按波特率把控制环拖住。本组件自己实现
**环形缓冲 + TX 中断**：

```
业务线程 ──memcpy 整帧──→ 环形缓冲(1KB) ──TX 中断──→ UART FIFO ──→ 总线
```

组帧开销只有一次几十字节的 `memcpy`；缓冲满（上位机没在收 / 波特率不够）时
**整帧丢弃**并计数，实时性优先于波形完整性。

## 换板卡 / 改串口

只改设备树别名，不动代码：

```dts
/ {
	aliases {
		vofa-uart = &uart7;    /* 换成你要用的串口 */
	};
};

&uart7 {
	current-speed = <921600>;
	status = "okay";
};
```

板卡没有这个别名时组件照常编译链接，`Init()` 会在日志里报一行警告、
`Send()` 变成空操作 —— 业务代码不用为板卡差异写 `#if`。

## 开关与初始化归属

| 符号 | 作用 |
|------|------|
| `CONFIG_TRD_VOFA` | 所有者组件（thread/vofa/trd_vofa.cpp）：只 REGISTER_INIT，不建线程 |
| `CONFIG_VOFA` | 组件本体（modules/vofa+）：协议、环形缓冲、TX 中断，由 TRD_VOFA select |

`prj.conf` 里写 `CONFIG_TRD_VOFA=y` 即可（它会 select `VOFA`，进而 select
`UART_INTERRUPT_DRIVEN`）。初始化失败只打一行警告不停机：`TRD_VOFA` 的
REGISTER_INIT 等级是 Mid（框架只对 High 等级停机），所以板卡没有
`vofa-uart` 别名时整车照常启动，`Send()` 变成空操作。

关掉 `CONFIG_VOFA` 后源码不参与编译，业务侧的 `vofa::` 调用需要一并去掉。
