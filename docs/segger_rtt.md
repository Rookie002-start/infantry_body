# SEGGER RTT 调试日志通道（下板 infantry_body）

RTT 用调试探针（SWD/JTAG）直接读写目标板 RAM 里的环形缓冲，把日志/变量实时送到电脑：
不占 UART、不阻塞控制线程、速度远高于串口。

本工程的部署方式与上板 `infantry_head` **完全一致**（`modules/segger/` 逐字节相同），
用的是 **OpenOCD 自带的 RTT 实现**，所以 CMSIS-DAP / ST-Link / J-Link 都能用，
不需要安装 SEGGER 的 J-Link 软件。

## 1. 一分钟上手

```bash
west build -d build
./flash.sh                # 或 ./flash.sh jlink

# 另开一个终端：起 RTT 服务（别关）
tools/rtt.sh

# 再开一个终端看输出
nc localhost 9090
#    或者用自带查看器（Ctrl-C 退出）：
tools/rtt_view.py
```

一步到位：`tools/rtt.sh -v`。

VS Code：`.vscode/launch.json` 里 6 个 cortex-debug 配置（CMSIS-DAP / ST-Link / J-Link
的 Debug 与 Attach）都已打开 RTT，`"address": "auto"` 会自动从 ELF 的 `_SEGGER_RTT`
符号取控制块地址，启动调试后 RTT 输出出现在 **RTT** 终端里。

> 探针同一时刻只能被一个进程占用：VS Code 调试会话占着的时候 `tools/rtt.sh` 连不上，反之亦然。

## 2. 代码里怎么打日志 / 打变量

```c
/* ① printk：Zephyr 默认 CONFIG_LOG_PRINTK=y，会被日志子系统接管 → 同时进 UART 和 RTT */
printk("vx=%d vy=%d\n", (int)(vx * 100), (int)(vy * 100));

/* ② LOG_*：推荐，支持浮点（工程已开 CONFIG_CBPRINTF_FP_SUPPORT） */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(my_mod, LOG_LEVEL_INF);
LOG_INF("yaw=%.3f pitch=%.3f", (double)yaw, (double)pitch);

/* ③ SEGGER_RTT_printf：最轻量，直接写 RTT 通道 0（注意：不支持 %f） */
#include <SEGGER_RTT.h>
SEGGER_RTT_printf(0, "wheel=%d %d %d %d\r\n", w0, w1, w2, w3);
```

## 3. 配置项（`prj.conf`）

| 配置 | 当前值 | 含义 |
| --- | --- | --- |
| `CONFIG_DEBUG_RTT` | `y` | 总开关（`Kconfig` 里定义，打开 `USE_SEGGER_RTT` + `LOG_BACKEND_RTT`） |
| `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP` | `4096` | 目标→主机缓冲；越大越不容易丢日志，代价是 RAM |
| `CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN` | `32` | 主机→目标缓冲（暂未用到输入） |
| `CONFIG_SEGGER_RTT_PRINTF_BUFFER_SIZE` | `128` | `SEGGER_RTT_printf()` 攒包缓冲 |
| `CONFIG_SEGGER_RTT_MODE_NO_BLOCK_SKIP` | `y` | 通道 0 非阻塞：主机没连 / 缓冲满就丢，**不阻塞线程** |
| `CONFIG_LOG_BACKEND_RTT_MODE_OVERWRITE` | `y` | 日志缓冲满时覆盖最旧数据（`BLOCK` 会阻塞，不建议） |
| `CONFIG_SEGGER_RTT_SECTION_CUSTOM` | `y`（默认） | 控制块/缓冲放 `.rtt_buff_data` 段，主机好找 |

内存代价：RTT 缓冲段约 **4.3 KB RAM**（4 KB 上行缓冲 + 控制块 + 下行缓冲），
`west build` 结尾的 `Memory region Used Size` 里能看到；不要时把 `CONFIG_DEBUG_RTT=n` 即可。

## 4. 只让日志走 RTT（可选）

现在是 "UART + RTT 双份"。串口（本板 console 是 uart7 @921600）带宽不够时：

1. `Kconfig` 的 `config DEBUG_LOG` 里注释掉 `select LOG_BACKEND_UART`；
2. （可选）`CONFIG_UART_CONSOLE=n` + `CONFIG_RTT_CONSOLE=y`，让 `printk` 直接写 RTT。

> 不要同时开 `UART_CONSOLE` 和 `RTT_CONSOLE`：两者都往 `printk` 挂钩子，谁生效取决于初始化顺序。

## 5. 常见问题

**Q: `tools/rtt.sh` 报 "在 ELF 里找不到 _SEGGER_RTT 符号"**
固件没开 `CONFIG_DEBUG_RTT` 或没重新编译。确认 `prj.conf` 里 `CONFIG_DEBUG_RTT=y` 后重新 `west build`。

**Q: OpenOCD 报 `No control block found`**
控制块是**固件运行时**初始化的：板子要已经烧录且跑起来；被 halt 住时用 `tools/rtt.sh -r` 复位后再连。

**Q: CMSIS-DAP 权限 / 设备被占**
本板用 `boards/st/board_dm_mc02/openocd-cmsis-dap-wrapper.sh`（与 `./flash.sh` 同一份），
里面会用 `sudo` 解绑 `cdc_acm`，和平时烧录一样需要 sudo 权限。

**Q: 日志偶尔缺行**
主机没跟上的那一瞬间缓冲满了。调大 `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP`，或依赖
`CONFIG_LOG_BACKEND_RTT_MODE_OVERWRITE`（默认）保证拿到最新数据。

## 6. 关掉 / 移除

* 临时：`prj.conf` 里 `CONFIG_DEBUG_RTT=n`；
* 彻底移除：删掉 `modules/segger/`、去掉根 `CMakeLists.txt` 里 `ZEPHYR_EXTRA_MODULES`
  的那一行、`Kconfig` 的 `DEBUG_RTT` 段、`prj.conf` 的 RTT 段。

> 如果以后 `west update` 拉到官方 `segger` 模块，注意不要与本目录同名共存：
> 删掉 `modules/segger/` 和那行 `ZEPHYR_EXTRA_MODULES` 即可，`prj.conf`/`Kconfig` 不用改。
