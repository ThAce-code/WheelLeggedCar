# CYT2BL3 开源库空工程 - 项目指南

> 本文件面向 AI 编码助手。阅读本文件前，默认读者对该项目一无所知。

## 1. 项目概述

本项目是 **成都逐飞科技（SEEKFREE）** 为 **Infineon CYT2BL3（Traveo II 系列）** 微控制器提供的第三方开源库空工程示例，用于智能车（smartcar）等嵌入式竞赛/应用开发。

- **项目名称**：CYT2BL3 Opensource Library 空工程
- **当前版本**：V3.4.3（版本历史见 `libraries/doc/version.txt`）
- **目标芯片**：Infineon CYT2BL3，ARM Cortex-M4 内核
- **开发环境**：IAR Embedded Workbench for ARM 9.40.1
- **许可证**：GPL-3.0（声明见 `libraries/doc/GPL3_permission_statement.txt`）
- **官方店铺/资料**：https://seekfree.taobao.com/

本工程是一个**最小可运行模板**，仅完成系统时钟与调试串口初始化，用户可在其中添加外设初始化代码与主循环逻辑。

## 2. 技术栈与运行时架构

### 2.1 技术栈

| 层级 | 内容 |
|------|------|
| 芯片官方 SDK | Infineon PDL（Peripheral Driver Library），位于 `libraries/sdk/` |
| 芯片外设封装 | `libraries/zf_driver/`，提供 GPIO、UART、PWM、PIT、ADC、SPI、IIC、Encoder、Flash 等驱动 |
| 外部设备驱动 | `libraries/zf_device/`，提供摄像头、屏幕、陀螺仪、测距、无线、按键等模块驱动 |
| 公共工具层 | `libraries/zf_common/`，提供时钟、调试、中断、FIFO、字体、类型定义等 |
| 组件层 | `libraries/zf_components/`，提供逐飞上位机助手（SeekFree Assistant）协议 |
| 用户代码 | `project/user/` 与 `project/code/` |

### 2.2 代码分层结构

```
libraries/
├── sdk/                # Infineon 官方 PDL SDK（common + tviibh4m 器件包）
│   ├── common/         # 通用驱动、启动文件、CMSIS、中间件
│   └── tviibh4m/       # CYT2BL3 相关头文件、寄存器定义、系统文件
├── zf_common/          # 公共头文件与工具
│   ├── zf_common_headfile.h   # 统一头文件，按层级包含所有驱动
│   ├── zf_common_typedef.h    # uint8/uint16 等类型别名与 ZF_ENABLE 宏
│   ├── zf_common_clock.h/.c   # 系统时钟初始化
│   ├── zf_common_debug.h/.c   # 调试串口、断言、日志
│   ├── zf_common_interrupt.h/.c # 中断封装
│   └── ...
├── zf_driver/          # MCU 片上外设驱动
│   ├── zf_driver_gpio.h/.c    # GPIO
│   ├── zf_driver_uart.h/.c    # UART
│   ├── zf_driver_pwm.h/.c     # PWM
│   ├── zf_driver_pit.h/.c     # 周期中断定时器
│   └── ...
├── zf_device/          # 外部模块设备驱动
│   ├── zf_device_mt9v03x.h/.c # 总钻风摄像头
│   ├── zf_device_imu660ra.h/.c # 六轴陀螺仪
│   ├── zf_device_ips200.h/.c   # IPS 屏幕
│   ├── zf_device_key.h/.c      # 按键
│   └── ...
└── zf_components/      # 应用组件
    └── seekfree_assistant*     # 逐飞上位机助手（参数调节、虚拟示波器、图像上传）

project/
├── user/
│   ├── main_cm4.c      # 主函数入口，完成 clock_init、debug_init
│   └── cm4_isr.c       # CM4 中断服务函数（PIT、UART、GPIO EXTI）
├── code/               # 用户新增代码文件存放目录（当前为空）
└── iar/                # IAR 工程文件、链接脚本、构建输出
    ├── cyt2bl3.eww     # IAR 工作区
    ├── project_config/
    │   ├── cyt2bl3.ewp # IAR 工程配置
    │   ├── cyt2bl3.ewt # C-STAT/C-RUN 配置
    │   └── cyt2bl3.ewx # 编译生成的 HEX（Intel HEX 格式）
    ├── icf/
    │   └── linker_directives_tviibe4m.icf  # IAR 链接脚本
    └── Debug_m4/         # 默认构建输出目录（.out / .hex / .map）
```

### 2.3 启动与运行流程

1. 复位后由 `libraries/sdk/common/src/startup/iar/startup_cm4.s` 启动。
2. 进入 `main_cm4.c` 的 `main()` 函数。
3. 必须首先调用 `clock_init(SYSTEM_CLOCK_160M)` 完成系统时钟与底层初始化。
4. 通常接着调用 `debug_init()` 初始化调试串口。
5. 在 `for(;;)` 主循环中编写用户逻辑。
6. 中断服务函数统一在 `cm4_isr.c` 中实现或扩展。

## 3. 构建系统

### 3.1 工程文件

- **工作区入口**：`project/iar/cyt2bl3.eww`
- **工程配置**：`project/iar/project_config/cyt2bl3.ewp`
- **当前配置**：`Debug`
- **目标设备**：`CYT2BL_M4`（Infineon CYT2BL_M4）

### 3.2 编译器与链接器关键配置

- **IDE 版本**：IAR EWARM 9.40.1.63870
- **内核**：ARM Cortex-M4，启用 FPU、DSP 扩展
- **C 标准**：默认 C 语言一致性模式（未强制 C99/C11）
- **优化等级**：Low（`CCOptLevel = 1`）
- **调试信息**：开启
- **预处理器宏定义**：
  - `tviibe4m`
  - `CYT2BL8CAE`
  - `CY_USE_PSVP=0`
  - `CY_MCU_rev_a`
  - `CPU_BOARD_REVC`
- **警告抑制**：`Pa082,Pa134`

### 3.3 头文件包含路径

工程已配置以下 include 路径（相对 `$PROJ_DIR$`）：

```
libraries/sdk/common/hdr
libraries/sdk/common/hdr/cmsis/include
libraries/sdk/common/src/drivers
libraries/sdk/common/src/mw
libraries/sdk/tviibh4m/hdr
libraries/sdk/tviibh4m/hdr/ip
libraries/sdk/tviibh4m/hdr/mcureg
libraries/sdk/tviibh4m/src
libraries/sdk/tviibh4m/src/interrupts
libraries/sdk/tviibh4m/src/mw
libraries/sdk/tviibh4m/src/drivers
libraries/sdk/tviibh4m/src/system
libraries/sdk/common/hdr/cmsis/dsp
libraries/zf_common
libraries/zf_driver
libraries/zf_device
libraries/zf_components
project/user
project/code
```

### 3.4 链接脚本

- **文件**：`project/iar/icf/linker_directives_tviibe4m.icf`
- **适用场景**：CYT2BL3 CM4 核心，Flash 下载方式（`_LINK_flash_`）
- **存储器规格**：
  - 总 SRAM：512 KB
  - 总 Code Flash：4160 KB
  - CM0+ 保留部分后，其余分配给 CM4
- **CM4 默认堆栈**：Heap 1 KB，Stack 8 KB

### 3.5 构建产物

构建成功后在 `project/iar/Debug_m4/` 生成：

- `Exe/cyt2bl3.out` —— ELF 调试文件
- `Exe/cyt2bl3.hex` —— 用于烧录的 Intel HEX 文件
- `List/cyt2bl3.map` —— 链接 MAP 文件
- `Obj/` —— 中间目标文件

### 3.6 构建命令

本项目依赖 IAR 图形化 IDE，无命令行构建脚本。标准操作：

1. 双击 `project/iar/cyt2bl3.eww` 打开 IAR。
2. 关闭所有已打开文件（移动工程位置后必须执行）。
3. 菜单选择 **Project → Clean**，等待进度条完成。
4. 菜单选择 **Project → Make**（或按 F7）编译。
5. 菜单选择 **Project → Download and Debug**（或按 Ctrl+D）下载调试。

> 注意：移动工程位置后，必须先 **关闭所有打开文件** 再执行 **Project → Clean**，否则可能出现路径缓存问题。

## 4. 代码组织与模块划分

### 4.1 用户代码应放置位置

- **`project/user/`**：存放 `main_cm4.c` 与 `cm4_isr.c` 等核心用户文件。
- **`project/code/`**：存放用户新增的额外代码文件。**不要在该目录下创建子文件夹**，直接将 `.c`/`.h` 文件放入即可，然后在 IAR 工程中右键添加文件到 `code` 组。

### 4.2 统一头文件

所有库文件通过 `libraries/zf_common/zf_common_headfile.h` 统一包含，按以下顺序组织：

1. 芯片 SDK 底层：`cy_project.h`、`cy_device_headers.h`
2. 开源库公共层：`zf_common_*`
3. 芯片外设驱动层：`zf_driver_*`
4. 外接设备驱动层：`zf_device_*`
5. 组件应用层：`seekfree_assistant*`

用户代码通常只需 `#include "zf_common_headfile.h"`。

### 4.3 中断服务函数

- **PIT 周期中断**：`pit0_ch0_isr()` ~ `pit0_ch21_isr()`，已提供模板，需在中断末尾调用 `pit_isr_flag_clear(PIT_CHx)`。
- **UART 中断**：`uart0_isr()` ~ `uart6_isr()`，通过 `uart_isr_mask()` 区分收发中断。
- **GPIO 外部中断**：`gpio_0_exti_isr()` ~ `gpio_23_exti_isr()`，通过 `exti_flag_get(pin)` 判断具体引脚。

默认 `uart0_isr()` 已集成调试串口接收中断处理（受 `DEBUG_UART_USE_INTERRUPT` 控制）。

### 4.4 调试与日志

- 调试串口默认使用 `UART_0`，波特率 `115200`，引脚 `UART0_TX_P00_1` / `UART0_RX_P00_0`。
- 可通过 `zf_common_debug.h` 修改 `DEBUG_UART_INDEX`、`DEBUG_UART_BAUDRATE`、`DEBUG_UART_TX_PIN`、`DEBUG_UART_RX_PIN`。
- 提供 `zf_assert(x)` 与 `zf_log(x, str)` 宏用于断言与日志输出。

## 5. 开发约定与代码风格

### 5.1 文件编码与换行

- **源文件编码**：GBK/GB2312（中文注释），非 UTF-8。
- **换行符**：CRLF（Windows 风格）。
- 新增文件建议保持 GBK + CRLF，以保证 IAR 与中文注释兼容。

### 5.2 命名规范

| 类型 | 命名风格 | 示例 |
|------|----------|------|
| 文件 | `zf_模块_功能.c/h` | `zf_driver_gpio.h` |
| 函数 | 小写 + 下划线 | `gpio_init()`、`uart_write_byte()` |
| 枚举 | 以 `_enum` 结尾 | `gpio_pin_enum`、`uart_index_enum` |
| 类型别名 | 小写 | `uint8`、`vuint32` |
| 宏/常量 | 全大写 + 下划线 | `ZF_ENABLE`、`GPIO_HIGH` |
| 弱化函数 | `ZF_WEAK` 属性 | `__attribute__((weak))` |

### 5.3 代码结构

- 每个 `.h` 文件顶部包含：
  - GPL-3.0 版权声明与公司信息。
  - 文件名称、版本信息、开发环境、适用平台。
  - 修改记录表。
- 函数声明按功能分区，使用 `//====...====` 注释块分隔。
- 头文件使用 `#ifndef _xxx_h_` / `#define _xxx_h_` / `#endif` 包含卫士。

### 5.4 关键宏与开关

- `USE_ZF_TYPEDEF (1)`：启用 `uint8/uint16/uint32` 等类型别名。
- `ZF_ENABLE / ZF_DISABLE`、`ZF_TRUE / ZF_FALSE`：布尔/使能语义。
- `COMPATIBLE_WITH_OLDER_VERSIONS`：兼容旧版接口（默认未启用）。
- `DEBUG_UART_USE_INTERRUPT (1)`：调试串口使用中断接收。

## 6. 测试策略

本项目为嵌入式硬件工程，**无自动化单元测试框架**。测试依赖：

1. **编译检查**：通过 IAR 编译确保无语法错误与链接错误。
2. **硬件在线调试**：使用 J-Link / IAR 调试器下载到 CYT2BL3 核心板。
3. **串口输出验证**：通过 `printf` / `zf_log` / `zf_assert` 输出到调试串口观察运行状态。
4. **外设功能验证**：逐一初始化外设并读取数据（如陀螺仪、摄像头、屏幕等）。

如需新增测试代码，建议写入 `project/code/` 并在 `main_cm4.c` 中调用。

## 7. 部署流程

1. 在 IAR 中完成编译，生成 `project/iar/Debug_m4/Exe/cyt2bl3.hex`。
2. 使用 IAR **Download and Debug** 或独立的烧录工具将 HEX 文件写入 CYT2BL3 芯片。
3. 断开调试器，上电运行。

## 8. 安全与注意事项

- **许可证约束**：本项目基于 GPL-3.0，修改后分发必须保留逐飞科技版权声明，并遵守 GPL 相关条款。
- **编码注意**：不要直接以 UTF-8 保存现有中文注释文件，否则中文注释会乱码。
- **工程迁移**：复制或移动工程目录后，必须先关闭 IAR 中所有打开文件，再执行 **Project → Clean**。
- **引脚冲突**：部分 SPI 与 UART 共用 SCB 外设资源，选择引脚时需注意资源冲突。
- **中断优先级**：使用 `interrupt_set_priority()` 与 `interrupt_init()` 配置，避免优先级倒置。
- **清理脚本**：`project/iar/删除临时文件IAR.bat` 可删除 IAR 生成的 settings、Debug、Obj 等临时目录与文件。

## 9. 常用文件速查

| 用途 | 路径 |
|------|------|
| 主函数入口 | `project/user/main_cm4.c` |
| 中断服务函数 | `project/user/cm4_isr.c` |
| 统一头文件 | `libraries/zf_common/zf_common_headfile.h` |
| 类型定义 | `libraries/zf_common/zf_common_typedef.h` |
| 调试输出 | `libraries/zf_common/zf_common_debug.h` |
| IAR 工作区 | `project/iar/cyt2bl3.eww` |
| IAR 工程 | `project/iar/project_config/cyt2bl3.ewp` |
| 链接脚本 | `project/iar/icf/linker_directives_tviibe4m.icf` |
| 版本日志 | `libraries/doc/version.txt` |
| 外设说明 | `libraries/zf_device/外设文件说明.txt` |
| 清理脚本 | `project/iar/删除临时文件IAR.bat` |
