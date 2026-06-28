# WheelLeggedCar

WheelLeggedCar 是一个基于逐飞 CYT2BL3 开源库空工程搭建的轮腿小车控制项目。当前目标是先把 IMU、调度器、舵机、电机、摄像头等模块拆清楚，再逐步完成姿态感知、轮腿执行和整车闭环控制。

## 当前状态

- 已完成仓库初始化和项目树整理。
- 已接入 LSM6DSV16X IMU，当前通过 SPI3 读取并使用芯片内置 SFLP/FIFO 输出姿态数据。
- 已接入 1 ms `app_scheduler`，当前处于 IMU-only 调度模式。
- 已接入 `telemetry`，通过 VOFA+ JustFloat 输出 roll、pitch、yaw。
- 已完成舵机模块初版，舵机 0 在 `P00_3` 输出 50 Hz PWM 已实测可用。
- BDS300 舵机已实测可动，当前自动往复测试已关闭，使用独立电源时必须与 MCU 共地。

## 目录结构

```text
docs/                  项目规划与架构文档
libraries/             逐飞库与芯片 SDK
project/code/          应用层模块代码
project/user/          main、ISR 等用户入口
project/iar/           IAR 工程文件
```

主要应用模块位于 `project/code/`：

- `sensor_imu.*`：IMU 采集与姿态状态发布。
- `telemetry.*`：VOFA+ 调试输出。
- `app_scheduler.*`：周期任务调度。
- `actuator_servo.*`：舵机 PWM 输出、限幅和缓动。
- `app_config.h`：当前周期、限幅、舵机通道等项目级配置入口。

## 已知引脚与资源

| 功能 | 资源/通道 | 引脚 | 状态 |
| --- | --- | --- | --- |
| VOFA+ / debug UART | `UART_0` | TX=`UART0_TX_P00_1`，RX=`UART0_RX_P00_0` | 已使用 |
| 心跳 LED | GPIO | `P23_7` | 已使用 |
| 系统 tick | `PIT_CH1` | 无外部引脚 | 已使用，1 ms |
| IMU SPI | `SPI_3` | CLK=`P13_2`，MOSI=`P13_1`，MISO=`P13_0` | 已验证 |
| IMU CS | GPIO | `P13_3` | 已验证 |
| 舵机 0 | `TCPWM_CH13_P00_3` | `P00_3` | 已验证 |
| 舵机 1 | `TCPWM_CH14_P00_2` | `P00_2` | 预留，未启用 |
| 舵机 2 | `TCPWM_CH17_P00_1` | `P00_1` | 预留，和 UART0 TX 冲突 |
| 舵机 3 | `TCPWM_CH18_P00_0` | `P00_0` | 预留，和 UART0 RX 冲突 |
| 摄像头 | TBD | TBD | 未接入 |
| 左右无刷电机 | TBD | TBD | 未接入 |
| 编码器/电调反馈 | TBD | TBD | 未接入 |

舵机当前参数：50 Hz，500-2500 us 脉宽对应 0-180 度。当前保留 `APP_SERVO_ACTIVE_MASK=0x01` 的舵机 0 通道配置；`APP_SERVO_TEST_ENABLE=0`，自动往复测试已关闭，等待四执行器联调规划后再打开。

## 编译与测试

1. 使用 IAR Embedded Workbench for ARM 打开 `project/iar/cyt2bl3.eww`。
2. 选择 `Debug` 配置后执行 Make/Rebuild。
3. 下载到 CYT2BL3 开发板运行。
4. 打开 VOFA+，串口波特率 115200，协议选择 JustFloat。
5. 当前遥测通道：`I0=roll`，`I1=pitch`，`I2=yaw`。

## 注意事项

- `project/iar/Debug_m4/` 是 IAR 生成的编译输出，不作为源代码维护重点。
- 舵机独立供电时，舵机电源 GND、MCU GND、电源负极必须共地。
- 默认调试串口占用 `P00_1/P00_0`，启用舵机 2/3 前需要先处理引脚冲突。
- 轮腿机构接上负载前，先完成舵机中位、方向、软限位和急停策略验证。
