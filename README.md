# WheelLeggedCar

基于 Infineon TRAVEO T2G CYT4BB7 的轮腿机器人控制项目。

## 芯片

| 分支 | 芯片 | 状态 |
|---|:---:|:---:|
| `main` | CYT4BB7 (TRAVEO T2G Body) | **活跃开发** |
| `hardware/cyt2bl3` | CYT2BL3 (TRAVEO T2G Lite) | 归档 |

IAR 项目文件：`project/iar/cyt4bb7.eww`

## 当前能力

- ✅ 两轮自平衡站立（>30 秒）
- ✅ 抗扰动恢复（推不倒也回正）
- ✅ BLDC 双路 FOC 电机控制（UART 二进制协议）
- ✅ LSM6DSV16X IMU（SFLP 120Hz + INT1 中断）
- ✅ 四状态 LQR 平衡控制器
- ✅ VOFA JustFloat 16 通道遥测
- ✅ 串口指令调试（BL/BS/BI/BP/BZ/STOP/C/D/M）
- ⬜ 腿/舵机协调控制
- ⬜ 前后行驶速度控制
- ⬜ 转弯控制

### 稳定版本

```text
balance-stand-v1 (tag)
  commit: f56185c
  参数:   BL,18,8,3.0,0 BS,5.0
  测试:   30s 站立 + 扰动恢复，bal 峰值 216 RPM（限幅 300）
  报告:   docs/balance-stand-v1-test-report.md
```

## 控制架构

```
                    ┌──────────────┐
    pitch_setpoint ─┤ K_angle (18) ├──┐
                    └──────────────┘  │
                    ┌──────────────┐  │
    pitch_rate ─────┤ K_rate  (8)  ├──┤
                    └──────────────┘  │
                    ┌──────────────┐  │    ┌─────┐    ┌──────────┐
    wheel_speed ────┤ K_speed (3)  ├──┼────┤ sum ├────┤ ±300 RPM ├── 电机
                    └──────────────┘  │    └─────┘    └──────────┘
                    ┌──────────────┐  │
    wheel_pos ──────┤ K_pos   (0)  ├──┘
                    └──────────────┘

    K_speed/K_angle = 0.167 °/RPM (减速倾角系数)
```

## 硬件连接

| 功能 | 接口 | 引脚 |
|---|---|---|
| 调试/VOFA 遥测 | UART0 | COM6 |
| BLDC 双路驱动 | UART1 | TX_P04_1, RX_P04_0 |
| IMU (LSM6DSV16X) | SPI2 | INT1=P19_3 |
| 舵机 ×4 | TCPWM | P00_3, P01_0, P01_1, P05_0 |

> **IMU 方向说明**：LSM6DSV16X 芯片在 PCB 上旋转 90°，固件已做 roll↔pitch 轴映射。

## 快速开始

### 1. 烧录

IAR 打开 `project/iar/cyt4bb7.eww`，编译并烧录 `cyt4bb7_cm_7_0`。

### 2. 开机校准

机器人竖直静置，VOFA 发送：

```text
IMU_ZERO
```

### 3. 启动平衡

```text
STOP
BL,18,8,3.0,0
BS,5.0
C,0,0
B,2
```

| 步骤 | 说明 |
|---|---|
| `STOP` | 清空所有状态 |
| `BL,...` | 设置四状态增益 |
| `BS,5.0` | 设定俯仰偏移（COM 补偿） |
| `C,0,0` | 启用底盘（零附加速度） |
| `B,2` | 激活平衡模式 |

> B,2 激活前请扶稳机器人，K_angle=18 对初始角度敏感。

### 4. 数据采集

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:BL,18,8,3.0,0;1:BS,5.0;1.5:B,1;2:B,2;2.5:C,0,0" `
  -Note "my_test"
```

## 串口命令

| 命令 | 说明 |
|---|---|
| `STOP` | 停止所有电机，清空 PID/激励/底盘 |
| `D,duty` | 开环占空比（双轮） |
| `M,rpm` | RPM 闭环目标（双轮） |
| `B,0` | 停止平衡模式 |
| `B,1` | 平衡待机（遥测仅读） |
| `B,2` | 平衡测试模式（电机输出） |
| `BL,angle,rate,speed,pos` | 设置全部四状态增益 |
| `BP,kp,kd` | 设置 PD 增益（兼容） |
| `BS,offset` | 设定俯仰 setpoint (°) |
| `BZ` | 清零轮位置积分和运动状态 |
| `BI,amp,period_ms` | 系统辨识方波激励 |
| `IMU_ZERO` | 陀螺仪零偏校准（需静止） |
| `C,forward,turn` | 底盘前进/转弯 RPM |

## 工具链

| 工具 | 路径 |
|---|---|
| 遥测采集 | `tools/collect_balance_data.ps1` |
| LQR 辨识 | `tools/identify_balance_model.m` |
| PD 分析 | `tools/analyze_balance_pd.m` |
| 电机阶跃 | `tools/collect_motor_steps.ps1` |

## 目录结构

```text
project/code/     应用层模块（平衡/电机/舵机/IMU/遥测/安全）
project/user/     main、ISR 入口
project/iar/      IAR 项目文件 (cyt4bb7.eww)
libraries/        逐飞库与芯片 SDK
tools/            MATLAB/PowerShell 工具链
data/             采集数据（CSV）
docs/             设计文档与测试报告
```
