# 电机速度闭环设计

## 目标

在当前 CYT2BL3 BLDC 串口通讯已经打通的基础上，实现主控端轮速闭环。测试目标是通过 VOFA 下行命令设置一个目标速度，让左右轮尽量达到同一速度，并且在目标变化时能及时响应。

第一版只做速度模式，不预留电流模式或力矩模式。这里的速度模式是主控端速度闭环：主控读取驱动板速度反馈，计算 duty，再通过 CYT2BL3 `A5 01` duty 帧发给驱动板。

## 范围

包含：

- 通用 PID 模块。
- VOFA/UART0 下行命令解析。
- 电机速度闭环状态和更新逻辑。
- 左右轮方向语义配置。
- 闭环遥测和硬件测试标准。

不包含：

- 轮腿平衡控制。
- 底盘运动学。
- 电流模式、力矩模式。
- 驱动板固件修改。
- 自动在线调参 UI。

## 当前工程前提

当前工程使用：

```c
APP_BLDC_UART_INDEX             UART_1
APP_BLDC_UART_BAUDRATE          460800
APP_BLDC_UART_TX_PIN            UART1_TX_P04_1
APP_BLDC_UART_RX_PIN            UART1_RX_P04_0
APP_BLDC_USE_ASCII_COMMANDS     0U
```

BLDC 控制帧为二进制 duty 协议：

```text
A5 01 left_H left_L right_H right_L checksum
```

速度反馈来自驱动板 `0x02` 功能帧。E07 例程只称其为速度/转速数据，未明确单位。因此第一版统一称为 `speed_count`，不假设它是 RPM。

## 总体架构

```text
VOFA COM6 / UART0
    |
    v
motor_command parser
    parses M,xxx and STOP
    |
    v
actuator_motor speed loop
    target_speed_count
    wheel_feedback.left_speed / right_speed
    left/right PID
    |
    v
duty output
    |
    v
bldc_foc_uart
    sends A5 01 duty frame on UART1
```

`bldc_foc_uart` 继续只负责协议收发。速度闭环放在 `actuator_motor` 层，不进入 UART 协议模块。

## VOFA 下行命令

VOFA 使用 `COM6 / UART0`。同一个串口同时承担：

```text
主控 -> VOFA: JustFloat 遥测
VOFA -> 主控: ASCII 下行命令
```

第一版支持这些命令：

```text
M,200
M,-200
M,0
STOP
```

含义：

- `M,200`：设置左右轮目标速度都为 `200 speed_count`，并启用速度闭环。
- `M,-200`：设置左右轮目标速度都为 `-200 speed_count`。
- `M,0`：目标速度为 0，闭环仍启用，用于观察刹停和稳态误差。
- `STOP`：关闭速度闭环，duty 立即归零，PID 积分清零。

命令以 `\r` 或 `\n` 结束。解析器忽略空白字符，非法命令不改变当前目标，只增加命令错误计数。

## 通用 PID 模块

新增通用 PID 模块：

```text
project/code/control_pid.h
project/code/control_pid.c
```

第一版保持小而稳定，不做动态内存，不绑定任何具体电机或传感器。

建议结构：

```c
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    uint8 first_update;
}pid_controller_struct;
```

建议接口：

```c
void control_pid_init(pid_controller_struct *pid);
void control_pid_set_gain(pid_controller_struct *pid, float kp, float ki, float kd);
void control_pid_set_limit(pid_controller_struct *pid, float integral_limit, float output_limit);
void control_pid_reset(pid_controller_struct *pid);
float control_pid_update(pid_controller_struct *pid, float target, float feedback, float dt_s);
```

行为要求：

- `error = target - feedback`。
- 积分项限幅到 `[-integral_limit, integral_limit]`。
- 输出限幅到 `[-output_limit, output_limit]`。
- `dt_s <= 0` 时不更新积分和微分，返回限幅后的比例输出。
- 第一次更新不计算 D 项，避免启动突跳。
- `control_pid_reset()` 清空积分、历史误差，并恢复首次更新状态。

轮速闭环第一版使用 PI，`kd = 0`。保留 `kd` 字段是为了后续同一 PID 库可以服务平衡、腿部和其它闭环。

## 电机速度闭环

`actuator_motor` 增加速度闭环状态。建议内部状态包括：

```text
enable
target_speed_count
left_target_speed_count
right_target_speed_count
left_pid
right_pid
left_duty_out
right_duty_out
last_loop_ms
command_error_count
```

第一版 VOFA 命令只设置一个 `target_speed_count`，内部映射为：

```text
left_target_speed_count  = target_speed_count
right_target_speed_count = target_speed_count
```

控制周期使用 `APP_MOTOR_PERIOD_MS`。若实际 `dt` 抖动，PID 使用 `now_ms - last_loop_ms` 计算 `dt_s`。

闭环更新逻辑：

```text
如果 app state 是 FAULT:
    stop, reset PID

如果 speed loop 未启用:
    stop

如果 wheel feedback offline:
    stop, reset PID

读取 left_speed/right_speed
应用方向语义修正
left_duty  = PID(left_target_speed, left_speed)
right_duty = PID(right_target_speed, right_speed)
应用 duty 方向语义修正
限幅后发送 duty
```

## 方向语义

第一版必须显式配置左右轮方向，不把方向修正写死在 PID 里。

建议配置项：

```c
#define APP_MOTOR_LEFT_DUTY_SIGN        (-1.0f)
#define APP_MOTOR_RIGHT_DUTY_SIGN       (-1.0f)
#define APP_MOTOR_LEFT_SPEED_SIGN       (1.0f)
#define APP_MOTOR_RIGHT_SPEED_SIGN      (1.0f)
```

含义：

- duty sign：把控制器输出的车体语义 duty 转成驱动板 duty。
- speed sign：把驱动板反馈速度转成车体语义速度。

标定目标：

```text
M,正数 -> 车体语义上左右轮都应前进
反馈 speed_count 在车体语义上应为正
```

如果某侧相反，只改 sign 配置，不改 PID 算法。

## 保护逻辑

速度闭环必须满足这些安全行为：

- `STOP` 命令立即输出 `0 duty` 并清 PID。
- `APP_STATE_FAULT` 时输出 `0 duty` 并清 PID。
- `wheel_feedback.online == APP_FALSE` 时输出 `0 duty` 并清 PID。
- 反馈 age 超过 `APP_BLDC_FEEDBACK_TIMEOUT_MS` 时认为 offline。
- duty 输出限幅使用独立闭环限幅，例如 `APP_MOTOR_SPEED_DUTY_LIMIT`。
- 积分限幅使用独立配置，例如 `APP_MOTOR_SPEED_INTEGRAL_LIMIT`。
- 目标速度限幅使用独立配置，例如 `APP_MOTOR_SPEED_TARGET_LIMIT`。

第一版不把 BLDC 反馈丢失直接升级为系统 FAULT，只停止电机闭环并在遥测里暴露状态。

## 配置建议

新增或调整配置项：

```c
#define APP_MOTOR_SPEED_LOOP_ENABLE             (1U)
#define APP_MOTOR_SPEED_TARGET_LIMIT            (1000.0f)
#define APP_MOTOR_SPEED_DUTY_LIMIT              (2000.0f)
#define APP_MOTOR_SPEED_INTEGRAL_LIMIT          (1000.0f)
#define APP_MOTOR_SPEED_KP                      (1.0f)
#define APP_MOTOR_SPEED_KI                      (0.0f)
#define APP_MOTOR_SPEED_KD                      (0.0f)
```

初始参数以能安全响应为主。建议先 `ki = 0` 做 P 控制，确认方向和反馈正确后再逐步加 `ki`。

## 遥测

保留现有 IMU、状态和 BLDC 诊断通道，同时增加或替换一组速度闭环通道：

```text
speed_loop_enable
target_speed_count
left_speed_count
right_speed_count
left_error
right_error
left_duty_out
right_duty_out
left_integral
right_integral
feedback_online
command_error_count
```

如果 VOFA 通道数量有限，优先保留：

```text
target_speed_count
left_speed_count
right_speed_count
left_duty_out
right_duty_out
feedback_online
app_state
safety_fault
```

## 测试计划

### 1. 通讯确认

构建并烧录 `cyt4bb7_cm_7_0`。在 VOFA 确认：

```text
COM6: JustFloat telemetry 正常
COM11: UART1 BLDC 二进制帧正常
```

驱动板连接后，开启速度反馈，确认 `wheel_feedback.online = 1`，静止时左右速度接近 0。

### 2. 方向标定

悬空车轮，使用低目标：

```text
M,100
M,-100
STOP
```

确认：

- 正目标时左右轮车体语义方向一致。
- 负目标时左右轮反向。
- 反馈速度符号和目标符号一致。
- 不一致时只调整 sign 配置。

### 3. 单目标速度响应

使用：

```text
M,200
M,400
M,0
M,-200
STOP
```

验收标准：

- 目标变化后 duty 立即变化。
- 速度反馈朝目标方向变化。
- 左右轮速度误差逐步收敛。
- `STOP` 后 duty 立即为 0。
- 反馈断开或超时后 duty 自动为 0。

### 4. 参数调整

先只调 `kp`：

```text
kp 太小: 响应慢，稳态误差大
kp 太大: 震荡或左右轮速度来回摆动
```

方向正确、P 响应稳定后再加 `ki` 消除稳态误差。

## 失败判据与排查顺序

如果 `M,200` 后轮子不动：

1. 看 `speed_loop_enable` 是否为 1。
2. 看 `feedback_online` 是否为 1。
3. 看 `app_state` 是否为 FAULT。
4. 看 `left_duty_out/right_duty_out` 是否非 0。
5. 看 UART1 是否有 `A5 01` duty 帧。
6. 查驱动板动力电、共地、RX/TX、使能/刹车输入。

如果轮子越调越反：

1. 不改 PID。
2. 先改 speed sign，让正转反馈为正。
3. 再改 duty sign，让正目标产生正反馈。

## 后续扩展

速度闭环稳定后，平衡层只应输出目标速度，不直接输出 duty：

```text
control_balance -> target_speed_count -> actuator_motor speed loop -> duty
```

这样平衡控制、腿部控制和 BLDC 协议保持解耦。

