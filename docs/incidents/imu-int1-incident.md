# IMU INT1 事件驱动上线事故记录

## 1. 背景

在轮式-腿式智能车项目中，为了降低 IMU 更新延迟、避免 SPI 读取阻塞电机控制时序，决定将 LSM6DSV16X 的 `INT1` 引脚作为 IMU 数据就绪中断源，替代原来的 5 ms 固定周期轮询。

目标链路：

```text
LSM6DSV16X FIFO watermark
  -> INT1 输出高电平
  -> CYT2BL3 P11_0 外部中断（上升沿）
  -> gpio_11_exti_isr() 置 data_ready_flag
  -> 调度器读取 FIFO 并更新姿态
```

相关硬件接线：

| 信号 | 引脚 |
|---|---|
| IMU INT1 | P11.0 |
| IMU SCK | P13.2 |
| IMU MOSI | P13.1 |
| IMU MISO | P13.0 |
| IMU CS | P13.3 |

## 2. 事故现象

第一次切到 `APP_IMU_USE_INT1 = 1` 后，VOFA+ 上：

- roll/pitch/yaw 不再更新；
- `imu_data_age` 持续增大；
- `int_count` 始终为 0；
- `P11_0` 电平一直为 1。

即：IMU 已经把 `INT1` 拉高了，但 MCU 的外部中断一次都没触发。

## 3. 根因分析

### 3.1 INT1 是什么信号

我们配置的是 **FIFO watermark（阈值）中断**：

- 当 FIFO 中样本数 ≥ watermark 时，`INT1` 被拉高；
- 当 MCU 读走 FIFO、样本数 < watermark 时，`INT1` 自动拉低；
- 因此它是一个**电平信号**，高电平期间表示“有数据待读取”。

### 3.2 初始化顺序导致漏沿

实际初始化顺序：

```text
sensor_imu_init()
  ├─ lsm6dsv16x_spi_init()
  ├─ lsm6dsv16x_init()          // 开加速度/陀螺仪
  ├─ lsm6dsv16x_sflp_init()     // 开 SFLP、FIFO watermark=1、INT1 路由
  │     从这一刻起 IMU 开始以 120Hz 往 FIFO 写 SFLP 数据
  │     第一个样本进入 FIFO 后，INT1 立即拉高 ↑
  │
  └─ exti_init(P11_0, RISING)   // 此时才打开 MCU 外部中断
```

问题：

1. `lsm6dsv16x_sflp_init()` 之后 IMU 已经开始发数据，`INT1` 的**上升沿在 `exti_init` 之前就已经出现**；
2. 我们用的是**上升沿触发**，而 `INT1` 拉高后会一直维持高电平；
3. MCU 没有捕获到这个上升沿，因此从不进入 `gpio_11_exti_isr()`；
4. 不进 ISR 就不置 `data_ready_flag`；
5. 不读 FIFO，`INT1` 永远不会被拉低；
6. 于是 `INT1` 永久高电平，系统进入死锁。

### 3.3 为什么轮询模式正常

轮询模式不看 `INT1`，每隔 5 ms 直接调用 `sensor_imu_update()` 读 FIFO。只要 FIFO 里有 SFLP 数据，就能正常更新姿态。这也反过来证明：IMU 中断路由本身和硬件接线都没有问题，问题只在 MCU 没捕获到第一个上升沿。

## 4. 处理过程

### 4.1 关键对话

> **我**：轮询模式下没问题。
> **Kimi**：已切回轮询。接下来可以加诊断通道排查 INT1。
>
> **我**：P11_0 是 P11.0 吗？
> **Kimi**：是的，`P11_0` 就是 P11.0。
>
> **我**：那接线没问题，加诊断切回 INT1 吧。
> **Kimi**：已切回 INT1，VOFA+ 增加 7 个诊断通道：roll/pitch/yaw/data_age/int_count/stale_count/P11_0 电平。
>
> *[VOFA+ 显示 I6=1, I4=0, I5=1, I3 很大]*
>
> **Kimi**：P11_0 一直高但 EXTI 计数为 0，说明上升沿在开中断前已出现，INT1 卡死。
>
> **我**：那为什么一开始不把它置成低电平呢？
> **Kimi**：`INT1` 是 IMU 输出，MCU 只能读不能写。把它拉低的唯一办法是读 FIFO，让 IMU 自己把 INT1 拉低。

### 4.2 修复方法

在 `project/code/sensor_imu.c` 的 `sensor_imu_init()` 中，打开 EXTI 之后主动同步一次 `INT1` 状态：

```c
#if (APP_IMU_USE_INT1 == 1U)
    exti_init(APP_IMU_INT1_PIN, EXTI_TRIGGER_RISING);
    (void)exti_flag_get(APP_IMU_INT1_PIN);
    if(GPIO_HIGH == gpio_get_level(APP_IMU_INT1_PIN))
    {
        sensor_imu_int1_isr();
    }
#endif
```

作用：

1. `exti_flag_get()` 清除可能已挂起的 EXTI 标志；
2. 如果 `P11_0` 已经处于高电平，说明上升沿已漏掉，手动触发一次 `data_ready_flag`；
3. 调度器读到 flag 后调用 `sensor_imu_update()`，把 FIFO 读空，`INT1` 自动拉低；
4. 下一个 SFLP 样本到达 watermark 时，产生新的上升沿，EXTI 恢复正常事件驱动。

### 4.3 相关代码提交

| Commit | 说明 |
|---|---|
| `a54b4bb` | 实现 IMU INT1 事件驱动更新 |
| `589d5de` | 修复 `servo_last_ms` 未使用 warning |
| `4668b0c` | 临时切换回 IMU 轮询模式 |
| `524c883` | 切回 INT1 并增加诊断通道 |
| `4148972` | 修复 INT1 初始化时可能已处于高电平导致漏沿 |

## 5. 总结

| 现象 | 根因 |
|---|---|
| 轮询正常，INT1 模式无数据 | IMU 在开 EXTI 之前已经把 INT1 拉高，上升沿漏掉 |
| P11_0 一直高、EXTI 计数为 0 | INT1 是电平信号，卡在高电平后不会再产生新的上升沿 |
| 修复后正常 | 初始化结束后主动同步一次 INT1 状态，打破死锁 |

### 经验教训

1. **电平型中断要防初始已 assert**：对于“高电平表示有数据”的源，打开中断后应检查当前电平，必要时手动触发第一次处理。
2. **不能靠 MCU 把外部中断输出拉低**：`INT1` 是传感器输出，MCU 只能读；让它变低的唯一办法是处理触发条件（这里是读 FIFO）。
3. **先开中断再使能源端**：理想顺序是 `exti_init()` 在 IMU 开始产生中断之前完成。如果源端已经使能，必须做同步/清标志/手动触发。
4. **诊断通道很有用**：在 VOFA+ 里同时看 `int_count` 和 `P11_0` 电平，能快速定位是“IMU 没发脉冲”还是“MCU 没收到脉冲”。

## 6. 日期

2026-06-28
