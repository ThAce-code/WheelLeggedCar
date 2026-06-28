# leg_config 建立计划

## 1. 目标

`leg_config` 用于集中描述四个轮腿舵机的安装位置、方向、中位角、软限位和调试参数。它的目标是把机械差异从控制算法中剥离出来，让 `control_leg` 只处理车身姿态逻辑。

最终效果：

1. 舵机 0/1/2/3 的物理含义清楚。
2. 每个舵机的正方向统一。
3. 每个舵机有独立中位和软限位。
4. 后续更换舵机、改接线或镜像安装时，只改配置，不改混控算法。

重要修正：当前轮腿机构为连杆约束结构，`neutral_deg` 不能默认理解为舵机电气中位 90 度。第一阶段应先标定“当前机械安全保持角”，确认不会顶死、拉扯或堵转后，再逐步收敛到真正的机械中位。

## 2. 建议文件

新增两个文件：

```text
project/code/leg_config.h
project/code/leg_config.c
```

如果后续希望把配置烧写到 Flash，再扩展 `leg_config_storage.*`。当前阶段先使用编译期常量，降低调试复杂度。

## 3. 执行范围与边界

本计划可以作为第一版执行计划，但执行时必须限制范围，避免把舵机、电机和姿态闭环一次性混在一起。

本次允许修改：

```text
project/code/leg_config.h
project/code/leg_config.c
project/code/control_leg.h
project/code/control_leg.c
project/code/app_types.h
project/iar/project_config/cyt2bl3.ewp
docs/project-tree.md
```

按需修改：

```text
project/code/app_config.h
```

本次不要修改：

```text
project/code/actuator_servo.c
project/code/actuator_servo.h
project/code/actuator_motor.c
project/code/app_scheduler.c
project/user/main_cm4.c
```

原因：

1. `actuator_servo` 当前已经能稳定输出 PWM，不在本次重构范围内。
2. `APP_SERVO_TEST_ENABLE` 当前应保持 `0U`，不要恢复自动往复测试。
3. 本次只建立配置层和腿部控制接口，不做 IMU 姿态闭环。
4. 本次不接入四舵机同步大动作，只让代码具备后续标定能力。

安全硬约束：

1. 禁止在连杆已连接且当前安全角未知时，直接输出 `90.0f`。
2. 禁止上电后自动启用四舵机同步动作。
3. 禁止用 `neutral_deg=90.0f` 作为默认动作输出依据。
4. 标定前必须保持 `APP_SERVO_TEST_ENABLE=0U`。
5. 第一阶段只允许“单舵机、小步进、可随时断电”的试探动作。
6. 如果出现舵机嗡鸣、堵转、连杆顶死、车身被拉扯，立即断电，不继续加角度。

## 4. 数据结构草案

建议先定义舵机位置枚举：

```c
typedef enum
{
    LEG_SERVO_FL = 0,
    LEG_SERVO_FR = 1,
    LEG_SERVO_RL = 2,
    LEG_SERVO_RR = 3,
    LEG_SERVO_COUNT = 4
}leg_servo_id_enum;
```

单个舵机配置：

```c
typedef struct
{
    uint8 servo_index;
    float safe_deg;
    float neutral_deg;
    float min_deg;
    float max_deg;
    float direction;
    float mount_x;
    float mount_y;
}leg_servo_config_struct;
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `servo_index` | 对应 `actuator_servo` 的物理通道编号 |
| `safe_deg` | 当前实车已确认不会损坏机构的保持角，第一阶段优先使用 |
| `neutral_deg` | 机械中位角，必须在安全标定后再确认 |
| `min_deg/max_deg` | 该舵机允许输出的机械软限位 |
| `direction` | 安装方向修正，统一正方向语义 |
| `mount_x` | 前后位置系数，用于 pitch 混控 |
| `mount_y` | 左右位置系数，用于 roll 混控 |

整车轮腿配置：

```c
typedef struct
{
    leg_servo_config_struct servo[LEG_SERVO_COUNT];
    float height_min;
    float height_max;
    float pitch_limit;
    float roll_limit;
}leg_config_struct;
```

建议额外定义腿部控制模式：

```c
typedef enum
{
    LEG_MODE_LOCK = 0,
    LEG_MODE_MANUAL,
    LEG_MODE_ATTITUDE
}leg_mode_enum;
```

第一版只实现 `LEG_MODE_LOCK` 和 `LEG_MODE_MANUAL` 的稳定输出；`LEG_MODE_ATTITUDE` 可以先保留枚举，不接入 IMU 闭环。

## 5. 初始映射与默认参数

基于当前布局图，先采用以下映射：

| ID | 位置 | `servo_index` | `mount_x` | `mount_y` |
| --- | --- | ---: | ---: | ---: |
| `LEG_SERVO_FL` | 前左 | 0 | `+1.0f` | `+1.0f` |
| `LEG_SERVO_FR` | 前右 | 1 | `+1.0f` | `-1.0f` |
| `LEG_SERVO_RL` | 后左 | 2 | `-1.0f` | `+1.0f` |
| `LEG_SERVO_RR` | 后右 | 3 | `-1.0f` | `-1.0f` |

初始角度参数先保守设置：

| 参数 | 初值 | 说明 |
| --- | ---: | --- |
| `safe_deg` | TBD | 必须由实车当前安全姿态测得，不能默认 90 度 |
| `neutral_deg` | TBD | 在 `safe_deg` 验证后再标定，不能默认 90 度 |
| `min_deg` | TBD | 从 `safe_deg` 开始小步进试探后确认 |
| `max_deg` | TBD | 从 `safe_deg` 开始小步进试探后确认 |
| `direction` | `+1.0f` | 初值，标定后按实车修改 |
| `height_min/max` | `0.0f / 0.0f` | 标定前禁止同步高度动作 |
| `pitch_limit` | `0.0f` | 标定前禁止俯仰补偿动作 |
| `roll_limit` | `0.0f` | 标定前禁止横滚补偿动作 |

执行者如果必须给出编译期初值，应使用当前单舵机已经实测安全的角度作为 `safe_deg`，而不是直接写 `90.0f`。在没有实测角度时，`control_leg` 默认不应使能舵机输出。

## 6. 对外接口

`leg_config.h` 建议提供只读接口：

```c
const leg_config_struct *leg_config_get(void);
const leg_servo_config_struct *leg_config_get_servo(uint8 leg_id);
```

接口行为要求：

1. `leg_config_get()` 永远返回全局静态配置地址。
2. `leg_config_get_servo()` 如果 `leg_id >= LEG_SERVO_COUNT`，返回空指针或返回 `LEG_SERVO_FL` 的配置；建议返回空指针，调用方负责处理。
3. 初期不提供运行时修改接口。
4. 调参时直接修改 `leg_config.c` 常量，重新编译烧录，避免运行时参数来源混乱。

## 7. control_leg 接入方式

`control_leg` 不直接写固定舵机编号，而是读取 `leg_config`：

```text
control_leg_update()
    -> 读取 leg_config
    -> 根据 mode 生成 height/pitch/roll/action_offset
    -> 混控得到四个目标角
    -> 按每个舵机 min/max 限幅
    -> 输出 servo_cmd_struct
```

`actuator_servo` 仍只负责 PWM、缓动和最终输出。

第一版 `control_leg` 建议提供以下接口：

```c
void control_leg_init(void);
void control_leg_update(uint32 now_ms);
void control_leg_set_mode(leg_mode_enum mode);
void control_leg_set_manual_angle(uint8 leg_id, float angle_deg);
void control_leg_set_body_cmd(float height_cmd, float pitch_cmd, float roll_cmd);
const servo_cmd_struct *control_leg_get_servo_cmd(void);
```

第一版行为：

1. `control_leg_init()` 默认进入 `LEG_MODE_LOCK`。
2. `LEG_MODE_LOCK` 优先输出四个舵机的 `safe_deg`，不是 `neutral_deg`。
3. `LEG_MODE_MANUAL` 使用手动角度，但仍必须经过每个舵机的 `min_deg/max_deg` 限幅。
4. `LEG_MODE_ATTITUDE` 暂时可以退化为 `LEG_MODE_LOCK`，不要做姿态闭环。
5. `control_leg_get_servo_cmd()` 返回最后一次计算得到的 `servo_cmd_struct`。
6. 在没有确认 `safe_deg` 前，`servo_cmd_struct.enable` 应保持关闭。

## 8. 标定流程

### 8.1 单舵机确认

1. 保持 `APP_SERVO_TEST_ENABLE=0`。
2. 优先拆下舵盘或断开连杆负载，只让舵机本体回到可控角度。
3. 如果不能拆舵盘，禁止直接输出 90 度。
4. 只启用一个舵机通道，其他舵机保持不输出。
5. 从当前已知安全姿态附近开始，记录该角度为 `safe_deg`。
6. 每次只做 `2-3` 度试探，不做大步进。
7. 观察是否出现嗡鸣、堵转、连杆顶死或车身被拉扯。
8. 小幅输出 `safe_deg + 2` 和 `safe_deg - 2`。
9. 记录“正方向”是否符合轮腿展开定义。
10. 如方向相反，将该舵机 `direction` 改为 `-1.0f`。

### 8.2 软限位确认

1. 从 `safe_deg` 开始，每次只增加或减少 `2-3` 度。
2. 观察是否接近机械干涉、舵机堵转或连杆极限。
3. 记录安全范围，更新 `min_deg/max_deg`。
4. 四个舵机分别完成，不做同步动作。
5. 完成软限位确认后，再判断是否可以把某个角度定义为 `neutral_deg`。

### 8.3 四舵机同步确认

1. 四个舵机先全部输出各自 `safe_deg`。
2. 确认无结构拉扯后，才允许输出各自 `neutral_deg`。
3. 小幅输入 `height_cmd`，观察四个轮腿是否同向展开。
4. 输入小幅 `pitch_cmd`，观察前后是否差动。
5. 输入小幅 `roll_cmd`，观察左右是否差动。
6. 所有同步动作建议从 `1-2` 度等效幅度开始。
7. 若方向不一致，只改 `direction`，不改混控公式。

## 9. 验收标准

`leg_config` 第一版完成后，应满足：

1. `LEG_SERVO_FL/FR/RL/RR` 与实车位置一致。
2. 每个舵机都有已确认安全的 `safe_deg`。
3. 四个舵机在 `safe_deg` 时不顶死、不拉扯机构。
4. `neutral_deg` 只能在 `safe_deg` 验证后确认。
5. `height_cmd > 0` 时四个轮腿动作方向一致。
6. `pitch_cmd > 0` 时前后轮腿出现预期差动。
7. `roll_cmd > 0` 时左右轮腿出现预期差动。
8. 任意控制命令都不会越过每个舵机的 `min_deg/max_deg`。
9. 上电默认不会让舵机自动大幅动作。

## 10. 实施顺序

### 10.1 第一步：只新增 leg_config

1. 新增 `project/code/leg_config.h`。
2. 新增 `project/code/leg_config.c`。
3. 写入 `leg_servo_id_enum`、`leg_servo_config_struct`、`leg_config_struct`。
4. 在 `leg_config.c` 中定义 `static const leg_config_struct leg_config_default`。
5. 实现 `leg_config_get()` 和 `leg_config_get_servo()`。
6. 把 `leg_config.c/.h` 加入 IAR 工程 `project/iar/project_config/cyt2bl3.ewp`。
7. 更新 `docs/project-tree.md`。

第一步验收：

1. `rg -n "leg_config" project/code project/iar/project_config docs` 能看到新文件、include 和工程引用。
2. 不修改 `actuator_servo`。
3. 不打开 `APP_SERVO_TEST_ENABLE`。

### 10.2 第二步：接入 control_leg 静态输出

1. `control_leg.c` include `leg_config.h`。
2. `control_leg_init()` 读取配置并初始化四个舵机目标为 `safe_deg`。
3. `control_leg_update()` 在 `LEG_MODE_LOCK` 下持续输出安全保持角。
4. 所有输出角度都通过本地限幅函数限制到 `min_deg/max_deg`。
5. `control_leg_get_servo_cmd()` 返回限幅后的 `servo_cmd_struct`。

第二步验收：

1. 编译期能找到 `leg_config.h`。
2. `control_leg` 中不再写死 0/1/2/3 的机械语义。
3. `LEG_MODE_LOCK` 下四个目标角都是各自 `safe_deg`。
4. 默认不因为 `neutral_deg` 未标定而输出 90 度。

### 10.3 第三步：增加手动标定接口

1. 增加 `control_leg_set_mode()`。
2. 增加 `control_leg_set_manual_angle()`。
3. `LEG_MODE_MANUAL` 下使用手动角度输出。
4. 手动角度仍必须经过软限位。
5. 手动测试时建议从 `safe_deg` 附近开始，不允许直接跳转到 90 度。

第三步验收：

1. 对任意舵机写入越界角度，输出会被限制到该舵机 `min_deg/max_deg`。
2. 不触发自动往复测试。
3. 不需要 IMU 数据参与。
4. 在 `safe_deg/min_deg/max_deg` 未确认前，不做四舵机同步动作。

### 10.4 第四步：预留姿态混控，不实车启用

1. 增加 `control_leg_set_body_cmd(height, pitch, roll)`。
2. 在代码中实现混控公式，但默认模式不进入 `LEG_MODE_ATTITUDE`。
3. `height/pitch/roll` 必须先按 `height_min/height_max/pitch_limit/roll_limit` 限幅。
4. 混控结果再按每个舵机 `min_deg/max_deg` 限幅。

第四步验收：

1. 代码中存在混控路径。
2. 默认运行仍是 `LEG_MODE_LOCK`。
3. 不因为 IMU 数据异常导致舵机动作。

## 11. 给执行者的具体要求

如果把本计划交给 Kimi 执行，请按以下约束执行：

1. 先读 `docs/leg-control-architecture.md`、`docs/leg-config-plan.md`、`project/code/app_types.h`、`project/code/control_leg.c/.h`、`project/code/actuator_servo.c/.h`。
2. 只做 `leg_config` 和 `control_leg` 第一版，不做电机联调。
3. 不删除现有舵机驱动。
4. 不改 `APP_SERVO_TEST_ENABLE=0U`。
5. 不让舵机上电后自动动作。
6. 不允许把 `neutral_deg` 默认写成 90 度并用于上电输出。
7. 必须新增 `safe_deg` 或等价字段，作为第一阶段安全保持角。
8. 新增源码文件保持 ASCII，CRLF 换行。
9. 如果本机没有 IAR 命令行，至少执行 `rg` 检查和 `git diff --check`。
10. 完成后报告改了哪些文件、默认模式是什么、舵机是否会自动动作。

建议执行者最终给出的验证命令：

```powershell
rg -n "leg_config|LEG_MODE|control_leg_set" project/code project/iar/project_config docs
rg -n "safe_deg|neutral_deg" project/code docs
rg -n "APP_SERVO_TEST_ENABLE" project/code/app_config.h
git diff --check
```

## 12. 当前建议

下一步先实现 `leg_config` 的静态配置和只读接口，不急着做姿态闭环。只要 `height/pitch/roll` 三个混控方向被验证正确，后面 IMU 姿态补偿和四执行器联调会顺很多。
