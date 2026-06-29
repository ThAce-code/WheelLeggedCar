/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          motor
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-18       pudding            first version
********************************************************************************************************************/

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"

//#define MOTOR_SPEED_OUT_PIN (ATOM1_CH2_P21_4)   // 电机旋转速度输出引脚
//#define MOTOR_DIR_OUT_PIN   (P21_5          )   // 电机运行方向输出引脚

#define FPWM                (uint16)(20000)     // PWM频率
#define PWM_PRIOD_LOAD      (uint16)(4000)      // PWM周期装载值

#define COMMUTATION_TIMEOUT (5000           )   // 换相超时时间  单位：10us
#define POLEPAIRS           (1              )   // 电机极对数
#define PWM_PIT_NUM         (60*FPWM        )


// 以下引脚为实际输入输出引脚
#define A_PHASE_PIN_H           (P09_0)    // 电机A相上桥控制引脚
#define A_PHASE_PIN_L           (P09_1)    // 电机A相下桥控制引脚
#define B_PHASE_PIN_H           (P05_0)    // 电机B相上桥控制引脚
#define B_PHASE_PIN_L           (P05_1)    // 电机B相下桥控制引脚
#define C_PHASE_PIN_H           (P05_2)    // 电机B相上桥控制引脚
#define C_PHASE_PIN_L           (P05_3)    // 电机B相下桥控制引脚

#define SPEED_OUTPUT_PWM_PIN    (P10_2)   
#define SPEED_OUTPUT_DIR_PIN    (P10_3)

typedef enum
{
    REVERSE,                                    // 反转
    FORWARD,                                    // 正转
}motor_dir_enum;

typedef enum
{
    MOTOR_ENABLE,                               // 驱动使能
    MOTOR_DISABLE,                              // 驱动关闭
}motor_en_state_enum;

typedef enum
{
    MOTOR_STOP,                                 // 电机未运行
    MOTOR_RUN,                                  // 电机正在运行
}motor_run_state_enum;

typedef enum
{
    USER_INPUT,                                 // 用户输入模式    通过用户输入的PWM占空比和DIR信号控制电机
    BOARD_POTENTIOMET,                          // 板载电位器模式  使用驱动板上的电位器控制转速 方向由DIR引脚控制
}motor_run_model_enum;

typedef enum
{
     BATTERY_HEALTH,                            // 电池电压正常    
     BATTERY_WARNING,                           // 电池低压警告
     BATTERY_ERROR                              // 电池电压异常
}battery_state_enum;

typedef struct
{
    motor_en_state_enum     en_status;          // 电机使能状态
    uint8                   brake_flag;         // 电机刹车标志位          1：触发刹车  0：未触发刹车
    motor_run_model_enum    run_model;          // 电机控制模式           
    motor_run_state_enum    run_state;          // 电机运行状态
    uint8                   motor_locked;       // 电机是否过流保护        1：过流保护  0：未过流保护
    uint16                  motor_duty_offset;  // 电机占空比偏差          用于限制输出功率防止烧板
    uint16                  motor_duty;         // 电机当前占空比
    motor_dir_enum          motor_set_dir;      // 电机当前设置方向        
    motor_dir_enum          motor_now_dir;      // 电机当前旋转方向        
    int32                   motor_speed;        // 电机当前转速            转速单位：RPM   
    int32                   motor_speed_filter; // 电机当前转速(滤波)      转速单位：RPM   
    float                  motor_power_num;     // 电机当前运行功率
    uint8                   hall_value_now;     // 当前霍尔值
    uint8                   hall_value_last;    // 上一次霍尔值
    battery_state_enum      battery_state;      // 电池当前状态
    uint8                   motor_control_angle;// 电机控制角度(默认110) 范围：60 - 180 角度越大 转速越快 力矩越小
}motor_struct;


extern motor_struct motor_control;


void motor_parameter_init       (void);
void motor_speed_output         (void);
void bldc_output                (uint8 hall_now);



#endif