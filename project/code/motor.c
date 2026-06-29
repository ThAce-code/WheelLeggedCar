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

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "sysint/cy_sysint.h"
#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_pwm.h"
#include "bldc_config.h"
#include "move_filter.h"
#include "board_gpio.h"
#include "board_adc.h"
#include "hall.h"
#include "motor.h"


motor_struct motor_control;                                                     // 定义电机参数对象

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PWM通道初始化
// 参数说明     void      
// 返回参数     void
// 使用示例     motor_pwm_output_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void motor_pwm_output_init(void)
{
    /******************************定义局部参数******************************/
    cy_stc_gpio_pin_config_t            pwm_pin_config;                         // 定义PWM引脚配置对象
    cy_stc_tcpwm_pwm_config_t           tcpwm_pwm_config;                       // 定义PWM定时器配置对象
    /******************************定义局部参数******************************/
    
    
    /******************************配置PWM引脚******************************/
    memset(&pwm_pin_config, 0, sizeof(pwm_pin_config));                         // 清除PWM引脚对象参数 
    
    pwm_pin_config.driveMode                     = CY_GPIO_DM_STRONG;           // 配置PWM输出引脚模式为强驱动模式
    // 初始化A相上桥引脚
    pwm_pin_config.hsiom                         = P9_0_TCPWM0_LINE24;
    Cy_GPIO_Pin_Init(get_port(A_PHASE_PIN_H), (A_PHASE_PIN_H % 8), &pwm_pin_config);
    // 初始化A相下桥引脚
    pwm_pin_config.hsiom                         = P9_1_TCPWM0_LINE_COMPL24;
    Cy_GPIO_Pin_Init(get_port(A_PHASE_PIN_L), (A_PHASE_PIN_L % 8), &pwm_pin_config);
    // 初始化B相上桥引脚
    pwm_pin_config.hsiom                         = P5_0_TCPWM0_LINE9;
    Cy_GPIO_Pin_Init(get_port(B_PHASE_PIN_H), (B_PHASE_PIN_H % 8), &pwm_pin_config);
    // 初始化B相下桥引脚
    pwm_pin_config.hsiom                         = P5_1_TCPWM0_LINE_COMPL9;
    Cy_GPIO_Pin_Init(get_port(B_PHASE_PIN_L), (B_PHASE_PIN_L % 8), &pwm_pin_config);
    // 初始化C相上桥引脚
    pwm_pin_config.hsiom                         = P5_2_TCPWM0_LINE11;
    Cy_GPIO_Pin_Init(get_port(C_PHASE_PIN_H), (C_PHASE_PIN_H % 8), &pwm_pin_config);
    // 初始化C相下桥引脚
    pwm_pin_config.hsiom                         = P5_3_TCPWM0_LINE_COMPL11;
    Cy_GPIO_Pin_Init(get_port(C_PHASE_PIN_L), (C_PHASE_PIN_L % 8), &pwm_pin_config);
    /******************************配置PWM引脚******************************/
    
    
    /****************************配置PWM定时器时钟***************************/
    // 初始化A相时钟  定时器通道24   使用16位分频器的通道1 不分频直接输出80Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS24, CY_SYSCLK_DIV_16_BIT, 2);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS24), CY_SYSCLK_DIV_16_BIT, 2, 0); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS24), CY_SYSCLK_DIV_16_BIT, 2);
    // 初始化B相时钟  定时器通道9    使用16位分频器的通道1 不分频直接输出80Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS9, CY_SYSCLK_DIV_16_BIT, 2);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS9), CY_SYSCLK_DIV_16_BIT, 2, 0); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS9), CY_SYSCLK_DIV_16_BIT, 2);
    // 初始化C相时钟  定时器通道11   使用16位分频器的通道1 不分频直接输出80Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS11, CY_SYSCLK_DIV_16_BIT, 2);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS11), CY_SYSCLK_DIV_16_BIT, 2, 0); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS11), CY_SYSCLK_DIV_16_BIT, 2);
    /****************************配置PWM定时器时钟***************************/
    
    
    /*****************************配置PWM定时器******************************/
    memset(&tcpwm_pwm_config, 0, sizeof(tcpwm_pwm_config));                     // 清除定时器配置参数
    
    tcpwm_pwm_config.pwmMode            = CY_TCPWM_PWM_MODE_DEADTIME      ;     // 当前模式为PWM死区模式 可以互补输出并且配置死区时间
    tcpwm_pwm_config.countDirection     = CY_TCPWM_COUNTER_COUNT_UP       ;     // 计数器向上计数
    tcpwm_pwm_config.cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_SET       ;     // 当比较寄存器0与计数器匹配时 拉高引脚输出
    tcpwm_pwm_config.cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR     ;     // 当比较寄存器1与计数器匹配时 拉低引脚输出
    tcpwm_pwm_config.overflowMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR     ;     // 当计数器上溢时 拉低引脚输出
    tcpwm_pwm_config.underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE ;     // 当计数器下溢时 引脚无需变化(向上计数模式不会触发计数下溢)
    tcpwm_pwm_config.deadTime           = 10                              ;     // 死区时间为10个时钟周期 80mhz时钟的10个周期为125ns
    tcpwm_pwm_config.deadTimeComp       = 10                              ;     // 互补通道死区时间也为10个时钟周期 
    tcpwm_pwm_config.runMode            = CY_TCPWM_PWM_CONTINUOUS         ;     // 持续运行 持续输出PWM信号
    tcpwm_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1      ;     // 时钟1分频 80Mhz时钟直接作用于定时器 单位时钟周期为12.5ns
    tcpwm_pwm_config.period             = (PWM_PRIOD_LOAD - 1)            ;     // 定时器周期为4000 80Mhz除以周期得到频率为20Khz
    tcpwm_pwm_config.compare0           = 0                               ;     // 比较寄存器0清空
    tcpwm_pwm_config.compare1           = 0                               ;     // 比较寄存器1清空
    tcpwm_pwm_config.enableCompare0Swap = true                            ;     // 比较器0交换使能 此处为预装载功能实现 用户的比较数据写入缓冲区 溢出事件触发主动交换缓冲区以及实际比较寄存器的值
    tcpwm_pwm_config.enableCompare1Swap = true                            ;     // 比较器1交换使能
    tcpwm_pwm_config.killMode           = CY_TCPWM_PWM_STOP_ON_KILL       ;     // 关闭定时器时停止计数
    tcpwm_pwm_config.countInputMode     = CY_TCPWM_INPUT_LEVEL            ;     // 计数事件为任意
    tcpwm_pwm_config.countInput         = 1uL                             ;     // 计数事件输入高电平

    // 初始化A相PWM通道   由于三个输出通道配置参数一致 则直接使用同一个配置对象对三个通道完成参数配置
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT24, &tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT24);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT24);
    
    // 初始化B相PWM通道   由于三个输出通道配置参数一致 则直接使用同一个配置对象对三个通道完成参数配置
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT9, &tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT9);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT9);
    
    // 初始化C相PWM通道   由于三个输出通道配置参数一致 则直接使用同一个配置对象对三个通道完成参数配置
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT11, &tcpwm_pwm_config);
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT11);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT11);
    /*****************************配置PWM定时器******************************/
    
    
    /***************************定时器计数同步配置****************************/
    // 由于三个通道都是独立的定时器 那么counter寄存器的计数值也是各自计数
    // 如果counter不同步的话 输出的三路PWM波形就无法对齐
    // 此处为最简单的伪同步方法：以最快的转速清除三路counter寄存器数据
    // 首先获取counter寄存器的地址
    uint32 pwm_ch24_count_address = 0x40580C00 + 0x00000008;                    // 通道24地址 + COUNTER寄存器偏移量
    uint32 pwm_ch09_count_address = 0x40580480 + 0x00000008;                    // 通道09地址 + COUNTER寄存器偏移量
    uint32 pwm_ch11_count_address = 0x40580580 + 0x00000008;                    // 通道11地址 + COUNTER寄存器偏移量
    // 通过指针访问寄存器地址
    volatile uint32_t *pwm_ch24_count_pointer = (volatile uint32_t *)pwm_ch24_count_address;
    volatile uint32_t *pwm_ch09_count_pointer = (volatile uint32_t *)pwm_ch09_count_address;
    volatile uint32_t *pwm_ch11_count_pointer = (volatile uint32_t *)pwm_ch11_count_address;
    // 直接对指针指向的寄存器赋值0
    *pwm_ch24_count_pointer     = 0;    
    *pwm_ch09_count_pointer     = 0;
    *pwm_ch11_count_pointer     = 0;
    /***************************定时器计数同步配置****************************/
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     三相通道占空比配置
// 参数说明     a_duty       A相占空比信息 最大值 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 参数说明     b_duty       A相占空比信息 最大值 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 参数说明     c_duty       A相占空比信息 最大值 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 返回参数     void
// 使用示例     motor_duty_set(200, 300, 400);
// 备注信息     更新比较器并且将计数器清零
//-------------------------------------------------------------------------------------------------------------------
void motor_duty_set(uint16 a_duty, uint16 b_duty, uint16 c_duty)
{
    // 由于三个通道都是独立的定时器 那么counter寄存器的计数值也是各自计数
    // 如果counter不同步的话 输出的三路PWM波形就无法对齐
    // 此处为最简单的伪同步方法：以最快的转速清除三路counter寄存器数据
    // 首先获取counter寄存器的地址
    uint32 pwm_ch24_count_address = 0x40580C00 + 0x00000008;                    // 通道24地址 + COUNTER寄存器偏移量
    uint32 pwm_ch09_count_address = 0x40580480 + 0x00000008;                    // 通道09地址 + COUNTER寄存器偏移量
    uint32 pwm_ch11_count_address = 0x40580580 + 0x00000008;                    // 通道11地址 + COUNTER寄存器偏移量
    // 通过指针访问寄存器地址
    volatile uint32_t *pwm_ch24_count_pointer = (volatile uint32_t *)pwm_ch24_count_address;
    volatile uint32_t *pwm_ch09_count_pointer = (volatile uint32_t *)pwm_ch09_count_address;
    volatile uint32_t *pwm_ch11_count_pointer = (volatile uint32_t *)pwm_ch11_count_address;
    // 中心对齐的PWM实现方式是通过两个比较器比较双边的值 counter寄存器仅向上计数
    // 当counter计数值等于比较器1的值时，拉高输出引脚
    // 当counter计数值等于比较器2的值时，拉低输出引脚
    TCPWM0_GRP0_CNT24->unCC0.u32Register = (PWM_PRIOD_LOAD - a_duty) / 2;  // 配置A相通道比较器1的值
    TCPWM0_GRP0_CNT24->unCC1.u32Register = (PWM_PRIOD_LOAD + a_duty) / 2;  // 配置A相通道比较器2的值
    TCPWM0_GRP0_CNT9->unCC0.u32Register  = (PWM_PRIOD_LOAD - b_duty) / 2;  // 配置B相通道比较器1的值
    TCPWM0_GRP0_CNT9->unCC1.u32Register  = (PWM_PRIOD_LOAD + b_duty) / 2;  // 配置B相通道比较器2的值
    TCPWM0_GRP0_CNT11->unCC0.u32Register = (PWM_PRIOD_LOAD - c_duty) / 2;  // 配置C相通道比较器1的值
    TCPWM0_GRP0_CNT11->unCC1.u32Register = (PWM_PRIOD_LOAD + c_duty) / 2;  // 配置C相通道比较器2的值
    // 直接对指针指向的寄存器赋值0
    *pwm_ch24_count_pointer     = 0;    
    *pwm_ch09_count_pointer     = 0;
    *pwm_ch11_count_pointer     = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     三相通道使能选择
// 参数说明     a_channel       0：关闭A相输出通道  1：开启A相输出通道  
// 参数说明     b_channel       0：关闭B相输出通道  1：开启B相输出通道  
// 参数说明     c_channel       0：关闭C相输出通道  1：开启C相输出通道  
// 返回参数     void
// 使用示例     motor_channel_set(1,0,1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void motor_channel_set(uint8 a_channel, uint8 b_channel, uint8 c_channel)
{     
    if(a_channel)                                                               // 当A相通道使能则开启通道
    {
        TCPWM0_GRP0_CNT24->unCTRL.u32Register       |= 0x80000000;              // 使能定时器
        TCPWM0_GRP0_CNT24->unTR_CMD.u32Register      = 0x00000004;              // 启动计数
    }
    else                                                                        // 否则关闭A相通道使能
    {
        TCPWM0_GRP0_CNT24->unCTRL.u32Register       &= 0x7FFFFFFF;              // 关闭A相通道使能
    }   
    if(b_channel)                                                               // 当B相通道使能则开启通道
    {                                                                             
        TCPWM0_GRP0_CNT9->unCTRL.u32Register        |= 0x80000000;              // 使能定时器 
        TCPWM0_GRP0_CNT9->unTR_CMD.u32Register       = 0x00000004;              // 启动计数 
    }                                                                             
    else                                                                        // 否则关闭B相通道使能
    {                                                                             
        TCPWM0_GRP0_CNT9->unCTRL.u32Register        &= 0x7FFFFFFF;              // 关闭B相通道使能 
    }
    if(c_channel)                                                               // 当C相通道使能则开启通道
    {                                                                               
        TCPWM0_GRP0_CNT11->unCTRL.u32Register       |= 0x80000000;              // 使能定时器  
        TCPWM0_GRP0_CNT11->unTR_CMD.u32Register      = 0x00000004;              // 启动计数  
    }                                                                               
    else                                                                        // 否则关闭C相通道使能
    {                                                                               
        TCPWM0_GRP0_CNT11->unCTRL.u32Register       &= 0x7FFFFFFF;              // 关闭C相通道使能  
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启A相以及B相下桥MOS 关闭B相上桥以及C相MOS A相输出占空比波形
// 参数说明     duty        占空比信息 PWM_PRIOD_LOAD (motor.h头文件有宏定义)
// 返回参数     void
// 使用示例     mos_q1q4_open(200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_q1q4_open(uint16 duty)
{
    motor_channel_set(1, 1, 0);         
    motor_duty_set(duty, 0, 0);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启A相以及C相下桥MOS 关闭C相上桥以及B相MOS A相输出占空比波形
// 参数说明     duty        占空比信息 PWM_PRIOD_LOAD (motor.h头文件有宏定义)        
// 返回参数     void
// 使用示例     mos_q1q6_open(200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_q1q6_open(uint16 duty)
{
    motor_channel_set(1, 0, 1);
    motor_duty_set(duty, 0, 0);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启B相以及A相下桥MOS 关闭A相上桥以及C相MOS B相输出占空比波形
// 参数说明     duty        占空比信息 PWM_PRIOD_LOAD (motor.h头文件有宏定义)        
// 返回参数     void
// 使用示例     mos_q3q2_open(200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_q3q2_open(uint16 duty)
{
    motor_channel_set(1, 1, 0);
    motor_duty_set(0, duty, 0);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启B相以及C相下桥MOS 关闭C相上桥以及A相MOS B相输出占空比波形
// 参数说明     duty        占空比信息 PWM_PRIOD_LOAD (motor.h头文件有宏定义)        
// 返回参数     void
// 使用示例     mos_q3q6_open(200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_q3q6_open(uint16 duty)
{
    motor_channel_set(0, 1, 1);
    motor_duty_set(0, duty, 0);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启C相以及A相下桥MOS 关闭A相上桥以及B相MOS C相输出占空比波形
// 参数说明     duty        占空比信息 PWM_PRIOD_LOAD (motor.h头文件有宏定义)        
// 返回参数     void
// 使用示例     mos_q5q2_open(200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_q5q2_open(uint16 duty)
{
    motor_channel_set(1, 0, 1);
    motor_duty_set(0, 0, duty);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开启C相以及B相下桥MOS 关闭B相上桥以及A相MOS C相输出占空比波形
// 参数说明     duty        占空比信息 PWM_PRIOD_LOAD (motor.h头文件有宏定义)        
// 返回参数     void
// 使用示例     mos_q5q4_open(200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_q5q4_open(uint16 duty)
{
    motor_channel_set(0, 1, 1);
    motor_duty_set(0, 0, duty);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     关闭所有MOS
// 参数说明     void        
// 返回参数     void
// 使用示例     mos_close();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mos_close(void)
{
    motor_duty_set(0, 0, 0);
    motor_channel_set(1, 1, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     BLDC六步换相输出
// 参数说明     hall_now        当前霍尔值
// 返回参数     void
// 使用示例     bldc_output(1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void bldc_output(uint8 hall_now)
{       
    uint16 output_duty      = 0;                     
    int8 hall_output        = 0;

    // 当使能信号有效且未触发硬件过流保护时 执行对应输出计算 否则直接关闭输出
    if(MOTOR_ENABLE == motor_control.en_status && 0 == motor_control.motor_locked)                                 
    {
        switch(motor_control.battery_state)                                     // 根据电池状态 执行不同的输出限制条件
        {
            case BATTERY_HEALTH :                                               // 当电池电压正常时 直接输出占空比
            {
                output_duty = func_limit_ab(motor_control.motor_duty - motor_control.motor_duty_offset, 0, PWM_PRIOD_LOAD / 100 * 99);                         
                break;
            }
            case BATTERY_WARNING:                                               // 当电池电压报警时 限制最高占空比为50%
            {
                if(motor_control.motor_duty > PWM_PRIOD_LOAD / 2)               
                {
                    output_duty = PWM_PRIOD_LOAD / 2;
                }
                else
                {
                    output_duty = motor_control.motor_duty - motor_control.motor_duty_offset;
                }
                break;
            }
            case BATTERY_ERROR  :                                               // 当电池电压异常时 关闭输出
            {
                output_duty = 0; 
                hall_now = 0;
                hall_output = 0;
                break;
            }
        }                                                
          
        if(0 != hall_now)                                                       // 当霍尔相位有效时执行相位偏移
        {
            // 用户设置参数为电机换相角度 而输出只能是固定角度 但结合延迟换相可以实现任意角度换相
            // 单相霍尔电角度是60° 所以此处先取有多少相：X = (motor_control.motor_control_angle / 60)
            // 当不满足完整相位时则直接算一整个相位 (motor_control.motor_control_angle % 60) == 0 ? X : X +1
            
            hall_output = (motor_control.motor_control_angle / 60);
            hall_output += ((motor_control.motor_control_angle % 60) > 0 ? 1 : 0);
            
            if(FORWARD == motor_control.motor_set_dir)                          // 设置电机正转时执行正转的偏移方向
            {
               // 加上实际霍尔的位置 可以得出需要输出的霍尔位置是多少
               hall_output = hall_hw_order_transition[hall_now] + hall_output;
               
               if(6 < hall_output)                                              // 当输出超过最大位置时 减去最大位置实现循环
               {
                    hall_output -= 6; 
               }
               
               hall_output = hall_hw_order[hall_output];                        // 获取对应位置的霍尔编码（上面是位置偏移计算 计算完成应该根据位置查询正确相位）
            }
            else                                                                // 设置电机反转时执行反转的偏移方向
            {
               // 减去实际霍尔的位置 可以得出需要输出的霍尔位置是多少
               hall_output = hall_hw_order_transition[hall_now] - hall_output;
            
               if(1 > hall_output)
               {
                  hall_output += 6;
               }
               hall_output = hall_hw_order[hall_output];  
            }
        }
    }
    // 根据计算好的输出相位调用对应的输出函数 使电机旋转至指定位置
    switch(hall_output)
    {
        case 1:     mos_q5q2_open(output_duty);    	break;	// 1
        case 2:     mos_q1q4_open(output_duty);    	break;	// 2  
        case 3:     mos_q5q4_open(output_duty);    	break;	// 3  
        case 4:     mos_q3q6_open(output_duty);		break;	// 4 
        case 5:     mos_q3q2_open(output_duty);		break;	// 5 
        case 6:     mos_q1q6_open(output_duty);    	break;	// 6  
        default:    mos_close();			        break;  
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机转速输出初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_speed_output_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_output_init(void)
{
    /******************************定义局部参数******************************/
    cy_stc_gpio_pin_config_t  output_pin_config         = {0};                  // 定义转速输出引脚参数对象
    cy_stc_tcpwm_pwm_config_t output_pwm_config;                                // 定义转速输出PWM参数对象
    /******************************定义局部参数******************************/
    
    
    /****************************配置转速输出引脚****************************/
    output_pin_config.driveMode                     = CY_GPIO_DM_STRONG;        // 输出引脚模式为强驱动模式
    output_pin_config.hsiom                         = P10_2_TCPWM0_LINE30;      // 输出引脚复用为定时器0的30通道输出
    Cy_GPIO_Pin_Init(get_port(SPEED_OUTPUT_PWM_PIN), (SPEED_OUTPUT_PWM_PIN % 8), &output_pin_config);
    /****************************配置转速输出引脚****************************/
    
    
    /*************************配置转速输出定时器时钟*************************/
    // 初始化转速输出定时器时钟  定时器通道30   使用16位分频器的通道0 80Mhz时钟被10分频输出到定时器(8Mhz)
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS30, CY_SYSCLK_DIV_16_BIT, 1);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS30), CY_SYSCLK_DIV_16_BIT, 1, 9); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS30), CY_SYSCLK_DIV_16_BIT, 1);
    /*************************配置转速输出定时器时钟*************************/
    
    /**************************配置转速输出定时器****************************/
    memset(&output_pwm_config, 0, sizeof(output_pwm_config));                   // 清除定时器配置参数
                                                                                
    output_pwm_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_8      ;    // 时钟8分频 8Mhz时钟被分频为1Mhz时钟
    output_pwm_config.pwmMode            = CY_TCPWM_PWM_MODE_PWM           ;    // 当前模式为PWM模式 
    output_pwm_config.countDirection     = CY_TCPWM_COUNTER_COUNT_UP       ;    // 计数器向上计数 
    output_pwm_config.cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR     ;    // 当比较寄存器0与计数器匹配时 拉低引脚输出
    output_pwm_config.cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE ;    // 当比较寄存器1与计数器匹配时 引脚无需变化
    output_pwm_config.overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET       ;    // 当计数器上溢时 拉高引脚输出
    output_pwm_config.underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE ;    // 当计数器下溢时 引脚无需变化(向上计数模式不会触发计数下溢)
    output_pwm_config.runMode            = CY_TCPWM_PWM_CONTINUOUS         ;    // 持续运行 持续输出PWM信号
    output_pwm_config.period             = 0                               ;    // 定时器周期为0 根据转速修改周期值
    output_pwm_config.compare0           = 0                               ;    // 定时器比较值为0 输出脉冲通常为50%占空比 修改周期值时将配置为 period / 2
    output_pwm_config.enableCompare0Swap = true                            ;    // 比较器0交换使能 此处为预装载功能实现 用户的比较数据写入缓冲区 溢出事件触发主动交换缓冲区以及实际比较寄存器的值
    output_pwm_config.enablePeriodSwap   = true                            ;    // 周期寄存器交换使能
    output_pwm_config.killMode           = CY_TCPWM_PWM_STOP_ON_KILL       ;    // 关闭定时器时停止计数
    output_pwm_config.countInputMode     = CY_TCPWM_INPUT_LEVEL            ;    // 计数事件为任意
    output_pwm_config.countInput         = 1uL                             ;    // 计数事件输入高电平
                                                  
    // 初始化转速输出定时器通道
    Cy_Tcpwm_Pwm_Init(TCPWM0_GRP0_CNT30, &output_pwm_config);                   
    Cy_Tcpwm_Pwm_Enable(TCPWM0_GRP0_CNT30);                                     
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT30);                                   
    /**************************配置转速输出定时器****************************/  
                                                                           
    gpio_init(SPEED_OUTPUT_DIR_PIN, GPO, 0, GPO_PUSH_PULL);                     // 最后初始化方向输出引脚 
}   

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机转速信息输出
// 参数说明     void
// 返回参数     void
// 使用示例     motor_speed_output();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_output(void)
{    
    if(100 < func_abs(motor_control.motor_speed_filter))                               // 当电机转速大于100RPM时(最小可检测转速为152.59RPM)输出携带转速信息的PWM脉冲
    {   // 转速为频率数据 设置的是周期值 所以此处赋值方式为 时钟/转速(频率)
        TCPWM0_GRP0_CNT30->unPERIOD_BUFF.u32Register = (1000000 / (func_abs(motor_control.motor_speed_filter)));
        // 占空比为50%
        TCPWM0_GRP0_CNT30->unCC0_BUFF.u32Register = (1000000 / (func_abs(motor_control.motor_speed_filter))) / 2;
        // 启动比较器交换 将在下次计数溢出时交换缓冲器和比较器的值
        TCPWM0_GRP0_CNT30->unTR_CMD.u32Register |= 0x11;                            

        motor_control.run_state = MOTOR_RUN;                                    // 电机旋转则更改电机状态为运行
    }
    else                                                                        // 否则清除所有输出
    {
        TCPWM0_GRP0_CNT30->unCOUNTER.u32Register = 0;
        TCPWM0_GRP0_CNT30->unPERIOD.u32Register = 1000;
        TCPWM0_GRP0_CNT30->unCC0.u32Register = 0;  
        motor_control.run_state = MOTOR_STOP;                                   // 电机未旋转则更改电机状态为停止
    }
    gpio_set_level(SPEED_OUTPUT_DIR_PIN, motor_control.motor_now_dir);          // 根据当前旋转方向设置转速输出方向
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_parameter_init();
// 备注信息     此处将完成电机控制通道初始化、转速输出通道初始化、基本控制参数配置
//-------------------------------------------------------------------------------------------------------------------
void motor_parameter_init(void)
{
    motor_pwm_output_init();                                                    // 电机通道PWM初始化
    motor_speed_output_init();                                                  // 转速信息输出通道初始化
    move_filter_init(&speed_filter);                                            // 转速滑动滤波器初始化
    
    motor_control.en_status     = (motor_en_state_enum)gpio_get_level(EN_PIN);  // 读取当前使能状态
    motor_control.run_model     = USER_INPUT;                                   // 初始默认为用户输入模式
    motor_control.brake_flag    = 0;                                            // 刹车标志清除
    motor_control.motor_control_angle = BLDC_MOTOR_ANGLE;                       // 电机换相角度初始配置
    
}










