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
* 文件名称          hall
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-19       pudding            first version
********************************************************************************************************************/
#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "gpio/cy_gpio.h"
#include "sysint/cy_sysint.h"
#include "zf_common_function.h"
#include "move_filter.h"
#include "board_gpio.h"
#include "hall.h"
#include "motor.h"

// 霍尔硬件换相序列
int8 hall_hw_order[7]                  = {0, 6, 2, 3, 1, 5, 4};        

// 霍尔硬件换相序列位置转换 
// 例：采集霍尔值为4 带入位置转换数组得出当前位置在6 对应霍尔硬件换相序列的第6个数据 
//     当前位置为6 如果我们想要驱动电机旋转则可以输出下一个位置或者上一个位置 
//     输出上一个位置则是5 带入霍尔硬件换相序列得出第5个数据值为5 则输出霍尔为5的相位
//     输出下一个位置则是1 带入霍尔硬件换相序列得出第1个数据值为6 则输出霍尔为6的相位
int8 hall_hw_order_transition[7]       = {0, 4, 2, 3, 6, 5, 1};                


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     霍尔换相超时中断 
// 参数说明     void
// 返回参数     void
// 使用示例     hall_over_time_isr();
// 备注信息     当霍尔信号超过65.535ms未出现变化则主动输出一次 用于启动电机
//-------------------------------------------------------------------------------------------------------------------
void hall_over_time_isr(void)
{
    Cy_Tcpwm_Counter_ClearTC_Intr(TCPWM0_GRP0_CNT21);                           // 清除霍尔换相超时中断标志位
    
    motor_control.hall_value_now  = gpio_get_level(HALL_A) + gpio_get_level(HALL_B) * 2 + gpio_get_level(HALL_C) * 4;   // 计算当前霍尔值
    
    bldc_output(motor_control.hall_value_now);                                  // 根据当前霍尔值主动输出一次
    
    motor_control.motor_now_dir = motor_control.motor_set_dir;                  // 电机当前旋转方向直接赋值为设置方向

    motor_control.motor_speed = 0;                                              // 电机转速清除 (由于是16位定时器 因此最小的可检测速度为152.59RPM)
    
    move_filter_calc(&speed_filter, motor_control.motor_speed);                 // 电机转速滑动滤波 使转速信息顺滑
    
    motor_control.motor_speed_filter = speed_filter.data_average;               // 获取滤波后的转速
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     霍尔延迟换相中断 
// 参数说明     void
// 返回参数     void
// 使用示例     hall_delay_isr();
// 备注信息     延迟一段时间后输出相位信息
//-------------------------------------------------------------------------------------------------------------------
void hall_delay_isr(void)
{
    Cy_Tcpwm_Counter_ClearTC_Intr(TCPWM0_GRP0_CNT20);                           // 清除霍尔延迟换相中断标志位
    
    bldc_output(motor_control.hall_value_now);                                  // 根据当前霍尔值主动输出一次
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     霍尔换相中断
// 参数说明     void
// 返回参数     void
// 使用示例     hall_isr();
// 备注信息     当三路霍尔信号任意一路电平变化时触发霍尔换相中断  
//-------------------------------------------------------------------------------------------------------------------
void hall_isr(void)
{
    uint16 hall_delay_phase_time = 0;                                           // 定义霍尔延迟换相的延迟时间
    
    static int16 motor_speed_last = 0;
    
    motor_control.hall_value_last = motor_control.hall_value_now;               // 保存上一次的霍尔值
    
    motor_control.hall_value_now  = gpio_get_level(HALL_A) + gpio_get_level(HALL_B) * 2 + gpio_get_level(HALL_C) * 4;   // 计算当前霍尔值
    
    if(motor_control.hall_value_last != motor_control.hall_value_now)           // 如果当前值与上一次值不一样代表正确换相 执行换相内容
    {
        int32 run_counter = Cy_Tcpwm_Counter_GetCounter(TCPWM0_GRP0_CNT19);     // 获取与上一次之间的换相时间(定时器时钟1Mhz,因此获取的时间单位刚好是微秒)
      
        Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT19, 0);                      // 清除单次换相定时器计数
        
        Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT19);                               // 触发定时器再次工作
        
        Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT21, 0);                      // 清除超时换相定时器计数 
        
        if(run_counter == 0)                                                    // 如果计时溢出则直接输出一次(定时器只工作一次 超过65.535ms则溢出停止)
        {       
            bldc_output(motor_control.hall_value_now);                          // 根据当前霍尔值主动输出一次
            
            motor_control.motor_now_dir = motor_control.motor_set_dir;          // 电机当前旋转方向直接赋值为设置方向
            
            motor_control.motor_speed = 0;                                      // 电机转速清除 (由于是16位定时器 因此最小的可检测速度为152.59RPM)
        }
        else
        {
            if(func_abs(motor_control.motor_speed) < 5000)                      // 当电机转速小于2000RPM时 根据霍尔值判断当前旋转方向
            {
                // 满足上一次霍尔值大于当前霍尔值且相差不超过3个相位时则认为当前电机为反转
                // 满足上一次霍尔值为1且当前霍尔值为6时则认为当前电机为反转
                // 否则认为电机正转
                if((hall_hw_order_transition[motor_control.hall_value_last] > hall_hw_order_transition[motor_control.hall_value_now] &&                
                  func_abs(hall_hw_order_transition[motor_control.hall_value_last] - hall_hw_order_transition[motor_control.hall_value_now] <= 3))|| 
                (hall_hw_order_transition[motor_control.hall_value_last] == 1 && hall_hw_order_transition[motor_control.hall_value_now] == 6))
                {
                    motor_control.motor_now_dir = REVERSE;                      // 更改电机旋转方向为反转
                }
                else
                {
                    motor_control.motor_now_dir = FORWARD;                      // 更改电机旋转方向为正转
                }
            }
            else                                                                // 当电机转速大于2000RPM时则直接使用当前设置的旋转方向 防止高速下霍尔值噪声导致转速波动
            {
                motor_control.motor_now_dir = motor_control.motor_set_dir;      // 电机当前旋转方向直接赋值为设置方向
            }
            
            if(REVERSE == motor_control.motor_now_dir)                          // 根据电机旋转方向 给滑动滤波器输入值赋予方向
            {
                run_counter = -run_counter;
            }
            
            motor_speed_last = motor_control.motor_speed;                       // 保存上次转速数据
              
            // 通过滑动滤波器的数据平均值 计算当前电机转速
            // 由于计数单位是微秒 因此电机旋转一圈的时间则是 6 * run_counter
            // 1秒钟电机旋转的圈数则是 1000000 / (6 * run_counter)
            // 1分钟电机旋转的圈数则是 60 * 1000000 / (6 * run_counter)
            // 简化后如下：
            motor_control.motor_speed = 10000000 / run_counter;                 // 计算电机转速 单位：RMP(每分钟旋转的圈数)

            if(func_abs(motor_control.motor_speed - motor_speed_last) > 5000) motor_control.motor_speed = motor_speed_last;     // 转速大幅度跳变则放弃(可能出现一次霍尔信号触发多次的情况，导致转速计算错误)
            
            if(motor_control.motor_control_angle % 60 == 0)                     // 如果完整相位换相 则可以直接输出
            {
                bldc_output(motor_control.hall_value_now);                      // 根据当前霍尔值主动输出一次
            }
            else                                                                // 否则通过定时器来实现等待一段时间后再输出
            {
                move_filter_calc(&hall_counter_filter, run_counter);            // 霍尔换相时间滤波 使延迟换相更加顺滑
                // 计算霍尔延迟换相时间 由于用户配置参数为换相角度 因此需要将角度转换为换相延迟时间
                // 霍尔单个相位电角度为60° 则先将换相角度除以60取余数 对应代码: (motor_control.motor_control_angle % 60)
                // 由于是六步换相 我们只能输出固定角度的相位 不能输出任意角度 所以只能通过延迟换相实现不同的换相角度
                // 比如电机当前角度为0° 我们需要100°换相  那么则等待电机旋转20°之后直接输出120°的相位，此时相位差为100°
                // 而等待电机旋转时间可以通过霍尔换相时间来估算本次换相的等待时长 对应代码：X * func_abs(hall_counter_filter.data_average) / 60; 其中X为预估等待角度
                // 因此我们需要计算的是延迟时间 也就是等待电机旋转的时间(上面例子的20°) 整体对应以下代码：
                hall_delay_phase_time = (60 - (motor_control.motor_control_angle % 60)) * func_abs(hall_counter_filter.data_average) / 60;    
              
                Cy_Tcpwm_Counter_SetPeriod(TCPWM0_GRP0_CNT20, hall_delay_phase_time);  //  配置定时器周期为延时时长 
                
                Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT20, 0);              // 清除当前定时器计数值
                
                Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT20);                       // 触发定时器启动计数
            }
            
        }
    }
    
    // 由于硬件霍尔同一时刻有且仅有一个霍尔相位改变状态 因此使用if else语句判断中断状态
    if(Cy_GPIO_GetInterruptStatusMasked(get_port(HALL_A), (HALL_A % 8)))        // 判断是否是A相霍尔触发中断
    {
        Cy_GPIO_ClearInterrupt(get_port(HALL_A), (HALL_A % 8));                 // 清除A相霍尔中断标志位
    }
    else if(Cy_GPIO_GetInterruptStatusMasked(get_port(HALL_B), (HALL_B % 8)))  // 判断是否是B相霍尔触发中断
    {
        Cy_GPIO_ClearInterrupt(get_port(HALL_B), (HALL_B % 8));                 // 清除B相霍尔中断标志位
    }
    else if(Cy_GPIO_GetInterruptStatusMasked(get_port(HALL_C), (HALL_C % 8)))  // 判断是否是C相霍尔触发中断
    {
        Cy_GPIO_ClearInterrupt(get_port(HALL_C), (HALL_C % 8));                 // 清除C相霍尔中断标志位
    }
    
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     霍尔初始化
// 参数说明     void
// 返回参数     void
// 使用示例     hall_init();
// 备注信息     初始化三路霍尔引脚及相关中断
//-------------------------------------------------------------------------------------------------------------------
void  hall_init(void)
{
    /******************************定义局部参数******************************/
    cy_stc_gpio_pin_config_t            hall_pin_config;                        // 定义霍尔引脚配置对象 
    cy_stc_tcpwm_counter_config_t       hall_counter_config;                    // 定义霍尔换相计数定时器对象
    cy_stc_tcpwm_counter_config_t       hall_delay_config;                      // 定义霍尔延时换相定时器对象
    cy_stc_tcpwm_counter_config_t       hall_over_time_config;                  // 定义霍尔超时换相定时器对象
    cy_stc_sysint_irq_t                 hall_irq_cfg;                           // 定义霍尔触发换相中断对象
    cy_stc_sysint_irq_t                 hall_delay_irq_cfg;                     // 定义霍尔延时换相中断对象
    cy_stc_sysint_irq_t                 hall_over_time_irq_cfg;                 // 定义霍尔超时换相中断对象
    /******************************定义局部参数******************************/
    
    
    /******************************配置HALL引脚******************************/
    memset(&hall_pin_config, 0, sizeof(hall_pin_config));                      // 清除霍尔引脚对象参数 
    
    hall_pin_config.driveMode = CY_GPIO_DM_HIGHZ;                               // 引脚模式为高阻模式 只能获取输入电平 无法输出
    hall_pin_config.intEdge   = 3;                                              // 触发方式为双边沿触发( 0:不触发  1:上升沿触发  2:下降沿触发  3:双边沿触发)
    hall_pin_config.intMask   = 1ul;                                            // 使能中断模式
    
    Cy_GPIO_Pin_Init(get_port(HALL_A), (HALL_A % 8), &hall_pin_config);         // 初始化霍尔A相引脚
    Cy_GPIO_Pin_Init(get_port(HALL_B), (HALL_B % 8), &hall_pin_config);         // 初始化霍尔B相引脚
    Cy_GPIO_Pin_Init(get_port(HALL_C), (HALL_C % 8), &hall_pin_config);         // 初始化霍尔C相引脚
    /******************************配置HALL引脚******************************/
    
    
    /******************************配置HALL中断******************************/
    hall_irq_cfg.sysIntSrc  = ioss_interrupts_gpio_dpslp_8_IRQn;                // 选择中断触发源为GPIO端口8
    hall_irq_cfg.intIdx     = CPUIntIdx7_IRQn;                                  // 选择中断触发通道为用户中断7
    hall_irq_cfg.isEnabled  = true;                                             // 允许中断触发
    
    Cy_SysInt_InitIRQ(&hall_irq_cfg);                                           // 初始化中断配置参数
    Cy_SysInt_SetSystemIrqVector(hall_irq_cfg.sysIntSrc, hall_isr);             // 配置当前中断回调函数为 hall_isr
    NVIC_SetPriority(hall_irq_cfg.intIdx, 0ul);                                 // 配置中断优先级为0 (最高优先级)
    NVIC_EnableIRQ(hall_irq_cfg.intIdx);                                        // 使能当前中断
    /******************************配置HALL中断******************************/
    
    
    /***************************配置HALL定时器时钟***************************/
    // 初始化HALL换相计数时钟  定时器通道19   使用16位分频器的通道0 10分频 输出8Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS19, CY_SYSCLK_DIV_16_BIT, 0);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS19), CY_SYSCLK_DIV_16_BIT, 0, 9); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS19), CY_SYSCLK_DIV_16_BIT, 0);
    
    // 初始化延迟换相控制时钟  定时器通道20   使用16位分频器的通道0 10分频 输出8Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS20, CY_SYSCLK_DIV_16_BIT, 0);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS20), CY_SYSCLK_DIV_16_BIT, 0, 9); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS20), CY_SYSCLK_DIV_16_BIT, 0);
    
    // 初始化超时换相控制时钟  定时器通道21   使用16位分频器的通道0 10分频 输出8Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS21, CY_SYSCLK_DIV_16_BIT, 0);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS21), CY_SYSCLK_DIV_16_BIT, 0, 9); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS21), CY_SYSCLK_DIV_16_BIT, 0);
    /***************************配置HALL定时器时钟***************************/
    
    
    /*************************配置HALL换相计数定时器*************************/
    memset(&hall_counter_config, 0, sizeof(hall_counter_config))          ;     // 清除换相计数定时器对象参数          
    
    hall_counter_config.period             = 0xffff                        ;    // 定时器周期为65535
    hall_counter_config.compare0           = 0xffff                        ;    // 定时器比较值为65535
    hall_counter_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_8    ;    // 时钟8分频 8Mhz输入时钟被8分频为1Mhz 因此定时器计数单位为微秒
    hall_counter_config.runMode            = CY_TCPWM_COUNTER_ONESHOT      ;    // 仅工作一次 溢出则停止运行
    hall_counter_config.countDirection     = CY_TCPWM_COUNTER_COUNT_UP     ;    // 向上计数
    hall_counter_config.compareOrCapture   = CY_TCPWM_COUNTER_MODE_COMPARE ;    // 比较模式
    hall_counter_config.countInputMode     = CY_TCPWM_INPUT_LEVEL          ;    // 计数事件为任意
    hall_counter_config.countInput         = 1uL                           ;    // 计数事件输入高电平
  
    Cy_Tcpwm_Counter_Init(TCPWM0_GRP0_CNT19, &hall_counter_config);             // 初始化定时器参数
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRP0_CNT19);                                 // 使能定时器
    Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT19, 0);                          // 清除定时器计数值
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT19);                                   // 触发定时器计数
    /*************************配置HALL换相计数定时器*************************/
    
    
    /*************************配置HALL延迟换相定时器*************************/
    hall_delay_config.period               = 0xffff                        ;    // 定时器周期为65535
    hall_delay_config.compare0             = 0xffff                        ;    // 定时器比较值为65535
    hall_delay_config.clockPrescaler       = CY_TCPWM_PRESCALER_DIVBY_8    ;    // 时钟8分频 8Mhz输入时钟被8分频为1Mhz 因此定时器计数单位为微秒
    hall_delay_config.runMode              = CY_TCPWM_COUNTER_ONESHOT      ;    // 仅工作一次 溢出则停止运行
    hall_delay_config.countDirection       = CY_TCPWM_COUNTER_COUNT_UP     ;    // 向上计数
    hall_delay_config.compareOrCapture     = CY_TCPWM_COUNTER_MODE_COMPARE ;    // 比较模式
    hall_delay_config.countInputMode       = CY_TCPWM_INPUT_LEVEL          ;    // 计数事件为任意
    hall_delay_config.countInput           = 1uL                           ;    // 计数事件输入高电平
    hall_delay_config.trigger0EventCfg     = CY_TCPWM_COUNTER_OVERFLOW     ;    // 中断触发事件0为溢出时触发中断
    hall_delay_config.trigger1EventCfg     = CY_TCPWM_COUNTER_OVERFLOW     ;    // 中断触发事件1为溢出时触发中断
    
    Cy_Tcpwm_Counter_Init(TCPWM0_GRP0_CNT20, &hall_delay_config);               // 初始化定时器参数
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRP0_CNT20);                                 // 使能定时器
    Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT20, 0);                          // 清除定时器计数值
    Cy_Tcpwm_Counter_SetTC_IntrMask(TCPWM0_GRP0_CNT20);                         // 使能定时器中断
    /*************************配置HALL延迟换相定时器*************************/
    
    
    /***********************配置HALL延迟换相定时器中断***********************/
    hall_delay_irq_cfg.sysIntSrc  = tcpwm_0_interrupts_20_IRQn             ;    // 选择中断触发源为定时器0的通道20
    hall_delay_irq_cfg.intIdx     = CPUIntIdx6_IRQn                        ;    // 选择中断触发通道为用户中断6
    hall_delay_irq_cfg.isEnabled  = true                                   ;    // 允许中断触发
                                                                                
    Cy_SysInt_InitIRQ(&hall_delay_irq_cfg);                                     // 初始化中断配置参数
    Cy_SysInt_SetSystemIrqVector(hall_delay_irq_cfg.sysIntSrc, hall_delay_isr); // 配置当前中断回调函数为 hall_delay_isr
    NVIC_SetPriority(hall_delay_irq_cfg.intIdx, 1u);                            // 配置中断优先级为1 
    NVIC_ClearPendingIRQ(hall_delay_irq_cfg.intIdx);                            // 清除中断请求的挂起状态
    NVIC_EnableIRQ(hall_delay_irq_cfg.intIdx);                                  // 使能当前中断
    /***********************配置HALL延迟换相定时器中断***********************/
    
    /*************************配置HALL超时换相定时器*************************/
    hall_over_time_config.period           = 0xffff                        ;    // 定时器周期为65535
    hall_over_time_config.compare0         = 0xffff                        ;    // 定时器比较值为65535
    hall_over_time_config.clockPrescaler   = CY_TCPWM_PRESCALER_DIVBY_8    ;    // 时钟8分频 8Mhz输入时钟被8分频为1Mhz 因此定时器计数单位为微秒
    hall_over_time_config.runMode          = CY_TCPWM_COUNTER_CONTINUOUS   ;    // 持续运行
    hall_over_time_config.countDirection   = CY_TCPWM_COUNTER_COUNT_UP     ;    // 向上计数
    hall_over_time_config.compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE ;    // 比较模式
    hall_over_time_config.countInputMode   = CY_TCPWM_INPUT_LEVEL          ;    // 计数事件为任意
    hall_over_time_config.countInput       = 1uL                           ;    // 计数事件输入高电平
    hall_over_time_config.trigger0EventCfg = CY_TCPWM_COUNTER_OVERFLOW     ;    // 中断触发事件0为溢出时触发中断
    hall_over_time_config.trigger1EventCfg = CY_TCPWM_COUNTER_OVERFLOW     ;    // 中断触发事件1为溢出时触发中断
    
    Cy_Tcpwm_Counter_Init(TCPWM0_GRP0_CNT21, &hall_over_time_config);           // 初始化定时器参数
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRP0_CNT21);                                 // 使能定时器
    Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT21, 0);                          // 清除定时器计数值
    Cy_Tcpwm_Counter_SetTC_IntrMask(TCPWM0_GRP0_CNT21);                         // 使能定时器中断
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT21);                                   // 触发定时器启动计数                        
    /*************************配置HALL超时换相定时器*************************/
    
    
    /***********************配置HALL超时换相定时器中断***********************/
    hall_over_time_irq_cfg.sysIntSrc  = tcpwm_0_interrupts_21_IRQn         ;    // 选择中断触发源为定时器0的通道20
    hall_over_time_irq_cfg.intIdx     = CPUIntIdx6_IRQn                    ;    // 选择中断触发通道为用户中断6
    hall_over_time_irq_cfg.isEnabled  = true                               ;    // 允许中断触发
                                                                                
    Cy_SysInt_InitIRQ(&hall_over_time_irq_cfg);                                 // 初始化中断配置参数
    Cy_SysInt_SetSystemIrqVector(hall_over_time_irq_cfg.sysIntSrc, hall_over_time_isr); // 配置当前中断回调函数为 hall_over_time_isr
    NVIC_SetPriority(hall_over_time_irq_cfg.intIdx, 1u);                        // 配置中断优先级为1 
    NVIC_ClearPendingIRQ(hall_over_time_irq_cfg.intIdx);                        // 清除中断请求的挂起状态
    NVIC_EnableIRQ(hall_over_time_irq_cfg.intIdx);                              // 使能当前中断
    /***********************配置HALL超时换相定时器中断***********************/
    
    move_filter_init(&hall_counter_filter);                                     // 霍尔计数器滑动滤波器初始化
}























