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
* 文件名称          user_pwm_in
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
#include "sysint/cy_sysint.h"
#include "gpio/cy_gpio.h"
#include "user_pwm_in.h"
#include "motor.h"

user_in_struct user_pwm_in_data;


void pwm_in_over_time_isr(void)
{
    if(Cy_Tcpwm_Counter_GetTC_IntrMasked(TCPWM0_GRP0_CNT18))                    // 检查是否是捕获超时中断标志位
    {
        Cy_Tcpwm_Counter_ClearTC_Intr(TCPWM0_GRP0_CNT18);                       // 清除捕获中断标志位
      
        if(gpio_get_level(PWM_IN_PIN))
        {
            user_pwm_in_data.get_duty = 1;                                      // 检测捕获IO电平状态 捕获超时并且IO是高电平则认为拉满输出
        }
        else
        {
            user_pwm_in_data.get_duty = 0;                                      // 捕获超时并且IO是低电平则认为关闭输出
        }
        user_pwm_in_data.get_dir  = gpio_get_level(DIR_IN_PIN);                 // 获取用户当前输入方向
        
        motor_control.motor_set_dir = (motor_dir_enum)user_pwm_in_data.get_dir; // 配置方向到电机控制
    }
}


void pwm_in_isr(void)
{
    if(Cy_Tcpwm_Counter_GetCC0_IntrMasked(TCPWM0_GRP1_CNT7))                    // 比较器0触发中断 代表检测到下降沿 
    {
        Cy_Tcpwm_Counter_ClearCC0_Intr(TCPWM0_GRP1_CNT7);                       // 清除比较器0触发中断 
    }
    

    if(Cy_Tcpwm_Counter_GetCC1_IntrMasked(TCPWM0_GRP1_CNT7))                    // 比较器1触发中断 代表检测到上升沿 
    {
        Cy_Tcpwm_Counter_ClearCC1_Intr(TCPWM0_GRP1_CNT7);                       // 清除比较器1触发中断 
        
        user_pwm_in_data.pwm_high_level = Cy_Tcpwm_Counter_GetCompare0(TCPWM0_GRP1_CNT7) + 2;   // 获取比较器0保存的计数值(检测到下降沿则说明保存的是高电平时长) 加2是因为后面清除计数额外用了2us
        user_pwm_in_data.pwm_period_num = Cy_Tcpwm_Counter_GetCompare1(TCPWM0_GRP1_CNT7) + 2;   // 获取比较器1保存的计数值(检测到上升沿则说明保存的是脉冲周期时长) 加2是因为后面清除计数额外用了2us
        
        Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP1_CNT7, 0);                       // 清除捕获定时器计数寄存器值
        Cy_Tcpwm_Counter_SetCounter(TCPWM0_GRP0_CNT18, 0);                      // 清除捕获超时定时器计数器值(运行到这里表示正确捕获到脉冲信号 超时则需要重新计时)
        
        if(user_pwm_in_data.pwm_high_level > user_pwm_in_data.pwm_period_num) user_pwm_in_data.pwm_high_level = 0;
        
        if(user_pwm_in_data.pwm_period_num - user_pwm_in_data.pwm_high_level > 2000)
        {
            user_pwm_in_data.pwm_high_level = 0;
            user_pwm_in_data.pwm_period_num = 0;
            user_pwm_in_data.get_duty = 0;
        }
        else
        {
            user_pwm_in_data.pwm_low_level = user_pwm_in_data.pwm_period_num - user_pwm_in_data.pwm_high_level;     // 计算低电平时长(低电平时长并没有用到，可以作为数据观察)
            user_pwm_in_data.get_duty = (float)((float)user_pwm_in_data.pwm_high_level / (float)user_pwm_in_data.pwm_period_num);   // 计算占空比 范围0-1 浮点类型
        }
        user_pwm_in_data.get_dir  = gpio_get_level(DIR_IN_PIN);                 // 获取用户当前输入方向
        
        motor_control.motor_set_dir = (user_pwm_in_data.get_dir == 1 ? REVERSE : FORWARD);  // 设置电机旋转方向
    }
}


void pwm_in_init(void)
{
    /******************************定义局部参数******************************/
    cy_stc_gpio_pin_config_t            pwm_in_pin_cfg;                         // 定义脉冲捕获引脚参数对象
    cy_stc_tcpwm_counter_config_t       pwm_in_counter_cfg;                     // 定义脉冲捕获定时器参数对象
    cy_stc_tcpwm_counter_config_t       pwm_in_over_time_cfg;                   // 定义脉冲捕获超时定时器参数对象
    cy_stc_sysint_irq_t                 pwm_in_irq_cfg;                         // 定义脉冲捕获中断参数对象
    cy_stc_sysint_irq_t                 pwm_in_over_time_irq_cfg;               // 定义脉冲捕获超时中断参数对象
    /******************************定义局部参数******************************/
    
    
    /**************************配置脉冲捕获输入引脚**************************/
    memset(&pwm_in_pin_cfg, 0 , sizeof(pwm_in_pin_cfg));                        // 清除脉冲捕获引脚对象参数
    
    pwm_in_pin_cfg.driveMode = CY_GPIO_DM_STRONG;                               // 配置引脚模式为强驱动模式
    pwm_in_pin_cfg.hsiom     = P7_6_TCPWM0_TR_ONE_CNT_IN789;                    // 配置引脚复用为定时器1通道7触发功能
    
    Cy_GPIO_Pin_Init(get_port(PWM_IN_PIN), (PWM_IN_PIN % 8), &pwm_in_pin_cfg);  // 初始化脉冲捕获引脚
    /**************************配置脉冲捕获输入引脚**************************/
    
    
    /*************************配置脉冲捕获定时器时钟*************************/
    // 初始化捕获通道时钟  定时器1通道7   使用16位分频器的通道0 10分频 输出8Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS263, CY_SYSCLK_DIV_16_BIT, 0);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS263), CY_SYSCLK_DIV_16_BIT, 0, 9); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS263), CY_SYSCLK_DIV_16_BIT, 0);
    // 初始化捕获超时通道时钟  定时器0通道18   使用16位分频器的通道0 10分频 输出8Mhz时钟到定时器 
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS18, CY_SYSCLK_DIV_16_BIT, 0);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS18), CY_SYSCLK_DIV_16_BIT, 0, 9); 
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS18), CY_SYSCLK_DIV_16_BIT, 0);
    /*************************配置脉冲捕获定时器时钟*************************/

    
    /**************************配置脉冲捕获定时器****************************/
    memset(&pwm_in_counter_cfg, 0, sizeof(pwm_in_counter_cfg));                 // 清除脉冲捕获定时器对象参数          
                                                                                
    pwm_in_counter_cfg.period             = 0xFFFF                       ;      // 定时器周期为65535                       
    pwm_in_counter_cfg.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_8   ; 	// 时钟8分频 8Mhz输入时钟被8分频为1Mhz 因此定时器计数单位为微秒
    pwm_in_counter_cfg.runMode            = CY_TCPWM_COUNTER_CONTINUOUS  ;      // 持续运行模式
    pwm_in_counter_cfg.countDirection     = CY_TCPWM_COUNTER_COUNT_UP    ;      // 向上计数 
    pwm_in_counter_cfg.debug_pause        = false                        ;      // 关闭“调试时暂停计数”
    pwm_in_counter_cfg.compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE;      // 捕获模式
    pwm_in_counter_cfg.interruptSources   = CY_TCPWM_INT_NONE            ;      // 自定义中断资源
    pwm_in_counter_cfg.capture0InputMode  = CY_TCPWM_INPUT_FALLING_EDGE  ;	// 捕获寄存器0捕获事件触发在下降沿	
    pwm_in_counter_cfg.capture0Input      = 2uL                          ;      // 选择定时器通道的复用引脚
    pwm_in_counter_cfg.capture1InputMode  = CY_TCPWM_INPUT_RISING_EDGE   ;      // 捕获寄存器1捕获事件触发在上升沿   
    pwm_in_counter_cfg.capture1Input      = 2uL                          ;      // 选择定时器通道的复用引脚
    pwm_in_counter_cfg.countInputMode     = CY_TCPWM_INPUT_LEVEL         ;      // 计数触发为水平信号
    pwm_in_counter_cfg.countInput         = 1uL                          ;      // 触发输入为持续高电平(直接计数)
    pwm_in_counter_cfg.trigger0EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH   ;      // 中断事件0配置为捕获寄存器0触发
    pwm_in_counter_cfg.trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW    ;      // 中断事件1配置为溢出时触发
    
    
    Cy_Tcpwm_Counter_Init(TCPWM0_GRP1_CNT7, &pwm_in_counter_cfg);               // 初始化定时器参数
    Cy_Tcpwm_Counter_SetCC0_IntrMask(TCPWM0_GRP1_CNT7);                         // 使能捕获寄存器0中断
    Cy_Tcpwm_Counter_SetCC1_IntrMask(TCPWM0_GRP1_CNT7);                         // 使能捕获寄存器1中断
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRP1_CNT7);                                  // 使能定时器
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP1_CNT7);                                    // 触发定时器计数
    /**************************配置脉冲捕获定时器****************************/
    
    
    /***************************配置脉冲捕获中断*****************************/
    pwm_in_irq_cfg.sysIntSrc  = tcpwm_0_interrupts_263_IRQn              ;      // 选择中断触发源为TCPWM定时器1的通道7
    pwm_in_irq_cfg.intIdx     = CPUIntIdx6_IRQn                          ;      // 选择中断触发通道为用户中断6
    pwm_in_irq_cfg.isEnabled  = true                                     ;      // 允许中断触发
    Cy_SysInt_InitIRQ(&pwm_in_irq_cfg);                                         // 初始化中断配置参数
    Cy_SysInt_SetSystemIrqVector(pwm_in_irq_cfg.sysIntSrc, pwm_in_isr);         // 配置当前中断回调函数为 hall_isr
    NVIC_SetPriority(pwm_in_irq_cfg.intIdx, 1u);                                // 配置中断优先级为1 
    NVIC_ClearPendingIRQ(pwm_in_irq_cfg.intIdx);                                // 清除当前中断的挂起状态
    NVIC_EnableIRQ(pwm_in_irq_cfg.intIdx);                                      // 使能当前中断
    /***************************配置脉冲捕获中断*****************************/

    
    /************************配置脉冲捕获超时定时器**************************/
    memset(&pwm_in_over_time_cfg, 0, sizeof(pwm_in_over_time_cfg));             // 清除脉冲捕获超时定时器对象参数       
    
    pwm_in_over_time_cfg.period             = 0xFFFF                       ;    // 定时器周期为65535                           
    pwm_in_over_time_cfg.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_8   ;    // 时钟8分频 8Mhz输入时钟被8分频为1Mhz 因此定时器计数单位为微秒
    pwm_in_over_time_cfg.runMode            = CY_TCPWM_COUNTER_CONTINUOUS  ;    // 持续运行模式
    pwm_in_over_time_cfg.countDirection     = CY_TCPWM_COUNTER_COUNT_UP    ;    // 向上计数 
    pwm_in_over_time_cfg.compareOrCapture   = CY_TCPWM_COUNTER_MODE_COMPARE;    // 比较模式
    pwm_in_over_time_cfg.countInputMode     = CY_TCPWM_INPUT_LEVEL         ;    // 选择定时器通道的复用引脚
    pwm_in_over_time_cfg.countInput         = 1uL                          ;    // 计数触发为水平信号
    pwm_in_over_time_cfg.trigger0EventCfg   = CY_TCPWM_COUNTER_OVERFLOW    ;    // 中断事件0配置为溢出时触发
    pwm_in_over_time_cfg.trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW    ;    // 中断事件1配置为溢出时触发
                                                                                   
    Cy_Tcpwm_Counter_Init(TCPWM0_GRP0_CNT18, &pwm_in_over_time_cfg);            // 初始化定时器参数 
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRP0_CNT18);                                 // 使能定时器 
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT18);                                   // 触发定时器计数 
    Cy_Tcpwm_Counter_SetTC_IntrMask(TCPWM0_GRP0_CNT18);                         // 使能计数器中断 
    /************************配置脉冲捕获超时定时器**************************/   
    
    
    /*************************配置脉冲捕获超时中断***************************/
    pwm_in_over_time_irq_cfg.sysIntSrc  = tcpwm_0_interrupts_18_IRQn     ;      // 选择中断触发源为TCPWM定时器0的通道18
    pwm_in_over_time_irq_cfg.intIdx     = CPUIntIdx6_IRQn                ;      // 选择中断触发通道为用户中断6
    pwm_in_over_time_irq_cfg.isEnabled  = true                           ;      // 允许中断触发
    Cy_SysInt_InitIRQ(&pwm_in_over_time_irq_cfg);                               // 初始化中断配置参数
    Cy_SysInt_SetSystemIrqVector(pwm_in_over_time_irq_cfg.sysIntSrc, pwm_in_over_time_isr);// 配置当前中断回调函数为 hall_isr
    NVIC_SetPriority(pwm_in_over_time_irq_cfg.intIdx, 1u);                      // 配置中断优先级为1 
    NVIC_ClearPendingIRQ(pwm_in_over_time_irq_cfg.intIdx);                      // 清除当前中断的挂起状态
    NVIC_EnableIRQ(pwm_in_over_time_irq_cfg.intIdx);                            // 使能当前中断
    /*************************配置脉冲捕获超时中断***************************/
    // 初始化输入方向引脚
    gpio_init(DIR_IN_PIN, GPI, 0, GPI_PULL_DOWN);
}































