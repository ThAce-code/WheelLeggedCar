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
* 文件名称          board_gpio
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

#include "bldc_config.h"
#include "motor.h"
#include "board_gpio.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     板载GPIO初始化
// 参数说明     void
// 返回参数     void
// 使用示例     board_gpio_init();
// 备注信息     初始化驱动板上的所有普通GPIO引脚
//-------------------------------------------------------------------------------------------------------------------
void board_gpio_init(void)
{
    gpio_init(LED_RUN_PIN   , GPO, 1, GPO_PUSH_PULL);           // 初始化运行指示灯
    gpio_init(LED_ERR_PIN   , GPO, 1, GPO_PUSH_PULL);           // 初始化故障指示灯
    gpio_init(LED_MODEL_PIN , GPO, 1, GPO_PUSH_PULL);           // 初始化模式指示灯
    gpio_init(EN_PIN        , GPI, 0, GPI_PULL_DOWN);           // 初始化使能开关
    gpio_init(EN_BREAK_PIN  , GPI, 0, GPI_PULL_DOWN);           // 初始化刹车检测引脚
    gpio_init(MODEL_KEY     , GPI, 0, GPI_PULL_DOWN);           // 初始化模式按键
    gpio_init(HALL_OR_SPI   , GPO, 0, GPO_PUSH_PULL);           // 初始化霍尔接口选择引脚
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     板载LED输出
// 参数说明     out_period       当前函数调用周期 单位秒 用于闪烁频率判断
// 返回参数     void
// 使用示例     board_led_output(0.05);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void board_led_output(float out_period)
{
    static float battery_warning_period = 0;

    motor_control.en_status = (motor_en_state_enum)gpio_get_level(EN_PIN);      // 读取当前使能开关状态
    
    if((gpio_get_level(EN_BREAK_PIN) && motor_control.en_status == MOTOR_ENABLE && BLDC_BRAKE_ENABLE) || motor_control.battery_state == BATTERY_ERROR || motor_control.motor_locked == 1)
    {   
        // 满足以下条件将开启故障灯：
        // 1.使能开关有效且刹车信号有效
        // 2.电池状态异常 可能电压过低
        // 3.电机过流锁定 
        gpio_set_level(LED_ERR_PIN, 0);                         // 开启故障灯 
    }
    else
    {
        if(motor_control.battery_state == BATTERY_WARNING)      // 当电池电压报警时，故障灯闪烁提醒
        {
            battery_warning_period += out_period;               // 积分当前函数调用周期
            if(battery_warning_period > 0.1)                    // 0.1秒翻转一次故障灯
            {
               gpio_toggle_level(LED_ERR_PIN);                  
               battery_warning_period -= 0.1;
            }
        }
        else
        {
            gpio_set_level(LED_ERR_PIN, 1);                     // 一切正常则关闭故障灯
        }
    }
    
    if(motor_control.run_state == MOTOR_RUN && motor_control.en_status == MOTOR_ENABLE && motor_control.motor_locked == 0)
    {
        gpio_set_level(LED_RUN_PIN, 0);                         // 电机运行时开启运行灯
    }
    else
    {
        gpio_set_level(LED_RUN_PIN, 1);                         // 电机未运行时关闭运行灯
    }
    
    if(motor_control.run_model)                                 // 根据运行模式修改模式灯
    {
        gpio_set_level(LED_MODEL_PIN, 1);                       // 板载电位器模式
    }
    else
    {
        gpio_set_level(LED_MODEL_PIN, 0);                       // 用户输入模式
    }
    
    
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     板载模式按键扫描
// 参数说明     void       
// 返回参数     void
// 使用示例     board_key_scan();
// 备注信息     用于切换电机运行模式  
//-------------------------------------------------------------------------------------------------------------------
void board_key_scan(void)
{
    static uint8 key_status = 1;                                // 当前按键状态
    
    static uint8 key_last_status;                               // 上一次按键状态
    
    key_last_status = key_status;                               // 保存按键状态
    
    key_status = gpio_get_level(MODEL_KEY);                     // 读取当前按键状态
    
    if(key_status && !key_last_status)                          // 检测松开按键的动作
    {
        if(motor_control.run_model == USER_INPUT)               // 按键触发则切换运行模式 
        {
            motor_control.run_model = BOARD_POTENTIOMET;        // 更改电机运行模式为板载电位器控制
        }
        else
        {
            motor_control.run_model = USER_INPUT;               // 更改电机运行模式为用户输入信号控制
        }
    }
}









