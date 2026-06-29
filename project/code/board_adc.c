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
* 文件名称          board_adc
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
#include "zf_common_function.h"
#include "board_adc.h"

adc_struct adc_information;             // 定义ADC参数对象

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC采集初始化
// 参数说明     void
// 返回参数     void
// 使用示例     adc_collection_init();
// 备注信息     初始化驱动板上的所有ADC通道
//-------------------------------------------------------------------------------------------------------------------
void adc_collection_init(void)
{
    adc_init(A_PHASE_PORT               , ADC_GATHER_RESOLUTION);               // 初始化A相电流采集端口
    adc_init(B_PHASE_PORT               , ADC_GATHER_RESOLUTION);               // 初始化B相电流采集端口
    adc_init(C_PHASE_PORT               , ADC_GATHER_RESOLUTION);               // 初始化C相电流采集端口
    adc_init(BUS_PHASE_PORT             , ADC_GATHER_RESOLUTION);               // 初始化母线电流采集端口
    adc_init(BOARD_POTENTIOMET_PORT     , ADC_GATHER_RESOLUTION);               // 初始化板载电位器采集端口
    adc_init(BATTERY_PHASE_PORT         , ADC_GATHER_RESOLUTION);               // 初始化电源电压采集端口
    adc_init(V_REFERENCE                , ADC_GATHER_RESOLUTION);               // 初始化参考电压采集端口
    
    
    adc_mean_filter_convert(A_PHASE_PORT                , 10);                  // 丢弃10次A相电流采集端口的值
    adc_mean_filter_convert(B_PHASE_PORT                , 10);                  // 丢弃10次B相电流采集端口的值
    adc_mean_filter_convert(C_PHASE_PORT                , 10);                  // 丢弃10次C相电流采集端口的值
    adc_mean_filter_convert(BUS_PHASE_PORT              , 10);                  // 丢弃10次母线电流采集端口的值
    adc_mean_filter_convert(BOARD_POTENTIOMET_PORT      , 10);                  // 丢弃10次板载电位器采集端口的值
    adc_mean_filter_convert(BATTERY_PHASE_PORT          , 10);                  // 丢弃10次电源电压采集端口的值
    adc_mean_filter_convert(V_REFERENCE                 , 10);                  // 丢弃10次参考电压采集端口的值
    
    adc_information.voltage_bus_filter  = 0; 
    
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电池电压检查
// 参数说明     void
// 返回参数     battery_state_enum  
// 使用示例     battery_check();
// 备注信息     返回当前电压对应的电池状态
//-------------------------------------------------------------------------------------------------------------------
battery_state_enum battery_check(void)
{
    battery_state_enum battery_state = BATTERY_ERROR;    
#if BATTERY_PROTECT
    static uint32 error_count_num = 0;
    static uint32 warning_count_num = 0;
    for(int i = 1; i <= 6; i ++)                                               // 检测电池电压是否满足锂电池电压范围 并且检测电压是否充足
    {
        if(adc_information.battery_voltage > (BATTERY_PROTECT_VALUE_MIN * i) && adc_information.battery_voltage < (BATTERY_PROTECT_VALUE_MAX * i))
        {
            if(adc_information.battery_voltage > (BATTERY_PROTECT_VALUE_MIN * i) && adc_information.battery_voltage < (BATTERY_WARNING_VALUE * i))
            {
                battery_state = BATTERY_WARNING;                                // 电池电压高于最小阈值但低于报警阈值
            }
            else
            {
                battery_state = BATTERY_HEALTH;                                 // 电池电压正常
                error_count_num = 0;
                warning_count_num = 0;
            }
            break;
        }
    }
    if(battery_state == BATTERY_WARNING && warning_count_num ++ < 500)
    {
         battery_state = BATTERY_HEALTH;                                        // 连续检测500次都低于报警阈值才认为应当报警
    }
    if(battery_state == BATTERY_ERROR && error_count_num ++ < 500)
    {
         battery_state = BATTERY_WARNING;                                       // 连续检测500次都低于最小阈值才直接关闭电源
    }
#else
    battery_state = BATTERY_HEALTH;                                             // 电池电压正常
#endif
    return battery_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC读取
// 参数说明     void
// 返回参数     void  
// 使用示例     adc_read();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void adc_read(void)
{
    uint16 adc_value_a_phase        = 0;
    uint16 adc_value_b_phase        = 0;
    uint16 adc_value_c_phase        = 0;
    uint16 adc_value_bus_phase      = 0;
    uint16 adc_value_battery_phase  = 0;
    uint16 adc_value_v_reference    = 0;
    static uint32 lock_count       = 0;
    static uint32 power_lock_count  = 0;

    adc_value_a_phase               = adc_convert(A_PHASE_PORT          ) - adc_information.current_a_offset;                   // 获取A相电流采集端口的值
    adc_value_b_phase               = adc_convert(B_PHASE_PORT          ) - adc_information.current_b_offset;                   // 获取B相电流采集端口的值
    adc_value_c_phase               = adc_convert(C_PHASE_PORT          ) - adc_information.current_c_offset;                   // 获取C相电流采集端口的值
    adc_value_bus_phase             = adc_convert(BUS_PHASE_PORT        ) - adc_information.voltage_bus_offset;                 // 获取母线电流采集端口的值
    
    
    adc_value_v_reference           = adc_mean_filter_convert(V_REFERENCE           , 5);                                      // 获取参考电压采集端口的值
    adc_value_battery_phase         = adc_mean_filter_convert(BATTERY_PHASE_PORT    , 5);                                      // 获取电源电压采集端口的值
    adc_information.current_board   = adc_mean_filter_convert(BOARD_POTENTIOMET_PORT, 5);                                      // 获取板载电位器电压
      
    adc_information.current_a       = (float)(adc_value_a_phase   - adc_value_v_reference)   * PHASE_CURRENT_TRANSITION_VALUE;                                // 计算A相电流
    adc_information.current_b       = (float)(adc_value_b_phase   - adc_value_v_reference)   * PHASE_CURRENT_TRANSITION_VALUE;                                // 计算B相电流
    adc_information.current_c       = (float)(adc_value_c_phase   - adc_value_v_reference)   * PHASE_CURRENT_TRANSITION_VALUE;                                // 计算C相电流
    adc_information.voltage_bus     = (float)(adc_value_bus_phase - adc_value_v_reference)   * BUS_CURRENT_TRANSITION_VALUE;                                  // 计算母线电流
    adc_information.battery_voltage = (float)adc_value_battery_phase / 4096 * 3.3 * 11 * BATTERY_OFFSET_VALUE;                  // 计算电池电压
    adc_information.v_reference     = adc_value_v_reference;                    // 保存参考电压采样值
    
    adc_information.voltage_bus_filter = adc_information.voltage_bus * 0.01 + adc_information.voltage_bus_filter * 0.99;         // 母线电流滤波 获得较为稳定的母线电流       
    
    if(motor_control.motor_duty == 0) adc_information.voltage_bus_filter_offset = adc_information.voltage_bus_filter;    
    
    adc_information.voltage_bus_filter = adc_information.voltage_bus_filter - adc_information.voltage_bus_filter_offset;
    
    adc_information.voltage_bus_filter = func_limit_ab(adc_information.voltage_bus_filter, 0, adc_information.voltage_bus_filter);
    
    if(BATTERY_ERROR != motor_control.battery_state)
    {
        motor_control.battery_state = battery_check();                          // 根据电池电压检测电池状态
    }
    
    if(MOTOR_LOCK_ENABLE && 0 == motor_control.motor_locked)
    {
        if(MOTOR_LOCKED_VALUE < func_abs(adc_information.voltage_bus_filter))
        {
            if(++ lock_count > 10)  motor_control.motor_locked = 1;
        }
        else 
        {
            lock_count = 0;
        }
    }
    
    if(MOTOR_POWER_LOCKED_ENABLE && 0 == motor_control.motor_locked)
    {
        motor_control.motor_power_num = (func_abs(adc_information.voltage_bus_filter) * adc_information.battery_voltage);
        if(MOTOR_POWER_LOCKED_VALUE < motor_control.motor_power_num)
        {
          if(++ power_lock_count > 100){
              motor_control.motor_duty_offset = func_limit_ab(motor_control.motor_duty_offset + (uint16)((motor_control.motor_power_num - MOTOR_POWER_LOCKED_VALUE) * MOTOR_POWER_LOCKED_KP), 0, motor_control.motor_duty * 9 / 10);
          }
        }
        else 
        {
            if(motor_control.motor_duty_offset > 0)
            {
                motor_control.motor_duty_offset = func_limit_ab(motor_control.motor_duty_offset - (uint16)((MOTOR_POWER_LOCKED_VALUE - motor_control.motor_power_num) * MOTOR_POWER_LOCKED_KP), 0, motor_control.motor_duty * 9 / 10);
            }
            power_lock_count = 0;
        }
    }
    
}










