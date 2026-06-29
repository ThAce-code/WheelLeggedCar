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

#ifndef _BOARD_ADC_H_
#define _BOARD_ADC_H_

#include "zf_driver_adc.h"
#include "motor.h"

// 相电流运放的放大倍数为25.5倍   采样电阻为2mΩ  运放有参考电压1.65V 因此计算实际电流公式如下：
// I = ((ADC / 4096) * 3.3 - 1.65) / 25.5 / 0.002
// 其中ADC为实际采值，I为计算后的实际电流 简化公式得：I = (ADC - 2048) * 0.001343
#define PHASE_CURRENT_TRANSITION_VALUE (0.0158)     // 定义相电流转换系数

// 母线电流运放的放大倍数为15倍   采样电阻为2mΩ  运放有参考电压1.65V 因此计算实际电流公式如下：
// I = ((ADC / 4096) * 3.3 - 1.65) / 15 / 0.002
// 其中ADC为实际采值，I为计算后的实际电流 简化公式得：I = (ADC - 2048) * 0.001343
#define BUS_CURRENT_TRANSITION_VALUE   (0.026855)     // 定义母线电流转换系数


#define ADC_GATHER_RESOLUTION   (ADC_12BIT    )     // ADC采集分辨率
#define BOARD_POTENTIOMET_PORT  (ADC0_CH18_P07_2)   // 板载电位器读取通道
#define BATTERY_PHASE_PORT      (ADC0_CH21_P07_5)   // 电源电压读取通道
#define A_PHASE_PORT            (ADC0_CH02_P06_2 )  // A相电流读取通道
#define B_PHASE_PORT            (ADC0_CH03_P06_3 )  // B相电流读取通道
#define C_PHASE_PORT            (ADC0_CH04_P06_4 )  // C相电流读取通道
#define BUS_PHASE_PORT          (ADC0_CH05_P06_5 )  // 母线电流读取通道
#define V_REFERENCE             (ADC0_CH16_P07_0)   // 电压跟随器输出的参考电压

typedef struct
{
    uint16 current_a_offset;                         // A相电流
    uint16 current_b_offset;                         // B相电流
    uint16 current_c_offset;                         // C相电流
    uint16 voltage_bus_offset;                       // 母线电流
    
    float voltage_bus;                              // 母线电流
    float voltage_bus_filter;                       // 母线电流(滤波)
    float voltage_bus_filter_offset;                // 母线电流(校准值)      
    float current_a;                                // A相电流
    float current_b;                                // B相电流
    float current_c;                                // C相电流
    float battery_voltage;                          // 板载电位器电压
    uint16 current_board;                           // 板载电位器电压
    uint16 v_reference;                             // 电压跟随器输出的参考电压采样值
}adc_struct;

extern adc_struct adc_information;                 // ADC参数对象声明全局

void    adc_collection_init     (void);            // ADC采集初始化
void    adc_read                (void);            // ADC采值

#endif
