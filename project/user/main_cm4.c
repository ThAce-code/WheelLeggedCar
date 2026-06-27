/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
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
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "lsm6dsv16x_test.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// **************************** 代码区域 ****************************

int main(void)
{
    uint8 id = 0;
    uint16 timeout = 0;
    uint8 led_count = 0;

    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    
    // LED 初始化：P23.7 推挽输出，默认高电平（低电平点亮，初始熄灭）
    gpio_init(P23_7, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    // 初始化 SPI3 与 CS 引脚
    lsm6dsv16x_spi_init();
    
    // 循环读取 WHO_AM_I，验证 SPI 通信是否正常
    while(timeout < 30)
    {
        id = lsm6dsv16x_read_reg(LSM6DSV_WHO_AM_I);
        if(id == LSM6DSV_ID)
        {
            break;
        }
        system_delay_ms(100);
        timeout++;
    }
    
    // WHO_AM_I 读取失败，快速闪烁 LED 提示硬件异常
    if(id != LSM6DSV_ID)
    {
        printf("LSM6DSV16X not found, check wiring!\r\n");
        while(1)
        {
            gpio_low(P23_7);            // LED 点亮
            system_delay_ms(100);
            gpio_high(P23_7);           // LED 熄灭
            system_delay_ms(100);
        }
    }
    
    // 初始化 IMU：使能加速度计和陀螺仪
    if(lsm6dsv16x_init())
    {
        printf("LSM6DSV16X init failed!\r\n");
        while(1);
    }
    if(lsm6dsv16x_sflp_init())
    {
        printf("LSM6DSV16X SFLP init failed!\r\n");
        while(1);
    }
    
    for(;;)
    {
        lsm6dsv16x_sflp_update();
        lsm6dsv16x_vofa_send();
        
        if(50 <= ++led_count)
        {
            led_count = 0;
            gpio_toggle_level(P23_7);
        }
        system_delay_ms(5);
    }
}

// **************************** 代码区域 ****************************
