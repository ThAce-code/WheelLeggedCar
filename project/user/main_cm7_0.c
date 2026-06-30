/*********************************************************************************************************************
* CYT4BB Opensourec Library (CYT4BB 开源库)：一个基于官方 SDK 接口的二次封装的底层开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件：
* 你可以根据自由软件基金会所发布的 GPL（GNU General Public License），GNU 通用公共许可证的条款，
* 即 GPL 的第3版（即 GPL3.0），或（您选择的）任何更高版本，对其进行再发布和/或修改。
*
* 本开源库的发布是希望它能发挥价值，但并未对其作任何的保证，
* 甚至没有针对其隐含的适销性和适合特定用途的保证。
* 详情参见 GPL
*
* 你应该已经随本开源库一同收到一份 GPL 的副本
* 如果没有，请查阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可协议 如果该协议不满足您的需求或另有用途
* 请联系淘宝客服或查阅 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 该许可证内的 libraries 文件夹 以及文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 修改内容时必须保留逐飞科技的版权声明和许可声明
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限责任公司
* 版本信息          查看 libraries/doc 文件夹内的 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 淘宝链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期             作者              备注
* 2024-1-4       pudding            first version
* 2026-6-29     移植IMU轮询测试
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "app.h"
#include "app_config.h"

static void led_blink_error_code(uint8 code)
{
    uint8 i;

    while(1)
    {
        for(i = 0; i < code; i++)
        {
            gpio_low(P19_0);
            system_delay_ms(150);
            gpio_high(P19_0);
            system_delay_ms(150);
        }
        system_delay_ms(800);
    }
}

int main(void)
{
    uint8 app_result;
    uint32 now_ms;
    uint32 led_last_ms = 0;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    // P19.0 板载LED, 低电平有效, 初始高电平=灭
    gpio_init(P19_0, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    app_result = app_init();
    if(0U != app_result)
    {
        // IMU 初始化失败: 闪 1 次=WHO_AM_I, 2 次=基础初始化, 3 次=SFLP/FIFO
        led_blink_error_code(app_result);
    }

    pit_ms_init(PIT_CH0, APP_TICK_PERIOD_MS);

    for(;;)
    {
        app_run_once();

        now_ms = app_get_ms();
        if(APP_HEARTBEAT_PERIOD_MS <= (now_ms - led_last_ms))
        {
            led_last_ms = now_ms;
            gpio_toggle_level(P19_0);
        }
    }
}

// **************************** 代码区域结束 ****************************
