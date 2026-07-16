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
* 文件名称          main_cm7_1
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "camera_frame_consumer.h"
#include "intercore_memory.h"
#include "sensor_gnss.h"
#include "intercore_transport.h"

static intercore_transport_struct gnss_transport;
static uint8 gnss_transport_attached;
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);

    if(1U != intercore_memory_configure())
    {
        while(true)
        {
        }
    }

    pit_ms_init(PIT_CH2, 1);
    (void)camera_frame_consumer_init();
    gnss_transport_attached = 0U;
    if(0U == sensor_gnss_init())
    {
        while(true) { }
    }

    while(true)
    {
        gnss_snapshot_struct snapshot;
        intercore_gnss_payload_struct payload = {0};
        uint32 now_ms;

        camera_frame_consumer_service();
        now_ms = camera_frame_consumer_now_ms();
        if(0U == gnss_transport_attached)
        {
            gnss_transport_attached = intercore_transport_cm7_1_attach(
                &gnss_transport, intercore_memory_get_layout());
        }
        sensor_gnss_service(now_ms);
        if(0U != sensor_gnss_take_snapshot(&snapshot))
        {
            payload.local_x_m = snapshot.local_x_m;
            payload.local_y_m = snapshot.local_y_m;
            payload.speed_mps = snapshot.speed_mps;
            payload.course_deg = snapshot.course_deg;
            payload.hdop = snapshot.hdop;
            payload.position_sigma_m = snapshot.position_sigma_m;
            payload.checksum_error_count = snapshot.checksum_error_count;
            payload.timeout_count = snapshot.timeout_count;
            payload.satellite_count = snapshot.satellite_count;
            payload.fix_valid = snapshot.fix_valid;
            payload.fix_quality = snapshot.fix_quality;
            payload.origin_valid = snapshot.origin_valid;
            if((0U != gnss_transport_attached) &&
               (0U == intercore_transport_publish_gnss(
                           &gnss_transport, &payload, snapshot.timestamp_ms)))
            {
                gnss_transport_attached = 0U;
            }
        }
    }
}

// **************************** 代码区域 ****************************
