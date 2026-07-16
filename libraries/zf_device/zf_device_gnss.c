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
* 文件名称          zf_device_gnss
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-01-12       pudding           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚             单片机管脚
*                   RX                  查看 zf_device_gnss.h 中 GNSS_RX 宏定义
*                   TX                  查看 zf_device_gnss.h 中 GNSS_TX 宏定义
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "math.h"
#include "zf_common_function.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_uart.h"

#include <string.h>

#include "zf_device_gnss.h"

#define GNSS_BUFFER_SIZE    ( 128 )

volatile uint8              gnss_flag = 0U;                                  // 1：采集完成等待处理数据 0：没有采集完成
gnss_info_struct            gnss;                                           // GPS解析之后的数据
    
static  uint8               gnss_state = 0;                                 // 1：GPS初始化完成
static  fifo_struct     gnss_receiver_fifo;                             // 
static  uint8               gnss_receiver_buffer[GNSS_BUFFER_SIZE];         // 数据存放数组

static  gps_state_enum      gnss_gga_state = GPS_STATE_RECEIVING;           // gga 语句状态
static  gps_state_enum      gnss_rmc_state = GPS_STATE_RECEIVING;           // rmc 语句状态
static  gps_state_enum      gnss_ths_state = GPS_STATE_RECEIVING;           // rmc 语句状态

static  uint8               gps_gga_buffer[GNSS_BUFFER_SIZE];
static  uint8               gps_rmc_buffer[GNSS_BUFFER_SIZE];
static  uint8               gps_ths_buffer[GNSS_BUFFER_SIZE];
static  uint32              gps_gga_length;
static  uint32              gps_rmc_length;
static  uint32              gps_ths_length;
static  uint8               gps_gga_truncated;
static  uint8               gps_rmc_truncated;
static  uint8               gps_ths_truncated;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取指定 ',' 后面的索引
// 参数说明     num             第几个逗号
// 参数说明     *str            字符串           
// 返回参数     uint8           返回索引
// 使用示例     get_parameter_index(1, s);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 get_parameter_index (uint8 num, char *str)
{
    uint8 i = 0, j = 0;
    char *temp = strchr(str, '\n');
    uint8 len = 0, len1 = 0;

    if(NULL != temp)
    {
        len = (uint8)((uint32)temp - (uint32)str + 1);
    }

    for(i = 0; i < len; i ++)
    {
        if(',' == str[i])
        {
            j ++;
        }
        if(j == num)
        {
            len1 =  i + 1;  
            break;
        }
    }

    return len1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给定字符串第一个 ',' 之前的数据转换为int
// 参数说明     *s              字符串
// 返回参数     float           返回数值
// 使用示例     get_int_number(&buf[get_parameter_index(7, buf)]);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
#if 0
static int get_int_number (char *s)
{
    char buf[10];
    uint8 i = 0;
    int return_value = 0;
    i = get_parameter_index(1, s);
    i = i - 1;
    strncpy(buf, s, i);
    buf[i] = 0;
    return_value = func_str_to_int(buf);
    return return_value;
}
                                                
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给定字符串第一个 ',' 之前的数据转换为float
// 参数说明     *s              字符串
// 返回参数     float           返回数值
// 使用示例     get_float_number(&buf[get_parameter_index(8, buf)]);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static float get_float_number (char *s)
{
    uint8 i = 0;
    char buf[15];
    float return_value = 0;
    
    i = get_parameter_index(1, s);
    i = i - 1;
    strncpy(buf, s, i);
    buf[i] = 0;
    return_value = (float)func_str_to_double(buf);
    return return_value;    
}
                                    
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给定字符串第一个 ',' 之前的数据转换为double 
// 参数说明     *s              字符串
// 返回参数     double          返回数值
// 使用示例     get_double_number(&buf[get_parameter_index(3, buf)]);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static double get_double_number (char *s)
{
    uint8 i = 0;
    char buf[15];
    double return_value = 0;
    
    i = get_parameter_index(1, s);
    i = i - 1;
    strncpy(buf, s, i);
    buf[i] = 0;
    return_value = func_str_to_double(buf);
    return return_value;
}
#endif

static uint8 gnss_copy_numeric_field (const char *source, char *destination,
                                      uint8 capacity, uint8 allow_empty)
{
    uint8 index = 0U;
    uint8 digit_seen = 0U;
    uint8 decimal_seen = 0U;
    uint8 value;

    if((NULL == source) || (NULL == destination) || (2U > capacity))
    {
        return 0U;
    }
    while(GNSS_BUFFER_SIZE > index)
    {
        value = (uint8)source[index];
        if((',' == value) || ('*' == value) || ('\r' == value) ||
           ('\n' == value) || (0U == value))
        {
            break;
        }
        if(index + 1U >= capacity)
        {
            return 0U;
        }
        destination[index] = (char)value;
        index++;
    }
    if(GNSS_BUFFER_SIZE == index)
    {
        return 0U;
    }
    destination[index] = 0;
    if(0U == index)
    {
        return allow_empty;
    }
    for(index = 0U; 0 != destination[index]; index++)
    {
        value = (uint8)destination[index];
        if((0U == index) && (('+' == value) || ('-' == value)))
        {
            continue;
        }
        if('.' == value)
        {
            if(0U != decimal_seen)
            {
                return 0U;
            }
            decimal_seen = 1U;
            continue;
        }
        if(('0' > value) || ('9' < value))
        {
            return 0U;
        }
        digit_seen = 1U;
    }
    return digit_seen;
}

static uint8 gnss_get_int_number (const char *source, uint8 allow_empty, int *value)
{
    char buffer[10];
    int candidate = 0;

    if((NULL == value) ||
       (0U == gnss_copy_numeric_field(source, buffer, sizeof(buffer), allow_empty)))
    {
        return 0U;
    }
    if(0 != buffer[0])
    {
        candidate = func_str_to_int(buffer);
    }
    *value = candidate;
    return 1U;
}

static uint8 gnss_get_float_number (const char *source, uint8 allow_empty, float *value)
{
    char buffer[15];
    float candidate = 0.0F;

    if((NULL == value) ||
       (0U == gnss_copy_numeric_field(source, buffer, sizeof(buffer), allow_empty)))
    {
        return 0U;
    }
    if(0 != buffer[0])
    {
        candidate = (float)func_str_to_double(buffer);
    }
    *value = candidate;
    return 1U;
}

static uint8 gnss_get_double_number (const char *source, uint8 allow_empty, double *value)
{
    char buffer[15];
    double candidate = 0.0;

    if((NULL == value) ||
       (0U == gnss_copy_numeric_field(source, buffer, sizeof(buffer), allow_empty)))
    {
        return 0U;
    }
    if(0 != buffer[0])
    {
        candidate = func_str_to_double(buffer);
    }
    *value = candidate;
    return 1U;
}

static float get_float_number (char *source)
{
    float value = 0.0F;

    if(0U == gnss_get_float_number(source, 1U, &value))
    {
        return 0.0F;
    }
    return value;
}

static uint8 gnss_parse_utc_ms (char *line, uint32 *utc_ms)
{
    uint8 index;
    uint8 fractional_digits = 0U;
    uint32 milliseconds = 0U;
    uint32 hour;
    uint32 minute;
    uint32 second;
    char *field;

    if((NULL == line) || (NULL == utc_ms))
    {
        return 0U;
    }
    index = get_parameter_index(1U, line);
    field = &line[index];
    for(index = 0U; 6U > index; index++)
    {
        if(('0' > field[index]) || ('9' < field[index]))
        {
            return 0U;
        }
    }
    hour = (uint32)(field[0] - '0') * 10U + (uint32)(field[1] - '0');
    minute = (uint32)(field[2] - '0') * 10U + (uint32)(field[3] - '0');
    second = (uint32)(field[4] - '0') * 10U + (uint32)(field[5] - '0');
    if((23U < hour) || (59U < minute) || (59U < second))
    {
        return 0U;
    }
    index = 6U;
    if('.' == field[index])
    {
        index++;
        while(('0' <= field[index]) && ('9' >= field[index]))
        {
            if(3U > fractional_digits)
            {
                milliseconds = milliseconds * 10U + (uint32)(field[index] - '0');
            }
            fractional_digits++;
            index++;
        }
        if(0U == fractional_digits)
        {
            return 0U;
        }
        while(3U > fractional_digits)
        {
            milliseconds *= 10U;
            fractional_digits++;
        }
    }
    if(',' != field[index])
    {
        return 0U;
    }
    *utc_ms = ((hour * 60U + minute) * 60U + second) * 1000U + milliseconds;
    return 1U;
}

static int8 gnss_hex_value (uint8 value)
{
    if(('0' <= value) && ('9' >= value))
    {
        return (int8)(value - '0');
    }
    if(('A' <= value) && ('F' >= value))
    {
        return (int8)(value - 'A' + 10U);
    }
    if(('a' <= value) && ('f' >= value))
    {
        return (int8)(value - 'a' + 10U);
    }
    return -1;
}

static uint8 gnss_sentence_checksum_valid (const uint8 *buffer, uint32 length, uint8 truncated)
{
    uint32 index;
    uint32 star_index = length;
    uint8 calculation = 0U;
    int8 high;
    int8 low;

    if((NULL == buffer) || (0U != truncated) || (7U > length) ||
       (GNSS_BUFFER_SIZE <= length) || ('$' != buffer[0]))
    {
        return 0U;
    }
    for(index = 1U; index < length; index++)
    {
        if('*' == buffer[index])
        {
            star_index = index;
            break;
        }
        if(('\r' == buffer[index]) || ('\n' == buffer[index]) || (0U == buffer[index]))
        {
            return 0U;
        }
        calculation ^= buffer[index];
    }
    if(star_index + 2U >= length)
    {
        return 0U;
    }
    if((star_index + 3U < length) && ('\r' != buffer[star_index + 3U]) &&
       ('\n' != buffer[star_index + 3U]) && (0U != buffer[star_index + 3U]))
    {
        return 0U;
    }

    high = gnss_hex_value(buffer[star_index + 1U]);
    low = gnss_hex_value(buffer[star_index + 2U]);
    return ((0 <= high) && (0 <= low) &&
            (calculation == (uint8)(((uint8)high << 4U) | (uint8)low))) ? 1U : 0U;
}





//-------------------------------------------------------------------------------------------------------------------
// 函数简介     世界时间转换为北京时间 
// 参数说明     *time           保存的时间
// 返回参数     void           
// 使用示例     utc_to_btc(&gnss->time);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static void utc_to_btc (gps_time_struct *time)
{
    uint8 day_num = 0;
    
    time->hour = time->hour + 8;
    if(23 < time->hour)
    {
        time->hour -= 24;
        time->day += 1;

        if(2 == time->month)
        {
            day_num = 28;
            if((0 == time->year % 4 && 0 != time->year % 100) || 0 == time->year % 400) // 判断是否为闰年 
            {
                day_num ++;                                                     // 闰月 2月为29天
            }
        }
        else
        {
            day_num = 31;                                                       // 1 3 5 7 8 10 12这些月份为31天
            if(4  == time->month || 6  == time->month || 9  == time->month || 11 == time->month )
            {
                day_num = 30;
            }
        }
        
        if(time->day > day_num)
        {
            time->day = 1;
            time->month ++;
            if(12 < time->month)
            {
                time->month -= 12;
                time->year ++;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     RMC语句解析
// 参数说明     *line           接收到的语句信息        
// 参数说明     *gnss            保存解析后的数据
// 返回参数     uint8           1：解析成功 0：数据有问题不能解析
// 使用示例     gps_gnrmc_parse((char *)data_buffer, &gnss);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_gnrmc_parse (char *line, gnss_info_struct *gnss)
{
    uint8 state = 0, temp = 0;
    uint32 utc_ms;
    
    double  latitude = 0;                                                       // 纬度
    double  longitude = 0;                                                      // 经度
    
    double lati_cent_tmp = 0, lati_second_tmp = 0;
    double long_cent_tmp = 0, long_second_tmp = 0;
    float speed_tmp = 0;
    char *buf = line;
    if(0U == gnss_parse_utc_ms(line, &utc_ms))
    {
        return 0U;
    }


    state = buf[get_parameter_index(2, buf)];

    if('A' == state || 'D' == state)                                                            // 如果数据有效 则解析数据
    {
        if((('N' != buf[get_parameter_index(4, buf)]) && ('S' != buf[get_parameter_index(4, buf)])) ||
           (('E' != buf[get_parameter_index(6, buf)]) && ('W' != buf[get_parameter_index(6, buf)])))
        {
            return 0U;
        }

        gnss->state = 1;
        gnss -> ns               = buf[get_parameter_index(4, buf)];
        gnss -> ew               = buf[get_parameter_index(6, buf)];

        if((0U == gnss_get_double_number(&buf[get_parameter_index(3, buf)], 0U, &latitude)) ||
           (0U == gnss_get_double_number(&buf[get_parameter_index(5, buf)], 0U, &longitude)))
        {
            return 0U;
        }

        if((!isfinite(latitude)) || (!isfinite(longitude)) ||
           (9000.0 < latitude) || (18000.0 < longitude))
        {
            return 0U;
        }

        gnss->latitude_degree    = (int)latitude / 100;                         // 纬度转换为度分秒
        lati_cent_tmp           = (latitude - gnss->latitude_degree * 100);
        gnss->latitude_cent      = (int)lati_cent_tmp;
        lati_second_tmp         = (lati_cent_tmp - gnss->latitude_cent) * 6000;
        gnss->latitude_second    = (int)lati_second_tmp;

        gnss->longitude_degree   = (int)longitude / 100;                        // 经度转换为度分秒
        long_cent_tmp           = (longitude - gnss->longitude_degree * 100);
        gnss->longitude_cent     = (int)long_cent_tmp;
        long_second_tmp         = (long_cent_tmp - gnss->longitude_cent) * 6000;
        gnss->longitude_second   = (int)long_second_tmp;

        gnss->latitude   = gnss->latitude_degree + lati_cent_tmp / 60;
        gnss->longitude  = gnss->longitude_degree + long_cent_tmp / 60;
        if((90.0 < gnss->latitude) || (180.0 < gnss->longitude))
        {
            return 0U;
        }


        speed_tmp       = get_float_number(&buf[get_parameter_index(7, buf)]);  // 速度(海里/小时)
        if((0U == gnss_get_float_number(&buf[get_parameter_index(7, buf)], 1U, &speed_tmp)) ||
           (0U == gnss_get_float_number(&buf[get_parameter_index(8, buf)], 1U, &gnss->direction)))
        {
            return 0U;
        }
        if(('S' == gnss->ns))
        {
            gnss->latitude = -gnss->latitude;
        }
        if(('W' == gnss->ew))
        {
            gnss->longitude = -gnss->longitude;
        }

        gnss->speed      = speed_tmp * 1.85f;                                   // 转换为公里/小时
        gnss->direction  = get_float_number(&buf[get_parameter_index(8, buf)]); // 角度           
    }
    else
    {
        gnss->state = 0;
    }

    // 在定位没有生效前也是有时间数据的，可以直接解析
    gnss->time.hour    = (buf[7] - '0') * 10 + (buf[8] - '0');                  // 时间
    gnss->time.minute  = (buf[9] - '0') * 10 + (buf[10] - '0');
    gnss->time.second  = (buf[11] - '0') * 10 + (buf[12] - '0');
    temp = get_parameter_index(9, buf);
    gnss->time.day     = (buf[temp + 0] - '0') * 10 + (buf[temp + 1] - '0');    // 日期
    gnss->rmc_utc_ms = utc_ms;
    if((0U == temp) || ('0' > buf[temp]) || ('9' < buf[temp]) ||
       ('0' > buf[temp + 1U]) || ('9' < buf[temp + 1U]) ||
       ('0' > buf[temp + 2U]) || ('9' < buf[temp + 2U]) ||
       ('0' > buf[temp + 3U]) || ('9' < buf[temp + 3U]) ||
       ('0' > buf[temp + 4U]) || ('9' < buf[temp + 4U]) ||
       ('0' > buf[temp + 5U]) || ('9' < buf[temp + 5U]))
    {
        return 0U;
    }
    gnss->time.month   = (buf[temp + 2] - '0') * 10 + (buf[temp + 3] - '0');
    gnss->time.year    = (buf[temp + 4] - '0') * 10 + (buf[temp + 5] - '0') + 2000;

    utc_to_btc(&gnss->time);

    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GGA语句解析
// 参数说明     *line           接收到的语句信息        
// 参数说明     *gnss            保存解析后的数据
// 返回参数     uint8           1：解析成功 0：数据有问题不能解析
// 使用示例     gps_gngga_parse((char *)data_buffer, &gnss);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_gngga_parse (char *line, gnss_info_struct *gnss)
{
    int fix_quality;
    int satellite_used;
    float hdop;
    float height;
    float geoid_height;
    uint32 utc_ms;
    char *buf = line;

    if(0U == gnss_parse_utc_ms(line, &utc_ms))
    {
        return 0U;
    }
    gnss->gga_utc_ms = utc_ms;

    if((0U == gnss_get_int_number(&buf[get_parameter_index(6, buf)], 1U, &fix_quality)) ||
       (0U == gnss_get_int_number(&buf[get_parameter_index(7, buf)], 1U, &satellite_used)) ||
       (0U == gnss_get_float_number(&buf[get_parameter_index(8, buf)], 1U, &hdop)) ||
       (0U == gnss_get_float_number(&buf[get_parameter_index(9, buf)], 1U, &height)) ||
       (0U == gnss_get_float_number(&buf[get_parameter_index(11, buf)], 1U, &geoid_height)) ||
       (0 > fix_quality) || (255 < fix_quality) ||
       (0 > satellite_used) || (255 < satellite_used))
    {
        return 0U;
    }
    gnss->fix_quality = (uint8)fix_quality;
    gnss->satellite_used = (uint8)satellite_used;
    gnss->hdop = hdop;
    gnss->height = height + geoid_height;

    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     THS语句解析
// 参数说明     *line           接收到的语句信息        
// 参数说明     *gnss            保存解析后的数据
// 返回参数     uint8           1：解析成功 0：数据有问题不能解析
// 使用示例     gps_gnths_parse((char *)data_buffer, &gnss);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_gnths_parse (char *line, gnss_info_struct *gnss)
{
    uint8 state = 0;
    char *buf = line;
    uint8 return_state = 0;
    float antenna_direction;

    state = buf[get_parameter_index(2, buf)];

    if('A' == state)
    {
        gnss->antenna_direction_state = 1;
        if(0U == gnss_get_float_number(&buf[get_parameter_index(1, buf)], 0U, &antenna_direction))
        {
            return 0U;
        }
        gnss->antenna_direction = antenna_direction;
        return_state = 1;
    }
    else
    {
        gnss->antenna_direction_state = 0;
    }
    
    return return_state;
}

static uint8 gnss_parse_sentence_transaction (const uint8 *buffer, uint32 length,
                                              uint8 truncated, gnss_info_struct *target)
{
    gnss_info_struct candidate;
    uint8 result = 0U;

    if((NULL == buffer) || (NULL == target) || (6U > length) ||
       (0U == gnss_sentence_checksum_valid(buffer, length, truncated)))
    {
        return 0U;
    }
    candidate = *target;
    if(0 == strncmp((const char *)&buffer[3], "RMC", 3))
    {
        result = gps_gnrmc_parse((char *)buffer, &candidate);
    }
    else if(0 == strncmp((const char *)&buffer[3], "GGA", 3))
    {
        result = gps_gngga_parse((char *)buffer, &candidate);
    }
    else if(0 == strncmp((const char *)&buffer[3], "THS", 3))
    {
        result = gps_gnths_parse((char *)buffer, &candidate);
    }
    if(0U == result)
    {
        return 0U;
    }
    *target = candidate;
    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的距离
// 参数说明     latitude1       第一个点的纬度
// 参数说明     longitude1      第一个点的经度
// 参数说明     latitude2       第二个点的纬度
// 参数说明     longitude2      第二个点的经度
// 返回参数     double          返回两点距离
// 使用示例     get_two_points_distance(latitude1_1, longitude1, latitude2, longitude2);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2)
{  
    const double EARTH_RADIUS = 6378137;                                        // 地球半径(单位：m)
    double rad_latitude1 = 0;
    double rad_latitude2 = 0;
    double rad_longitude1 = 0;
    double rad_longitude2 = 0;
    double distance = 0;
    double a = 0;
    double b = 0;
    
    rad_latitude1 = ANGLE_TO_RAD(latitude1);                                    // 根据角度计算弧度
    rad_latitude2 = ANGLE_TO_RAD(latitude2);
    rad_longitude1 = ANGLE_TO_RAD(longitude1);
    rad_longitude2 = ANGLE_TO_RAD(longitude2);

    a = rad_latitude1 - rad_latitude2;
    b = rad_longitude1 - rad_longitude2;

    distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rad_latitude1) * cos(rad_latitude2) * pow(sin(b / 2), 2)));   // google maps 里面实现的算法
    distance = distance * EARTH_RADIUS;  

    return distance;  
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的方位角
// 参数说明     latitude1       第一个点的纬度
// 参数说明     longitude1      第一个点的经度
// 参数说明     latitude2       第二个点的纬度
// 参数说明     longitude2      第二个点的经度
// 返回参数     double          返回方位角（0至360）
// 使用示例     get_two_points_azimuth(latitude1_1, longitude1, latitude2, longitude2);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2)
{
    latitude1 = ANGLE_TO_RAD(latitude1);
    latitude2 = ANGLE_TO_RAD(latitude2);
    longitude1 = ANGLE_TO_RAD(longitude1);
    longitude2 = ANGLE_TO_RAD(longitude2);

    double x = sin(longitude2 - longitude1) * cos(latitude2);
    double y = cos(latitude1) * sin(latitude2) - sin(latitude1) * cos(latitude2) * cos(longitude2 - longitude1);
    double angle = RAD_TO_ANGLE(atan2(x, y));
    return ((0 < angle) ? angle : (angle + 360));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析GPS数据
// 参数说明     void
// 返回参数     uint8           0-解析成功 1-解析失败 可能数据包错误
// 使用示例     gps_data_parse();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint32 gnss_next_sequence (uint32 sequence)
{
    sequence++;
    return (0U != sequence) ? sequence : 1U;
}

uint8 gnss_data_parse (void)
{
    uint8 return_state = 0U;
    do
    {
        if(GPS_STATE_RECEIVED == gnss_rmc_state)
        {
            gnss_rmc_state = GPS_STATE_PARSING;
            if(0U == gnss_parse_sentence_transaction(gps_rmc_buffer, gps_rmc_length,
                                                      gps_rmc_truncated, &gnss))
            {
                return_state = 1U;
            }
            else
            {
                gnss.rmc_sequence = gnss_next_sequence(gnss.rmc_sequence);
            }
        }
        gnss_rmc_state = GPS_STATE_RECEIVING;

        if(GPS_STATE_RECEIVED == gnss_gga_state)
        {
            gnss_gga_state = GPS_STATE_PARSING;
            if(0U == gnss_parse_sentence_transaction(gps_gga_buffer, gps_gga_length,
                                                      gps_gga_truncated, &gnss))
            {
                return_state = 1U;
            }
            else
            {
                gnss.gga_sequence = gnss_next_sequence(gnss.gga_sequence);
            }
        }
        gnss_gga_state = GPS_STATE_RECEIVING;

        if(GPS_STATE_RECEIVED == gnss_ths_state)
        {
            gnss_ths_state = GPS_STATE_PARSING;
            if(0U == gnss_parse_sentence_transaction(gps_ths_buffer, gps_ths_length,
                                                      gps_ths_truncated, &gnss))
            {
                return_state = 1U;
            }
        }
        gnss_ths_state = GPS_STATE_RECEIVING;
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS串口回调函数
// 参数说明     void            
// 返回参数     void            
// 使用示例     gps_uart_callback();
// 备注信息     此函数需要在串口接收中断内进行调用
//-------------------------------------------------------------------------------------------------------------------
void gnss_uart_callback (void)
{
    uint8 temp_gps[6];
    uint32 temp_length = 0;

    if(gnss_state)
    {
        uint8 dat;
        while(uart_query_byte(GNSS_UART, &dat))
        {
            fifo_write_buffer(&gnss_receiver_fifo, &dat, 1);
        }
        
        if('\n' == dat)
        {
            // 读取前6个数据 用于判断语句类型
            temp_length = 6;
            fifo_read_buffer(&gnss_receiver_fifo, temp_gps, &temp_length, FIFO_READ_ONLY);
            
            // 根据不同类型将数据拷贝到不同的缓冲区
            if(0 == strncmp((char *)&temp_gps[3], "RMC", 3))
            {
                // 如果没有在解析数据则更新缓冲区的数据
                if(GPS_STATE_PARSING != gnss_rmc_state)
                {
                    gnss_rmc_state = GPS_STATE_RECEIVED;
                    temp_length = fifo_used(&gnss_receiver_fifo);
                    gps_rmc_truncated = (GNSS_BUFFER_SIZE <= temp_length) ? 1U : 0U;
                    if(0U != gps_rmc_truncated)
                    {
                        temp_length = GNSS_BUFFER_SIZE - 1U;
                    }
                    gps_rmc_length = temp_length;
                    fifo_read_buffer(&gnss_receiver_fifo, gps_rmc_buffer, &temp_length, FIFO_READ_AND_CLEAN);
                    gps_rmc_buffer[gps_rmc_length] = 0U;
                }
            }
            else if(0 == strncmp((char *)&temp_gps[3], "GGA", 3))
            {
                // 如果没有在解析数据则更新缓冲区的数据
                if(GPS_STATE_PARSING != gnss_gga_state)
                {
                    gnss_gga_state = GPS_STATE_RECEIVED;
                    temp_length = fifo_used(&gnss_receiver_fifo);
                    gps_gga_truncated = (GNSS_BUFFER_SIZE <= temp_length) ? 1U : 0U;
                    if(0U != gps_gga_truncated)
                    {
                        temp_length = GNSS_BUFFER_SIZE - 1U;
                    }
                    gps_gga_length = temp_length;
                    fifo_read_buffer(&gnss_receiver_fifo, gps_gga_buffer, &temp_length, FIFO_READ_AND_CLEAN);
                    gps_gga_buffer[gps_gga_length] = 0U;
                }
            }
            else if(0 == strncmp((char *)&temp_gps[3], "THS", 3))
            {
                // 如果没有在解析数据则更新缓冲区的数据
                if(GPS_STATE_PARSING != gnss_ths_state)
                {
                    gnss_ths_state = GPS_STATE_RECEIVED;
                    temp_length = fifo_used(&gnss_receiver_fifo);
                    gps_ths_truncated = (GNSS_BUFFER_SIZE <= temp_length) ? 1U : 0U;
                    if(0U != gps_ths_truncated)
                    {
                        temp_length = GNSS_BUFFER_SIZE - 1U;
                    }
                    gps_ths_length = temp_length;
                    fifo_read_buffer(&gnss_receiver_fifo, gps_ths_buffer, &temp_length, FIFO_READ_AND_CLEAN);
                    gps_ths_buffer[gps_ths_length] = 0U;
                }
            }
            
            // 统一将FIFO清空
            fifo_clear(&gnss_receiver_fifo);

            gnss_flag = 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS初始化
// 参数说明     void
// 返回参数     void
// 使用示例     gps_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void gnss_init (gps_device_enum gps_device)
{
    const uint8 set_rate[]      = {0xF1, 0xD9, 0x06, 0x42, 0x14, 0x00, 0x00, 0x0A, 0x05, 0x00, 0x64, 0x00, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xB8, 0xED};
    const uint8 open_gga[]      = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10};
    const uint8 open_rmc[]      = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x01, 0x00, 0x1A};
    
    const uint8 close_gll[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
    const uint8 close_gsa[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
    const uint8 close_grs[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    const uint8 close_gsv[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
    const uint8 close_vtg[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B};
    const uint8 close_zda[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D};
    const uint8 close_gst[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F};
    const uint8 close_txt[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x40, 0x00, 0x3A, 0x8F};
    const uint8 close_txt_ant[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x20, 0x00, 0x1A, 0x4F};

    memset(&gnss, 0, sizeof(gnss));
    gnss_flag = 0U;
    gps_rmc_length = 0U;
    gps_gga_length = 0U;
    gps_ths_length = 0U;
    gps_rmc_truncated = 0U;
    gps_gga_truncated = 0U;
    gps_ths_truncated = 0U;
    
    if((TAU1201 == gps_device) || (GN42A == gps_device))
    {
        fifo_init(&gnss_receiver_fifo, FIFO_DATA_8BIT, gnss_receiver_buffer, GNSS_BUFFER_SIZE);
        system_delay_ms(500);                                                   // 等待GPS启动后开始初始化
        uart_init(GNSS_UART, 115200, GNSS_RX, GNSS_TX);

        uart_write_buffer(GNSS_UART, (uint8 *)set_rate, sizeof(set_rate));      // 设置GPS更新速率为10hz 如果不调用此语句则默认为1hz
        system_delay_ms(200);   
            
        uart_write_buffer(GNSS_UART, (uint8 *)open_rmc, sizeof(open_rmc));      // 开启rmc语句
        system_delay_ms(50);    
        uart_write_buffer(GNSS_UART, (uint8 *)open_gga, sizeof(open_gga));      // 开启gga语句
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gll, sizeof(close_gll));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gsa, sizeof(close_gsa));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_grs, sizeof(close_grs));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gsv, sizeof(close_gsv));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_vtg, sizeof(close_vtg));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_zda, sizeof(close_zda));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gst, sizeof(close_gst));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_txt, sizeof(close_txt));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_txt_ant, sizeof(close_txt_ant));
        system_delay_ms(50);

        gnss_state = 1;
        uart_rx_interrupt(GNSS_UART, 1);
    }
    else if(GN43RFA == gps_device)
    {
        // GN43RFA RTK模块不需要进行参数设置，如果需要修改参数应该使用专用的上位机修改参数
        fifo_init(&gnss_receiver_fifo, FIFO_DATA_8BIT, gnss_receiver_buffer, GNSS_BUFFER_SIZE);
        uart_init(GNSS_UART, 115200, GNSS_RX, GNSS_TX);
        gnss_state = 1;
        uart_rx_interrupt(GNSS_UART, 1);
    }
    
}

#if defined(GNSS_HOST_TEST)
uint8 gnss_host_parse_sentence (const char *sentence, uint32 length, gnss_info_struct *parsed)
{
    uint8 buffer[GNSS_BUFFER_SIZE];

    if((NULL == sentence) || (NULL == parsed) || (GNSS_BUFFER_SIZE <= length))
    {
        return 0U;
    }
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, sentence, length);
    return gnss_parse_sentence_transaction(buffer, length, 0U, parsed);
}
#endif
