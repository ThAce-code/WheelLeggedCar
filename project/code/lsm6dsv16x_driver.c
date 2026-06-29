/*********************************************************************************************************************
* File: lsm6dsv16x_driver.c
* Description: SPI and SFLP driver for LSM6DSV16X 6-axis IMU.
* Platform: CYT4BB7
********************************************************************************************************************/

#include "lsm6dsv16x_driver.h"
#include "math.h"

#define LSM6DSV_RAD_TO_DEG              (57.2957795f)
#define LSM6DSV_ACC_SENS_4G             (0.0001220703125f)
#define LSM6DSV_GYRO_SENS_2000DPS       (0.07f)
#define LSM6DSV_COMPLEMENTARY_ALPHA     (0.98f)
#define LSM6DSV_GYRO_OFFSET_SAMPLES     (200)
#define LSM6DSV_FUNC_CFG_MAIN_BANK      (0x00)
#define LSM6DSV_FUNC_CFG_EMB_BANK       (0x80)
#define LSM6DSV_SFLP_GAME_ENABLE        (0x02)
#define LSM6DSV_SFLP_FIFO_ENABLE        (0x02)
#define LSM6DSV_SFLP_ODR_120HZ          (0x18)
#define LSM6DSV_ACC_FS_4G               (0x01)
#define LSM6DSV_GYRO_FS_2000DPS         (0x04)
#define LSM6DSV_FIFO_STREAM_MODE        (0x06)
#define LSM6DSV_FIFO_GAME_TAG           (0x13)
#define LSM6DSV_SPI_READ                (0x80)
#define LSM6DSV_SPI_MAX_READ_LEN        (7U)

static lsm6dsv16x_angle_data_struct lsm6dsv16x_angle_data;
static float lsm6dsv16x_gyro_offset_x = 0.0f;
static float lsm6dsv16x_gyro_offset_y = 0.0f;
static float lsm6dsv16x_gyro_offset_z = 0.0f;
static uint8 lsm6dsv16x_angle_ready = 0;

volatile uint8 lsm6dsv16x_debug_who_am_i = 0;
volatile uint8 lsm6dsv16x_debug_sflp_step = 0;
volatile uint8 lsm6dsv16x_debug_sflp_reg = 0;
volatile uint8 lsm6dsv16x_debug_bank = 0;

static float lsm6dsv16x_inv_sqrt_arg(float value)
{
    return (0.0f < value) ? sqrtf(value) : 0.0f;
}

static void lsm6dsv16x_select_bank(uint8 bank)
{
    lsm6dsv16x_write_reg(LSM6DSV_FUNC_CFG_ACCESS, bank);
    lsm6dsv16x_debug_bank = bank;
    system_delay_ms(1);
}

static void lsm6dsv16x_read_regs(uint8 reg, uint8 *data, uint32 len)
{
    uint8 tx[LSM6DSV_SPI_MAX_READ_LEN + 1U];
    uint8 rx[LSM6DSV_SPI_MAX_READ_LEN + 1U];
    uint32 i;

    if(LSM6DSV_SPI_MAX_READ_LEN < len)
    {
        return;
    }

    tx[0] = (uint8)(reg | LSM6DSV_SPI_READ);
    for(i = 0; i < len; i++)
    {
        tx[i + 1U] = 0U;
        rx[i + 1U] = 0U;
    }

    gpio_low(LSM6DSV_CS_PIN);
    spi_transfer_8bit(SPI_2, tx, rx, len + 1U);
    system_delay_us(1);
    gpio_high(LSM6DSV_CS_PIN);

    for(i = 0; i < len; i++)
    {
        data[i] = rx[i + 1U];
    }
}

static float lsm6dsv16x_half_to_float(uint16 half)
{
    uint16 half_exp = half & 0x7C00U;
    uint16 half_sig = half & 0x03FFU;
    uint16 exp = 0U;
    uint32 sign = ((uint32)half & 0x8000U) << 16;
    uint32 bits;
    union
    {
        uint32 u32;
        float f32;
    } conv;

    if(0x0000U == half_exp)
    {
        if(0U == half_sig)
        {
            bits = sign;
        }
        else
        {
            half_sig <<= 1;
            while(0U == (half_sig & 0x0400U))
            {
                half_sig <<= 1;
                exp++;
            }
            bits = sign + (((uint32)(127 - 15 - exp)) << 23) + (((uint32)(half_sig & 0x03FFU)) << 13);
        }
    }
    else if(0x7C00U == half_exp)
    {
        bits = sign + 0x7F800000U + (((uint32)half_sig) << 13);
    }
    else
    {
        bits = sign + (((uint32)(half & 0x7FFFU) + 0x1C000U) << 13);
    }

    conv.u32 = bits;
    return conv.f32;
}

static void lsm6dsv16x_quat_to_angle(float qw, float qx, float qy, float qz)
{
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    sinr_cosp = 2.0f * (qw * qx + qy * qz);
    cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    lsm6dsv16x_angle_data.roll = atan2f(sinr_cosp, cosr_cosp) * LSM6DSV_RAD_TO_DEG;

    sinp = 2.0f * (qw * qy - qz * qx);
    if(1.0f < sinp)
    {
        sinp = 1.0f;
    }
    else if(-1.0f > sinp)
    {
        sinp = -1.0f;
    }
    lsm6dsv16x_angle_data.pitch = asinf(sinp) * LSM6DSV_RAD_TO_DEG;

    siny_cosp = 2.0f * (qw * qz + qx * qy);
    cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    lsm6dsv16x_angle_data.yaw = atan2f(siny_cosp, cosy_cosp) * LSM6DSV_RAD_TO_DEG;
}

void lsm6dsv16x_spi_init(void)
{
    spi_init(SPI_2, SPI_MODE0, 8 * 1000 * 1000,
             SPI2_CLK_P15_2, SPI2_MOSI_P15_1, SPI2_MISO_P15_0, SPI_CS_NULL);
    gpio_init(LSM6DSV_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(20);
}

void lsm6dsv16x_write_reg(uint8 reg, uint8 data)
{
    uint8 tx[2];
    uint8 rx[2];

    tx[0] = (uint8)(reg & (uint8)~LSM6DSV_SPI_READ);
    tx[1] = data;

    gpio_low(LSM6DSV_CS_PIN);
    spi_transfer_8bit(SPI_2, tx, rx, 2);
    system_delay_us(1);
    gpio_high(LSM6DSV_CS_PIN);
}

uint8 lsm6dsv16x_read_reg(uint8 reg)
{
    uint8 data;
    lsm6dsv16x_read_regs(reg, &data, 1U);
    return data;
}

uint8 lsm6dsv16x_self_check(void)
{
    lsm6dsv16x_debug_who_am_i = lsm6dsv16x_read_reg(LSM6DSV_WHO_AM_I);
    return (lsm6dsv16x_debug_who_am_i == LSM6DSV_ID) ? 0 : 1;
}

uint8 lsm6dsv16x_init(void)
{
    if(lsm6dsv16x_self_check())
    {
        return 1;
    }

    lsm6dsv16x_write_reg(LSM6DSV_CTRL3, 0x44);  // BDU + IF_INC
    lsm6dsv16x_write_reg(LSM6DSV_CTRL1, 0x06);  // ACC: 120Hz, high-performance
    lsm6dsv16x_write_reg(LSM6DSV_CTRL2, 0x06);  // GYRO: 120Hz, high-performance
    lsm6dsv16x_write_reg(LSM6DSV_CTRL6, LSM6DSV_GYRO_FS_2000DPS);  // GYRO: +-2000dps
    lsm6dsv16x_write_reg(LSM6DSV_CTRL8, LSM6DSV_ACC_FS_4G);         // ACC: +-4g
    return 0;
}

void lsm6dsv16x_get_acc(int16 *x, int16 *y, int16 *z)
{
    uint8 dat[6];
    lsm6dsv16x_read_regs(LSM6DSV_OUTX_L_A, dat, 6U);

    *x = (int16)(((uint16)dat[1] << 8) | dat[0]);
    *y = (int16)(((uint16)dat[3] << 8) | dat[2]);
    *z = (int16)(((uint16)dat[5] << 8) | dat[4]);
}

void lsm6dsv16x_get_gyro(int16 *x, int16 *y, int16 *z)
{
    uint8 dat[6];
    lsm6dsv16x_read_regs(LSM6DSV_OUTX_L_G, dat, 6U);

    *x = (int16)(((uint16)dat[1] << 8) | dat[0]);
    *y = (int16)(((uint16)dat[3] << 8) | dat[2]);
    *z = (int16)(((uint16)dat[5] << 8) | dat[4]);
}

void lsm6dsv16x_angle_init(void)
{
    int16 acc_x, acc_y, acc_z;
    int16 gyro_x, gyro_y, gyro_z;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16 i;

    system_delay_ms(100);

    for(i = 0; i < LSM6DSV_GYRO_OFFSET_SAMPLES; i++)
    {
        lsm6dsv16x_get_gyro(&gyro_x, &gyro_y, &gyro_z);
        sum_x += (float)gyro_x * LSM6DSV_GYRO_SENS_2000DPS;
        sum_y += (float)gyro_y * LSM6DSV_GYRO_SENS_2000DPS;
        sum_z += (float)gyro_z * LSM6DSV_GYRO_SENS_2000DPS;
        system_delay_ms(5);
    }

    lsm6dsv16x_gyro_offset_x = sum_x / (float)LSM6DSV_GYRO_OFFSET_SAMPLES;
    lsm6dsv16x_gyro_offset_y = sum_y / (float)LSM6DSV_GYRO_OFFSET_SAMPLES;
    lsm6dsv16x_gyro_offset_z = sum_z / (float)LSM6DSV_GYRO_OFFSET_SAMPLES;

    lsm6dsv16x_get_acc(&acc_x, &acc_y, &acc_z);
    lsm6dsv16x_angle_data.acc_x = (float)acc_x * LSM6DSV_ACC_SENS_4G;
    lsm6dsv16x_angle_data.acc_y = (float)acc_y * LSM6DSV_ACC_SENS_4G;
    lsm6dsv16x_angle_data.acc_z = (float)acc_z * LSM6DSV_ACC_SENS_4G;

    lsm6dsv16x_angle_data.roll = atan2f(lsm6dsv16x_angle_data.acc_y,
                                        lsm6dsv16x_angle_data.acc_z) * LSM6DSV_RAD_TO_DEG;
    lsm6dsv16x_angle_data.pitch = atan2f(-lsm6dsv16x_angle_data.acc_x,
                                         lsm6dsv16x_inv_sqrt_arg(lsm6dsv16x_angle_data.acc_y * lsm6dsv16x_angle_data.acc_y +
                                                                 lsm6dsv16x_angle_data.acc_z * lsm6dsv16x_angle_data.acc_z)) * LSM6DSV_RAD_TO_DEG;
    lsm6dsv16x_angle_data.yaw = 0.0f;
    lsm6dsv16x_angle_data.quat_w = 1.0f;
    lsm6dsv16x_angle_data.quat_x = 0.0f;
    lsm6dsv16x_angle_data.quat_y = 0.0f;
    lsm6dsv16x_angle_data.quat_z = 0.0f;
    lsm6dsv16x_angle_ready = 1;
}

void lsm6dsv16x_angle_update(float dt_s)
{
    int16 acc_x, acc_y, acc_z;
    int16 gyro_x, gyro_y, gyro_z;
    float acc_roll;
    float acc_pitch;
    float acc_yz_norm;

    lsm6dsv16x_get_acc(&acc_x, &acc_y, &acc_z);
    lsm6dsv16x_get_gyro(&gyro_x, &gyro_y, &gyro_z);

    lsm6dsv16x_angle_data.acc_x = (float)acc_x * LSM6DSV_ACC_SENS_4G;
    lsm6dsv16x_angle_data.acc_y = (float)acc_y * LSM6DSV_ACC_SENS_4G;
    lsm6dsv16x_angle_data.acc_z = (float)acc_z * LSM6DSV_ACC_SENS_4G;

    lsm6dsv16x_angle_data.gyro_x = (float)gyro_x * LSM6DSV_GYRO_SENS_2000DPS - lsm6dsv16x_gyro_offset_x;
    lsm6dsv16x_angle_data.gyro_y = (float)gyro_y * LSM6DSV_GYRO_SENS_2000DPS - lsm6dsv16x_gyro_offset_y;
    lsm6dsv16x_angle_data.gyro_z = (float)gyro_z * LSM6DSV_GYRO_SENS_2000DPS - lsm6dsv16x_gyro_offset_z;

    acc_yz_norm = lsm6dsv16x_inv_sqrt_arg(lsm6dsv16x_angle_data.acc_y * lsm6dsv16x_angle_data.acc_y +
                                          lsm6dsv16x_angle_data.acc_z * lsm6dsv16x_angle_data.acc_z);
    acc_roll = atan2f(lsm6dsv16x_angle_data.acc_y,
                      lsm6dsv16x_angle_data.acc_z) * LSM6DSV_RAD_TO_DEG;
    acc_pitch = atan2f(-lsm6dsv16x_angle_data.acc_x, acc_yz_norm) * LSM6DSV_RAD_TO_DEG;

    if(!lsm6dsv16x_angle_ready)
    {
        lsm6dsv16x_angle_data.roll = acc_roll;
        lsm6dsv16x_angle_data.pitch = acc_pitch;
        lsm6dsv16x_angle_data.yaw = 0.0f;
        lsm6dsv16x_angle_ready = 1;
    }
    else
    {
        lsm6dsv16x_angle_data.roll = LSM6DSV_COMPLEMENTARY_ALPHA *
                                     (lsm6dsv16x_angle_data.roll + lsm6dsv16x_angle_data.gyro_x * dt_s) +
                                     (1.0f - LSM6DSV_COMPLEMENTARY_ALPHA) * acc_roll;
        lsm6dsv16x_angle_data.pitch = LSM6DSV_COMPLEMENTARY_ALPHA *
                                      (lsm6dsv16x_angle_data.pitch + lsm6dsv16x_angle_data.gyro_y * dt_s) +
                                      (1.0f - LSM6DSV_COMPLEMENTARY_ALPHA) * acc_pitch;
        lsm6dsv16x_angle_data.yaw += lsm6dsv16x_angle_data.gyro_z * dt_s;
    }
}

void lsm6dsv16x_vofa_send(void)
{
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    float vofa_data[3];

    vofa_data[0] = lsm6dsv16x_angle_data.roll;
    vofa_data[1] = lsm6dsv16x_angle_data.pitch;
    vofa_data[2] = lsm6dsv16x_angle_data.yaw;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}

uint8 lsm6dsv16x_sflp_init(void)
{
    uint8 reg;

    lsm6dsv16x_debug_sflp_step = 1;
    lsm6dsv16x_write_reg(LSM6DSV_FIFO_CTRL4, 0x00);

    lsm6dsv16x_debug_sflp_step = 2;
    lsm6dsv16x_select_bank(LSM6DSV_FUNC_CFG_EMB_BANK);

    reg = lsm6dsv16x_read_reg(LSM6DSV_SFLP_ODR);
    reg = (uint8)((reg & (uint8)~0x38U) | LSM6DSV_SFLP_ODR_120HZ);
    lsm6dsv16x_write_reg(LSM6DSV_SFLP_ODR, reg);

    lsm6dsv16x_debug_sflp_step = 3;
    reg = lsm6dsv16x_read_reg(LSM6DSV_EMB_FUNC_FIFO_EN_A);
    lsm6dsv16x_write_reg(LSM6DSV_EMB_FUNC_FIFO_EN_A, (uint8)(reg | LSM6DSV_SFLP_FIFO_ENABLE));
    reg = lsm6dsv16x_read_reg(LSM6DSV_EMB_FUNC_FIFO_EN_A);
    lsm6dsv16x_debug_sflp_reg = reg;
    if(0U == (reg & LSM6DSV_SFLP_FIFO_ENABLE))
    {
        lsm6dsv16x_select_bank(LSM6DSV_FUNC_CFG_MAIN_BANK);
        return 1;
    }

    lsm6dsv16x_debug_sflp_step = 4;
    reg = lsm6dsv16x_read_reg(LSM6DSV_EMB_FUNC_EN_A);
    lsm6dsv16x_write_reg(LSM6DSV_EMB_FUNC_EN_A, (uint8)(reg | LSM6DSV_SFLP_GAME_ENABLE));
    reg = lsm6dsv16x_read_reg(LSM6DSV_EMB_FUNC_EN_A);
    lsm6dsv16x_debug_sflp_reg = reg;
    if(0U == (reg & LSM6DSV_SFLP_GAME_ENABLE))
    {
        lsm6dsv16x_select_bank(LSM6DSV_FUNC_CFG_MAIN_BANK);
        return 1;
    }

    lsm6dsv16x_debug_sflp_step = 5;
    lsm6dsv16x_select_bank(LSM6DSV_FUNC_CFG_MAIN_BANK);

    lsm6dsv16x_write_reg(LSM6DSV_FIFO_CTRL1, APP_IMU_FIFO_WTM);
    lsm6dsv16x_write_reg(LSM6DSV_INT1_CTRL, LSM6DSV_INT1_FIFO_TH);
    lsm6dsv16x_write_reg(LSM6DSV_FIFO_CTRL4, LSM6DSV_FIFO_STREAM_MODE);
    system_delay_ms(20);

    lsm6dsv16x_debug_sflp_step = 0;
    return 0;
}

uint8 lsm6dsv16x_sflp_update(void)
{
    uint8 status[2];
    uint8 frame[7];
    uint8 tag;
    uint16 fifo_count;
    uint16 i;
    uint16 raw_x;
    uint16 raw_y;
    uint16 raw_z;
    float qx;
    float qy;
    float qz;
    float qw;
    float sumsq;
    float norm;
    uint8 got_game_vector = 0;

    lsm6dsv16x_read_regs(LSM6DSV_FIFO_STATUS1, status, 2U);

    fifo_count = (uint16)status[0] | (uint16)((status[1] & 0x01U) << 8);
    if(0U == fifo_count)
    {
        return 1;
    }

    for(i = 0; i < fifo_count; i++)
    {
        lsm6dsv16x_read_regs(LSM6DSV_FIFO_DATA_TAG, frame, 7U);

        tag = frame[0] >> 3;
        if(LSM6DSV_FIFO_GAME_TAG == tag)
        {
            raw_x = (uint16)frame[1] | ((uint16)frame[2] << 8);
            raw_y = (uint16)frame[3] | ((uint16)frame[4] << 8);
            raw_z = (uint16)frame[5] | ((uint16)frame[6] << 8);
            qx = lsm6dsv16x_half_to_float(raw_x);
            qy = lsm6dsv16x_half_to_float(raw_y);
            qz = lsm6dsv16x_half_to_float(raw_z);
            sumsq = qx * qx + qy * qy + qz * qz;

            if(1.0f < sumsq)
            {
                norm = sqrtf(sumsq);
                qx /= norm;
                qy /= norm;
                qz /= norm;
                sumsq = 1.0f;
            }

            qw = sqrtf(1.0f - sumsq);
            lsm6dsv16x_angle_data.quat_w = qw;
            lsm6dsv16x_angle_data.quat_x = qx;
            lsm6dsv16x_angle_data.quat_y = qy;
            lsm6dsv16x_angle_data.quat_z = qz;
            lsm6dsv16x_quat_to_angle(qw, qx, qy, qz);
            got_game_vector = 1;
        }
    }

    return got_game_vector ? 0 : 1;
}

lsm6dsv16x_angle_data_struct *lsm6dsv16x_get_angle_data(void)
{
    return &lsm6dsv16x_angle_data;
}
