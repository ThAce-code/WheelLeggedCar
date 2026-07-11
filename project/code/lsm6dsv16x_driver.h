/*********************************************************************************************************************
* File: lsm6dsv16x_driver.h
* Description: SPI and SFLP driver for LSM6DSV16X 6-axis IMU.
* Platform: CYT4BB7
********************************************************************************************************************/

#ifndef _lsm6dsv16x_driver_h_
#define _lsm6dsv16x_driver_h_

#include "zf_common_headfile.h"

#define LSM6DSV_CS_PIN          P15_3

#define LSM6DSV_ANGLE_DT_S      (0.01f)

#define APP_IMU_FIFO_WTM        (1U)

#define LSM6DSV_WHO_AM_I        0x0F
#define LSM6DSV_IF_CFG          0x03
#define LSM6DSV_CTRL1           0x10
#define LSM6DSV_CTRL2           0x11
#define LSM6DSV_CTRL3           0x12
#define LSM6DSV_CTRL6           0x15
#define LSM6DSV_CTRL8           0x17
#define LSM6DSV_FIFO_CTRL1      0x07
#define LSM6DSV_FIFO_CTRL4      0x0A
#define LSM6DSV_INT1_CTRL       0x0D
#define LSM6DSV_FIFO_STATUS1    0x1B
#define LSM6DSV_FIFO_STATUS2    0x1C
#define LSM6DSV_FIFO_OVR_IA     (0x40U)
#define LSM6DSV_OUT_TEMP_L      0x20
#define LSM6DSV_OUTX_L_G        0x22
#define LSM6DSV_OUTX_L_A        0x28
#define LSM6DSV_FIFO_DATA_TAG   0x78

#define LSM6DSV_INT1_FIFO_TH    (0x08U)

#define LSM6DSV_FUNC_CFG_ACCESS     0x01
#define LSM6DSV_EMB_FUNC_EN_A       0x04
#define LSM6DSV_EMB_FUNC_EXEC_STATUS 0x07
#define LSM6DSV_EMB_FUNC_FIFO_EN_A  0x44
#define LSM6DSV_SFLP_ODR            0x5E
#define LSM6DSV_EMB_FUNC_INIT_A     0x66

#define LSM6DSV_ID              0x70

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
}lsm6dsv16x_angle_data_struct;

void    lsm6dsv16x_spi_init     (void);
uint8   lsm6dsv16x_read_reg     (uint8 reg);
void    lsm6dsv16x_write_reg    (uint8 reg, uint8 data);
uint8   lsm6dsv16x_self_check   (void);
uint8   lsm6dsv16x_init         (void);
void    lsm6dsv16x_get_acc      (int16 *x, int16 *y, int16 *z);
void    lsm6dsv16x_get_gyro     (int16 *x, int16 *y, int16 *z);
void    lsm6dsv16x_angle_init   (void);
void    lsm6dsv16x_angle_update (float dt_s);
void    lsm6dsv16x_vofa_send    (void);
uint8   lsm6dsv16x_sflp_init    (void);
uint8   lsm6dsv16x_sflp_update  (void);
uint32  lsm6dsv16x_get_invalid_sample_count(void);
uint8   lsm6dsv16x_gyro_offset_init(void);
void    lsm6dsv16x_gyro_update(void);
lsm6dsv16x_angle_data_struct *lsm6dsv16x_get_angle_data(void);

#endif
