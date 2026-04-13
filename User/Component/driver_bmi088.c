#include "driver_bmi088.h"
#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "bsp_time.h"
#include "device_instance.h"
#include <stdint.h>
#include <math.h>

void driver_bmi088_init(void)
{
    // 初始化BMI088设备
    // 初始化加速度计
    bmi08xa_init(&bmi08_dev);
    
    // 加速度计必须先从 SUSPEND 模式唤醒到 ACTIVE 模式，才能可靠地配置参数
    bmi08a_set_power_mode(&bmi08_dev);
    
    // 使用 bmi08xa 版本的函数才能同时设置 ODR、BW 以及刚刚讨论的 Range (3G)
    bmi08xa_set_meas_conf(&bmi08_dev);

    // 初始化陀螺仪
    bmi08g_init(&bmi08_dev);
    bmi08g_set_power_mode(&bmi08_dev);
    bmi08g_set_meas_conf(&bmi08_dev);
}

BMI08_INTF_RET_TYPE bmi088_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint16_t cs_pin = *((uint16_t *)intf_ptr);
    HAL_GPIO_WritePin(GPIOD, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 1000);
    HAL_SPI_Receive(&hspi2, reg_data, len, 1000);
    HAL_GPIO_WritePin(GPIOD, cs_pin, GPIO_PIN_SET);
    return BMI08_INTF_RET_SUCCESS;
}

BMI08_INTF_RET_TYPE bmi088_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint16_t cs_pin = *((uint16_t *)intf_ptr);
    HAL_GPIO_WritePin(GPIOD, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 1000);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)reg_data, len, 1000);
    HAL_GPIO_WritePin(GPIOD, cs_pin, GPIO_PIN_SET);
    return BMI08_INTF_RET_SUCCESS;
}

void bmi088_delay_us(uint32_t period, void *intf_ptr)
{
    Delay_us(period);
}

// ============================================================================
// 数据转换函数：LSB -> 物理单位
// ============================================================================

/**
 * @brief 将陀螺仪的 LSB 数据转换为 rad/s (弧度/秒)
 * @note  当前量程配置为 500 DPS (分辨率: 65.6 LSB/dps)
 *        1 dps = 0.0174532925f rad/s
 *        转换系数 = (1.0f / 65.6f) * 0.0174532925f = 0.000266056f
 */
void driver_bmi088_gyro_to_rad_s(struct bmi08_sensor_data *raw_gyro, struct bmi08_sensor_data_f *out_gyro_rad, struct imu_offset *offset)
{
    const float GYRO_RAD_FACTOR = 0.000266056f; 
    
    // 1. 转换为物理单位
    out_gyro_rad->x = (float)raw_gyro->x * GYRO_RAD_FACTOR;
    out_gyro_rad->y = (float)raw_gyro->y * GYRO_RAD_FACTOR;
    out_gyro_rad->z = (float)raw_gyro->z * GYRO_RAD_FACTOR;

    // 2. 减去零偏 (如果已校准)
    if (offset->is_calibrated) {
        out_gyro_rad->x -= offset->gyro_x;
        out_gyro_rad->y -= offset->gyro_y;
        out_gyro_rad->z -= offset->gyro_z;
    }
}

/**
 * @brief 将加速度计的 LSB 数据转换为 m/s^2 并去除零偏
 * @note  当前量程配置为 3G (分辨率: 10920 LSB/g)
 *        转换系数 = (1.0f / 10920.0f) * STANDARD_GRAVITY_MS2
 */
void driver_bmi088_accel_to_ms2(struct bmi08_sensor_data *raw_accel, struct bmi08_sensor_data_f *out_accel_ms2, struct imu_offset *offset)
{
    // (1.0f / 10920.0f) * 9.80665f = 0.00089804487f
    const float ACCEL_MS2_FACTOR = (1.0f / 10920.0f) * STANDARD_GRAVITY_MS2;
    
    // 1. 转换为物理单位
    out_accel_ms2->x = (float)raw_accel->x * ACCEL_MS2_FACTOR;
    out_accel_ms2->y = (float)raw_accel->y * ACCEL_MS2_FACTOR;
    out_accel_ms2->z = (float)raw_accel->z * ACCEL_MS2_FACTOR;

    // 2. 减去零偏 (如果已校准)
    if (offset->is_calibrated) {
        out_accel_ms2->x -= offset->accel_x;
        out_accel_ms2->y -= offset->accel_y;
        out_accel_ms2->z -= offset->accel_z;
    }
}

// ============================================================================
// 姿态解算：四元数 -> 欧拉角 (Euler Angles)
// ============================================================================

/**
 * @brief 将 Mahony 算法输出的四元数转换为欧拉角 (Pitch, Roll, Yaw)
 * @param[in]  q0, q1, q2, q3 : Mahony 输出的全局四元数 (通常在 MahonyAHRS.c 中声明为 extern)
 * @param[out] euler          : 指向 euler_angles 结构体的指针，用于存储计算得到的欧拉角（单位：弧度）
 * @note  使用的旋转顺序为 Z-Y-X (Yaw-Pitch-Roll)，符合航空航天标准。
 *        如果需要角度 (Degree)，请在外部乘以 (180.0f / PI)。
 */
void driver_bmi088_quaternion_to_euler(float q0, float q1, float q2, float q3, struct euler_angles *euler)
{
    // 提前计算四元数的乘积，减少运算量
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 计算 Pitch (俯仰角)，绕 Y 轴旋转
    // asin() 范围是 [-pi/2, pi/2]
    // 限制在 [-1, 1] 之间防止浮点误差导致 asin 报错 (NaN)
    float sin_p = 2.0f * (q0q2 - q1q3);
    if (sin_p > 1.0f) sin_p = 1.0f;
    if (sin_p < -1.0f) sin_p = -1.0f;
    euler->pitch_rad = asinf(sin_p);

    // 计算 Roll (横滚角)，绕 X 轴旋转
    // atan2() 范围是 [-pi, pi]
    euler->roll_rad = atan2f(2.0f * (q0q1 + q2q3), 1.0f - 2.0f * (q1q1 + q2q2));

    // 计算 Yaw (偏航角)，绕 Z 轴旋转
    // atan2() 范围是 [-pi, pi]
    euler->yaw_rad = atan2f(2.0f * (q0q3 + q1q2), 1.0f - 2.0f * (q2q2 + q3q3)); 

    // 计算对应的角度 (Degree)
    const float RAD_TO_DEG = 57.2957795131f; // 180 / PI
    euler->pitch_deg = euler->pitch_rad * RAD_TO_DEG;
    euler->roll_deg  = euler->roll_rad  * RAD_TO_DEG;
    euler->yaw_deg   = euler->yaw_rad   * RAD_TO_DEG;
}
