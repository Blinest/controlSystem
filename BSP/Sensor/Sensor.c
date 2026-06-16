/**
 * @file Sensor.c
 * @brief 传感器驱动函数实现
 * 
 * 实现传感器初始化、数据读取、自检等功能
 * 假设使用MPU9250作为主要传感器
 * 
 * @date 2026-04-02
 * @author blin
 */

#include "Sensor.h"
#include "mpu9250.h"
#include <stdio.h>

#include "CMCU-06.h"
#include "IMU.h"


// 初始化全局传感器数组
SensorContext sensor_context[SENSOR_NUM];


/**
 * @brief 传感器初始化函数
 * 
 * 初始化所有传感器，包括：
 * 1. 配置I2C接口
 * 2. 配置传感器参数
 * 3. 执行自检
 */
void sensor_init(void)
{
    // 1. 初始化传感器硬件
    // 这里调用实际的MPU9250初始化函数
	sensor_context->parser_mode = 0;
    // mpu9250_init();
	// IMU_Init();
	CMCU_06_Init();

    // 2. 配置传感器参数
    // 设置量程、采样率、滤波器等
    
    // 3. 初始化传感器数据结构


}

/**
 * @brief 单传感器数据读取函数
 * @param sensor_id 传感器ID (1-4)
 * 
 * 读取指定传感器的数据并更新全局结构体
 */
void sensor_single_read(uint8_t sensor_id)
{
    // IMU数据读取
	// IMU_single_read(sensor_id);
	// 压力传感器数据读取
	CMCU_06_single_read(sensor_id);
}

/**
 * @brief 多传感器数据读取函数
 * 
 * 批量读取所有传感器的数据
 */
void sensor_multi_read(void)
{
    
    for (int i = 0; i < SENSOR_NUM; i++) {
        // 调用单传感器读取函数
        sensor_single_read(i + 1);
    }

}


/**
 * @brief 获取传感器原始数据
 * @param sensor_id 传感器ID
 * @param x 指向X轴数据的指针
 * @param y 指向Y轴数据的指针
 * @param z 指向Z轴数据的指针
 */
void sensor_get_raw_data(uint8_t sensor_id, int16_t* x, int16_t* y, int16_t* z)
{
    if (sensor_id < 1 || sensor_id > SENSOR_NUM) {
        *x = *y = *z = 0;
        return;
    }
    
    int idx = sensor_id - 1;
    *x = (int16_t)sensor_context[idx].global_sensor.imu.pitch;
    *y = (int16_t)sensor_context[idx].global_sensor.imu.roll;
    *z = (int16_t)sensor_context[idx].global_sensor.imu.yaw;
}

/**
 * @brief 获取传感器角度数据
 * @param sensor_id 传感器ID
 * @param pitch 指向俯仰角的指针
 * @param roll 指向横滚角的指针
 * @param yaw 指向偏航角的指针
 * 
 * 将原始数据转换为角度值（单位：度）
 */
void sensor_get_angle_data(uint8_t sensor_id, float* pitch, float* roll, float* yaw)
{
    if (sensor_id < 1 || sensor_id > SENSOR_NUM) {
        *pitch = *roll = *yaw = 0.0f;
        return;
    }
    
    int idx = sensor_id - 1;
    
    // 假设原始数据单位是0.01度
    *pitch = (float)sensor_context[idx].global_sensor.imu.pitch / 100.0f;
    *roll = (float)sensor_context[idx].global_sensor.imu.roll / 100.0f;
    *yaw = (float)sensor_context[idx].global_sensor.imu.yaw / 100.0f;
}