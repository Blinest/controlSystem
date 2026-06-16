/**
 * @file Sensor.h
 * @brief 传感器驱动头文件
 * 
 * 定义传感器数据结构和函数接口
 * 
 * @date 2026-04-02
 * @author Psyduck
 */

#ifndef CONTROLSYSTEM_SENSOR_H
#define CONTROLSYSTEM_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "SensorParser.h"

#define SENSOR_NUM 1

/**
 * @brief IMU 数据结构（纯数据）
 */
typedef struct {
	float pitch;   /**< X轴角度 */
	float roll;    /**< Y轴角度 */
	float yaw;     /**< Z轴角度 */
	uint8_t id;    /**< 传感器ID */
} IMU;

/**
 * @brief 压力传感器数据结构（纯数据）
 */
typedef struct {
	float val;        /**< 转换后的压力值（单位：根据实际定义） */
	int32_t raw_val;  /**< 原始32位有符号值 */
	uint8_t id;       /**< 传感器ID */
} PressSensor;

/**
 * @brief 全局传感器数据结构（纯数据聚合）
 */
typedef struct {
	IMU imu;
	PressSensor press_sensor;
} GlobalSensor;

typedef enum {
	SENSOR_PARSER_MODE_NONE = 0,
	SENSOR_PARSER_MODE_IMU,
	SENSOR_PARSER_MODE_CMCU
} SensorParserMode;

/**
 * @brief 传感器解析上下文（参考 MotorContext 设计）
 * 将传感器数据与对应的协议解析器封装在一起
 */
typedef struct {
	GlobalSensor global_sensor;   /**< 传感器物理数据 */
	// 结构体：IMU 和压力传感器使用不同的解析器
	// SensorParser   imu_parser;   /**< IMU 协议解析器 */
	// CMCU_Parser    press_parser; /**< 压力传感器协议解析器 */

	//使用联合体（如果同一时刻只有一种传感器工作，可节省内存）
	union {
	    SensorParser   imu_parser;
	    CMCU_Parser    press_parser;
	} parser;

	SensorParserMode parser_mode;

	// 可选：超时、状态标志等
	uint8_t last_response_time;
	uint8_t timeout_threshold;
	bool    is_online;
} SensorContext;

/**
 * @brief 传感器初始化函数
 */
void sensor_init(void);

/**
 * @brief 单传感器数据读取函数
 * @param sensor_id 传感器ID (1-4)
 */
void sensor_single_read(uint8_t sensor_id);

/**
 * @brief 多传感器数据读取函数
 */
void sensor_multi_read(void);

/**
 * @brief 获取传感器原始数据
 * @param sensor_id 传感器ID
 * @param x 指向X轴数据的指针
 * @param y 指向Y轴数据的指针
 * @param z 指向Z轴数据的指针
 */
void sensor_get_raw_data(uint8_t sensor_id, int16_t* x, int16_t* y, int16_t* z);

/**
 * @brief 获取传感器角度数据
 * @param sensor_id 传感器ID
 * @param pitch 指向俯仰角的指针
 * @param roll 指向横滚角的指针
 * @param yaw 指向偏航角的指针
 */
void sensor_get_angle_data(uint8_t sensor_id, float* pitch, float* roll, float* yaw);

// 全局传感器数组声明
extern SensorContext sensor_context[SENSOR_NUM];

#endif //CONTROLSYSTEM_SENSOR_H