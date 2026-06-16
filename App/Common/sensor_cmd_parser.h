/**
 * @file sensor_cmd_parser.h
 * @brief 下位机指令解析库头文件
 * 
 * 负责解析外设（电机/传感器）反馈的指令，更新全局结构体
 * 
 * @date 2026-04-23
 * @author blin
 */

#ifndef SENSOR_CMD_PARSER_H
#define SENSOR_CMD_PARSER_H

#include <stdint.h>
#include "Sensor/Sensor.h"          // SensorContext, SENSOR_NUM
#include "cmd_packer.h"      // cmd_packer_send_status_frame 的新接口

struct ContinuumRobot;
typedef struct ContinuumRobot ContinuumRobot;

/* 帧头定义 */
#define SENSOR_ID           0x01    // IMU 传感器地址
#define CMCU_ADDR_DEFAULT   0x02    // CMCU 压力传感器默认地址

/* 功能码定义 */
#define FUNC_SENSOR_FEEDBACK  0x03  // 传感器反馈功能码
#define CMCU_FUNC_READ        0x03  // 读保持寄存器
#define CMCU_FUNC_WRITE       0x06  // 写单个寄存器

/* 初始化解析器 */
void SensorParser_Init(SensorParser *parser);
void SensorContext_Init(SensorContext *context);

/**
 * @param motors  电机数组指针（用于状态打包）
 * @param lqts    系统状态指针（用于状态打包）
 * @note  若一帧有效且校验通过，会更新 sensors 并自动调用打包发送函数
 */
void SensorParser_IMU_Feed(uint8_t byte,
		               SensorContext* sensors,
		               const MotorContext* motors,
		               const ContinuumRobot* CR);

void CMCU_Parser_Feed(uint8_t byte,
				  SensorContext* sensors,
				  const MotorContext* motors,
				  ContinuumRobot* CR);
#endif