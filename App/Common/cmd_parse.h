/**
* @file cmd_parse.h
 * @brief 指令解析模块：解析串口指令帧并触发电机/传感器控制
 *
 * 电机帧格式：[0xAA][cmd][数据...]
 *   cmd = 0x01: 紧急停止
 *   cmd = 0x02: 单机运行 [addr][speed_L][speed_H][pos_L][pos_H]
 *   cmd = 0x03: 多机同步 [pos1_L][pos1_H]...[pos6_L][pos6_H]
 *
 * 用法：在接收任务中每收到一字节调用 cmd_parse_feed_byte(byte)，
 *       解析到完整帧后内部会执行对应控制并自动复位状态。
 */

#ifndef __CMD_PARSE_H
#define __CMD_PARSE_H

#include <stdint.h>
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"

// 喂入一字节数据，处理来自 PC 的控制指令 (0xAA/0xBB)
void cmd_parse_feed_byte(uint8_t byte);

// 喂入一字节数据，处理来自外设的反馈数据 (0xAA)
void cmd_parse_feed_periph_byte(uint8_t byte);

// 重置控制指令解析状态机
void cmd_parse_reset(void);

// 重置外设反馈解析状态机
void cmd_periph_reset(void);

// 大端序读取 short
int16_t read_short_be(const uint8_t* buf, uint16_t index);

/**
 * @brief 打包系统状态帧 (test_frame 格式)
 * @param frame 存储打包后的数据缓冲区
 * @param motor_pos 电机位置数组 [MOTOR_NUM][3]
 * @param sensor_angle 传感器角度数组 [SENSOR_NUM][3]
 * @param scale 缩放比例
 * @param state 系统状态
 * @return 打包后的总长度
 */
uint16_t cmd_pack_status_frame(uint8_t* frame, float motor_pos[MOTOR_NUM][3], float sensor_angle[SENSOR_NUM][3], float scale, uint8_t state);

#endif /* __CMD_PARSE_H */