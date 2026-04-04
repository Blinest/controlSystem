/**
 * @file cmd_parse_unified.h
 * @brief 统一的指令解析与打包接口头文件
 * 
 * 提供统一的接口，内部调用三个独立的库：
 * 1. pc_cmd_parser - 上位机指令解析库
 * 2. periph_cmd_parser - 下位机指令解析库
 * 3. cmd_packer - 指令打包库
 * 
 * @date 2026-03-30
 * @author Psyduck
 */

#ifndef __CMD_PARSE_UNIFIED_H
#define __CMD_PARSE_UNIFIED_H

#include <stdint.h>
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "CR/CR.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *   兼容性接口（保持原有函数名）
 * ================================================================ */

/**
 * @brief 喂入一字节数据，处理来自 PC 的控制指令 (0xAA/0xBB)
 * 兼容原有接口，内部调用 pc_cmd_parser_feed_byte
 */
void cmd_parse_feed_byte(uint8_t byte);

/**
 * @brief 喂入一字节数据，处理来自外设的反馈数据 (0xAA)
 * 兼容原有接口，内部调用 periph_cmd_parser_feed_byte
 */
void cmd_parse_feed_periph_byte(uint8_t byte);

/**
 * @brief 基于 Emm42 协议的新外设反馈解析函数
 * 兼容原有接口，内部调用 periph_cmd_parser_feed_byte_new
 */
void cmd_parse_feed_periph_byte_new(uint8_t byte);

/**
 * @brief 重置控制指令解析状态机
 * 兼容原有接口，内部调用 pc_cmd_parser_reset_all
 */
void cmd_parse_reset(void);

/**
 * @brief 重置外设反馈解析状态机
 * 兼容原有接口，内部调用 periph_cmd_parser_reset_all
 */
void cmd_periph_reset(void);

/* ================================================================
 *   打包函数接口（保持原有函数名）
 * ================================================================ */

/**
 * @brief 打包系统状态帧 (test_frame 格式)
 * 兼容原有接口，内部调用 cmd_packer_pack_status_frame
 */
uint16_t cmd_pack_status_frame(uint8_t* frame, GlobalMotor motor[MOTOR_NUM], GlobalSensor sensor[SENSOR_NUM], LQTS lqts, uint8_t state);


/**
 * @brief 打包关键系统状态帧
 * 兼容原有接口，内部调用 cmd_packer_pack_critical_frame
 */
uint16_t cmd_pack_critical_frame(uint8_t* frame, uint8_t state);

/* ================================================================
 *   高级接口
 * ================================================================ */

/**
 * @brief 发送系统状态帧到上位机
 * 内部调用 cmd_packer_send_status_frame
 */
void cmd_send_status_frame(void);





#ifdef __cplusplus
}
#endif

#endif /* __CMD_PARSE_UNIFIED_H */