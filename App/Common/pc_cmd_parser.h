/**
 * @file pc_cmd_parser.h
 * @brief 上位机指令解析器 — 帧协议 [0xAA][func][len][data...][cs]
 * 
 * @date 2026-07-24
 * @author blin
 */
#ifndef __PC_CMD_PARSER_H
#define __PC_CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/* 帧定义 */
#define FRAME_HEAD      0xAA
#define FUNC_CLOSE      0x00
#define FUNC_ENABLE     0x01
#define FUNC_ESTOP      0x02
#define FUNC_DEFLECT    0x03
#define FUNC_REVERSER	0x04
#define FUNC_HOMING		0x05

#define SPECIAL_UP      0xFD
#define SPECIAL_DOWN    0xFE
#define SPECIAL_RESET   0xFC

/**
 * @brief 喂一字节给帧状态机，完整帧校验通过后自动执行
 */
void pc_cmd_parser_feed_byte(uint8_t byte);

/**
 * @brief 重置解析器状态
 */
void pc_cmd_parser_reset(void);

#endif
