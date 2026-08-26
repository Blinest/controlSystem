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
#define FUNC_BEND       0x03
#define FUNC_CYCLE      0x05

#define FUNC_PARA		0x07
#define SPECIAL_UP      0xFD
#define SPECIAL_DOWN    0xFE
#define SPECIAL_RESET   0xFC

/* FUNC_CYCLE 动作组控制 */
#define CYCLE_STOP      0x01   /* AA 05 01 cs: 关闭循环并回初始状态 */
#define CYCLE_START     0x00   /* AA 05 00 cs: 启动循环动作组 */

/**
 * @brief 喂一字节给帧状态机，完整帧校验通过后自动执行
 */
void pc_cmd_parser_feed_byte(uint8_t byte);

/**
 * @brief 重置解析器状态
 */
void pc_cmd_parser_reset(void);

#endif
