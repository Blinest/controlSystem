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

// 喂入一字节数据，通过状态机完成状态解析
void cmd_parse_feed_byte(uint8_t byte);

// 重置状态机
void cmd_parse_reset(void);

#endif /* __CMD_PARSE_H */