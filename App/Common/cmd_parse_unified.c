/**
 * @file cmd_parse_unified.c
 * @brief 统一的指令解析与打包接口实现
 * 
 * 提供统一的接口，内部调用三个独立的库
 * 
 * @date 2026-03-30
 * @author Psyduck
 */

#include "cmd_parse_unified.h"
#include "pc_cmd_parser.h"
#include "periph_cmd_parser.h"
#include "cmd_packer.h"

/* ================================================================
 *   兼容性接口实现
 * ================================================================ */

void cmd_parse_feed_byte(uint8_t byte) {
    pc_cmd_parser_feed_byte(byte);
}



void cmd_parse_reset(void) {
    pc_cmd_parser_reset_all();
}


/* ================================================================
 *   打包函数接口实现
 * ================================================================ */

uint16_t cmd_pack_status_frame(uint8_t* frame, GlobalMotor motor[MOTOR_NUM], GlobalSensor sensor[SENSOR_NUM], float scale, uint8_t state) {
    return cmd_packer_pack_status_frame(frame, motor, sensor, scale, state);
}

/* ================================================================
 *   高级接口实现
 * ================================================================ */

void cmd_send_status_frame(void) {
    cmd_packer_send_status_frame();
}

