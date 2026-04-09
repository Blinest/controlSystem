/**
 * Emm42_V5.0 步进闭环驱动 — 电机控制库
 *
 * Motor 结构体 + 指令构建 + 指令解析
 */

#ifndef EMM42_MOTOR_H
#define EMM42_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Motor.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  功能码
 * ================================================================ */
#define EMM42_FC_MOTOR_ENABLE       0xF3
#define EMM42_FC_SPEED_MODE         0xF6
#define EMM42_FC_POSITION_MODE      0xFD
#define EMM42_FC_IMMEDIATE_STOP     0xFE
#define EMM42_FC_SYNC_MOTION        0xFF
#define EMM42_FC_SET_HOME           0x93
#define EMM42_FC_TRIGGER_HOMING     0x9A
#define EMM42_FC_ABORT_HOMING       0x9C
#define EMM42_FC_READ_HOMING_PARAM  0x22
#define EMM42_FC_WRITE_HOMING_PARAM 0x4C
#define EMM42_FC_READ_HOMING_STATUS 0x3B
#define EMM42_FC_TRIGGER_CALIB      0x06
#define EMM42_FC_CLEAR_POSITION     0x0A
#define EMM42_FC_RELEASE_STALL      0x0E
#define EMM42_FC_FACTORY_RESET      0x0F
#define EMM42_FC_READ_FW_VERSION    0x1F
#define EMM42_FC_READ_PHASE_RL      0x20
#define EMM42_FC_READ_POS_PID       0x21
#define EMM42_FC_READ_BUS_VOLTAGE   0x24
#define EMM42_FC_READ_PHASE_CURRENT 0x27
#define EMM42_FC_READ_ENCODER       0x31
#define EMM42_FC_READ_INPUT_PULSES  0x32
#define EMM42_FC_READ_TARGET_POS    0x33
#define EMM42_FC_READ_REALTIME_TPOS 0x34
#define EMM42_FC_READ_REALTIME_SPD  0x35
#define EMM42_FC_READ_REALTIME_POS  0x36
#define EMM42_FC_READ_POS_ERROR     0x37
#define EMM42_FC_READ_MOTOR_STATUS  0x3A
#define EMM42_FC_READ_DRV_CONFIG    0x42
#define EMM42_FC_READ_SYS_STATUS    0x43
#define EMM42_FC_WRITE_MICROSTEP    0x84
#define EMM42_FC_WRITE_ID_ADDR      0xAE
#define EMM42_FC_WRITE_LOOP_MODE    0x46
#define EMM42_FC_WRITE_OPEN_CURRENT 0x44
#define EMM42_FC_WRITE_DRV_CONFIG   0x48
#define EMM42_FC_WRITE_POS_PID      0x4A
#define EMM42_FC_STORE_SPEED_PARAM  0xF7
#define EMM42_FC_WRITE_VEL_SCALE    0x4F

#define EMM42_DIR_CW    0x00
#define EMM42_DIR_CCW   0x01
#define EMM42_POS_REL   0x00
#define EMM42_POS_ABS   0x01

#define EMM42_ST_OK          0x02
#define EMM42_ST_COND_FAIL   0xE2
#define EMM42_ST_ERROR       0xEE
#define EMM42_ST_REACHED     0x9F

/* ================================================================
 *  返回值
 * ================================================================ */
typedef enum {
    EMM42_OK             =  0,
    EMM42_ERR_TOO_SHORT  = -1,
    EMM42_ERR_UNKNOWN_FC = -2,
    EMM42_ERR_CHECKSUM   = -3,
    EMM42_ERR_LENGTH     = -4,
	EMM42_ERR_ADDR		 = -5
} emm42_result_t;


/* ================================================================
 *  工具
 * ================================================================ */
static inline uint16_t emm42_be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}
static inline uint32_t emm42_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

emm42_result_t emm42_parse_can(GlobalMotor *m, uint32_t can_ext_id,const uint8_t *buf, uint32_t len, bool verify_cs);
emm42_result_t emm42_build(GlobalMotor *m, uint8_t fc);
const char* emm42_cmd_name(uint8_t fc);
void demo_parse(const char *desc, GlobalMotor *m, const uint8_t *buf, int len);
void demo_build(const char *desc, GlobalMotor *m, uint8_t fc);

#ifdef __cplusplus
}
#endif
#endif