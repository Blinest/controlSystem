/**
 * @file pc_cmd_parser.c
 * @brief 上位机指令解析器实现
 *
 * @date 2026-07-24
 * @author blin
 * 
 * 帧格式: [0xAA] [func(1)] [len(1)] [data...] [cs]
 *   cs = 前面所有字节的和 & 0xFF
 *
 * FuncCode:
 *   0x00  关闭失能   (空)
 *   0x01  启动使能   (空)
 *   0x02  紧急停止   (空)
 *   0x03  喷管偏转   special_addr(1) + direction(1) + angle(2)
 *         SpecialAddr:
 *           0xFE  向下偏转
 *           0xFD  向上偏转
 *           0xFC  偏转复位
 */

#include "pc_cmd_parser.h"
#include "SW/SW.h"
#include "Motor/Motor.h"
#include "usart.h"

/* 解析缓冲区 */
#define RX_BUF_SIZE 256

typedef enum {
    ST_HEAD = 0,
    ST_FUNC,
    ST_LEN,
    ST_DATA,
    ST_CS
} ParseState;

static uint8_t s_buf[RX_BUF_SIZE];
static uint8_t s_len;
static uint8_t s_idx;
static ParseState s_state = ST_HEAD;

void pc_cmd_parser_reset(void)
{
    s_state = ST_HEAD;
    s_len = 0;
    s_idx = 0;
}

static void parse_and_execute(void)
{
    uint8_t func = s_buf[1];
    uint8_t dlen = s_buf[2];

    switch (func)
    {
    case FUNC_CLOSE:
        for (int i = 0; i < MOTOR_NUM; i++)
            SW_kinematic_control(i, 0, (float[]){0.0f, 0.0f});
        break;

    case FUNC_ENABLE:
        /* 舵机无需使能，空操作 */
        break;

    case FUNC_ESTOP:
        motor_feedback_reset();
        break;

    case FUNC_BEND:
        if (dlen >= 4)
        {
            uint8_t special = s_buf[3];
            uint8_t dir     = s_buf[4];
            int16_t raw     = (int16_t)((s_buf[5] << 8) | s_buf[6]);
            float angle     = (float)raw / 100.0f;

            if (angle < 0.0f) angle = 0.0f;
            if (angle > 90.0f) angle = 90.0f;

            switch (special)
            {
            case SPECIAL_DOWN:
                SW_kinematic_control(0, 0, ((float[]){angle, SW.joint_space.target_theta[1]}));
                break;
            case SPECIAL_UP:
                SW_kinematic_control(1, dir, ((float[]){SW.joint_space.target_theta[0], angle}));
                break;
            case SPECIAL_RESET:
                for (int i = 0; i < MOTOR_NUM; i++)
                    SW_kinematic_control(i, 0, ((float[]){0.0f, 0.0f}));
                break;
            default:
                break;
            }
        }
        break;

    case FUNC_CYCLE:
        /* 循环动作组: [0x00]=启动, [0x01]=关闭并回初始状态 */
        if (dlen >= 1)
        {
            if (s_buf[3] == CYCLE_START)
                SW_action_group_start_default();
            else if (s_buf[3] == CYCLE_STOP)
                SW_action_group_stop();
        }
        break;
    case FUNC_PARA:
        /* 参数设置: 峰值速度(1, °/s) — 全局统一作用于所有电机, 限幅 0~50 */
        if (dlen >= 1)
        {

            float vel = (float)s_buf[3];
            if (vel > SERVO_SPEED_MAX) vel = SERVO_SPEED_MAX;
            for (int i = 0; i < MOTOR_NUM; i++)
                global_motor[i].servo.target_speed = vel;
        }
        break;
    }
}

void pc_cmd_parser_feed_byte(uint8_t byte)
{
    switch (s_state)
    {
    case ST_HEAD:
        if (byte == FRAME_HEAD)
        {
            s_buf[0] = byte;
            s_idx = 1;
            s_state = ST_FUNC;
        }
        break;

    case ST_FUNC:
        s_buf[1] = byte;
        s_len = 2;
        s_state = ST_LEN;
        break;

    case ST_LEN:
        s_len = byte;
        s_buf[2] = byte;
        if (s_len + 4 > RX_BUF_SIZE) { pc_cmd_parser_reset(); break; }
        s_idx = 3;
        s_state = (s_len == 0) ? ST_CS : ST_DATA;
        break;

    case ST_DATA:
        if (s_idx < RX_BUF_SIZE)
            s_buf[s_idx++] = byte;
        if (s_idx >= (uint8_t)(3 + s_len))
            s_state = ST_CS;
        break;

    case ST_CS:
    {
        s_buf[s_idx] = byte;
        uint16_t sum = 0;
        for (int i = 0; i < s_idx; i++)
            sum += s_buf[i];
        if ((sum & 0xFF) == byte)
            parse_and_execute();
        pc_cmd_parser_reset();
        break;
    }
    default:
        pc_cmd_parser_reset();
        break;
    }
}
