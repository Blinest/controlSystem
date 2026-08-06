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
 *   0x03  喷管角度控制   special_addr(1) + direction(1) + angle(2)
 *   0x04  喷管偏转控制	status(1)
 *   0x04  SS电机测试 id(1) + direction(1) + speed_x100(2) + angle_x100(2)
 */

#include "pc_cmd_parser.h"
#include "LYZ/LYZ.h"
#include "Motor/Motor.h"
#include "usart.h"

/* 解析缓冲区 */
#define RX_BUF_SIZE 512

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
	uint8_t dir = s_buf[3];
    switch (func)
    {
    case FUNC_CLOSE:

    	break;

    case FUNC_ENABLE:
    	/* 空操作 */
    	break;

    case FUNC_ESTOP:
    	motor_stop_all();
    	/* 同步清零 SW 状态 */
    	for (int i = 0; i < MOTOR_NUM; i++)
    	{
    		LYZ.target_theta = 0.0f;
    		LYZ.current_theta = 0.0f;
    	}
    	break;

    case FUNC_DEFLECT:
    	uint8_t special_id = s_buf[4];
    	uint16_t val = (s_buf[5] << 8) | s_buf[6];
    	float fval = (float)val / 100.0f;
    	switch (special_id)
    	{
    		case 0xFE:
    			// 喷管偏转控制
    			LYZ_deflect_kinematic_control(dir, fval);
    			break;
    		case 0xFF:
    			// 喷管截面面积控制，接收位移值，这里位移值单位为mm，
    			LYZ_cross_section_kinematic_control(dir, fval);
    			break;
    	}

        break;

    case FUNC_REVERSER:
    	if (dir == 0)
    	{
    		LYZ_thrust_reverser_kinematic_control(dir,0);
    	} else
    	{
    		LYZ_thrust_reverser_kinematic_control(dir,360 * 35);
    	}

    	break;
    case FUNC_HOMING:
		LYZ_homing();
        break;

    default:
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
