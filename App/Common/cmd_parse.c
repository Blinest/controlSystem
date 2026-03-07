/**
* @file cmd_parse.c
 * @brief 指令解析实现：状态机 + 电机/传感器执行
 *
 * 帧格式统一为：
 *   Byte0: 帧头       - 0xAA: 电机指令; 0xBB: 传感器指令
 *   Byte1: 功能码     - 区分具体功能（单电机控制、多电机同步等）
 *   Byte2: 数据长度 L - （可选）仅当该功能需要数据时存在，表示后续数据字节数
 *   Byte3+: 数据区    - 长度为 L 的参数区（电机地址、个数、位移等）
 *
 * 对于"无数据"的功能（例如：整体停止、多电机基于运动学启动），
 * 帧可以仅由 [帧头][功能码] 两字节组成，不包含长度与数据。
 */

#include "cmd_parse.h"
#include "usart.h"
/* 帧头定义 */
#define FRAME_HEAD_MOTOR      0xAA    /* 电机指令帧头 */
#define FRAME_HEAD_SENSOR     0xBB    /* 传感器指令帧头 */



/* 电机功能码定义 */
typedef enum
{
 FUNC_MOTOR_ENABLE     = 0x01,   /* 电机使能 */
 FUNC_MOTOR_STOP       = 0x02,   /* 多电机停止指令 */
 FUNC_MOTOR_SINGLE     = 0x03,   /* 单电机控制：addr + direction + distance */
 FUNC_MOTOR_SYNC       = 0x04,   /* 多电机同步：count + start_addr + distances... */
 FUNC_MOTOR_KINEMATIC  = 0x05,   /* 基于运动学的协同控制（无数据段） */
 FUNC_MOTOR_CUSTOM     = 0x06,   /* 自定义多电机控制：count + [addr, direction, distance]... */
} MotorFuncCode_t;

/* 传感器功能码定义 */
typedef enum
{
 FUNC_SENSOR_INIT      = 0x00,   /* 传感器初始化（无数据段） */
 FUNC_SENSOR_SINGLE_RD = 0x01,   /* 单传感器数据读取（需要传感器ID） */
 FUNC_SENSOR_MULTI_RD  = 0x02,   /* 多传感器批量读取（无数据段） */
 FUNC_SENSOR_SELF_TEST = 0x03,   /* 传感器自检（需要传感器ID） */
} SensorFuncCode_t;

/* 帧解析状态机 */
typedef enum {
 CMD_STATE_HEAD = 0,   /* 等待帧头 0xAA / 0xBB */
 CMD_STATE_FUNC,       /* 已收到帧头，等待功能码 */
 CMD_STATE_LEN,        /* 已收到功能码，等待数据长度 L */
 CMD_STATE_DATA,       /* 已知 L，接收 L 字节数据 */
} CmdParseState_t;

/* 发送指令缓冲区参数定义 */
#define CMD_BUF_SIZE  32
static uint8_t s_cmdBuf[CMD_BUF_SIZE];     /* 指令缓冲区 */
static uint8_t s_cmdLen;                  /* 数据长度 L（仅数据区字节数），无数据则为 0 */
static uint8_t s_cmdIdx;                  /* 当前缓冲区索引 */
static CmdParseState_t s_state = CMD_STATE_HEAD; /* 当前解析状态 */

/* 过程函数声明 */
static void cmd_reset(void);
static void cmd_parse_and_execute(void);

/* 电机相关过程函数 */
/*
 * TODO:
 * 电机初始化函数
 * 电机使能函数
 * 单电机位置控制函数
 * 多电机位置控制函数
 * 基于运动学算法的多电机位置控制函数
 * 测试函数(自定义)
*/

/* 传感器相关过程函数 */
/*
 * TODO:
 * 传感器初始化函数
 * 单传感器数据读取函数
 * 多传感器数据读取函数
 * 测试函数(自定义)
*/


/* 状态机相关 */
/**
 * @brief 状态机核心解析函数
 * @param byte
*/
void cmd_parse_feed_byte(uint8_t byte)
{
	uint8_t receive = byte;
	switch (s_state)
	{
		case CMD_STATE_HEAD:
			/* 识别有效帧头: AA/BB */
			if (receive == FRAME_HEAD_MOTOR || receive == FRAME_HEAD_SENSOR)
			{
				// 解析下一个字节，并将帧头存入发送数据缓冲区
				s_cmdBuf[0] = receive;
				s_cmdIdx = 1;
				s_state = CMD_STATE_FUNC;
			}
			break;
		/* 根据帧头进行不同的控制操作 */
		case CMD_STATE_FUNC:
			/* 保存功能码 */
			s_cmdBuf[1] = receive;
			s_cmdLen = 2;
			if (s_cmdBuf[0] == FRAME_HEAD_MOTOR)
			{
				// 根据功能码执行特定动作
				switch ((MotorFuncCode_t)receive)
				{
					case FUNC_MOTOR_SINGLE:
					case FUNC_MOTOR_KINEMATIC:
					case FUNC_MOTOR_CUSTOM:
						s_state = CMD_STATE_LEN;
						break;
					case FUNC_MOTOR_ENABLE: // TODO: 电机使能函数
					case FUNC_MOTOR_STOP: // TODO: 电机停止函数
						s_cmdLen = 0;
						// 进入指令解析执行函数，本部分不提供具体的函数实现
						cmd_parse_and_execute();
						// 状态重置，等待下一个指令
						cmd_reset();
						break;
					default:
						// 无效功能码，直接重置状态
						cmd_reset();
						break;
					;
				}
			}
			else if (s_cmdBuf[0] == FRAME_HEAD_SENSOR)
			{
				switch ((SensorFuncCode_t)receive)
				{
					case FUNC_SENSOR_INIT:
						cmd_parse_and_execute();
						cmd_reset();
						break;
					case FUNC_SENSOR_SELF_TEST:
						s_state = CMD_STATE_LEN;
						break;
					default:
						/* 无效功能码，重置状态 */
						cmd_reset();
						break;
				}
			}
			else
			{
				// 无效帧头，直接重置
				cmd_reset();
				break;
			}
			break;
		case CMD_STATE_LEN:
			/* 保存数据长度 L */
			s_cmdLen = receive;
			s_cmdBuf[2] = receive;
			// 过滤非法数据长度
			if (s_cmdLen + 3 > CMD_BUF_SIZE || s_cmdLen == 0)
			{
				cmd_reset();
				break;
			}
			s_cmdIdx = 3;
			s_state = CMD_STATE_DATA;
			break;
		case CMD_STATE_DATA:
			/* 数据段接收 */
		/* 接收数据字节 */
		if (s_cmdIdx < CMD_BUF_SIZE)
			s_cmdBuf[s_cmdIdx++] = receive;

		/* 检查是否接收完成 */
		if (s_cmdIdx >= (uint8_t)(3 + s_cmdLen))
		{
			/* 已接收 [帧头][功能码][Len] + Len 字节的数据 */
			cmd_parse_and_execute();
			cmd_reset();
		}
		break;
		default:
			cmd_reset();
			break;
	}
}

/**
 * @brief 解析并执行完整指令
 * @details 根据帧头和功能码执行相应的操作
 */
static void cmd_parse_and_execute(void)
{
	uint8_t head     = s_cmdBuf[0];
	uint8_t func     = s_cmdBuf[1];
	uint8_t data_len = s_cmdLen;
	uint8_t *data    = (data_len > 0) ? &s_cmdBuf[3] : NULL;
	/* 长度安全检查：若有数据，则应保证缓冲区足够 */
	if (data_len > 0 && (uint8_t)(3 + data_len) > CMD_BUF_SIZE)
		return;
	switch (head)
	{
		case FRAME_HEAD_MOTOR:
			switch (func)
			{
				case FUNC_MOTOR_SINGLE: // TODO: 测试函数(自定义)
				case FUNC_MOTOR_KINEMATIC: // TODO: 基于运动学的电机控制函数
				case FUNC_MOTOR_SYNC: // TODO: 多电机同步控制
				case FUNC_MOTOR_CUSTOM: // TODO: 测试函数(自定义)

				default:
					break;
			}
		case FRAME_HEAD_SENSOR:
		default:
			break;
	}
}