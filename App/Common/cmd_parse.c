/**
* @file cmd_parse.c
 * @brief 指令解析实现：状态机 + 电机/传感器执行
 *
 * 帧格式统一为：
 *   Byte0: 帧头       - 0xAA: 电机指令; 0xBB: 传感器指令
 *   Byte1: 功能码     - 区分具体功能（单电机控制、多电机同步等）
 *   Byte2: 数据长度 L - （可选）仅当该功能需要数据时存在，表示后续数据字节数，2^8-1=255字节上限
 *   Byte3+: 数据区    - 长度为 L 的参数区（电机地址、个数、位移等）
 *
 * 对于"无数据"的功能（例如：整体停止、多电机基于运动学启动），
 * 帧可以仅由 [帧头][功能码] 两字节组成，不包含长度与数据。
 */

#include "cmd_parse.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"

/* 电机相关过程函数库 */
#include "Motor/motor.h"
/* 传感器相关过程函数库 */
#include "Sensor/IMU.h"

/* 引用外部队列 */
extern osMessageQueueId_t SensorMessageQueueHandle;

/* 引用全局状态变量 (定义在 CmdCtrlTask.c 中) */
extern float motor_pos[MOTOR_NUM][3];
extern float sensor_angle[SENSOR_NUM][3];
extern float scale_val;
extern uint8_t sys_state;
extern bool is_connected;

/* 帧头定义 */
#define FRAME_HEAD_MOTOR      0xAA    /* 电机指令帧头 */
#define FRAME_HEAD_SENSOR     0xBB    /* 传感器指令帧头 */
#define FRAME_HEAD_PERIPH     0xAA    /* 外设反馈帧头 */

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
 CMD_STATE_CHECK       /* 等待校验和字节 */
} CmdParseState_t;

/* --- 控制指令解析缓冲区与状态 --- */
#define CTRL_BUF_SIZE  32
static uint8_t s_ctrlBuf[CTRL_BUF_SIZE];
static uint8_t s_ctrlLen;
static uint8_t s_ctrlIdx;
static CmdParseState_t s_ctrlState = CMD_STATE_HEAD;

/* --- 外设反馈解析缓冲区与状态 --- */
#define PERIPH_BUF_SIZE  64
static uint8_t s_periphBuf[PERIPH_BUF_SIZE];
static uint8_t s_periphLen;
static uint8_t s_periphIdx;
static CmdParseState_t s_periphState = CMD_STATE_HEAD;

/* 过程函数声明 */
static void cmd_ctrl_reset(void);
static void cmd_periph_reset_internal(void);
static void cmd_parse_and_execute(void);

/**
 * @brief 内部调用：将系统状态打包并发送至反馈队列
 */
static void send_test_frame_to_queue(void) {
    uint8_t frame[64];
    uint16_t len = cmd_pack_status_frame(frame, motor_pos, sensor_angle, scale_val, sys_state);

    // 发送到 SensorMessageQueue 缓冲，由 DataTask 统一调度发送
    for (int i = 0; i < len; i++) {
        uint16_t msg = frame[i];
        osMessageQueuePut(SensorMessageQueueHandle, &msg, 0, 0);
    }
}

/* 传感器相关过程函数 */


/* --- 状态机相关 --- */

/**
 * @brief 控制指令解析函数 (PC -> STM32)
 * @param byte 接收到的单字节数据
 */
void cmd_parse_feed_byte(uint8_t byte)
{
	uint8_t receive = byte;
	switch (s_ctrlState)
	{
		case CMD_STATE_HEAD:
			if (receive == FRAME_HEAD_MOTOR || receive == FRAME_HEAD_SENSOR)
			{
				// 解析下一个字节，并将帧头存入发送数据缓冲区
				s_ctrlBuf[0] = receive;
				s_ctrlIdx = 1;
				s_ctrlState = CMD_STATE_FUNC;
			}
			break;
		/* 根据帧头进行不同的控制操作 */
		case CMD_STATE_FUNC:
			/* 保存功能码 */
			s_ctrlBuf[1] = receive;
			s_ctrlLen = 2;
			if (s_ctrlBuf[0] == FRAME_HEAD_MOTOR)
			{
				// 根据功能码执行特定动作
				switch ((MotorFuncCode_t)receive)
				{
					case FUNC_MOTOR_SINGLE:
					case FUNC_MOTOR_KINEMATIC:
					case FUNC_MOTOR_CUSTOM:
						s_ctrlState = CMD_STATE_LEN;
						break;
					case FUNC_MOTOR_ENABLE:
					case FUNC_MOTOR_STOP:
						s_ctrlLen = 0;
						// 进入指令解析执行函数，本部分不提供具体的函数实现
						cmd_parse_and_execute();
						cmd_ctrl_reset();
						break;
					default:
						cmd_ctrl_reset();
						break;
				}
			}
			else if (s_ctrlBuf[0] == FRAME_HEAD_SENSOR)
			{
				switch ((SensorFuncCode_t)receive)
				{
					case FUNC_SENSOR_INIT:
						cmd_parse_and_execute();
						cmd_ctrl_reset();
						break;
					case FUNC_SENSOR_SELF_TEST:
						s_ctrlState = CMD_STATE_LEN;
						break;
					default:
						cmd_ctrl_reset();
						break;
				}
			}
			break;
		case CMD_STATE_LEN:
			s_ctrlLen = receive;
			s_ctrlBuf[2] = receive;
			if (s_ctrlLen + 4 > CTRL_BUF_SIZE) // 帧头(1) + 功能(1) + 长度(1) + 校验(1)
			{
				cmd_ctrl_reset();
				break;
			}
			s_ctrlIdx = 3;
			s_ctrlState = (s_ctrlLen == 0) ? CMD_STATE_CHECK : CMD_STATE_DATA;
			break;
		case CMD_STATE_DATA:
			s_ctrlBuf[s_ctrlIdx++] = receive;
			if (s_ctrlIdx >= (uint8_t)(3 + s_ctrlLen))
			{
				s_ctrlState = CMD_STATE_CHECK;
			}
			break;
		case CMD_STATE_CHECK:
			s_ctrlBuf[s_ctrlIdx] = receive; // 存入校验位
			uint16_t sum = 0;
			for (int i = 0; i < s_ctrlIdx; i++) sum += s_ctrlBuf[i];
			if ((sum & 0xFF) == receive) {
				is_connected = true; // 收到暗号，解锁连接
				cmd_parse_and_execute();
			}
			cmd_ctrl_reset();
			break;
		default:
			cmd_ctrl_reset();
			break;
	}
}

/**
 * @brief 外设反馈解析函数 (Peripheral -> STM32)
 * @param byte 接收到的单字节数据
 */
void cmd_parse_feed_periph_byte(uint8_t byte)
{
	uint8_t receive = byte;
	switch (s_periphState)
	{
		case CMD_STATE_HEAD:
			if (receive == FRAME_HEAD_PERIPH)
			{
				s_periphBuf[0] = receive;
				s_periphIdx = 1;
				s_periphState = CMD_STATE_FUNC;
			}
			break;
		case CMD_STATE_FUNC:
			s_periphBuf[1] = receive;
			s_periphIdx = 2;
			s_periphState = CMD_STATE_LEN;
			break;
		case CMD_STATE_LEN:
			s_periphLen = receive;
			s_periphBuf[2] = receive;
			if (s_periphLen > (PERIPH_BUF_SIZE - 4) || s_periphLen == 0)
			{
				cmd_periph_reset_internal();
				break;
			}
			s_periphIdx = 3;
			s_periphState = CMD_STATE_DATA;
			break;
		case CMD_STATE_DATA:
		// 数据解析
			if (s_periphIdx < PERIPH_BUF_SIZE)
				s_periphBuf[s_periphIdx++] = receive;

			if (s_periphIdx >= (uint8_t)(3 + s_periphLen + 1)) // 帧头+功能+长度 + 数据 + 校验
			{
				// 字节校验，累加数据字节并取低八位
				uint16_t sum = 0;
				for (int i = 0; i < s_periphIdx - 1; i++) sum += s_periphBuf[i];
				if ((sum & 0xFF) == receive) {
					uint8_t func = s_periphBuf[1];
					if (func == 0x01 && s_periphLen == 7) {
						uint8_t m_id = s_periphBuf[3];
						if (m_id >= 1 && m_id <= MOTOR_NUM) {
							motor_pos[m_id-1][0] = (float)read_short_be(s_periphBuf, 4) / 100.0f; // 位移
							motor_pos[m_id-1][1] = (float)read_short_be(s_periphBuf, 6) / 100.0f; // 速度
							motor_pos[m_id-1][2] = (float)read_short_be(s_periphBuf, 8) / 100.0f; // 加速度
						}
					} else if (func == 0x03 && s_periphLen == 7) {
						uint8_t s_id = s_periphBuf[3];
						if (s_id >= 1 && s_id <= SENSOR_NUM) {
							sensor_angle[s_id-1][0] = (float)read_short_be(s_periphBuf, 4) / 100.0f; // 俯仰
							sensor_angle[s_id-1][1] = (float)read_short_be(s_periphBuf, 6) / 100.0f; // 横滚
							sensor_angle[s_id-1][2] = (float)read_short_be(s_periphBuf, 8) / 100.0f; // 偏航
						}
					}
					send_test_frame_to_queue();
				}
				cmd_periph_reset_internal();
			}
			break;
		default:
			cmd_periph_reset_internal();
			break;
	}
}

/**
 * @brief 解析并执行控制指令
 */
static void cmd_parse_and_execute(void)
{
	uint8_t head     = s_ctrlBuf[0];
	uint8_t func     = s_ctrlBuf[1];
	uint8_t data_len = s_ctrlLen;
	if (data_len > 0 && (uint8_t)(3 + data_len) > CTRL_BUF_SIZE)
		return;
	switch (head)
	{
		case FRAME_HEAD_MOTOR:
			switch (func)
			{
				case FUNC_MOTOR_SINGLE:

					break;
				case FUNC_MOTOR_KINEMATIC: // TODO: 基于运动学的电机控制函数
				case FUNC_MOTOR_SYNC: // TODO: 多电机同步控制
					for (int i = 0; i < data_len; i++)
					{

					}
				case FUNC_MOTOR_CUSTOM: // TODO: 测试函数

					break;
				case FUNC_MOTOR_ENABLE: // TODO: 电机使能
					for (int i = 0; i < MOTOR_NUM; i++)
					{
						motor_enable(motor[i].id);
					}
					break;
				case FUNC_MOTOR_STOP: // TODO: 电机停止
					motor_stop();
					break;
				default:
					break;
			}
            break;
		case FRAME_HEAD_SENSOR:
			switch (func)
			{
				case FUNC_SENSOR_INIT: // TODO: 传感器初始化函数
				case FUNC_SENSOR_SINGLE_RD: // TODO: 单传感器数据读取函数
				case FUNC_SENSOR_MULTI_RD: // TODO: 多传感器数据读取函数
				case FUNC_SENSOR_SELF_TEST: // TODO: 传感器自检函数
				default:
					break;
			}
            break;
		default:
			break;
	}
}

/**
 * @brief 重置控制指令解析状态
 */
void cmd_parse_reset(void) {
	cmd_ctrl_reset();
}

static void cmd_ctrl_reset(void) {
	s_ctrlState = CMD_STATE_HEAD;
	s_ctrlLen = 0;
	s_ctrlIdx = 0;
}

/**
 * @brief 重置外设反馈解析状态
 */
void cmd_periph_reset(void) {
	cmd_periph_reset_internal();
}

static void cmd_periph_reset_internal(void) {
	s_periphState = CMD_STATE_HEAD;
	s_periphLen = 0;
	s_periphIdx = 0;
}

/**
 * @brief 打包系统状态帧 (test_frame 格式)
 * @param frame 存储打包后的数据缓冲区
 * @param motor_pos 电机位置数组 [MOTOR_NUM][3]
 * @param sensor_angle 传感器角度数组 [SENSOR_NUM][3]
 * @param scale 缩放比例
 * @param state 系统状态
 * @return 打包后的总长度
 */
uint16_t cmd_pack_status_frame(uint8_t* frame, float motor_pos[MOTOR_NUM][3], float sensor_angle[SENSOR_NUM][3], float scale, uint8_t state) {
    uint16_t idx = 0;

    frame[idx++] = 0xBB; // 帧头
    frame[idx++] = FUNC_SENSOR_MULTI_RD; // 功能码
    frame[idx++] = 47;   // 数据长度 (MOTOR_NUM(1) + SENSOR_NUM(1) + 18 + 24 + 2 + 1)
    frame[idx++] = MOTOR_NUM;
    frame[idx++] = SENSOR_NUM;

    // 电机数据
    for (int i = 0; i < MOTOR_NUM; i++) {
        int16_t x = (int16_t)(motor_pos[i][0] * 100);
        int16_t y = (int16_t)(motor_pos[i][1] * 100);
        int16_t z = (int16_t)(motor_pos[i][2] * 100);
        frame[idx++] = (x >> 8) & 0xFF; frame[idx++] = x & 0xFF;
        frame[idx++] = (y >> 8) & 0xFF; frame[idx++] = y & 0xFF;
        frame[idx++] = (z >> 8) & 0xFF; frame[idx++] = z & 0xFF;
    }

    // 传感器数据
    for (int i = 0; i < SENSOR_NUM; i++) {
        int16_t x = (int16_t)(sensor_angle[i][0] * 100);
        int16_t y = (int16_t)(sensor_angle[i][1] * 100);
        int16_t z = (int16_t)(sensor_angle[i][2] * 100);
        frame[idx++] = (x >> 8) & 0xFF; frame[idx++] = x & 0xFF;
        frame[idx++] = (y >> 8) & 0xFF; frame[idx++] = y & 0xFF;
        frame[idx++] = (z >> 8) & 0xFF; frame[idx++] = z & 0xFF;
    }

    // scale & state
    int16_t s_val = (int16_t)(scale * 100);
    frame[idx++] = (s_val >> 8) & 0xFF; frame[idx++] = s_val & 0xFF;
    frame[idx++] = state;

    // 校验和
    uint16_t checksum = 0;
    for (int i = 0; i < idx; i++) {
        checksum += frame[i];
    }
    frame[idx++] = checksum & 0xFF;

    return idx;
}

// 辅助函数：大端序读取 short
int16_t read_short_be(const uint8_t* buf, uint16_t index) {
    return (int16_t)((buf[index] << 8) | buf[index + 1]);
}