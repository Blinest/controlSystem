/**
 * @file cmd_packer.c
 * @brief 指令打包库
 *
 * 负责将全局结构体中的数据打包成数据包，发送给上位机
 *
 * @date 2026-03-30
 * @author blin
 */

#include "cmd_packer.h"
#include "cmsis_os2.h"
#include "SW/SW.h"
// 引用外部队列
extern osMessageQueueId_t SensorMessageQueueHandle;

/**
 * @brief 打包系统状态帧
 * @param frame 存储打包后的数据缓冲区
 * @param motor 电机全局结构体
 * @param state 系统状态
 * @return 打包后的总长度
 */
uint16_t cmd_packer_pack_status_frame(uint8_t* frame, GlobalMotor motor[MOTOR_NUM] , uint8_t state) {
    // 打包帧格式：帧头 + 数据长度 + 电机数量 + 传感器数量
	uint16_t idx = 0;
    frame[idx++] = 0xCC; // 帧头, 用于设备标识
    frame[idx++] = 0x02; // 用于设备标识

	// 预留数据长度字节的位置，稍后计算
	uint16_t data_length_pos = idx;
	idx += 1; // 为数据长度预留位置

    frame[idx++] = MOTOR_NUM;

    // 电机数据
    for (int i = 0; i < MOTOR_NUM; i++) {
        int16_t pos = (int16_t)(motor[i].servo.filt_angle * 100);
        int16_t vel = (int16_t)(motor[i].servo.current_speed * 100);
    	int8_t motor_state = (int8_t)(motor[i].status);
    	// 填入数据包
        frame[idx++] = (pos >> 8) & 0xFF; frame[idx++] = pos & 0xFF;
        frame[idx++] = (vel >> 8) & 0xFF; frame[idx++] = vel & 0xFF;
    	frame[idx++] = motor_state & 0xFF;
    }

    // 向下偏转
    int16_t s_val = (int16_t)(SW.joint_space.current_theta[0] * 100);
    frame[idx++] = (s_val >> 8) & 0xFF; frame[idx++] = s_val & 0xFF;
    
	// 向上偏转
	int16_t s_val2 = (int16_t)(SW.joint_space.current_theta[1] * 100);
	frame[idx++] = s_val2 >> 8; frame[idx++] = s_val2 & 0xFF;
    frame[idx++] = state; // 使用传入的 state

	// 计算数据长度（从电机数量字节开始到state字节的所有字节）
	// 数据长度 = 当前idx - 3 （因为0:帧头,1:功能码,2:数据长度）
	frame[data_length_pos] = (uint8_t)(idx-3);

    // 校验和
    uint16_t checksum = 0;
    for (int i = 0; i < idx; i++) {
        checksum += frame[i];
    }
    frame[idx++] = checksum & 0xFF;

    return idx;
}
