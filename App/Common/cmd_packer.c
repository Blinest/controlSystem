/**
 * @file cmd_packer.c
 * @brief 指令打包库
 * 
 * 负责将全局结构体中的数据打包成数据包，发送给上位机
 * 
 * @date 2026-03-30
 * @author Psyduck
 */

#include "cmd_packer.h"
#include "CR/CR.h"
#include "cmsis_os2.h"


// 引用外部队列
extern osMessageQueueId_t SensorMessageQueueHandle;

/**
 * @brief 打包系统状态帧 (test_frame 格式)
 * @param frame 存储打包后的数据缓冲区
 * @param motor 电机全局结构体
 * @param sensor 传感器全局结构体
 * @param scale 缩放比例
 * @param state 系统状态
 * @return 打包后的总长度
 */
uint16_t cmd_packer_pack_status_frame(uint8_t* frame, GlobalMotor motor[MOTOR_NUM], GlobalSensor sensor[SENSOR_NUM], float scale, uint8_t state) {
    // 打包帧格式：帧头 + 功能码 + 数据长度 + 电机数量 + 传感器数量
	uint16_t idx = 0;
    frame[idx++] = 0xBB; // 帧头
    frame[idx++] = 0x02; // 功能码 (多传感器批量读取)
    frame[idx++] = 47;   // 数据长度 (MOTOR_NUM(1) + SENSOR_NUM(1) + 18 + 24 + 2 + 1)
    frame[idx++] = MOTOR_NUM;
    frame[idx++] = SENSOR_NUM;

    // 电机数据
    for (int i = 0; i < MOTOR_NUM; i++) {
        int16_t pos = (int16_t)(motor[i].stepper_motor.current_pos * 100); // mm
        int16_t vel = (int16_t)(motor[i].stepper_motor.current_vel * 100); // mm/s
        int16_t acc = (int16_t)(motor[i].stepper_motor.current_acc * 100); // mm/s^2
    	int8_t state = (int8_t)(motor[i].state);
    	// 填入数据包
        frame[idx++] = (pos >> 8) & 0xFF; frame[idx++] = pos & 0xFF;
        frame[idx++] = (vel >> 8) & 0xFF; frame[idx++] = vel & 0xFF;
        frame[idx++] = (acc >> 8) & 0xFF; frame[idx++] = acc & 0xFF;
    	frame[idx++] = state & 0xFF;
    }

    // 传感器数据
    for (int i = 0; i < SENSOR_NUM; i++) {
        int16_t x = (int16_t)(sensor[i].x * 100);
        int16_t y = (int16_t)(sensor[i].y * 100);
        int16_t z = (int16_t)(sensor[i].z * 100);

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


/**
 * @brief 发送打包后的帧到队列
 * @param frame 打包后的数据帧
 * @param frame_len 帧长度
 */
void cmd_packer_send_frame_to_queue(uint8_t* frame, uint16_t frame_len) {
    for (int i = 0; i < frame_len; i++) {
        uint8_t msg = frame[i];
        osMessageQueuePut(SensorMessageQueueHandle, &msg, 0, 0);
    }
}

/**
 * @brief 发送系统状态帧到上位机
 * 这是供其他模块调用的高级接口
 */
void cmd_packer_send_status_frame(void) {
    uint8_t packed_frame[64];
	float scale = lqts.operation_space.scale;
	uint8_t state = lqts.state;
    uint16_t frame_len = cmd_packer_pack_status_frame(packed_frame, global_motor, global_sensor, scale, state);
    cmd_packer_send_frame_to_queue(packed_frame, frame_len);
}
