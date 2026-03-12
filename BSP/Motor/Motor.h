//
// Created by blin on 2026/3/7.
//

#ifndef CONTROLSYSTEM_MOTOR_H
#define CONTROLSYSTEM_MOTOR_H
#include "stdint.h"

#include <stdbool.h>
#include <stdint.h>
/**********************************************************
***	编写作者：Lin

***	qq：1071378062
**********************************************************/

#define MOTOR_NUM 3 // 定义电机数量
#define MOTOR_ID 4 // 定义电机起始 ID

// ==================== 电机外设反馈数据结构体 ====================
typedef struct MotorFeedback {
    uint8_t addr;          // 电机地址
    uint8_t func;          // 功能码
	uint8_t totol_byte;
	uint8_t configure;
	uint8_t vol; //电压
	uint16_t current; //电流
	uint16_t encoding; //磁编码器值
	uint8_t motor_data_target[3];
    int16_t motor_data_cur[3];   // pos, vel, acc 数据 (单位: 0.01mm, 0.01mm/s, 0.01mm/s^2),目前电机反馈只有位置和速度，无法实现加速度
    uint8_t state;         // 电机状态 (02: 运行, E2:无响应 ,EE: 异常)，判断时出问题直接判断EE即可
} MotorFeedback;

// ==================== 步进电机参数 ====================

typedef struct StepperMotor
{
	uint8_t daocheng;
	uint8_t xifen;
	double step_angle;
	uint8_t current_pos; // mm
	uint8_t target_pos; // mm
	uint8_t current_vel; // mm/s
	uint8_t target_vel; // mm/s
} StepperMotor;

typedef struct ServoMotor
{
	uint8_t daocheng;
	uint8_t xifen;
	double step_angle;
	uint8_t current_pos; // mm
	uint8_t target_pos; // mm
	uint8_t current_vel; // mm/s
	uint8_t target_vel; // mm/s
} ServoMotor;

// ==================== 电机参数 ====================
typedef struct Motor
{
	int id;
	bool state;
	StepperMotor stepper_motor;
	ServoMotor servo_motor;
	uint8_t last_response_time;
	uint8_t timeout_threshold;
	uint16_t current_pos; // rad
	uint16_t current_vel; // rpm -> rad/s
	uint16_t current_acc; // rad/s^2
	uint16_t target_pos;
	uint16_t target_vel;
	uint16_t target_acc;
	float vel_max;
	uint8_t size;
	uint8_t cmd[32];
} Motor;

void motor_init();
void motor_run(int addr, uint16_t speed, float target, bool snf);
void motor_enable(uint8_t addr);
void motor_stop();
void motor_single_control(uint8_t addr, uint8_t direction, uint16_t distance);
void motor_sync_control(uint8_t count, uint8_t start_addr, uint16_t distance[]);

typedef void (*Kinematic)(float CR[], float deltaL[]);
void motor_kinematic_control(Kinematic kinematic, uint8_t* R, float* theta, float* phi, float deltaL[]);
void motor_custom_control(uint8_t count, uint8_t *params);


float motor_angle_to_displacement(uint8_t motor_index, float angle);
float motor_displacement_to_angle(uint8_t motor_index, float displacement);
extern Motor motor[MOTOR_NUM];
#endif //CONTROLSYSTEM_MOTOR_H