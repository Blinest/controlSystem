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
	uint8_t current_pos; // rad
	uint8_t current_vel; // rpm -> rad/s
	uint8_t current_acc; // rad/s^2
	uint8_t target_pos;
	uint8_t target_vel;
	uint8_t target_acc;
	float vel_max;
	uint8_t size;
	uint8_t cmd[32];
} Motor;

void motor_init();
void motor_enable(uint8_t addr);
void motor_stop();
void motor_single_control(uint8_t addr, uint8_t direction, uint16_t distance);
void motor_sync_control(uint8_t count, uint8_t start_addr, uint16_t *distances);
void motor_kinematic_control();
void motor_custom_control(uint8_t count, uint8_t *params);
extern Motor motor[MOTOR_NUM];
#endif //CONTROLSYSTEM_MOTOR_H