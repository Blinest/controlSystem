//
// Created by blin on 2026/3/7.
//
/**
* @file motor.c
 * @brief 电机指令处理模块
 *
 * 本模块提供电机指令处理功能：
 * - motor_init()：电机初始化，初始化流程包括电机参数设置、
 * - motor_run()：启动电机，并设置绝对目标位置
 * - motor_position_control_snf()：
 * - motor_emergency_stop_all()：紧急停止所有电机
 *
 */
#include "Motor.h"
#include "Emm_V5.h"
#include "math.h"
Motor motor[MOTOR_NUM];
//电机初始化函数
void motor_init()
{
	//TODO: 初始化电机相关的GPIO、定时器、PWM等硬件资源
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		motor[i].id = MOTOR_ID + i;
		motor[i].stepper_motor.daocheng = 2; // 根据丝杠导程设置，2mm
		motor[i].stepper_motor.xifen = 128; // 平滑控制
		motor[i].stepper_motor.step_angle = 1.8; // 步距角
		motor[i].vel_max = 50; // 最大速度建议50rpm以下，速度过高，步进电机(FUYU35, 2025年)会出现抖动
		motor[i].current_acc = 0; // 由于位移量较小，为提高响应速度，直接启动，不做加减速处理 (0-255)
	}
}
// 电机使能函数
void motor_enable(uint8_t addr)
{
	//TODO: 根据电机地址发送使能指令，可能涉及串口通信或GPIO控制
	Emm_V5_En_Control(addr, 1, 0);

	for (int i = 0; i < MOTOR_NUM; i++)
	{
		if (motor[i].id == addr)
		{
			// 更新电机状态，(电机使能状态 0x02)
			motor[i].state = 0x02;
			break;
		}
	}

}
/**
  * @brief 启动步进电机，并达到指定位置
  * @param addr: 电机地址
  * @param speed: 速度值, rpm
  * @param target: 目标位置(绝对位置), mm
  * @param snf: 同步标志位，true同步
  */
void motor_run(int addr, uint16_t speed, float target, bool snf) {
	int idx = (addr >= MOTOR_ID && addr < MOTOR_ID + MOTOR_NUM) ? (addr - MOTOR_ID) : 0;
	int xifen = motor[idx].stepper_motor.xifen;
	int daocheng = motor[idx].stepper_motor.daocheng;
	double step_angle = motor[idx].stepper_motor.step_angle;
	// 位置计算
	float distance = xifen * 360.0f / step_angle * target / daocheng;

	const int dir = distance > 0 ? 1:0;
	const uint32_t clk = (uint32_t)fabs(distance);
	Emm_V5_Pos_Control(addr, dir, speed, motor[idx].current_acc, clk, true, snf);
}
// 电机停止函数
void motor_stop()
{
	char message[64];
	// 发送紧急停止消息
	// sprintf(message, "stop all motors！\r\n");
	// UART2_SendString(message);
	for(int i = 0; i < MOTOR_NUM; i++) {
		Emm_V5_Stop_Now(motor[i].id, false);
		osDelay(5);
		// 更新电机状态(电机停止状态0xEE)
		motor[i].state = 0xEE;
	}
	//TODO: 发送停止指令给所有电机，确保它们立即停止运动
}
// 单电机同步控制函数
void motor_single_control(uint8_t addr, uint8_t direction, uint16_t distance)
{
	//TODO: 根据电机地址、运动方向和距离发送控制指令，可能涉及串口通信或GPIO控制


}
// 多电机同步控制函数
void motor_sync_control(uint8_t count, uint8_t start_addr, uint16_t distance[])
{
	//TODO: 根据电机数量、起始地址和距离数组发送同步控制指令，确保所有电机同时开始运动并在指定距离停止
	int size = count;
	if (size <= 0 || size > MOTOR_NUM) return;
	float max_distance = 0;
	uint16_t speed[size];
	// 找到最长路径（cur/target 下标 1..size 对应电机 1..size）
	// 动态路径分配
	for (int i = 0; i < size; i++)
	{
		distance[i] = fabs(motor[i].stepper_motor.target_pos - motor[i].stepper_motor.target_pos);
		max_distance = fmax(max_distance, distance[i]);
	}
	if (max_distance == 0) return;
	// 动态速度调整：线性比例控制
	for (int i = 0; i < size; i++)
		speed[i] = (uint16_t)(distance[i] * motor[i].vel_max / max_distance);

	Emm_V5_Synchronous_motion(0);
	osDelay(20);
	for (int i = 0; i < size; i++)
	{
		motor_run(motor[i].id, speed[i], motor[i].stepper_motor.target_pos, true);
	}
	Emm_V5_Synchronous_motion(0);
}
// 基于运动学的多电机控制函数，通过回调函数实现
void motor_kinematic_control (Kinematic kinematic, int* R, int* theta, float* phi, float* deltaL[])
{
	//TODO: 实现基于运动学的多电机控制算法，计算每个电机的目标位置和速度，并发送相应的控制指令
	kinematic(R, theta, phi, deltaL);
	motor_sync_control(MOTOR_NUM, 0, deltaL);
	// 进行单电机控制
}
// 自定义多电机控制函数
void motor_custom_control(uint8_t count, uint8_t *params)
{
	//TODO: 根据自定义参数格式解析控制指令，并发送相应的控制命令给指定数量的电机

}

/**
 * @brief 将步进电机的角度信息转换为位移信息
 * @param motor_index: 电机索引
 * @param angle: 角度信息（单位：度）
 * @return 位移信息（单位：mm）
 */
float motor_angle_to_displacement(uint8_t motor_index, float angle)
{
    if (motor_index >= MOTOR_NUM) {
        return 0.0f;
    }
    
    StepperMotor *stepper = &motor[motor_index].stepper_motor;
    
    // 计算每转的步数
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;
    
    // 计算角度对应的步数
    float steps = angle / 360.0f * steps_per_rev;
    
    // 计算位移：步数 * 导程 / 每转步数
    float displacement = steps * stepper->daocheng / (360.0f / stepper->step_angle * stepper->xifen);
    
    // 更新电机结构体中的位置信息
    stepper->current_pos = displacement;
    motor[motor_index].current_pos = angle * 180.0f / 3.1415926f; // 转换为弧度并存储
    
    return displacement;
}

/**
 * @brief 将位移信息转换为步进电机的角度信息
 * @param motor_index: 电机索引
 * @param displacement: 位移信息（单位：mm）
 * @return 角度信息（单位：度）
 */
float motor_displacement_to_angle(uint8_t motor_index, float displacement)
{
    if (motor_index >= MOTOR_NUM) {
        return 0.0f;
    }
    
    StepperMotor *stepper = &motor[motor_index].stepper_motor;
    
    // 计算每转的步数
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;
    
    // 计算位移对应的步数
    float steps = displacement * steps_per_rev / stepper->daocheng;
    
    // 计算角度：步数 / 每转步数 * 360度
    float angle = steps / steps_per_rev * 360.0f;
    
    // 更新电机结构体中的位置信息
    stepper->current_pos = displacement;
    motor[motor_index].current_pos = angle * 180.0f / 3.1415926f; // 转换为弧度并存储
    
    return angle;
}
