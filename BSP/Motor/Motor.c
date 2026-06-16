/**
* @file motor.c
 * @brief 电机指令处理模块（基于 MotorContext 架构）
 *
 * 本模块提供电机指令处理功能：
 * - motor_init()：初始化所有电机上下文（参数 + 解析器）
 * - motor_run()：启动步进电机绝对位置控制
 * - motor_enable()：使能/失能电机
 * - motor_stop_all()：紧急停止所有电机
 * - motor_sync_control()：多电机同步位置控制
 * - 运动学接口：motor_kinematic_control()
 * - 辅助函数：角度/位移换算、状态检查等
 */

#include "Motor.h"
#include "math.h"

#include <stdio.h>
#include "usart.h"
#include "Emm_V5.h"
#include "Common/XV2_cmd_parser.h"
#include "CR/CR.h"
#include "CR/kinematic.h"
#include "string.h"

// 创建电机与电机反馈数据结构体
MotorContext motor_ctx[MOTOR_NUM];

//电机初始化函数
void motor_init()
{

	for (int i = 0; i < MOTOR_NUM; i++) {
		GlobalMotor *gm = &motor_ctx[i].global_motor;
		gm->id = MOTOR_ID + i;
		gm->stepper_motor.daocheng = 2;
		gm->stepper_motor.xifen = 256;
		gm->stepper_motor.step_angle = 1.8f;
		gm->stepper_motor.target_vel = 10.0f;
		gm->stepper_motor.current_vel = 10.0f;
		gm->vel_max = 60.0f;
		gm->current_acc = 0.0f;

		// 初始化每个电机的串口解析器
		X_V2_SerialParser_Init(&motor_ctx[i].parser);
		// motor_auto_calibrate(i);
	}

}


/* ==================== 使能控制 ==================== */
/**
 * @param addr 电机地址（ID）
 * @param enable true：使能，false：失能
 */
void motor_enable(uint8_t addr, bool enable)
{
	// 直接调用底层驱动，地址与 ID 一致
	Emm_V5_En_Control(addr, enable, 0);

}

/* ==================== 自动校准（占位） ==================== */
/**
 * 自动校准指令，注意需要进行至少40s的延迟操作
 * @param idx
 */
void motor_auto_calibrate(uint8_t idx)
{
	Emm_V5_Calibrate(motor_ctx[idx].global_motor.id);
	HAL_Delay(40000);
}

/* ==================== 单电机绝对位置控制 ==================== */
/**
 * @param idx     电机在 motor_ctx 中的索引（0~MOTOR_NUM-1）
 * @param vel_rpm 速度（RPM）
 * @param target  目标位置（角度，度）
 * @param snf     同步标志（1=同步触发）
 */
void motor_run(int idx, float vel_rpm, float target, uint8_t snf)
{
	if (idx < 0 || idx >= MOTOR_NUM) return;

	GlobalMotor *gm =  &motor_ctx[idx].global_motor;
	StepperMotor *sm = &gm->stepper_motor;
	float step_angle = gm->stepper_motor.step_angle;
	float xifen = gm->stepper_motor.xifen;

	// 方向：目标角度正负决定方向（0正1负）
	uint8_t dir = (target >= 0) ? 0 : 1;
	float angle_abs = fabsf(target);

	// 计算脉冲数
	uint32_t clk = (uint32_t)(angle_abs / step_angle * xifen);

	// 加速度 (RPM/s^2)，可根据需要扩展
	uint16_t acc = 10;

	// 更新期望值
	gm->target_vel = vel_rpm;
	sm->target_vel = vel_rpm;
	gm->target_pos = target;
	sm->target_pos = target;

	// 调用底层驱动（位置模式）
	Emm_V5_Pos_Control(gm->id, dir, (uint16_t)vel_rpm, acc, clk, 1, snf);

}

/**
 * @brief 速度模式驱动电机（用于外部位置环）
 * @param idx       电机索引
 * @param vel_rpm   目标速度 (RPM)，可为正或负，内部自动处理方向
 * @param acc_rpm_s 加速度 (RPM/s)
 */
void motor_run_velocity_mode(uint8_t idx, float vel_rpm, uint16_t acc_rpm_s) {
	if (idx < 0 || idx >= MOTOR_NUM) return;

	GlobalMotor *gm = &motor_ctx[idx].global_motor;
	uint8_t dir = (vel_rpm >= 0) ? 0 : 1;
	float abs_vel = fabsf(vel_rpm);

	Emm_V5_Vel_Control(gm->id, dir, abs_vel, 10, false);
	// 记录当前目标速度
	gm->target_vel = vel_rpm;
}

/**
 * 多电机停止函数f
 *
 */
void motor_stop_all()
{
	for(int i = 0; i < MOTOR_NUM; i++) {
		GlobalMotor *gm = &motor_ctx[i].global_motor;
		Emm_V5_Stop_Now(gm->id, false);
		gm->state = 0;
	}
}

/* ==================== 单电机相对位移控制 ==================== */
/**
 * @param idx       电机索引
 * @param direction 方向（0=正，1=负）
 * @param distance  位移（mm）
 * @param vel       速度（mm/s）
 */
void motor_single_control(uint8_t idx, uint8_t direction, float distance, float vel)
{
	if (idx >= MOTOR_NUM) return;

	GlobalMotor *gm = &motor_ctx[idx].global_motor;
	float displacement = (direction == 0) ? distance : -distance;

	// 速度转换为 RPM
	float vel_rpm = vel * 60.0f / gm->stepper_motor.daocheng;
	// 位移转换为角度
	float angle = displacement * 360.0f / gm->stepper_motor.daocheng;

	// char str[30];
	// sprintf(str, "target: %d", (int)(gm->stepper_motor.daocheng * 100));
	// Usart_SendString(&huart1, str,strlen(str));
	motor_run(idx, vel_rpm, angle, 0);
}

/* ==================== 多电机同步控制 ==================== */
/**
 * @param count     参与同步的电机数量
 * @param start_idx 起始索引（在 motor_ctx 中）
 * @param target_distance  各电机目标位移数组（mm），长度 count，起始从0开始
 * @param current_distance 各电机当前位移数组（mm），长度 count，起始从0开始
 */
void motor_sync_control(uint8_t count, uint8_t start_idx, float target_distance[], float current_distance[])
{
	if (start_idx + count > MOTOR_NUM) return;

	// 1. 计算最大位移，用于速度同步比例分配
	float max_abs_dis = 0.0f;
	float vel_rpm,abs_dis[count];
	for (uint8_t i = 0; i < count; i++) {
		abs_dis[i] = fabsf(target_distance[i] - current_distance[i]);
		if (abs_dis[i] > max_abs_dis) max_abs_dis = abs_dis[i];
	}

	// 2. 为每个电机计算速度并发送命令（snf = true）
	for (uint8_t i = 0; i < count; i++) {
		uint8_t idx = start_idx + i;
		GlobalMotor *gm = &motor_ctx[idx].global_motor;

		// 速度按比例分配（最大速度 * 位移占比）
		float ratio = (max_abs_dis > 0) ? abs_dis[i] / max_abs_dis : 0.0f;
		vel_rpm = ratio * gm->vel_max;

		// 转为角度并调用 motor_run，snf = 1 表示暂存，最后统一触发
		float angle = target_distance[i] * 360.0f / gm->stepper_motor.daocheng;
		motor_run(idx, vel_rpm, angle, 1);
		HAL_Delay(8);  // 避免总线拥塞
	}

	// 3. 发送同步触发命令（地址 0 为广播触发，取决于协议）
	Emm_V5_Synchronous_motion(0);
	HAL_Delay(10);
}



/* ==================== 角度/位移换算 ==================== */
float motor_angle_to_displacement(uint8_t motor_index, float angle)
{
    if (motor_index >= MOTOR_NUM) return 0.0f;
    GlobalMotor *gm = &motor_ctx[motor_index].global_motor;
    StepperMotor *sm = &gm->stepper_motor;

    float steps_per_rev = 360.0f / sm->step_angle * sm->xifen;
    float steps = angle / 360.0f * steps_per_rev;
    float displacement = steps * sm->daocheng / steps_per_rev;

    sm->current_pos = displacement;
    gm->current_pos = angle * (float)(3.14159265f / 180.0f); // 度转弧度
    return displacement;
}

float motor_displacement_to_angle(uint8_t motor_index, float displacement)
{
    if (motor_index >= MOTOR_NUM) return 0.0f;
    GlobalMotor *gm = &motor_ctx[motor_index].global_motor;
    StepperMotor *sm = &gm->stepper_motor;

    float steps_per_rev = 360.0f / sm->step_angle * sm->xifen;
    float steps = displacement * steps_per_rev / sm->daocheng;
    float angle = steps / steps_per_rev * 360.0f;

    sm->current_pos = displacement;
    gm->current_pos = angle * (float)(3.14159265f / 180.0f);
    return angle;
}

/* ==================== 电机状态检查 ==================== */
void motor_status_check(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        GlobalMotor *gm = &motor_ctx[i].global_motor;
        // 假设读取当前位置和速度的命令
        Emm_V5_Read_Sys_Params(gm->id, S_CPOS);
        HAL_Delay(10);
        Emm_V5_Read_Sys_Params(gm->id, S_VEL);
        HAL_Delay(10);
    }
}

/* ==================== 完整控制接口 ==================== */
void motor_full_control(uint8_t idx, uint8_t dir, float dist, float velocity, float acceleration)
{
    if (idx >= MOTOR_NUM) return;
    GlobalMotor *gm = &motor_ctx[idx].global_motor;
    float displacement = (dir == 0) ? -dist : dist;

    gm->stepper_motor.target_pos = gm->stepper_motor.current_pos + displacement;
    gm->stepper_motor.target_vel = velocity;
    gm->stepper_motor.current_acc = acceleration;

    motor_run(idx, velocity / 100.0f, gm->stepper_motor.current_pos + displacement, 0);
}