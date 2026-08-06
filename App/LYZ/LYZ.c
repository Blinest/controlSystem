/**********************************************************
***	编写作者：blinest
***	qq：1071378062
*
* brief 上层控制实现
* 功能包括：
* 1.基于运动学模型的电机控制，主要用于系统运动
* 2.传感器数据读取，主要用于环境感知
**********************************************************/
#include "LYZ.h"
#include "usart.h"
#include "Motor/Motor.h"
#include <stdio.h>
#include "math.h"
#include "cmsis_os2.h"

LYZNozzle LYZ;

#define SERVO_VEL 30
void LYZ_init(void)
{
    // 系统初始化
	LYZ.current_phi = 0.0f;
	LYZ.current_theta = 0.0f;
	LYZ.current_S = 0.0f;
	LYZ.target_phi = 0.0f;
	LYZ.target_theta = 0.0f;
	LYZ.target_S = 0.0f;

    motor_init();

}

/**
 * @brief 反推控制函数，用于实现LYZ反推
 * @param dir: 反推方向
 * @param theta: 开合角度
 */

void LYZ_thrust_reverser_kinematic_control(const uint8_t dir, const float theta)
{
	LYZ.current_theta = theta;
	// 调用底层步进电机运动控制函数
	switch (dir)
	{
		case 0:
			motor_run_SS_abs(0, 0, 10, theta);
			osDelay(5);
			motor_run_SS_abs(1, 0, 10, theta);
			break;
		case 1:
			motor_run_SS_abs(1, 1, 10, theta);
			osDelay(5);
			motor_run_SS_abs(0, 1, 10, theta);
			break;
		default:
			break;
	}
}

/**
 * @brief 截面面积控制函数，用于实现LYZ喷管喷嘴截面面积控制
 * @param dir: 面积增减方向，0增大，1减小
 * @param S: 开合时推杆的移动距离，这里采用绝对位置控制
 */
void LYZ_cross_section_kinematic_control(const uint8_t dir, const float S)
{
	//LYZ.current_S = S;
	// 调用底层电推杆绝对位置控制函数
	motor_run_AQ_abs(2, dir, 15000, S);
}

/**
 * @brief 偏转控制函数，用于使用LYZ喷管的左右偏转
 * @param dir: 偏转方向，0左，1右
 * @param phi: 偏转角度
 */
void LYZ_deflect_kinematic_control(const uint8_t dir, const float phi)
{
	// 调用底层舵机控制函数
	motor_run_servo(3, dir, SERVO_VEL, phi);
}

/**
 * @brief 状态归为函数
 */
void LYZ_homing()
{
	LYZ.current_phi = 0;
	LYZ.current_theta = 0;
	LYZ.current_S = 0;
	LYZ_deflect_kinematic_control(0,0);
	LYZ_cross_section_kinematic_control(0,0);
	LYZ_thrust_reverser_kinematic_control(0,0);
}