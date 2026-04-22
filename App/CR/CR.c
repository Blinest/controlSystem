/**
 *上层控制实现，用于完成顶层喷管的弯曲与截面变化
 *功能包括：
 *1. 喷管弯曲控制与前馈补偿
 *2. 喷管截面变化控制
 *3. 喷管初始参数设定
 */

#include "CR.h"
#include "usart.h"
#include "kinematic.h"

#include <stdio.h>
#include "math.h"
#include "Sensor/Sensor.h"


#define LQTS_THETA1_MAX 60

#define LQTS_THETA2_MAX 60

#define LQTS_ANGLE_RANGE 30
#define pi 3.1415926535


/*
 臂体补偿器
*/
bool tendon_comp = true;

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

// 创建 lqts 结构体
LQTS lqts;

void LQTS_init(void)
{
    lqts.arm_params = (ArmParams){
        .L = 400, // 喷管长度
        // .tendon_preload = 16.0, // 预紧力，不考虑
        // .friction_coeff = 0.1, // 摩擦系数，无张力反馈，不考虑
        // .backbone_stiffness = 1000.0, // 弯曲刚度，不考虑
        // .material_damping = 0.05, // 材料阻尼，不考虑
        // .calibrate_offset = {0.01, 0, 0}, // 零点偏移，不考虑
        .direction_gain = {0.95, 1.75, 0.8, 1.75} // 方向补偿(urdl)
    };

	lqts.operation_space.scale = 100;
	lqts.joint_space.theta = 0;
    lqts.parameter.r = 80; // 肌腱与中心孔距离 80mm
    motor_init();
	sensor_init();
}

/**
 * @brief 用于控制喷管弯曲
 * @param seg :段
 * @param direction：方向 0,1
 * @param val
 * @return
 */
uint8_t armBend(int seg, char direction, double val)
{

	lqts.joint_space.theta = direction == 1 ? val : -val;
    return armBend_edit(seg, direction, val, 0, 0.15, 0, 0.18, 90.0, 60.0);
}

void deltaL_update(void)
{
    // 存储当前位置
    float cur_pos[3];
	for(int i = 0; i < 3; i++)
	{
		cur_pos[i] = lqts.joint_space.deltaL[i];
	}
	// 调用运动学模型，更新位置
	calculate_L(lqts.parameter.r, lqts.joint_space.theta,lqts.joint_space.phi,lqts.joint_space.deltaL);
	// char response[128];
	// float v0 = lqts.joint_space.deltaL[0];
	// float v1 = lqts.joint_space.deltaL[1];
	// float v2 = lqts.joint_space.deltaL[2];
	//
	// int i0 = (int)v0, f0 = (int)((v0 - i0) * 100 + 0.5f); if (f0 >= 100) { f0 -= 100; i0++; }
	// int i1 = (int)v1, f1 = (int)((v1 - i1) * 100 + 0.5f); if (f1 >= 100) { f1 -= 100; i1++; }
	// int i2 = (int)v2, f2 = (int)((v2 - i2) * 100 + 0.5f); if (f2 >= 100) { f2 -= 100; i2++; }
	//
	// int size = snprintf(response, sizeof(response), "Parameters: %d.%02d, %d.%02d, %d.%02d\r\n", i0, f0, i1, f1, i2, f2);
	// Usart_SendString(&huart1, (uint8_t*)response, size);

	HAL_Delay(20);
	// 调用底层多电机控制库，目前使用的是1-3号电机
	motor_sync_control(3, 1, lqts.joint_space.deltaL);

}

void auto_straight(void)
{
    lqts.joint_space.theta = 0;
    lqts.joint_space.phi = 0;
	lqts.operation_space.scale = 0;

    deltaL_update();
}

int direction_to_index(char direction) {
    switch(direction) {
        case 'u': return 0;
        case 'r': return 1;
        case 'd': return 2;
        case 'l': return 3;
        default: return 0; // 默认返回'u'的索引
    }
}

// 补偿模型：基于力矩平衡实现
double tendonCompensation(int seg, char direction, double angle_deg) {
	const double ANGLE_THRESH = 0.3;      // 几何补偿起始阈值 (rad)
	const double GEO_SCALE = 0.15 / 1.2;  // 0.125
	const double MAX_RATIO = 1.3;
	const double MIN_RATIO = 0.7;

	int dir_idx = direction_to_index(direction);
	double angle_rad = angle_deg * M_PI / 180.0;
	double dir_gain = lqts.arm_params.direction_gain[dir_idx];

	// 基础补偿角度
	double theta_comp = angle_rad * dir_gain;

	// 几何非线性补偿（大角度）
	if (angle_rad > ANGLE_THRESH) {
		double cos_theta = cos(angle_rad);
		theta_comp *= (1.0 + GEO_SCALE * (angle_rad - ANGLE_THRESH) * (1.0 - cos_theta));
	}

	// 重力补偿
	double delta_cos = 1.0 - cos(angle_rad);
	if (direction == 'u') {
		theta_comp *= (1.0 + 0.08 * delta_cos);
	} else if (direction == 'd') {
		theta_comp *= (1.0 - 0.03 * delta_cos);
	}

	// 限幅
	double min_allowed = MIN_RATIO * angle_rad;
	double max_allowed = MAX_RATIO * angle_rad;
	theta_comp = fmin(fmax(theta_comp, min_allowed), max_allowed);

	return theta_comp;
}

/**
 * @brief 用于控制截面面积收缩
 * @param direction 1正
 * @param val 目前的取值范围为 50.0f-100.0f，百分数
 */
void scale_squared(uint8_t direction, float val)
{
	// 限制条件
	if (val < 75) return;

	// 运动学推导
	lqts.operation_space.scale = direction == 1? val : -val;
	float R = 50;
	float target;
	float val_sqrt = sqrtf(val) / 10.0f;
	target = 2.0f * pi * (R -  val_sqrt * R);


	// float testVal = target;
	// int int_part = (int)testVal;
	// int frac_part = (int)((testVal - int_part) * 100 + 0.5);  // 保留两位小数，四舍五入
	// if (frac_part < 0) frac_part = -frac_part;  // 小数部分取绝对值
	// if (frac_part >= 100) {  // 处理进位，如 1.999 -> 2.00
	// 	int_part += 1;
	// 	frac_part -= 100;
	// }
	//
	// char test[32];
	// int len = snprintf(test, sizeof(test), "%d.%02d", int_part, frac_part);
	// if (len > 0 && len < sizeof(test)) {
	// 	Usart_SendString(&huart1, (uint8_t*)test, len);
	// } else {
	// 	Usart_SendString(&huart1, (uint8_t*)"ERR_FMT\r\n", 9);
	// }
	motor_run(0, 10, target ,false);
}

/**
 * @brief: 带补偿的臂体弯曲函数
 * @param seg
 * @param direction ：弯曲方向：urdl
 * @param val ：弯曲角度: deg
 * @param g_u ：向上补偿
 * @param g_r ：向右补偿
 * @param g_d ：向下补偿
 * @param g_l ：向左补偿
 * @param seg1_limit
 * @param seg2_limit
 * @return
	 */
uint8_t armBend_edit(int seg, char direction, double val, double g_u, double g_r, double g_d, double g_l, double seg1_limit, double seg2_limit)
{
    // 节段、角度限制检查
    if(seg != 1 && seg != 2) return 1;
    if (seg == 1 && (val > seg1_limit || val < 0)) return 1;
    if (seg == 2 && (val > seg2_limit || val < 0)) return 1;
    double val_rad = val * pi / 180.0;

    // 使用肌腱补偿器
    double compensated_angle_rad = 0;
    if(tendon_comp) {
       compensated_angle_rad = tendonCompensation(seg, direction, val);
    } else {
        compensated_angle_rad = val * pi / 180.0;
    }

    // 检查补偿后的角度是否超出安全范围
    double compensated_deg = compensated_angle_rad * 180.0 / pi;
    double max_angle = (seg == 1) ? 120.0 : 60.0;  // 允许一定的超调，目前第一段臂体可以超调到120°左右，第二段臂体为50°左右，由于第二段的力臂较大，会出现拉脱情况。
    if (compensated_deg > max_angle) {
        compensated_angle_rad = max_angle * pi / 180.0;
    }

    // 设置 phi 角度，并进行简单的扭转补偿
    double phi = 0;
    switch (direction)
    {
        case 'u': phi = 0; break;
        case 'r': phi = pi / 2 - val_rad * g_r; break;
        case 'd': phi = pi;; break;
        case 'l': phi = 3 * pi / 2 + val_rad * g_l; break;
        default: return 1;
    }
    // 更新补偿后的关节角度
    lqts.joint_space.theta = compensated_angle_rad;
    lqts.joint_space.phi = phi;
    deltaL_update();
    return 0;
}
