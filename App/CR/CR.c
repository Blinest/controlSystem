/**
 *上层控制实现，用于处理上层指令解析和执行，提供电机控制和传感器数据读取等功能
 *功能包括：
 *1. 电机控制：基于运动学的电机控制
 *2. 传感器数据读取
 *3. 指令解析：解析上层指令，执行相应的操作，如控制电机、读取传感器数据等
 *4. 样机控制：根据指令控制样机的运动，如弯曲等
 *5. 错误处理：处理指令解析错误、通信错误等情况，确保系统稳定运行
 */

#include "CR.h"
#include "usart.h"
#include "kinematic.h"
#include "Motor/Motor.h"
#include <stdio.h>
#include "math.h"
#include "Sensor/Sensor.h"


#define LQTS_THETA1_MAX 60
#define LQTS_THETA1_MIN -40
#define LQTS_THETA2_MAX 60
#define LQTS_THETA2_MIN -40
#define LQTS_ANGLE_RANGE 30
#define pi 3.1415926535
#define MOTOR_NUM 3

/*
 臂体补偿器
*/
bool tendon_comp = true;

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

LQTS lqts;

void LQTS_init(void)
{
    lqts.arm_params[0] = (ArmParams){
        .L = 200, // 臂体长度
        .tendon_preload = 16.0, // 预紧力，不考虑
        .friction_coeff = 0.1, // 摩擦系数，无张力反馈，不考虑
        .backbone_stiffness = 1000.0, // 弯曲刚度
        .material_damping = 0.05, // 材料阻尼
        .calibrate_offset = {0.01, 0, 0}, // 零点偏移
        .direction_gain = {0.95, 1.75, 0.8, 1.75} // 方向补偿(urdl)
    };

    lqts.arm_params[1] = (ArmParams){
        .L = 200,
        .tendon_preload = 8.0,// 预紧力，不考虑
        .friction_coeff = 0.1,// 摩擦系数，无张力反馈，不考虑
        .backbone_stiffness = 400.0,// 弯曲刚度
        .material_damping = 0.05,// 材料阻尼
        .calibrate_offset = {0, 0, 0}, // 零点偏移
        .direction_gain = {1.25, 0.95, 1.35, 1.15} // 方向补偿
    };
	lqts.operation_space.scale = 20;
	lqts.joint_space.theta = 30;
    lqts.parameter.r = 5; // 肌腱与中心孔距离 5mm
    motor_init();
	sensor_init();
}

// 用于控制喷管弯曲
uint8_t armBend(int seg, char direction, double val)
{
    return armBend_edit(seg, direction, val, 0, 0.15, 0, 0.18, 90.0, 60.0);
}

void deltaL_update(void)
{
    // 存储当前位置
    // float cur_pos[MOTOR_NUM + 1];
    for(int i = 1; i <= 6; i++)
    {
        // cur_pos[i] = LQTS.joint_space.deltaL[i];
    }

    char response[128];
    sprintf(response, "Parameters: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",lqts.joint_space.deltaL[1], lqts.joint_space.deltaL[2],lqts.joint_space.deltaL[3],lqts.joint_space.deltaL[4],lqts.joint_space.deltaL[5],lqts.joint_space.deltaL[6]);

}

void autostraight(void)
{
    lqts.joint_space.theta = 0;
    lqts.joint_space.phi = 0;

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
double tendonCompensation(int seg, char direction, double angle_deg)
{
    int dir_idx = direction_to_index(direction);
    double angle_rad = angle_deg * pi / 180.0;
    double dir_gain =  lqts.arm_params[seg-1].direction_gain[dir_idx];
    double theta_ideal = angle_rad;
    // 摩擦引起的角度损失，理论上与肌腱张力成正比，由于目前没有张力反馈，不进行张力补偿，
    // double r = lqts.parameter.r / 1000;
    // double friction_loss_rad = friction_torque / (bending_stiffness_Nm2 + 0.001)  * (1.0 - exp(-angle_rad)); // 使用指数函数平滑

    //材料弹性引起的角度损失，考虑弹性恢复力矩，目前不需要考虑
    //double elastic_coeff = (lqts.arm_params[seg-1].backbone_stiffness / (lqts.arm_params[seg-1].L* lqts.arm_params[seg-1].L));
    //double elastic_loss = elastic_coeff * angle_rad * lqts.arm_params[seg-1].material_damping;

    // 大角度时的几何非线性补偿
    // 当弯曲时，肌腱的有效力臂会减小：R_eff = R * cos(theta/2)
    double geometric_factor = 1.0;
    if (angle_rad > 0.3) { // 大约17度以上开始考虑
    // 使用平滑过渡，避免突变
        double t = (angle_rad - 0.3) / 1.2; // 归一化到[0,1]，假设最大90度=1.57弧度

        geometric_factor = 1.0 + 0.15 * t * (1.0 - cos(angle_rad));
    }
    // 重力补偿
    double gravity_factor = 1.0;
    if (direction == 'u') {
        // 向上弯曲，对抗重力，需要额外补偿
        gravity_factor = 1.0 + 0.08 * (1.0 - cos(angle_rad));
    } else if (direction == 'd') {
        // 向下弯曲，重力辅助，可以减少补偿
        gravity_factor = 1.0 - 0.03 * (1.0 - cos(angle_rad));
    }


    double theta_compensated = theta_ideal * dir_gain * geometric_factor * gravity_factor;

    double max_ratio = 1.3;
    double min_ratio = 0.7;

    double min_allowed = min_ratio * angle_rad;
    double max_allowed = max_ratio * angle_rad;

    if (theta_compensated < min_allowed) {
        theta_compensated = min_allowed;
    }
    else if (theta_compensated > max_allowed) {
        theta_compensated = max_allowed;

    }
    return  theta_compensated;
}

// 用于控制截面面积收缩
void scale_squared(double val)
{

}

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
