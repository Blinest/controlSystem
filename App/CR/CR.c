/**
 * @file CR.c
 * @brief 连续体机器人控制模块（统一运动学控制 + 弯曲增益补偿，正确索引映射）
 */

#include "CR.h"
#include "kinematic.h"
#include <math.h>
#include <string.h>

// 全局实例定义
ContinuumRobot CR;

// 内部常量
#define PI              3.1415926535f
#define MOTOR_NUM       6
#define MAX_DELTA_L     10.0f   // 肌腱最大变化量 (mm)
#define MIN_DELTA_L    -10.0f

// 辅助函数：将弧度角度归一化到 [0, 2π)
static float normalize_phi(float phi) {
    float two_pi = 2.0f * PI;
    float res = fmodf(phi, two_pi);
    if (res < 0) res += two_pi;
    return res;
}

// 辅助函数：根据 phi 角度判断方向索引 (0:u,1:r,2:d,3:l)
static int phi_to_direction_index(float phi) {
    float eps = 0.01f;
    phi = normalize_phi(phi);
    if (phi < eps || fabs(phi - 2*PI) < eps) return 0;          // 0° -> 上
    if (fabs(phi - PI/2) < eps) return 1;                      // 90° -> 右
    if (fabs(phi - PI) < eps) return 2;                        // 180° -> 下
    if (fabs(phi - 3*PI/2) < eps) return 3;                   // 270° -> 左
    // 如果不是标准方向，找最近的方向（避免出错）
    float angles[4] = {0, PI/2, PI, 3*PI/2};
    int best = 0;
    float min_diff = fabs(phi - 0);
    for (int i = 1; i < 4; i++) {
        float diff = fabs(phi - angles[i]);
        if (diff < min_diff) {
            min_diff = diff;
            best = i;
        }
    }
    return best;
}

// ========== 补偿回调实现：弯曲增益补偿 ==========
void bending_gain_compensation(float deltaL[])
{
    float theta[2];
    float phi[2];
    for (int i = 0; i < 2; i++) {
        theta[i] = CR.joint_space.target_theta[i];
        phi[i]   = CR.joint_space.target_phi[i];
    }

    // 电机索引映射：段0 -> 0,2,4 ; 段1 -> 1,3,5
    int motor_indices[2][3] = {
        {0, 2, 4},
        {1, 3, 5}
    };

    for (int seg = 0; seg < 2; seg++) {
        int dir_idx = phi_to_direction_index(phi[seg]);
        double gain = CR.arm_params[seg].direction_gain[dir_idx];

        for (int i = 0; i < 3; i++) {
            int motor_idx = motor_indices[seg][i];
            deltaL[motor_idx] *= (float)gain;
        }
    }

    // 安全限幅
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (deltaL[i] > MAX_DELTA_L) deltaL[i] = MAX_DELTA_L;
        if (deltaL[i] < MIN_DELTA_L) deltaL[i] = MIN_DELTA_L;
    }
}

// ========== 简单限幅补偿（备选） ==========
void default_tendon_compensation(float deltaL[])
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (deltaL[i] > MAX_DELTA_L) deltaL[i] = MAX_DELTA_L;
        if (deltaL[i] < MIN_DELTA_L) deltaL[i] = MIN_DELTA_L;
    }
}

// ========== 辅助函数 ==========
int direction_to_index(char direction) {
    switch(direction) {
        case 'u': return 0;
        case 'r': return 1;
        case 'd': return 2;
        case 'l': return 3;
        default:  return 0;
    }
}

static float dir_to_phi(int dir) {
    const float phi_vals[4] = {0.0f, PI / 2.0f, PI, 3.0f * PI / 2.0f};
    return phi_vals[dir % 4];
}

// ========== 核心运动学控制接口 ==========
void CR_kinematic_control(KinematicFunc kinematic, CompensationFunc compensate,
                             float R, float theta[], int dir[], float deltaL[])
{
    if (kinematic == NULL) return;

    float phi[2];
    phi[0] = dir_to_phi(dir[0]);
    phi[1] = dir_to_phi(dir[1]);

    // 先将目标角度同步到全局结构（以便补偿回调可以读取）
    for (int i = 0; i < 2; i++) {
        CR.joint_space.target_theta[i] = theta[i];
        CR.joint_space.target_phi[i]   = phi[i];
    }

    // 1. 调用运动学计算原始 deltaL，这里为 CR 结构体里的 target_deltaL
    kinematic(R, theta, phi, deltaL);

    // 2. 如果提供了补偿回调，则进行后处理（例如应用方向增益）
    if (compensate != NULL) {
        compensate(deltaL);
    }

    // 3. 发送到电机
    motor_sync_control(MOTOR_NUM, 0, deltaL, CR.joint_space.current_deltaL);
}

// ========== 内部状态更新（不经过补偿，直接使用当前目标值） ==========
void deltaL_update(void)
{
    calculate_L(CR.parameter.r,
                CR.joint_space.target_theta,
                CR.joint_space.target_phi,
                CR.joint_space.target_deltaL);
    motor_sync_control(MOTOR_NUM, 0, CR.joint_space.target_deltaL, CR.joint_space.current_deltaL);
}

// ========== 自动回到伸直状态 ==========
void autoStraight(void) {
	CR.joint_space.target_theta[0] = 0.0f;
	CR.joint_space.target_theta[1] = 0.0f;
	CR.joint_space.target_phi[0]   = 0.0f;
	CR.joint_space.target_phi[1]   = 0.0f;
	deltaL_update();   // 直接按当前目标值更新电机
}

// ========== 初始化 ==========
void CR_init(void)
{
    // 第一段臂体参数
    CR.arm_params[0] = (ArmParams){
        .L = 200.0,
        .tendon_preload = 16.0,
        .friction_coeff = 0.1,
        .backbone_stiffness = 1000.0,
        .material_damping = 0.05,
        .calibrate_offset = {0.01, 0.0, 0.0},
        .direction_gain = {1, 1, 1, 1}
    };
    // 第二段臂体参数
    CR.arm_params[1] = (ArmParams){
        .L = 200.0,
        .tendon_preload = 8.0,
        .friction_coeff = 0.1,
        .backbone_stiffness = 400.0,
        .material_damping = 0.05,
        .calibrate_offset = {0.0, 0.0, 0.0},
        .direction_gain = {1, 1, 1, 1}
    };
    CR.parameter.r = 5.0;
    // 清空关节空间目标值
    CR.joint_space.target_theta[0] = 0.0f;
    CR.joint_space.target_theta[1] = 0.0f;
    CR.joint_space.target_phi[0]   = 0.0f;
    CR.joint_space.target_phi[1]   = 0.0f;
    for (int i = 0; i < MOTOR_NUM; i++) {
        CR.joint_space.target_deltaL[i] = 0.0f;
    }
    motor_init();
}