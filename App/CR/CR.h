#ifndef __CR_H
#define __CR_H

#include <stdint.h>
#include <stdbool.h>
#include "Motor/Motor.h"

/**********************************************************
***	编写作者：blinest
***	qq：1071378062
**********************************************************/

// 运动学回调函数类型
// 参数: R - 半径(mm), theta[] - 弯曲角(rad), phi[] - 弯曲平面角(rad), deltaL[] - 输出肌腱长度变化(mm)
typedef void (*KinematicFunc)(float R, float theta[], float phi[], float deltaL[]);

// 补偿回调函数类型（后处理）
// 参数: deltaL[] - 肌腱长度变化数组（6个电机），可直接修改
typedef void (*CompensationFunc)(float deltaL[]);

typedef struct CR_Parameter
{
    float r;                       // 肌腱与中心孔距离 (mm)
} CR_Parameter;

typedef struct Joint_Space
{
    float current_phi[2];           // 当前弯曲平面角 (rad)
    float current_theta[2];         // 当前弯曲角 (rad)
    float target_phi[2];            // 目标弯曲平面角 (rad)
    float target_theta[2];          // 目标弯曲角 (rad)
    float current_deltaL[6];        // 当前肌腱长度变化 (mm)
    float target_deltaL[6];         // 目标肌腱长度变化 (mm)
} Joint_Space;

typedef struct ArmParams
{
    double L;                       // 臂体长度 (mm)
    double tendon_preload;          // 预紧力 (N) – 预留
    double friction_coeff;          // 摩擦系数 – 预留
    double backbone_stiffness;      // 弯曲刚度 (N·mm/rad)
    double material_damping;        // 材料阻尼系数 – 预留
    double calibrate_offset[3];     // 肌腱零点偏移 (mm)
    double direction_gain[4];       // 方向增益 (u, r, d, l) – 可用于补偿回调
} ArmParams;

typedef struct ContinuumRobot
{
    Joint_Space joint_space;
    CR_Parameter parameter;
    ArmParams arm_params[2];
    bool state;
} ContinuumRobot;

// 全局实例
extern ContinuumRobot CR;

// 初始化机器人
void CR_init(void);

// 根据当前目标 theta/phi 重新计算 deltaL 并发送给电机（不经过补偿回调，仅用于内部状态同步）
void deltaL_update(void);

// 方向字符转索引 (u->0, r->1, d->2, l->3)
static float normalize_phi(float phi);
int direction_to_index(char direction);
static int phi_to_direction_index(float phi);

// 肌腱补偿实现
void default_tendon_compensation(float deltaL[]);
void bending_gain_compensation(float deltaL[]);

// ========== 核心统一运动学控制接口 ==========
/**
 * @brief 两段连续体机器人运动学控制（整合方向选择 + 补偿回调）
 * @param kinematic   运动学计算回调函数（不能为 NULL）
 * @param compensate  补偿回调函数（可以为 NULL，表示不补偿）
 * @param R           半径 (mm)
 * @param theta       两段弯曲角度数组 (rad)，长度为2
 * @param dir         两段弯曲方向数组，取值范围 0~3 (0:上,1:右,2:下,3:左)
 * @param deltaL      输出 tendons 长度变化数组，长度为6（调用者需保证空间）
 */
void CR_kinematic_control(KinematicFunc kinematic, CompensationFunc compensate,
                             float R, float theta[], int dir[], float deltaL[]);
void autoStraight(void);
#endif /* __CR_H */