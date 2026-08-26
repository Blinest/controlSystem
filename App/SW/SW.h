/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/
#ifndef __SW_H
#define __SW_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* sw几何常量 */
#define SEGMENT_COUNT       2

typedef struct JointSpace
{
    float target_theta[SEGMENT_COUNT];
    float current_theta[SEGMENT_COUNT];
    float total_target_theta;
} JointSpace;

typedef struct ArmParams
{
    double L;                /**< 每段长度 (m) */
    double direction_gain[4]; /**< 方向增益，对应(u,r,d,l) */
} ArmParams;

typedef struct SWNozzle
{
    JointSpace joint_space;
    ArmParams arm_params[2];
    bool state;
} SWNozzle;

void SW_init(void);

/** SW运动学模型*/
void SW_kinematic_control(const uint8_t idx, const uint8_t dir, const float theta[]);

/* ==================== 循环动作组 ==================== */

typedef struct
{
    float    bend_theta[SEGMENT_COUNT]; /**< 偏转目标角度 (每个关节, °) */
    uint8_t  bend_dir;                  /**< 偏转方向: 0=正向, 1=反向 */
    float    hold_bend_s;               /**< 偏转 (DOWN/UP) 到位后的保持时间 (s) */
    float    hold_return_s;             /**< 归正 (RESET) 到位后的保持时间 (s) */
    uint8_t  repeat;                    /**< 循环次数, 0=无限循环 */
} SWActionGroupCfg;

/**
 * @brief 启动循环动作组: DOWN(电机0) -> RESET -> UP(电机1) -> RESET -> 循环
 *        偏转角度取 bend_theta[0]/bend_theta[1], 方向取 bend_dir
 * @param cfg 动作组参数, NULL 时不动作
 */
void SW_action_group_start(const SWActionGroupCfg *cfg);

/**
 * @brief 以默认参数启动循环动作组
 */
void SW_action_group_start_default(void);

/**
 * @brief 停止循环动作组
 */
void SW_action_group_stop(void);

/**
 * @brief 查询动作组是否运行中
 */
bool SW_action_group_running(void);

/**
 * @brief 动作组状态机, 需按控制周期周期调用 (建议与控制节拍同步)
 */
void SW_action_group_control(void);

extern SWNozzle SW;
#endif
