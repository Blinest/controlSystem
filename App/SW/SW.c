/**********************************************************
***	编写作者：blinest
***	qq：1071378062
*
* brief 上层控制实现
* 功能包括：
* 1.基于运动学模型的电机控制，主要用于系统运动
* 2.传感器数据读取，主要用于环境感知
**********************************************************/
#include "SW.h"
#include "usart.h"
#include "Motor/Motor.h"
#include <stdio.h>
#include "math.h"
#include "cmsis_os2.h"

SWNozzle SW;

void SW_init(void)
{
    // 系统初始化
    SW.joint_space.target_theta[0] = 0.0f;
    SW.joint_space.target_theta[1] = 0.0f;
    SW.joint_space.current_theta[0]   = 0.0f;
    SW.joint_space.current_theta[1]   = 0.0f;

    motor_init();

}
void SW_kinematic_control(const uint8_t idx, const uint8_t dir, const float theta[])
{
    if(idx >= MOTOR_NUM) return;

    SW.joint_space.target_theta[0] = theta[0];
    SW.joint_space.target_theta[1] = theta[1];

    /* 峰值速度 (°/s): 由上位机 FUNC_PARA 写入 target_speed, motor_init 预置默认值 */
    float vel = global_motor[idx].servo.target_speed;

    motor_run(idx, dir, vel, theta[idx]);
}

/* ==================== 循环动作组 ====================
 * 动作组由 SW_kinematic_control 驱动完成上下偏转与归正的循环控制:
 *   DOWN(电机0 下偏转) -> 保持 -> RESET(全电机归零) -> 保持
 *   -> UP(电机1 上偏转) -> 保持 -> RESET(全电机归零) -> 保持 -> (按 repeat 循环)
 * 运动完成以 motor_run 的 S 曲线状态 (status 清零) 判断,
 * 保持阶段用软件定时计数, 因此整个状态机需按控制周期周期调用。
 */

typedef enum
{
    AG_IDLE = 0,      /* 未运行 */
    AG_DOWN,          /* 下偏转 (电机0) */
    AG_UP,            /* 上偏转 (电机1) */
    AG_DOWN_HOLD,     /* 下偏转到位保持 */
    AG_UP_HOLD,       /* 上偏转到位保持 */
    AG_RESET,         /* 归正复位 (全电机回 0°) */
    AG_RESET_HOLD     /* 归正到位保持 */
} SWActionGroupState;

static SWActionGroupCfg s_ag_cfg;
static SWActionGroupState s_ag_state = AG_IDLE;
static uint32_t s_ag_hold_cnt = 0;   /* 保持阶段剩余计时 (控制周期个数) */
static uint32_t s_ag_cycle = 0;      /* 已完成的循环数 */
static bool s_ag_last_was_down = true; /* 上一轮动作是否为 DOWN (用于交替) */

/* 默认动作组: DOWN(电机0 下偏转 25°) -> RESET -> UP(电机1 上偏转 25°) -> RESET -> 无限循环
 * 角度/方向用法与 pc_cmd_parser 的 SPECIAL_DOWN/UP/RESET 一致 (dir=0 正向, bend_theta[i] 取各电机角度) */
static const SWActionGroupCfg s_ag_default_cfg = {
    .bend_theta    = {25.0f, 25.0f},
    .bend_dir      = 0,
    .hold_bend_s   = 1.0f,
    .hold_return_s = 1.0f,
    .repeat        = 0,   /* 无限循环 */
};

void SW_action_group_start(const SWActionGroupCfg *cfg)
{
    if (cfg == NULL) return;
    s_ag_cfg = *cfg;
    s_ag_state = AG_DOWN;
    s_ag_hold_cnt = 0;
    s_ag_cycle = 0;
    SW_kinematic_control(0, s_ag_cfg.bend_dir, s_ag_cfg.bend_theta);
}

void SW_action_group_start_default(void)
{
    SW_action_group_start(&s_ag_default_cfg);
}

void SW_action_group_stop(void)
{
    s_ag_state = AG_IDLE;
    s_ag_hold_cnt = 0;
    s_ag_cycle = 0;
    /* 停止循环并归正回初始状态 (回 0°) */
    SW_kinematic_control(0, 0, ((float[]){0.0f, 0.0f}));
    SW_kinematic_control(1, 0, ((float[]){0.0f, 0.0f}));
}

bool SW_action_group_running(void)
{
    return (s_ag_state != AG_IDLE);
}

/* 所有电机均已到位 (S 曲线运动结束) */
static bool action_group_motion_done(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
        if (global_motor[i].status) return false;
    return true;
}

void SW_action_group_control(void)
{
    if (s_ag_state == AG_IDLE) return;

    switch (s_ag_state)
    {
    case AG_DOWN:
        /* 电机0 下偏转到位后保持 hold_bend 秒 */
        if (action_group_motion_done())
        {
            s_ag_hold_cnt = (uint32_t)(s_ag_cfg.hold_bend_s / CONTROL_DT);
            if (s_ag_hold_cnt > 0) s_ag_state = AG_DOWN_HOLD;
            else goto ag_do_reset;
        }
        break;

    case AG_UP:
        /* 电机1 上偏转到位后保持 hold_bend 秒 */
        if (action_group_motion_done())
        {
            s_ag_hold_cnt = (uint32_t)(s_ag_cfg.hold_bend_s / CONTROL_DT);
            if (s_ag_hold_cnt > 0) s_ag_state = AG_UP_HOLD;
            else goto ag_do_reset;
        }
        break;

    case AG_DOWN_HOLD:
    case AG_UP_HOLD:
        if (s_ag_hold_cnt > 0)
            s_ag_hold_cnt--;
        else
            goto ag_do_reset;
        break;

    case AG_RESET:
        if (action_group_motion_done())
        {
            s_ag_hold_cnt = (uint32_t)(s_ag_cfg.hold_return_s / CONTROL_DT);
            if (s_ag_hold_cnt > 0) s_ag_state = AG_RESET_HOLD;
            else goto ag_next;
        }
        break;

    case AG_RESET_HOLD:
        if (s_ag_hold_cnt > 0)
            s_ag_hold_cnt--;
        else
            goto ag_next;
        break;

    default:
        s_ag_state = AG_IDLE;
        break;
    }
    return;

ag_do_reset:
    /* 归正复位: 全电机回 0° (与 SPECIAL_RESET 一致) */
    for (int i = 0; i < MOTOR_NUM; i++)
        SW_kinematic_control(i, 0, ((float[]){0.0f, 0.0f}));
    s_ag_state = AG_RESET;
    return;

ag_next:
    /* 一个循环完成: DOWN -> RESET -> UP -> RESET */
    s_ag_cycle++;
    if (s_ag_cfg.repeat != 0 && s_ag_cycle >= s_ag_cfg.repeat)
    {
        s_ag_state = AG_IDLE;   /* 循环完成 */
        return;
    }
    /* 交替上下偏转: 上一动作为 DOWN 则下一轮转 UP, 否则转 DOWN */
    if (s_ag_last_was_down)
    {
        s_ag_last_was_down = false;
        s_ag_state = AG_UP;
        SW_kinematic_control(1, s_ag_cfg.bend_dir, s_ag_cfg.bend_theta);
    }
    else
    {
        s_ag_last_was_down = true;
        s_ag_state = AG_DOWN;
        SW_kinematic_control(0, s_ag_cfg.bend_dir, s_ag_cfg.bend_theta);
    }
    return;
}