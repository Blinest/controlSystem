//
// 舵机齿轮控制实现
// 角度→脉冲: 大齿轮角度 × 传动比 → 舵机角度 → 脉冲
//

#include "Motor/Motor.h"
#include "PWM/servo_pwm.h"
#include "SW/SW.h"
#include <math.h>

/* ==================== 全局变量 ==================== */
GlobalMotor global_motor[MOTOR_NUM];

/* ==================== 五次 S 曲线 ==================== */
static float s_curve(float s)
{
    float s2 = s * s;
    float s3 = s2 * s;
    return 10.0f * s3 - 15.0f * s3 * s + 6.0f * s3 * s2;
}

/* ==================== 角度 → 脉冲 ====================
 * 舵机角度 = 大齿轮角度 × 大齿轮齿数 / 小齿轮齿数
 * 脉冲 = 零位脉冲 + 方向 × 舵机角度 × (最大脉冲-最小脉冲) / 舵机总行程
 */
static float gear_angle_to_pulse(int id, float gear_deg)
{
    float pulse;

    if (id == 0) {
        pulse = SERVO1_ZERO_PULSE
              + SERVO1_DIR
              * gear_deg * BIG1_TEETH / SMALL1_TEETH
              * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
              / SERVO1_TOTAL_ANGLE;
    } else {
        pulse = SERVO2_ZERO_PULSE
              + SERVO2_DIR
              * gear_deg * BIG2_TEETH / SMALL2_TEETH
              * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
              / SERVO2_TOTAL_ANGLE;
    }

    if (pulse < SERVO_MIN_PULSE) pulse = SERVO_MIN_PULSE;
    if (pulse > SERVO_MAX_PULSE) pulse = SERVO_MAX_PULSE;
    return pulse;
}

/* ==================== 初始化 ==================== */

void motor_init(void)
{
    servo_pwm_init();

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        global_motor[i].id = i;
        global_motor[i].servo.current_angle = 0.0f;
        global_motor[i].servo.filt_angle    = 0.0f;
        global_motor[i].servo.target_angle = 0.0f;
        global_motor[i].servo.current_speed = 0.0f;
        global_motor[i].servo.target_speed = 20;   /* 默认速度 (°/s), 供 FUNC_PARA 覆盖 */
        global_motor[i].status = 0;
        global_motor[i].start_angle = 0.0f;
        global_motor[i].t = 0.0f;
        global_motor[i].total_t = 0.0f;
    }

    /* 舵机回零 */
    for (int i = 0; i < MOTOR_NUM; i++)
        servo_set_pulse((uint8_t)i, (uint16_t)gear_angle_to_pulse(i, 0.0f));
}

/* ==================== motor_run ==================== */

void motor_run(int id, uint8_t direction, float speed, float angle)
{
    if (id < 0 || id >= MOTOR_NUM) return;

    GlobalMotor *m = &global_motor[id];

    if (speed > SERVO_SPEED_MAX) speed = SERVO_SPEED_MAX;   /* 限幅: 0~50°/s */
    float target = (direction == 0) ? angle : -angle;

    /* 如果目标角度和速度都没变，忽略重复调用 */
    if (m->status &&
        fabsf(m->servo.target_angle - target) < 0.01f &&
        fabsf(m->servo.target_speed - speed) < 0.01f)
        return;

    m->start_angle         = m->servo.current_angle;
    m->servo.target_angle  = target;
    m->servo.target_speed  = speed;
    m->t                   = 0.0f;

    float diff = fabsf(m->servo.target_angle - m->start_angle);
    /* speed 语义为 S 曲线峰值速度: 换算为等效平均速度再算时长 */
    float v_avg = speed * (1.0f / S_CURVE_PEAK_FACTOR);
    m->total_t = (v_avg > 0.001f) ? (diff / v_avg) : MIN_MOVE_TIME;
    if (m->total_t < MIN_MOVE_TIME)
        m->total_t = MIN_MOVE_TIME;

    /* 同步 SW 目标角度 */
    SW.joint_space.target_theta[id] = target;

    m->status = 1;
}

/* ==================== 50Hz 控制节拍 ==================== */

void motor_control_step(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        GlobalMotor *m = &global_motor[i];
        if (!m->status) continue;

        m->t += CONTROL_DT;

        if (m->t >= m->total_t)
        {
            m->servo.current_angle = m->servo.target_angle;
            m->status = 0;
        }
        else
        {
            float s = m->t / m->total_t;
            float p = s_curve(s);
            m->servo.current_angle = m->start_angle
                                   + (m->servo.target_angle - m->start_angle) * p;
        }

        /* 带传动比的脉冲映射 */
        float pulse = gear_angle_to_pulse(i, m->servo.current_angle);
        servo_set_pulse((uint8_t)i, (uint16_t)pulse);

        /* ---- 反馈仿真: 一阶低通模拟舵机实际响应, 使反馈贴近实体位置 ----
         * filt_angle 向命令轨迹收敛, 收敛快慢由 SERVO_TIME_CONSTANT 决定;
         * current_speed 为滤波位置的变化率, 上报的峰值速度将 ≲ speed。 */
        float cmd = m->servo.current_angle;
        float prev = m->servo.filt_angle;
        float next;
        if (m->status == 0 && fabsf(cmd - prev) < FEEDBACK_SNAP_DEG)
        {
            next = cmd;                       /* 运动结束且已收敛: 直接对齐, 消除残留误差 */
        }
        else
        {
            next = prev + FEEDBACK_ALPHA * (cmd - prev);
        }
        m->servo.filt_angle   = next;
        m->servo.current_speed = (next - prev) / CONTROL_DT;

        /* 将仿真实际位置同步到 SW 结构体 (反馈上报用) */
        SW.joint_space.current_theta[i] = m->servo.filt_angle;
    }
}

/* 紧急停止: 冻结舵机位置, 命令与反馈软件状态归零 */
void motor_feedback_reset(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        GlobalMotor *m = &global_motor[i];
        m->status = 0;
        m->servo.target_angle = 0.0f;
        m->servo.current_angle = 0.0f;
        m->servo.filt_angle    = 0.0f;
        m->servo.current_speed = 0.0f;
        SW.joint_space.target_theta[i]  = 0.0f;
        SW.joint_space.current_theta[i] = 0.0f;
    }
}

/* ==================== 控制辅助 ==================== */

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        GlobalMotor *m = &global_motor[i];
        m->status = 0;
        m->servo.target_angle = m->servo.current_angle;
        /* 舵机物理停转: 反馈位置冻结在当前滤波位置, 速度归零 */
        m->servo.current_speed = 0.0f;
    }
}

void motor_status_check(void)
{
}
