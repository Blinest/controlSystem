//
// 舵机齿轮控制 — S 弯喷管
// motor_run(id, direction, speed, angle)
//   id        — 舵机逻辑 ID（对应不同 IO/TIM 通道）
//   direction — 0=正转(角度从零增大), 1=反转(角度从零减小)
//   speed     — 峰值角速度 (°/s)，S 曲线按"峰值=速度"换算时长，反馈峰值不会超过该值
//   angle     — 目标角度 (°)，经五次 S 曲线平滑后到达
//

#ifndef CONTROLSYSTEM_MOTOR_H
#define CONTROLSYSTEM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 电机数量 ==================== */
#define MOTOR_NUM  2

/* ==================== 控制参数 ==================== */
#define CONTROL_PERIOD_MS  20      /* 50Hz 控制周期 */
#define CONTROL_DT          0.02f
#define MIN_MOVE_TIME       0.5f   /* 最短运动时间 (s) — 仅防除零, 不得掩盖速度差异 */

/* ==================== 速度语义与反馈仿真 ==================== */
#define S_CURVE_PEAK_FACTOR   1.875f          /* 五次S曲线峰值速度系数 (max d(s_curve)/ds = 1.875) */
#define SERVO_SPEED_MAX       50.0f           /* 可设置速度上限 (°/s) — 上限以上被滤波时间常数掩盖 */
#define SERVO_TIME_CONSTANT   0.10f           /* 舵机响应时间常数 τ (s), 反馈低通滤波用 */
#define FEEDBACK_ALPHA        (CONTROL_DT / (CONTROL_DT + SERVO_TIME_CONSTANT))
#define FEEDBACK_SNAP_DEG     0.01f           /* 反馈与命令误差小于此值时直接对齐 */

/* ==================== 传动比 ====================
 * 齿轮1: 小齿轮 20齿 -> 大齿轮 188齿
 * 齿轮2: 小齿轮 25齿 -> 大齿轮 233齿
 */
#define SMALL1_TEETH    20.0f
#define BIG1_TEETH      188.0f
#define SMALL2_TEETH    25.0f
#define BIG2_TEETH      233.0f

/* ==================== 舵机参数 ==================== */
#define SERVO_MIN_PULSE         500.0f
#define SERVO_MAX_PULSE         2500.0f

#define SERVO1_TOTAL_ANGLE      360.0f
#define SERVO2_TOTAL_ANGLE      360.0f

/* 舵机安装零点脉冲 (us) */
#define SERVO1_ZERO_PULSE       2000.0f
#define SERVO2_ZERO_PULSE       800.0f

/* 舵机旋转方向：+1 正向，-1 反向 */
#define SERVO1_DIR              -1.0f
#define SERVO2_DIR              1.0f

/* ==================== 全局电机状态 ==================== */
typedef struct Servo {
    float current_angle;  /* 大齿轮命令角度 (°) — S 曲线轨迹 */
    float target_angle;   /* 大齿轮目标角度 (°) */
    float filt_angle;     /* 模拟舵机实际位置 (°) — 低通滤波, 用于反馈上报 */
    float current_speed;  /* 反馈角速度 (°/s) — 滤波实际位置的变化率 */
    volatile float target_speed;	  /* 目标峰值速度 (°/s) */
} Servo;

typedef struct {
    int    id;           /* 舵机逻辑 ID（0-based） */
    Servo servo;
    bool   status;      /* S 曲线运动中 */

    /* S 曲线运行时状态 */
    float  start_angle; /* 本段起始角度 (°) */
    float  t;           /* 本段已用时 (s) */
    float  total_t;     /* 本段预计总时间 (s) */
} GlobalMotor;

/* ==================== 函数声明 ==================== */

void motor_init(void);

void motor_run(int id, uint8_t direction, float speed, float angle);

/** 复位命令与反馈软件状态 (紧急停止用): 冻结舵机, 命令/反馈归零 */
void motor_feedback_reset(void);

void motor_control_step(void);

void motor_stop_all(void);

void motor_status_check(void);

extern GlobalMotor global_motor[MOTOR_NUM];

#endif
