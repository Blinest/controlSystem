//
// 舵机齿轮控制 — S 弯喷管
// motor_run(id, direction, speed, angle)
//   id        — 舵机逻辑 ID（对应不同 IO/TIM 通道）
//   direction — 0=正转(角度从零增大), 1=反转(角度从零减小)
//   speed     — 角速度 (°/s)，决定 S 曲线运动时长
//   angle     — 目标角度 (°)，经五次 S 曲线平滑后到达
//

#ifndef CONTROLSYSTEM_MOTOR_H
#define CONTROLSYSTEM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 电机数量 ==================== */
#define MOTOR_NUM  4

/* ==================== 控制参数 ==================== */
#define CONTROL_PERIOD_MS  20      /* 50Hz 控制周期 */
#define CONTROL_DT          0.02f
#define MIN_MOVE_TIME       2.5f   /* 最短运动时间 (s) */

/* ==================== 传动比 ====================
 * 齿轮1: 小齿轮 20齿 -> 大齿轮 20齿
 */
#define SMALL1_TEETH    20.0f
#define BIG1_TEETH      20.0f

/* ==================== 舵机参数 ==================== */
#define SERVO_MIN_PULSE         500.0f
#define SERVO_MAX_PULSE         2500.0f

#define SERVO1_TOTAL_ANGLE      360.0f

/* 舵机安装零点脉冲 (us) */
#define SERVO1_ZERO_PULSE       1500.0f

/* 舵机旋转方向：+1 正向，-1 反向 */
#define SERVO1_DIR              1.0f

/* ==================== 全局电机状态 ==================== */
typedef struct Servo {
	uint8_t addr;          /* PWM 舵机地址 */
    float current_angle;  /* 当前角度 (°) */
    float target_angle;   /* 目标角度 (°) */
    float current_speed;
    float target_speed;
} Servo;

typedef struct StepperMotor {
    uint8_t addr;          /* SS_R485 从机地址 */
    float current_angle;   /* 大齿轮当前角度 (°) */
    float target_angle;    /* 大齿轮目标角度 (°) */
    float current_speed;   /* 大齿轮当前角速度 (°/s) */
    float target_speed;    /* 大齿轮目标角速度 (°/s) */
    int32_t current_pulse; /* 当前累计脉冲 */
    int32_t target_pulse;  /* 目标累计脉冲 */
    uint16_t target_rpm;   /* 电机侧目标转速 */
	uint16_t xifen;			/* 电机细分 */
	float step_angle;		/* 步距角 */
    bool status;           /* SS_R485 运动中 */
} StepperMotor;

typedef struct DCMotor {
	uint8_t addr;
	float current_pos;   /* 当前位移量 (mm) */
	float target_pos;    /* 目标位移量 (mm) */
	float current_speed;   /* 当前速度 (mm/s) */
	float target_speed;    /* 目标角速度 (mm/s) */
	int32_t current_pulse; /* 当前累计脉冲 */
	int32_t target_pulse;  /* 目标累计脉冲 */
	uint16_t target_rpm;   /* 电机侧目标转速 */
	bool status;
} DCMotor;

typedef union Motor
{
	DCMotor dc_motor;
	Servo servo;
	StepperMotor stepper_motor;
} Motor;


typedef enum {
    MOTOR_TYPE_STEPPER = 0,  /* 0、1 号：步进电机 (SS_R485) */
    MOTOR_TYPE_DC      = 1,  /* 2 号：直流电机 (AQMD245NS) */
    MOTOR_TYPE_SERVO   = 2,  /* 3 号：舵机 (PWM) */
} MotorType;

typedef struct {
    int    id;           /* 舵机逻辑 ID（0-based） */
	Motor motor;
    MotorType type;      /* 电机类型 */
    bool   status;      /* 电机运动中 */

    /* S 曲线运行时状态 */
    float  start_angle; /* 本段起始角度 (°) */
    float  t;           /* 本段已用时 (s) */
    float  total_t;     /* 本段预计总时间 (s) */
} GlobalMotor;

/* ==================== 函数声明 ==================== */

void motor_init(void);

void motor_run(int id, uint8_t direction, float speed, float angle);

void motor_run_SS(int id, uint8_t direction, float speed, float angle);

void motor_run_SS_abs(int id, uint8_t direction, float speed, float angle);

void motor_run_AQ_abs(int id, uint8_t direction, float speed, float displacement);

void motor_run_servo(int idx, uint8_t direction, float speed, float angle);

void motor_control_step(void);

void motor_stop_all(void);

void motor_status_check(void);

extern GlobalMotor global_motor[MOTOR_NUM];

#endif
