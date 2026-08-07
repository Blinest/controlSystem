//
// 舵机齿轮控制实现
// 角度→脉冲: 大齿轮角度 × 传动比 → 舵机角度 → 脉冲
//

#include "Motor/Motor.h"
#include "Motor/SS_R485.h"
#include "Motor/AQMD245NS.h"
#include "PWM/servo_pwm.h"
#include "LYZ/LYZ.h"
#include <math.h>
#include "usart.h"
/* ==================== 全局变量 ==================== */
GlobalMotor global_motor[MOTOR_NUM];

extern int platform_uart_send(const uint8_t *data, uint16_t len);

#define SS_R485_ADDR_BASE       1U
#define AQMD245NS_ADDR_BASE     1U
#define SERVO_ADDR              4U
#define SS_SCREW_LEAD_MM        1.0f    /* 丝杆导程: mm/rev */
#define PI                      3.141535

static uint8_t ss_r485_addr_from_id(int id)
{
    return (uint8_t)(SS_R485_ADDR_BASE + (uint8_t)id);
}

static uint8_t aq_addr_from_id(int id)
{
    return (uint8_t)(AQMD245NS_ADDR_BASE + (uint8_t)id);
}

static uint16_t aq_speed_to_u16(float speed)
{
    float value = fabsf(speed);

    if (value > 65535.0f) value = 65535.0f;
    return (uint16_t)(value + 0.5f);
}



static int32_t ss_angle_to_pulse(uint8_t idx, float angle)
{
    float pulse =  angle * global_motor[idx].motor.stepper_motor.xifen / global_motor[idx].motor.stepper_motor.step_angle;

    return (int32_t)((pulse >= 0.0f) ? (pulse + 0.5f) : (pulse - 0.5f));
}

static uint16_t ss_screw_speed_to_rpm(float speed_mm_s)
{
    float rpm = fabsf(speed_mm_s) * 60.0f / SS_SCREW_LEAD_MM;

    if (rpm < 1.0f) rpm = 1.0f;
    if (rpm > 65535.0f) rpm = 65535.0f;
    return (uint16_t)(rpm + 0.5f);
}

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

static uint16_t servo_signed_angle_to_pulse(float signed_angle)
{
    float pulse = 1500.0f + signed_angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180.0f;

    if (pulse < SERVO_MIN_PULSE) pulse = SERVO_MIN_PULSE;
    if (pulse > SERVO_MAX_PULSE) pulse = SERVO_MAX_PULSE;
    return (uint16_t)(pulse + 0.5f);
}

static uint16_t servo_angle_to_pulse(uint8_t direction, float angle)
{
    float signed_angle = (direction == 0) ? angle : -angle;
    return servo_signed_angle_to_pulse(signed_angle);
}

/* ==================== 初始化 ==================== */

void motor_init(void)
{
    servo_pwm_init();
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        global_motor[i].id = i;

        switch (i)
        {
        case 0:
        case 1:
            /* 步进电机 (SS_R485) */
            global_motor[i].type = MOTOR_TYPE_STEPPER;
            global_motor[i].motor.stepper_motor.addr = ss_r485_addr_from_id(i);
            global_motor[i].motor.stepper_motor.current_angle = 0.0f;
            global_motor[i].motor.stepper_motor.target_angle = 0.0f;
            global_motor[i].motor.stepper_motor.current_speed = 0.0f;
            global_motor[i].motor.stepper_motor.target_speed = 0.0f;
            global_motor[i].motor.stepper_motor.current_pulse = 0;
            global_motor[i].motor.stepper_motor.target_pulse = 0;
            global_motor[i].motor.stepper_motor.target_rpm = 0;
            global_motor[i].motor.stepper_motor.xifen = 20;
            global_motor[i].motor.stepper_motor.step_angle = 1.8f;
            global_motor[i].motor.stepper_motor.status = false;
            break;

        case 2:
            /* 直流电机 (AQMD245NS) */
            global_motor[i].type = MOTOR_TYPE_DC;
            global_motor[i].motor.dc_motor.addr = aq_addr_from_id(i);
            global_motor[i].motor.dc_motor.current_pos = 0.0f;
            global_motor[i].motor.dc_motor.target_pos = 0.0f;
            global_motor[i].motor.dc_motor.current_speed = 0.0f;
            global_motor[i].motor.dc_motor.target_speed = 0.0f;
            global_motor[i].motor.dc_motor.current_pulse = 0;
            global_motor[i].motor.dc_motor.target_pulse = 0;
            global_motor[i].motor.dc_motor.target_rpm = 0;
            global_motor[i].motor.dc_motor.status = false;
            break;

        case 3:
            /* 舵机 (PWM) */
            global_motor[i].type = MOTOR_TYPE_SERVO;
            global_motor[i].motor.servo.addr = SERVO_ADDR;
            global_motor[i].motor.servo.current_angle = 0.0f;
            global_motor[i].motor.servo.target_angle = 0.0f;
            global_motor[i].motor.servo.current_speed = 0.0f;
            global_motor[i].motor.servo.target_speed = 0.0f;
            break;

        default:
            break;
        }

        global_motor[i].status = 0;
        global_motor[i].start_angle = 0.0f;
        global_motor[i].t = 0.0f;
        global_motor[i].total_t = 0.0f;
    }

    /* 舵机回零 */
    servo_set_pulse(SERVO_ADDR, servo_angle_to_pulse(0, 0.0f));
	/* 舵机回零 */
}

/* ==================== motor_run ==================== */

void motor_run_SS(int idx, uint8_t direction, float speed, float angle)
{
    if (idx < 0 || idx >= MOTOR_NUM) return;

    Motor *m = &global_motor[idx].motor;

    float target = (direction == 0) ? angle : -angle;

    float start = m->stepper_motor.current_angle;
    float diff = target;
    int32_t pulse = ss_angle_to_pulse(idx, diff);
    uint16_t rpm = ss_screw_speed_to_rpm(speed);
    uint8_t addr = m->stepper_motor.addr;

    m->stepper_motor.target_angle = start + diff;
    m->stepper_motor.target_speed = speed;
    m->stepper_motor.target_pulse = m->stepper_motor.current_pulse + pulse;
    m->stepper_motor.target_rpm = rpm;

    global_motor[idx].start_angle = start;
    global_motor[idx].t = 0.0f;
    global_motor[idx].total_t = (speed > 0.001f) ? (fabsf(diff) / speed) : MIN_MOVE_TIME;
    if (global_motor[idx].total_t < MIN_MOVE_TIME)
        global_motor[idx].total_t = MIN_MOVE_TIME;

    if (pulse == 0)
    {
        m->stepper_motor.status = false;
        global_motor[idx].status = false;
        return;
    }

    /* SS_R485 位置控制：先设置目标速度，再按相对脉冲位置运行 */
    // if (SS_set_motor_target_speed(addr, rpm) != 0)
    // {
    //     m->stepper_motor.status = false;
    //     global_motor[idx].status = false;
    //     return;
    // }
    if (SS_cmd_run_pulse_any(addr, pulse) != 0)
    {
        m->stepper_motor.status = false;
        global_motor[idx].status = false;
        return;
    }

    m->stepper_motor.current_angle = m->stepper_motor.target_angle;
    m->stepper_motor.current_pulse = m->stepper_motor.target_pulse;
    m->stepper_motor.current_speed = speed;
    m->stepper_motor.status = true;
    global_motor[idx].status = true;
}

void motor_run_SS_abs(int idx, uint8_t direction, float speed, float angle)
{
    if (idx < 0 || idx >= MOTOR_NUM) return;
    if (global_motor[idx].type != MOTOR_TYPE_STEPPER) return;

    Motor *m = &global_motor[idx].motor;
    float target = (direction == 0) ? angle : -angle;

    if (target < -36000.0f) target = -36000.0f;
    if (target > 36000.0f) target = 36000.0f;

    int32_t abs_pulse = ss_angle_to_pulse(idx, target);
    uint16_t rpm = ss_screw_speed_to_rpm(speed);
    uint8_t addr = m->stepper_motor.addr;

    m->stepper_motor.target_angle = target;
    m->stepper_motor.target_speed = speed;
    m->stepper_motor.target_pulse = abs_pulse;
    m->stepper_motor.target_rpm = rpm;

    global_motor[idx].start_angle = m->stepper_motor.current_angle;
    global_motor[idx].t = 0.0f;
    global_motor[idx].total_t = (speed > 0.001f) ? (fabsf(target - m->stepper_motor.current_angle) / speed) : MIN_MOVE_TIME;
    if (global_motor[idx].total_t < MIN_MOVE_TIME)
        global_motor[idx].total_t = MIN_MOVE_TIME;

    if (SS_cmd_run_abs_any(addr, abs_pulse) != 0)
    {
        m->stepper_motor.status = false;
        global_motor[idx].status = false;
        return;
    }

    m->stepper_motor.current_angle = target;
    m->stepper_motor.current_pulse = abs_pulse;
    m->stepper_motor.current_speed = speed;
    m->stepper_motor.status = true;
    global_motor[idx].status = true;
}
void motor_run_AQ(int idx, uint8_t direction, float speed, float displacement)
{
	if (idx < 0 || idx >= MOTOR_NUM) return;
	if (global_motor[idx].type != MOTOR_TYPE_DC) return;

	Motor *m = &global_motor[idx].motor;
	float target = (direction == 0) ? displacement : -displacement;
	int16_t target_speed = direction == 0 ? aq_speed_to_u16(speed) : -aq_speed_to_u16(speed);
	uint8_t addr = m->dc_motor.addr;
	float pulse_f = target * AQM_POSITION_PULSE_PER_MM;
	int32_t pulse = (int32_t)((pulse_f >= 0.0f) ? (pulse_f + 0.5f) : (pulse_f - 0.5f));

	m->dc_motor.target_pos = target;
	m->dc_motor.target_speed = speed;
	m->dc_motor.target_pulse = pulse;
	m->dc_motor.target_rpm = target_speed;

	aqm_frame_t frame;
	aqm_set_speed(&frame, addr, target_speed);

	/* 同样先清残留、锁总线并发位置命令时消费回显，避免脏字节污染 position-read ring */
	uart1_drain_stale_bytes();
	uart1_bus_lock();
	bool ok = (platform_uart_send(frame.buf, frame.len) == 0);
	if (ok)
	{
		uint8_t echo[8];                 /* 0x06 写单寄存器回显: addr+func+reg(2)+val(2)+crc(2) */
		if (platform_uart_recv(echo, sizeof(echo), 50) == 0)
		{
			ok = (echo[0] == addr);
		}
	}
	uart1_bus_unlock();

	m->dc_motor.current_speed = speed;
	m->dc_motor.status = ok;
	global_motor[idx].status = ok;
}
void motor_run_AQ_abs(int idx, uint8_t direction, float speed, float displacement)
{
    if (idx < 0 || idx >= MOTOR_NUM) return;
    if (global_motor[idx].type != MOTOR_TYPE_DC) return;

    Motor *m = &global_motor[idx].motor;
    float target = (direction == 0) ? displacement : -displacement;
    uint16_t target_speed = aq_speed_to_u16(speed);
    uint8_t addr = m->dc_motor.addr;
    float pulse_f = target * AQM_POSITION_PULSE_PER_MM;
    int32_t pulse = (int32_t)((pulse_f >= 0.0f) ? (pulse_f + 0.5f) : (pulse_f - 0.5f));
    bool expect_echo = false;

    m->dc_motor.target_pos = target;
    m->dc_motor.target_speed = speed;
    m->dc_motor.target_pulse = pulse;
    m->dc_motor.target_rpm = target_speed;

    aqm_frame_t frame;
    aqm_set_position(&frame, addr, target_speed, AQM_POS_MODE_ABS, pulse);

    /* 写位置命令会得到从机回显应答。若不放总线锁并消费掉这帧回显，
     * 残留字节会堆在 UART1 ring 里，使后续 motor_status_check 的位置读取
     * 取到脏数据而 CRC 失败，最终 LYZ.current_S 一直不更新。 */
    uart1_drain_stale_bytes();
    uart1_bus_lock();
    if (platform_uart_send(frame.buf, frame.len) == 0)
    {
        uint8_t echo[8];                 /* 0x10 写多寄存器回显: addr+func+reg(2)+cnt(2)+crc(2) */
        if (platform_uart_recv(echo, sizeof(echo), 50) == 0)
        {
            expect_echo = (echo[0] == addr);
        }
    }
    uart1_bus_unlock();

    m->dc_motor.current_speed = speed;
    m->dc_motor.status = expect_echo;
    global_motor[idx].status = expect_echo;
}

void motor_run_servo(int idx, uint8_t direction, float speed, float angle)
{
    if (idx < 0 || idx >= MOTOR_NUM) return;
    if (global_motor[idx].type != MOTOR_TYPE_SERVO) return;

    Motor *m = &global_motor[idx].motor;
    float target = (direction == 0) ? angle : -angle;

	float diff = target - m->servo.current_angle;
    m->servo.target_angle = target;
    m->servo.target_speed = speed;
    m->servo.current_speed = speed;
    global_motor[idx].start_angle = m->servo.current_angle;
    global_motor[idx].t = 0.0f;
	// 计算运动时间
    global_motor[idx].total_t = (speed > 0.001f) ? (fabsf(diff) / speed) : MIN_MOVE_TIME;
    if (global_motor[idx].total_t < MIN_MOVE_TIME)
        global_motor[idx].total_t = MIN_MOVE_TIME;

    if (fabsf(diff) < 0.001f)
    {
        m->servo.current_speed = 0.0f;
        global_motor[idx].status = false;
        return;
    }

    global_motor[idx].status = true;
}

void motor_run(int id, uint8_t direction, float speed, float angle)
{
    if (id < 0 || id >= MOTOR_NUM) return;

    switch (global_motor[id].type)
    {
    case MOTOR_TYPE_STEPPER:
        motor_run_SS_abs(id, direction, speed, angle);
        break;
    case MOTOR_TYPE_DC:
        motor_run_AQ_abs(id, direction, speed, angle);
        break;
    case MOTOR_TYPE_SERVO:
        motor_run_servo(id, direction, speed, angle);
        break;
    default:
        break;
    }
}
/* ==================== 50Hz 控制节拍 ==================== */

void motor_control_step(void)
{
    GlobalMotor *gm = &global_motor[3];
    Motor *m = &gm->motor;

    if (gm->type != MOTOR_TYPE_SERVO) return;
    if (!gm->status) return;
    if (gm->total_t <= 0.001f) return;

    gm->t += CONTROL_DT;
    float s = gm->t / gm->total_t;

    if (s >= 1.0f)
    {
        m->servo.current_angle = m->servo.target_angle;
        m->servo.current_speed = 0.0f;
        servo_set_pulse(m->servo.addr, servo_signed_angle_to_pulse(m->servo.current_angle));
        LYZ.current_phi = m->servo.current_angle;
        gm->status = false;
        return;
    }

    // 计算当前值
    float ratio = s_curve(s);
    m->servo.current_angle = gm->start_angle +
                             (m->servo.target_angle - gm->start_angle) * ratio;
    servo_set_pulse(m->servo.addr, servo_signed_angle_to_pulse(m->servo.current_angle));
    LYZ.current_phi = m->servo.current_angle;
}
/* ==================== 控制辅助 ==================== */

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        GlobalMotor *gm = &global_motor[i];
        Motor *m = &gm->motor;

        switch (gm->type)
        {
        case MOTOR_TYPE_STEPPER:
            (void)SS_cmd_run_stop(m->stepper_motor.addr, 0);
            m->stepper_motor.current_speed = 0.0f;
            m->stepper_motor.status = false;
            break;

        case MOTOR_TYPE_DC:
        {
            aqm_frame_t frame;
            aqm_set_speed(&frame, m->dc_motor.addr, 0);
            (void)platform_uart_send(frame.buf, frame.len);
            m->dc_motor.current_speed = 0.0f;
            m->dc_motor.status = false;
            break;
        }

        default:
            break;
        }

        gm->status = false;
    }
}

void motor_status_check(void)
{
    /* 先处理 UART1 上其他设备主动上报的数据。
     * 这些字节经中断收进 ring 后，被 platform_uart_recv 在等待电机回复时
     * 以「非目标从机首字节」切进上报缓冲，统一在这里消费，避免缓冲溢出。
     * TODO: 按具体上报设备的 帧头+定长 解析各字段。
     * 例：
     *   uint8_t frame[64]; uint16_t len = sizeof(frame);
     *   while (uart1_get_reporting_frame(frame, &len) > 0) { ...解析...; len = sizeof(frame); }
     */
    {
        uint8_t frame[64];
        uint16_t len = sizeof(frame);
        while (uart1_get_reporting_frame(frame, &len) > 0)
        {
            /* 收集到一帧主动上报数据，当前留待按协议解析 */
            len = sizeof(frame);
        }
    }

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (global_motor[i].type != MOTOR_TYPE_DC) continue;

        Motor *m = &global_motor[i].motor;
        int32_t pulse;
        float position_mm;

        if (aqm_get_current_position(m->dc_motor.addr, &pulse, &position_mm) != 0)
            continue;

        m->dc_motor.current_pulse = pulse;
    	LYZ.current_S = pulse / 1400;
        m->dc_motor.current_pos = position_mm;
    }
}
