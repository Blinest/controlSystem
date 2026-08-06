#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* ==================== 舵机通道映射 ====================
 * id → {TIM, Channel, GPIO} 由 servo_register() 注册
 * 最大支持的舵机数量 */
#define SERVO_MAX_CNT  8

/* 单路舵机描述符 */
typedef struct {
    TIM_TypeDef  *tim;         /* TIM 实例（TIM1~TIM17） */
    uint32_t      channel;     /* TIM_CHANNEL_1 ~ _4      */
    uint16_t      min_pulse;   /* 最小脉宽 (us)           */
    uint16_t      max_pulse;   /* 最大脉宽 (us)           */
} ServoDesc;

/* ==================== 全局函数 ==================== */

/* 初始化 TIM3 PWM 硬件（默认注册 1 路舵机，地址 4） */
void servo_pwm_init(void);

/**
 * 注册一路舵机 — 调用后可用 id 设置脉冲
 * @param id      逻辑 ID（0-based）
 * @param tim     TIM 实例指针
 * @param channel TIM_CHANNEL_x
 * @param min_pulse  最小脉宽 (us)
 * @param max_pulse  最大脉宽 (us)
 * @return 0=成功, -1=id 超出范围
 */
int servo_register(uint8_t id, TIM_TypeDef *tim, uint32_t channel,
                   uint16_t min_pulse, uint16_t max_pulse);

/**
 * 通过逻辑 ID 设置舵机脉宽
 * @param id       逻辑 ID
 * @param pulse_us 脉宽 (us)，会自动限幅到 [min, max]
 */
void servo_set_pulse(uint8_t id, uint16_t pulse_us);

/* 默认舵机寄存器宏 — 快速声明 */
#define SERVO_REG_DEFAULT(id, tim, ch) \
    servo_register(id, tim, TIM_CHANNEL_##ch, 500, 2500)

#endif
