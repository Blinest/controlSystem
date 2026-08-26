#include "servo_pwm.h"
#include "gpio.h"

TIM_HandleTypeDef g_servo_tim_handle;
TIM_OC_InitTypeDef g_servo_oc_handle;

/* ==================== id → TIM 通道映射表 ==================== */
static ServoDesc s_servo_tbl[SERVO_MAX_CNT];
static uint8_t   s_servo_cnt = 0;

int servo_register(uint8_t id, TIM_TypeDef *tim, uint32_t channel,
                   uint16_t min_pulse, uint16_t max_pulse)
{
    if (id >= SERVO_MAX_CNT) return -1;
    s_servo_tbl[id].tim       = tim;
    s_servo_tbl[id].channel   = channel;
    s_servo_tbl[id].min_pulse = min_pulse;
    s_servo_tbl[id].max_pulse = max_pulse;
    if (id >= s_servo_cnt) s_servo_cnt = id + 1;
    return 0;
}

void servo_set_pulse(uint8_t id, uint16_t pulse_us)
{
    if (id >= s_servo_cnt) return;
    const ServoDesc *d = &s_servo_tbl[id];
    if (pulse_us < d->min_pulse) pulse_us = d->min_pulse;
    if (pulse_us > d->max_pulse) pulse_us = d->max_pulse;

    /* 直接写 TIM3 CCR 寄存器 */
    if (d->channel == TIM_CHANNEL_1)
        TIM3->CCR1 = pulse_us;
    else if (d->channel == TIM_CHANNEL_2)
        TIM3->CCR2 = pulse_us;
    else if (d->channel == TIM_CHANNEL_3)
        TIM3->CCR3 = pulse_us;
    else
        TIM3->CCR4 = pulse_us;
}

/* ==================== TIM3 PWM 初始化（默认 2 路） ==================== */

void servo_pwm_init(void)
{
    g_servo_tim_handle.Instance = TIM3;
    g_servo_tim_handle.Init.Prescaler = 72 - 1;         /* 1MHz */
    g_servo_tim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_servo_tim_handle.Init.Period = 20000 - 1;         /* 50Hz */
    g_servo_tim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_servo_tim_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&g_servo_tim_handle);

    g_servo_oc_handle.OCMode = TIM_OCMODE_PWM1;
    g_servo_oc_handle.Pulse = 1500;
    g_servo_oc_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
    g_servo_oc_handle.OCFastMode = TIM_OCFAST_DISABLE;

    /* CH2 (PB5) — id=0 */
    HAL_TIM_PWM_ConfigChannel(&g_servo_tim_handle, &g_servo_oc_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&g_servo_tim_handle, TIM_CHANNEL_2);

    /* CH1 (PB4) — id=1 */
    HAL_TIM_PWM_ConfigChannel(&g_servo_tim_handle, &g_servo_oc_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_servo_tim_handle, TIM_CHANNEL_1);

    /* 注册默认映射: id 0 → CH2(PB5), id 1 → CH1(PB4) */
    servo_register(0, TIM3, TIM_CHANNEL_2, 500, 2500);
    servo_register(1, TIM3, TIM_CHANNEL_1, 500, 2500);
}

/* ==================== MSP 初始化 ==================== */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        GPIO_InitTypeDef gpio = {0};

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_AFIO_CLK_ENABLE();

        __HAL_AFIO_REMAP_SWJ_NOJTAG();
        __HAL_AFIO_REMAP_TIM3_PARTIAL();

        gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}
