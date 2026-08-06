/**********************************************************
***	编写作者：blinest
***	qq：1071378062
*** date: 2026.07.24
* brief AQMD2405NS-MT直流电机驱动器通信实现骨架
**********************************************************/
#ifndef CONTROLSYSTEM_AQMD245NS_H
#define CONTROLSYSTEM_AQMD245NS_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== Modbus 功能码 ==================== */
#define MB_FUNC_READ_HOLDING        0x03
#define MB_FUNC_WRITE_SINGLE        0x06
#define MB_FUNC_WRITE_MULTI         0x10

/* ==================== 电机控制相关寄存器 ==================== */
#define AQM_REG_DEVICE_ID           0x0000  /* 设备标识 (R)      */
#define AQM_REG_REAL_PWM            0x0010  /* 实时PWM值 (R)     */
#define AQM_REG_ERROR_STATUS        0x0017  /* 错误状态码 (R)    */
#define AQM_REG_SET_SPEED           0x0040  /* 设置速度/占空比 (W) */
#define AQM_REG_POSITION_CTRL       0x0046  /* 位置控制 (W) */
#define AQM_REG_CURRENT_PULSE       0x002C  /* 当前脉冲数 (R) */

#define AQM_POS_MODE_ABS            0x0000U /* 绝对位置 */
#define AQM_POS_MODE_REL            0x1101U /* 相对位置，小端发送为 01 11 */
#define AQM_POSITION_PULSE_PER_MM   1400.0f /* 位置反馈换算系数 */

/* ==================== Modbus 帧缓冲区 ==================== */
typedef struct {
    uint8_t buf[256];
    uint16_t len;
} aqm_frame_t;

/* ==================== CRC16 ==================== */
uint16_t aqm_crc16(const uint8_t *data, uint16_t len);

/* ==================== 帧构建 ==================== */

/**
 * @brief  构建读保持寄存器帧 (0x03)
 */
uint16_t aqm_build_read(aqm_frame_t *frame, uint8_t slave,
                        uint16_t reg, uint16_t count);

/**
 * @brief  构建写单寄存器帧 (0x06)
 */
uint16_t aqm_build_write(aqm_frame_t *frame, uint8_t slave,
                         uint16_t reg, uint16_t value);

/**
 * @brief  构建写多个寄存器帧 (0x10)，data 按设备要求原样写入
 */
uint16_t aqm_build_write_multi(aqm_frame_t *frame, uint8_t slave,
                               uint16_t reg, uint16_t count,
                               const uint8_t *data);

/* ==================== 帧解析 ==================== */

/**
 * @brief  解析读保持寄存器响应
 * @param  frame  接收帧
 * @param  values 输出寄存器值数组
 * @param  count  期望寄存器数量
 * @return true=成功
 */
bool aqm_parse_read(const aqm_frame_t *frame, uint16_t *values, uint16_t count);

/**
 * @brief  解析写单寄存器响应 (回显校验)
 * @param  frame  接收帧
 * @param  reg    期望寄存器地址
 * @param  value  期望写入值
 * @return true=成功
 */
bool aqm_parse_write(const aqm_frame_t *frame, uint16_t reg, uint16_t value);

/**
 * @brief  解析当前位置反馈帧
 * @param  frame       接收帧，如 03 03 04 58 88 00 00 7A 3D
 * @param  pulse       输出脉冲数，如 0x00008858 = 34904
 * @param  position_mm 输出实际位置，单位 mm，pulse / 1400
 * @return true=成功
 */
bool aqm_parse_position_feedback(const aqm_frame_t *frame,
                                 int32_t *pulse,
                                 float *position_mm);

/* ==================== 电机控制 API ==================== */

/**
 * @brief  设置电机速度/占空比
 * @param  frame  输出帧
 * @param  slave  从站地址
 * @param  duty   速度值
 *                - 正转: 0 ~ 1000 (对应 0.0% ~ 100.0%)
 *                - 反转: -1 ~ -1000 (对应 -0.1% ~ -100.0%)
 *                例: duty=500  → 正转 50.0%, 写入 0x01F4
 *                    duty=-500 → 反转 50.0%, 写入 0xFE0C
 * @return 帧长度
 */
uint16_t aqm_set_speed(aqm_frame_t *frame, uint8_t slave, int16_t duty);

/**
 * @brief  构建位置控制帧
 * @param  frame     输出帧
 * @param  slave     从站地址
 * @param  speed     速度
 * @param  mode      位置模式: AQM_POS_MODE_ABS / AQM_POS_MODE_REL
 * @param  pulse  脉冲数，高字节在前发送，如 3010 -> 00 00 0B C2
 * @return 帧长度
 */
uint16_t aqm_set_position(aqm_frame_t *frame, uint8_t slave,
                          uint16_t speed, uint16_t mode,
                          int32_t pulse);

/**
 * @brief  构建当前脉冲数读取帧
 * @param  frame  输出帧
 * @param  slave  从站地址
 * @return 帧长度
 */
uint16_t aqm_read_current_pulse(aqm_frame_t *frame, uint8_t slave);

/**
 * @brief  读取并解析当前位置
 * @param  slave       从站地址
 * @param  pulse       输出脉冲数
 * @param  position_mm 输出实际位置，单位 mm
 * @return 0 成功，非0 失败
 */
int aqm_get_current_position(uint8_t slave, int32_t *pulse, float *position_mm);

/**
 * @brief  读取实时 PWM 值
 * @param  frame  输出帧
 * @param  slave  从站地址
 * @return 帧长度
 * @note   解析后: pwm × 0.1% = 实际占空比
 */
uint16_t aqm_read_pwm(aqm_frame_t *frame, uint8_t slave);

/**
 * @brief  读取错误状态
 * @param  frame  输出帧
 * @param  slave  从站地址
 * @return 帧长度
 */
uint16_t aqm_read_error(aqm_frame_t *frame, uint8_t slave);

/**
 * @brief  读取设备标识
 * @param  frame  输出帧
 * @param  slave  从站地址
 * @return 帧长度
 */
uint16_t aqm_read_dev_id(aqm_frame_t *frame, uint8_t slave);

#ifdef __cplusplus
}
#endif

#endif
