/**********************************************************
***	编写作者：blinest
***	qq：1071378062
*** date: 2026.07.24
* brief 山社R485型驱动器 Modbus RTU 通信实现骨架
* 功能包括：
* 向下位机发送控制指令
**********************************************************/

#include "SS_R485.h"
#include <string.h>

#include "usart.h"

/* ====================================================================
 *  平台相关 — 需用户实现
 * ==================================================================== */
extern int platform_uart_send(const uint8_t *data, uint16_t len);
extern int platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/* ====================================================================
 *  底层 Modbus 收发
 * ==================================================================== */

static int modbus_read_registers_raw(uint8_t addr, uint16_t start_reg,
                                     uint16_t count, uint8_t *rx_data)
{
    uint8_t tx[8];
    uint8_t rx[5 + count * 2 + 2];  /* 响应: addr+func+byteCount+data+crc */
    uint16_t data_len;
    uint16_t crc_calc, crc_rx;
    int ret;

    /* 构建请求报文 */
    BUILD_READ_REQ(tx, addr, start_reg, count);

    uart1_bus_lock();   /* 保护「发请求→读回复」整条事务，防止与其他任务写入交错 */

    /* 发送 */
    if (platform_uart_send(tx, sizeof(tx)) != 0)
    {
        ret = -1;
        goto out;
    }

    /* 接收: 最小响应长度 5 字节 */
    data_len = (uint16_t)(count * 2);
    if (platform_uart_recv(rx, (uint16_t)(5 + data_len), 500) != 0)
    {
        ret = -2;
        goto out;
    }

    /* 校验 CRC */
    crc_rx = rx[5 + data_len] | (uint16_t)(rx[5 + data_len + 1] << 8);
    crc_calc = modbus_crc16(rx, (uint16_t)(5 + data_len));
    if (crc_calc != crc_rx)
    {
        ret = -3;
        goto out;
    }

    /* 复制数据 */
    memcpy(rx_data, rx + 3, data_len);

    ret = 0;

out:
    uart1_bus_unlock();
    return ret;
}

static int modbus_write_register_raw(uint8_t addr, uint16_t reg_addr,
                                     uint16_t value)
{
    uint8_t tx[8];

    BUILD_WRITE_REQ(tx, addr, reg_addr, value);

    if (platform_uart_send(tx, sizeof(tx)) != 0) return -1;

    /* 写操作返回与发送相同报文，可省略接收校验 */
    return 0;
}

/* ====================================================================
 *  高层 API
 * ==================================================================== */
/**
 * @brief 读寄存器
 * @param slave_addr 从机地址
 * @param reg_addrr 寄存器地址
 * @param *value 数据指针
 * @return 
 */
int SS_read_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value)
{
    uint8_t data[2];
    if (modbus_read_registers_raw(slave_addr, reg_addr, 1, data) != 0)
        return -1;
    *value = ((uint16_t)data[0] << 8) | data[1];
    return 0;
}

/**
 * @brief 读多个连续寄存器（原始字节流，不进行字节序转换）
 * @param slave_addr 从机地址
 * @param reg_addr   起始寄存器地址
 * @param count      要读取的寄存器个数（每个寄存器占2字节）
 * @param buf        输出缓冲区，长度至少 count*2 字节
 * @return 底层 modbus_read_registers_raw 的返回值（0 成功，非0 失败）
 * @note 此函数直接透传底层数据，不处理字节序，适用于批量读取或原始数据解析
 */
int SS_read_registers(uint8_t slave_addr, uint16_t reg_addr,
                        uint16_t count, uint8_t *buf)
{
    return modbus_read_registers_raw(slave_addr, reg_addr, count, buf);
}

/**
 * @brief 写单个寄存器（直接透传，数据按大端写入）
 * @param slave_addr 从机地址
 * @param reg_addr   寄存器地址
 * @param value      要写入的16位数值
 * @return 底层 modbus_write_register_raw 的返回值（0 成功，非0 失败）
 */
int SS_write_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t value)
{
    return modbus_write_register_raw(slave_addr, reg_addr, value);
}

/**
 * @brief 写多个连续寄存器（原始字节流，数据按大端顺序排列）
 * @param slave_addr 从机地址
 * @param reg_addr   起始寄存器地址
 * @param count      要写入的寄存器个数（每个寄存器占2字节）
 * @param data       待写入数据，长度至少 count*2 字节
 * @return 0 成功，非0 发送失败或参数错误
 */
int SS_write_registers(uint8_t slave_addr, uint16_t reg_addr,
                       uint16_t count, const uint8_t *data)
{
    if (count == 0U || data == NULL) return -1;

    uint8_t tx[9 + count * 2];
    BUILD_WRITE_MULTI_REQ(tx, slave_addr, reg_addr, count, data);

    if (platform_uart_send(tx, (uint16_t)sizeof(tx)) != 0) return -2;

    /* 写多寄存器返回 addr+func+reg+count+crc，可按需要扩展接收校验 */
    return 0;
}

/* ====================================================================
 *  便捷读写函数（含注释）
 * ==================================================================== */

/**
 * @brief 获取硬件版本号（两个ASCII字符）
 * @param addr    从机地址
 * @param ver_str 输出缓冲区，至少2字节，存放两个ASCII字符（如 'V','1'）
 * @return 0 成功，-1 读取失败
 */
int SS_get_hw_version(uint8_t addr, uint8_t *ver_str)
{
    uint16_t val;
    if (SS_read_register(addr, REG_HW_VERSION, &val) != 0) return -1;
    ver_str[0] = (uint8_t)(val >> 8);
    ver_str[1] = (uint8_t)(val & 0xFF);
    return 0;
}

/**
 * @brief 获取软件版本号（两个ASCII字符）
 * @param addr    从机地址
 * @param ver_str 输出缓冲区，至少2字节，存放两个ASCII字符
 * @return 0 成功，-1 读取失败
 */
int SS_get_sw_version(uint8_t addr, uint8_t *ver_str)
{
    uint16_t val;
    if (SS_read_register(addr, REG_SW_VERSION, &val) != 0) return -1;
    ver_str[0] = (uint8_t)(val >> 8);
    ver_str[1] = (uint8_t)(val & 0xFF);
    return 0;
}

/**
 * @brief 获取电机实时位置（步数）
 * @param addr 从机地址
 * @param pos  输出参数，当前位置值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_position(uint8_t addr, uint16_t *pos)
{
    return SS_read_register(addr, REG_MOTOR_REAL_POS, pos);
}

/**
 * @brief 获取运行状态输入（位标志）
 * @param addr   从机地址
 * @param status 输出参数，状态位
 * @return 0 成功，-1 读取失败
 */
int SS_get_run_status(uint8_t addr, uint16_t *status)
{
    return SS_read_register(addr, REG_RUN_INPUT_STATUS, status);
}

/**
 * @brief 设置串口超时时间（毫秒）
 * @param addr       从机地址
 * @param timeout_ms 超时值（毫秒）
 * @return 0 成功，非0 写入失败
 */
int SS_set_serial_timeout(uint8_t addr, uint16_t timeout_ms)
{
    return SS_write_register(addr, REG_SERIAL_TIMEOUT, timeout_ms);
}

/**
 * @brief 获取串口超时时间（毫秒）
 * @param addr    从机地址
 * @param timeout 输出参数，当前超时值
 * @return 0 成功，-1 读取失败
 */
int SS_get_serial_timeout(uint8_t addr, uint16_t *timeout)
{
    return SS_read_register(addr, REG_SERIAL_TIMEOUT, timeout);
}

/**
 * @brief 设置波特率（枚举值）
 * @param addr 从机地址
 * @param rate 波特率枚举（BaudRate_t）
 * @return 0 成功，非0 写入失败
 */
int SS_set_baud_rate(uint8_t addr, BaudRate_t rate)
{
    return SS_write_register(addr, REG_BAUD_RATE, (uint16_t)rate);
}

/**
 * @brief 获取波特率（枚举值）
 * @param addr 从机地址
 * @param rate 输出参数，当前波特率数值
 * @return 0 成功，-1 读取失败
 */
int SS_get_baud_rate(uint8_t addr, uint16_t *rate)
{
    return SS_read_register(addr, REG_BAUD_RATE, rate);
}

/**
 * @brief 设置平滑滤波常数
 * @param addr 从机地址
 * @param val  滤波常数值
 * @return 0 成功，非0 写入失败
 */
int SS_set_smooth_const(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_SMOOTH_CONST, val);
}

/**
 * @brief 获取平滑滤波常数
 * @param addr 从机地址
 * @param val  输出参数，当前滤波常数值
 * @return 0 成功，-1 读取失败
 */
int SS_get_smooth_const(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_SMOOTH_CONST, val);
}

/**
 * @brief 设置位置误差运行阈值（步数）
 * @param addr  从机地址
 * @param steps 阈值步数
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_err_run_thresh(uint8_t addr, uint16_t steps)
{
    return SS_write_register(addr, REG_POS_ERR_RUN_THRESH, steps);
}

/**
 * @brief 获取位置误差运行阈值（步数）
 * @param addr  从机地址
 * @param steps 输出参数，当前阈值
 * @return 0 成功，-1 读取失败
 */
int SS_get_pos_err_run_thresh(uint8_t addr, uint16_t *steps)
{
    return SS_read_register(addr, REG_POS_ERR_RUN_THRESH, steps);
}

/**
 * @brief 设置位置误差停止阈值（步数）
 * @param addr  从机地址
 * @param steps 阈值步数
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_err_stop_thresh(uint8_t addr, uint16_t steps)
{
    return SS_write_register(addr, REG_POS_ERR_STOP_THRESH, steps);
}

/**
 * @brief 获取位置误差停止阈值（步数）
 * @param addr  从机地址
 * @param steps 输出参数，当前阈值
 * @return 0 成功，-1 读取失败
 */
int SS_get_pos_err_stop_thresh(uint8_t addr, uint16_t *steps)
{
    return SS_read_register(addr, REG_POS_ERR_STOP_THRESH, steps);
}

/**
 * @brief 设置电机额定电流（单位视硬件定义，通常为mA或0.1A）
 * @param addr 从机地址
 * @param curr 电流值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_rated_curr(uint8_t addr, uint16_t curr)
{
    return SS_write_register(addr, REG_MOTOR_RATED_CURR, curr);
}

/**
 * @brief 获取电机额定电流
 * @param addr 从机地址
 * @param curr 输出参数，当前电流值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_rated_curr(uint8_t addr, uint16_t *curr)
{
    return SS_read_register(addr, REG_MOTOR_RATED_CURR, curr);
}

/**
 * @brief 设置空闲/运行最小电流
 * @param addr 从机地址
 * @param val  电流值
 * @return 0 成功，非0 写入失败
 */
int SS_set_idle_run_min_curr(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_IDLE_RUN_MIN_CURR, val);
}

/**
 * @brief 获取空闲/运行最小电流
 * @param addr 从机地址
 * @param val  输出参数，当前电流值
 * @return 0 成功，-1 读取失败
 */
int SS_get_idle_run_min_curr(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_IDLE_RUN_MIN_CURR, val);
}

/**
 * @brief 设置编码器分辨率（线数/圈）
 * @param addr  从机地址
 * @param lines 线数
 * @return 0 成功，非0 写入失败
 */
int SS_set_encoder_res(uint8_t addr, uint16_t lines)
{
    return SS_write_register(addr, REG_ENCODER_RES, lines);
}

/**
 * @brief 获取编码器分辨率（线数/圈）
 * @param addr  从机地址
 * @param lines 输出参数，当前线数
 * @return 0 成功，-1 读取失败
 */
int SS_get_encoder_res(uint8_t addr, uint16_t *lines)
{
    return SS_read_register(addr, REG_ENCODER_RES, lines);
}

/**
 * @brief 设置电机电感量（单位视硬件定义）
 * @param addr 从机地址
 * @param val  电感值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_inductance(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_MOTOR_INDUCTANCE, val);
}

/**
 * @brief 获取电机电感量
 * @param addr 从机地址
 * @param val  输出参数，当前电感值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_inductance(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_MOTOR_INDUCTANCE, val);
}

/**
 * @brief 设置电机电阻（单位视硬件定义）
 * @param addr 从机地址
 * @param val  电阻值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_resistance(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_MOTOR_RESISTANCE, val);
}

/**
 * @brief 获取电机电阻
 * @param addr 从机地址
 * @param val  输出参数，当前电阻值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_resistance(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_MOTOR_RESISTANCE, val);
}

/**
 * @brief 设置位置过冲误差告警阈值（步数）
 * @param addr  从机地址
 * @param steps 阈值步数
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_over_err_warn(uint8_t addr, uint16_t steps)
{
    return SS_write_register(addr, REG_POS_OVER_ERR_WARN, steps);
}

/**
 * @brief 获取位置过冲误差告警阈值（步数）
 * @param addr  从机地址
 * @param steps 输出参数，当前阈值
 * @return 0 成功，-1 读取失败
 */
int SS_get_pos_over_err_warn(uint8_t addr, uint16_t *steps)
{
    return SS_read_register(addr, REG_POS_OVER_ERR_WARN, steps);
}

/**
 * @brief 获取实际位置过冲误差（步数）
 * @param addr 从机地址
 * @param val  输出参数，当前过冲误差值
 * @return 0 成功，-1 读取失败
 */
int SS_get_actual_pos_over_err(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_ACTUAL_POS_OVER_ERR, val);
}

/**
 * @brief 设置报警状态掩码（位使能）
 * @param addr 从机地址
 * @param mask 报警掩码
 * @return 0 成功，非0 写入失败
 */
int SS_set_alarm_status(uint8_t addr, uint16_t mask)
{
    return SS_write_register(addr, REG_ALARM_STATUS, mask);
}

/**
 * @brief 获取报警状态
 * @param addr   从机地址
 * @param status 输出参数，当前报警状态位
 * @return 0 成功，-1 读取失败
 */
int SS_get_alarm_status(uint8_t addr, uint16_t *status)
{
    return SS_read_register(addr, REG_ALARM_STATUS, status);
}

/**
 * @brief 获取最大运行速度（记录值）
 * @param addr  从机地址
 * @param speed 输出参数，最大速度值
 * @return 0 成功，-1 读取失败
 */
int SS_get_max_run_speed(uint8_t addr, uint16_t *speed)
{
    return SS_read_register(addr, REG_MAX_RUN_SPEED, speed);
}

/**
 * @brief 清除最大运行速度记录（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_clear_max_run_speed(uint8_t addr)
{
    return SS_write_register(addr, REG_MAX_RUN_SPEED, 0);
}

/**
 * @brief 获取实时速度
 * @param addr  从机地址
 * @param speed 输出参数，当前速度值
 * @return 0 成功，-1 读取失败
 */
int SS_get_real_time_speed(uint8_t addr, uint16_t *speed)
{
    return SS_read_register(addr, REG_REAL_TIME_SPEED, speed);
}

/**
 * @brief 获取实时电流
 * @param addr 从机地址
 * @param curr 输出参数，当前电流值
 * @return 0 成功，-1 读取失败
 */
int SS_get_real_time_curr(uint8_t addr, uint16_t *curr)
{
    return SS_read_register(addr, REG_REAL_TIME_CURR, curr);
}

/**
 * @brief 设置输入信号滤波延迟（毫秒）
 * @param addr      从机地址
 * @param input_idx 输入通道索引（0~7）
 * @param delay_ms  延迟时间（毫秒）
 * @return 0 成功，-1 输入索引超出范围，其他非0 写入失败
 */
int SS_set_input_delay(uint8_t addr, uint8_t input_idx, uint16_t delay_ms)
{
    static const uint16_t delay_regs[] = {
        REG_INPUT0_DELAY, REG_INPUT1_DELAY, REG_INPUT2_DELAY,
        REG_INPUT3_DELAY, REG_INPUT4_DELAY, REG_INPUT5_DELAY,
        REG_INPUT6_DELAY, REG_INPUT7_DELAY
    };
    if (input_idx > 7) return -1;
    return SS_write_register(addr, delay_regs[input_idx], delay_ms);
}

/**
 * @brief 获取输入信号滤波延迟（毫秒）
 * @param addr      从机地址
 * @param input_idx 输入通道索引（0~7）
 * @param delay     输出参数，当前延迟值
 * @return 0 成功，-1 输入索引超出范围或读取失败
 */
int SS_get_input_delay(uint8_t addr, uint8_t input_idx, uint16_t *delay)
{
    static const uint16_t delay_regs[] = {
        REG_INPUT0_DELAY, REG_INPUT1_DELAY, REG_INPUT2_DELAY,
        REG_INPUT3_DELAY, REG_INPUT4_DELAY, REG_INPUT5_DELAY,
        REG_INPUT6_DELAY, REG_INPUT7_DELAY
    };
    if (input_idx > 7) return -1;
    return SS_read_register(addr, delay_regs[input_idx], delay);
}

/**
 * @brief 设置位置积分常数
 * @param addr 从机地址
 * @param val  积分常数值
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_integral(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_POS_INTEGRAL, val);
}

/**
 * @brief 获取位置积分常数
 * @param addr 从机地址
 * @param val  输出参数，当前积分常数值
 * @return 0 成功，-1 读取失败
 */
int SS_get_pos_integral(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_POS_INTEGRAL, val);
}

/* --------------------------------------------------------------------
 *  最大值/最小值记录读取及清除（写0清零）
 * -------------------------------------------------------------------- */

/**
 * @brief 获取母线电压最大值
 * @param addr 从机地址
 * @param val  输出参数，记录的最大电压值
 * @return 0 成功，-1 读取失败
 */
int SS_get_bus_voltage_max(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_BUS_VOLTAGE_MAX, val);
}

/**
 * @brief 清除母线电压最大值记录（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_clear_bus_voltage_max(uint8_t addr)
{
    return SS_write_register(addr, REG_BUS_VOLTAGE_MAX, 0);
}

/**
 * @brief 获取升压电流最大值
 * @param addr 从机地址
 * @param val  输出参数，记录的最大电流值
 * @return 0 成功，-1 读取失败
 */
int SS_get_boost_curr_max(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_BOOST_CURR_MAX, val);
}

/**
 * @brief 清除升压电流最大值记录（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_clear_boost_curr_max(uint8_t addr)
{
    return SS_write_register(addr, REG_BOOST_CURR_MAX, 0);
}

/**
 * @brief 获取滞后脉冲最大值
 * @param addr 从机地址
 * @param val  输出参数，记录的最大滞后脉冲数
 * @return 0 成功，-1 读取失败
 */
int SS_get_lag_pulse_max(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_LAG_PULSE_MAX, val);
}

/**
 * @brief 清除滞后脉冲最大值记录（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_clear_lag_pulse_max(uint8_t addr)
{
    return SS_write_register(addr, REG_LAG_PULSE_MAX, 0);
}

/**
 * @brief 获取超前脉冲最大值
 * @param addr 从机地址
 * @param val  输出参数，记录的最大超前脉冲数
 * @return 0 成功，-1 读取失败
 */
int SS_get_lead_pulse_max(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_LEAD_PULSE_MAX, val);
}

/**
 * @brief 清除超前脉冲最大值记录（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_clear_lead_pulse_max(uint8_t addr)
{
    return SS_write_register(addr, REG_LEAD_PULSE_MAX, 0);
}

/**
 * @brief 获取母线电压最小值
 * @param addr 从机地址
 * @param val  输出参数，记录的最小电压值
 * @return 0 成功，-1 读取失败
 */
int SS_get_bus_voltage_min(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_BUS_VOLTAGE_MIN, val);
}

/**
 * @brief 清除母线电压最小值记录（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_clear_bus_voltage_min(uint8_t addr)
{
    return SS_write_register(addr, REG_BUS_VOLTAGE_MIN, 0);
}

/**
 * @brief 设置编码器最小分辨率（线数/圈）
 * @param addr 从机地址
 * @param val  分辨率值
 * @return 0 成功，非0 写入失败
 */
int SS_set_encoder_min_res(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_ENCODER_MIN_RES, val);
}

/**
 * @brief 获取编码器最小分辨率
 * @param addr 从机地址
 * @param val  输出参数，当前最小分辨率值
 * @return 0 成功，-1 读取失败
 */
int SS_get_encoder_min_res(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_ENCODER_MIN_RES, val);
}

/* ====================================================================
 *  32位读写辅助函数（内部使用）
 * ==================================================================== */

/**
 * @brief 读取一个32位有符号整数（占两个连续寄存器）
 * @param addr 从机地址
 * @param reg  起始寄存器地址（高16位）
 * @param val  输出参数，存放读取的32位值
 * @return 0 成功，非0 失败（来自底层读寄存器）
 * @note 数据为大端序，先读高16位，再读低16位
 */
static int _read_i32(uint8_t addr, uint16_t reg, int32_t *val)
{
    uint8_t buf[4];
    int r = SS_read_registers(addr, reg, 2, buf);
    if (r == 0) {
        uint32_t low  = ((uint32_t)buf[0] << 8) | (uint32_t)buf[1];
        uint32_t high = ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
        *val = (int32_t)((high << 16) | low);
    }
    return r;
}

/**
 * @brief 写入一个32位有符号整数（占两个连续寄存器）
 * @param addr 从机地址
 * @param reg  起始寄存器地址（高16位）
 * @param val  要写入的32位值
 * @return 0 成功，非0 失败（来自底层写寄存器）
 * @note 数据按大端序拆分，先写高16位，再写低16位
 */
static int _write_i32(uint8_t addr, uint16_t reg, int32_t val)
{
    uint32_t u = (uint32_t)val;
    uint8_t buf[4] = {
        (uint8_t)(u >> 8),
        (uint8_t)(u),
        (uint8_t)(u >> 24),
        (uint8_t)(u >> 16)
    };
    return SS_write_registers(addr, reg, 2, buf);
}

/* ====================================================================
 *  电机方向 & 反转输入
 * ==================================================================== */

/**
 * @brief 获取电机方向（正转/反转）
 * @param addr 从机地址
 * @param dir  输出参数，方向值（具体含义见硬件手册）
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_direction(uint8_t addr, uint16_t *dir)
{
    return SS_read_register(addr, REG_MOTOR_DIR, dir);
}

/**
 * @brief 设置电机方向
 * @param addr 从机地址
 * @param dir  方向值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_direction(uint8_t addr, uint16_t dir)
{
    return SS_write_register(addr, REG_MOTOR_DIR, dir);
}

/**
 * @brief 设置反转输入电平（极性配置）
 * @param addr 从机地址
 * @param val  电平值
 * @return 0 成功，非0 写入失败
 */
int SS_set_rev_input_level(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_REV_INPUT_LEVEL, val);
}

/* ====================================================================
 *  软件限位 (32位)
 * ==================================================================== */

/**
 * @brief 获取软件负限位位置（绝对位置，步数）
 * @param addr 从机地址
 * @param val  输出参数，负限位值
 * @return 0 成功，非0 读取失败
 */
int SS_get_soft_neg_limit(uint8_t addr, int32_t *val)
{
    return _read_i32(addr, REG_SOFT_NEG_LIMIT, val);
}

/**
 * @brief 设置软件负限位位置
 * @param addr 从机地址
 * @param val  负限位值（步数）
 * @return 0 成功，非0 写入失败
 */
int SS_set_soft_neg_limit(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_SOFT_NEG_LIMIT, val);
}

/**
 * @brief 获取软件正限位位置（绝对位置，步数）
 * @param addr 从机地址
 * @param val  输出参数，正限位值
 * @return 0 成功，非0 读取失败
 */
int SS_get_soft_pos_limit(uint8_t addr, int32_t *val)
{
    return _read_i32(addr, REG_SOFT_POS_LIMIT, val);
}

/**
 * @brief 设置软件正限位位置
 * @param addr 从机地址
 * @param val  正限位值（步数）
 * @return 0 成功，非0 写入失败
 */
int SS_set_soft_pos_limit(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_SOFT_POS_LIMIT, val);
}

/* ====================================================================
 *  速度/加减速/目标速度 (16位)
 * ==================================================================== */

/**
 * @brief 获取电机启动速度（单位视硬件定义）
 * @param addr 从机地址
 * @param s    输出参数，启动速度值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_start_speed(uint8_t addr, uint16_t *s)
{
    return SS_read_register(addr, REG_MOTOR_START_SPEED, s);
}

/**
 * @brief 设置电机启动速度
 * @param addr 从机地址
 * @param s    启动速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_start_speed(uint8_t addr, uint16_t s)
{
    return SS_write_register(addr, REG_MOTOR_START_SPEED, s);
}

/**
 * @brief 获取电机停止速度
 * @param addr 从机地址
 * @param s    输出参数，停止速度值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_stop_speed(uint8_t addr, uint16_t *s)
{
    return SS_read_register(addr, REG_MOTOR_STOP_SPEED, s);
}

/**
 * @brief 设置电机停止速度
 * @param addr 从机地址
 * @param s    停止速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_stop_speed(uint8_t addr, uint16_t s)
{
    return SS_write_register(addr, REG_MOTOR_STOP_SPEED, s);
}

/**
 * @brief 获取电机加速时间（单位视硬件定义）
 * @param addr 从机地址
 * @param t    输出参数，加速时间值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_accel_time(uint8_t addr, uint16_t *t)
{
    return SS_read_register(addr, REG_MOTOR_ACCEL_TIME, t);
}

/**
 * @brief 设置电机加速时间
 * @param addr 从机地址
 * @param t    加速时间值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_accel_time(uint8_t addr, uint16_t t)
{
    return SS_write_register(addr, REG_MOTOR_ACCEL_TIME, t);
}

/**
 * @brief 获取电机减速时间
 * @param addr 从机地址
 * @param t    输出参数，减速时间值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_decel_time(uint8_t addr, uint16_t *t)
{
    return SS_read_register(addr, REG_MOTOR_DECEL_TIME, t);
}

/**
 * @brief 设置电机减速时间
 * @param addr 从机地址
 * @param t    减速时间值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_decel_time(uint8_t addr, uint16_t t)
{
    return SS_write_register(addr, REG_MOTOR_DECEL_TIME, t);
}

/**
 * @brief 获取电机目标速度
 * @param addr 从机地址
 * @param s    输出参数，目标速度值
 * @return 0 成功，-1 读取失败
 */
int SS_get_motor_target_speed(uint8_t addr, uint16_t *s)
{
    return SS_read_register(addr, REG_MOTOR_TARGET_SPEED, s);
}

/**
 * @brief 设置电机目标速度
 * @param addr 从机地址
 * @param s    目标速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_motor_target_speed(uint8_t addr, uint16_t s)
{
    return SS_write_register(addr, REG_MOTOR_TARGET_SPEED, s);
}

/* ====================================================================
 *  硬件限位/原点/力矩/运行模式
 * ==================================================================== */

/**
 * @brief 获取硬件限位设置
 * @param addr 从机地址
 * @param val  输出参数，限位配置值
 * @return 0 成功，-1 读取失败
 */
int SS_get_hw_limit(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_HW_LIMIT_SET, val);
}

/**
 * @brief 设置硬件限位
 * @param addr 从机地址
 * @param val  限位配置值
 * @return 0 成功，非0 写入失败
 */
int SS_set_hw_limit(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_HW_LIMIT_SET, val);
}

/**
 * @brief 获取原点端口使能状态
 * @param addr 从机地址
 * @param val  输出参数，使能值
 * @return 0 成功，-1 读取失败
 */
int SS_get_enable_home_port(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_ENABLE_HOME_PORT, val);
}

/**
 * @brief 设置原点端口使能
 * @param addr 从机地址
 * @param val  使能值
 * @return 0 成功，非0 写入失败
 */
int SS_set_enable_home_port(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_ENABLE_HOME_PORT, val);
}

/**
 * @brief 获取第二原点设置
 * @param addr 从机地址
 * @param val  输出参数，第二原点配置
 * @return 0 成功，-1 读取失败
 */
int SS_get_second_home(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_SECOND_HOME_SET, val);
}

/**
 * @brief 设置第二原点
 * @param addr 从机地址
 * @param val  第二原点配置值
 * @return 0 成功，非0 写入失败
 */
int SS_set_second_home(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_SECOND_HOME_SET, val);
}

/**
 * @brief 获取力矩模式设置
 * @param addr 从机地址
 * @param val  输出参数，力矩模式值
 * @return 0 成功，-1 读取失败
 */
int SS_get_torque_mode(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_TORQUE_MODE_SET, val);
}

/**
 * @brief 设置力矩模式
 * @param addr 从机地址
 * @param val  力矩模式值
 * @return 0 成功，非0 写入失败
 */
int SS_set_torque_mode(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_TORQUE_MODE_SET, val);
}

/**
 * @brief 获取运行模式
 * @param addr 从机地址
 * @param mode 输出参数，运行模式值
 * @return 0 成功，-1 读取失败
 */
int SS_get_run_mode(uint8_t addr, uint16_t *mode)
{
    return SS_read_register(addr, REG_RUN_MODE_SET, mode);
}

/**
 * @brief 设置运行模式
 * @param addr 从机地址
 * @param mode 运行模式值
 * @return 0 成功，非0 写入失败
 */
int SS_set_run_mode(uint8_t addr, uint16_t mode)
{
    return SS_write_register(addr, REG_RUN_MODE_SET, mode);
}

/* ====================================================================
 *  输出端口 & 报警
 * ==================================================================== */

/**
 * @brief 置位输出端口（按位掩码）
 * @param addr 从机地址
 * @param mask 需要置位的端口掩码
 * @return 0 成功，非0 写入失败
 */
int SS_output_set_on(uint8_t addr, uint16_t mask)
{
    return SS_write_register(addr, REG_OUTPUT_SET_ON, mask);
}

/**
 * @brief 复位输出端口（按位掩码）
 * @param addr 从机地址
 * @param mask 需要复位的端口掩码
 * @return 0 成功，非0 写入失败
 */
int SS_output_set_off(uint8_t addr, uint16_t mask)
{
    return SS_write_register(addr, REG_OUTPUT_SET_OFF, mask);
}

/**
 * @brief 获取输出端口状态
 * @param addr 从机地址
 * @param s    输出参数，当前输出状态
 * @return 0 成功，-1 读取失败
 */
int SS_get_output_status(uint8_t addr, uint16_t *s)
{
    return SS_read_register(addr, REG_OUTPUT_READ_STATUS, s);
}

/**
 * @brief 读取报警状态（只读）
 * @param addr 从机地址
 * @param s    输出参数，报警状态位
 * @return 0 成功，-1 读取失败
 */
int SS_get_alarm_read(uint8_t addr, uint16_t *s)
{
    return SS_read_register(addr, REG_ALARM_READ_STATUS, s);
}

/**
 * @brief 清除报警（写入0）
 * @param addr 从机地址
 * @return 0 成功，非0 写入失败
 */
int SS_alarm_clear(uint8_t addr)
{
    return SS_write_register(addr, REG_ALARM_CLEAR, 0);
}

/**
 * @brief 设置报警输出分配
 * @param addr 从机地址
 * @param val  输出分配值
 * @return 0 成功，非0 写入失败
 */
int SS_set_alarm_out_assign(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_ALARM_OUT_ASSIGN, val);
}

/**
 * @brief 设置运行输出分配
 * @param addr 从机地址
 * @param val  输出分配值
 * @return 0 成功，非0 写入失败
 */
int SS_set_run_out_assign(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_RUN_OUT_ASSIGN, val);
}

/**
 * @brief 设置位置到达输出分配
 * @param addr 从机地址
 * @param val  输出分配值
 * @return 0 成功，非0 写入失败
 */
int SS_set_in_pos_out_assign(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_IN_POS_OUT_ASSIGN, val);
}

/* ====================================================================
 *  位置提醒 & 表格参数
 * ==================================================================== */

/**
 * @brief 设置位置提醒 X11（32位绝对位置）
 * @param addr 从机地址
 * @param val  位置值（步数）
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_remind_x11(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_POS_REMIND_X11, val);
}

/**
 * @brief 设置表格大小
 * @param addr 从机地址
 * @param size 表格大小
 * @return 0 成功，非0 写入失败
 */
int SS_set_table_size(uint8_t addr, uint16_t size)
{
    return SS_write_register(addr, REG_TABLE_SIZE, size);
}

/**
 * @brief 设置表格指针
 * @param addr 从机地址
 * @param ptr  表格指针值
 * @return 0 成功，非0 写入失败
 */
int SS_set_table_ptr(uint8_t addr, uint16_t ptr)
{
    return SS_write_register(addr, REG_TABLE_PTR, ptr);
}

/**
 * @brief 设置表格起始地址
 * @param addr  从机地址
 * @param start 起始地址
 * @return 0 成功，非0 写入失败
 */
int SS_set_table_start(uint8_t addr, uint16_t start)
{
    return SS_write_register(addr, REG_TABLE_START_ADDR, start);
}

/* ====================================================================
 *  急停 & 回原点输出
 * ==================================================================== */

/**
 * @brief 获取急停状态
 * @param addr 从机地址
 * @param val  输出参数，急停状态值
 * @return 0 成功，-1 读取失败
 */
int SS_get_emergency_stop(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_EMERGENCY_STOP, val);
}

/**
 * @brief 设置急停（使能/禁止）
 * @param addr 从机地址
 * @param val  急停值
 * @return 0 成功，非0 写入失败
 */
int SS_set_emergency_stop(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_EMERGENCY_STOP, val);
}

/**
 * @brief 获取回原点完成输出状态
 * @param addr 从机地址
 * @param val  输出参数，完成状态
 * @return 0 成功，-1 读取失败
 */
int SS_get_home_done_out(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_HOME_DONE_OUT, val);
}

/**
 * @brief 设置回原点完成输出
 * @param addr 从机地址
 * @param val  完成输出值
 * @return 0 成功，非0 写入失败
 */
int SS_set_home_done_out(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_HOME_DONE_OUT, val);
}

/* ====================================================================
 *  快速转换速度（多端口）
 * ==================================================================== */

/**
 * @brief 设置快速速度端口 X0
 * @param addr 从机地址
 * @param val  速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_quick_speed_port_x0(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_QUICK_SPEED_PORT_X0, val);
}

/**
 * @brief 设置快速速度端口 X17
 * @param addr 从机地址
 * @param val  速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_quick_speed_port_x17(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_QUICK_SPEED_PORT_X17, val);
}

/**
 * @brief 设置快速速度值1
 * @param addr 从机地址
 * @param val  速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_quick_speed_val1(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_QUICK_SPEED_VAL1, val);
}

/**
 * @brief 设置快速速度值2
 * @param addr 从机地址
 * @param val  速度值
 * @return 0 成功，非0 写入失败
 */
int SS_set_quick_speed_val2(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_QUICK_SPEED_VAL2, val);
}

/* ====================================================================
 *  触发运行 (32位)
 * ==================================================================== */

/**
 * @brief 获取触发运行脉冲数
 * @param addr 从机地址
 * @param val  输出参数，脉冲数（步数）
 * @return 0 成功，非0 读取失败
 */
int SS_get_trig_run_pulse(uint8_t addr, int32_t *val)
{
    return _read_i32(addr, REG_TRIG_RUN_PULSE, val);
}

/**
 * @brief 设置触发运行脉冲数
 * @param addr 从机地址
 * @param val  脉冲数（步数）
 * @return 0 成功，非0 写入失败
 */
int SS_set_trig_run_pulse(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_TRIG_RUN_PULSE, val);
}

/**
 * @brief 获取触发启动运行值（32位）
 * @param addr 从机地址
 * @param val  输出参数，触发值
 * @return 0 成功，非0 读取失败
 */
int SS_get_trig_start_run(uint8_t addr, int32_t *val)
{
    return _read_i32(addr, REG_TRIG_START_RUN, val);
}

/**
 * @brief 设置触发启动运行值
 * @param addr 从机地址
 * @param val  触发值
 * @return 0 成功，非0 写入失败
 */
int SS_set_trig_start_run(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_TRIG_START_RUN, val);
}

/* ====================================================================
 *  闭环 PID (含16位有符号)
 * ==================================================================== */

/**
 * @brief 获取同步偏差（有符号16位）
 * @param addr 从机地址
 * @param val  输出参数，偏差值（有符号）
 * @return 0 成功，-1 读取失败
 */
int SS_get_sync_deviation(uint8_t addr, int16_t *val)
{
    uint16_t u;
    int r = SS_read_register(addr, REG_SYNC_DEVIATION, &u);
    if (r == 0) *val = (int16_t)u;
    return r;
}

/**
 * @brief 设置同步偏差（有符号16位）
 * @param addr 从机地址
 * @param val  偏差值（有符号）
 * @return 0 成功，非0 写入失败
 */
int SS_set_sync_deviation(uint8_t addr, int16_t val)
{
    return SS_write_register(addr, REG_SYNC_DEVIATION, (uint16_t)val);
}

/**
 * @brief 获取位置环比例系数
 * @param addr 从机地址
 * @param val  输出参数，比例值
 * @return 0 成功，-1 读取失败
 */
int SS_get_pos_proportion(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_POS_PROPORTION, val);
}

/**
 * @brief 设置位置环比例系数
 * @param addr 从机地址
 * @param val  比例值
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_proportion(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_POS_PROPORTION, val);
}

/**
 * @brief 获取速度环比例系数
 * @param addr 从机地址
 * @param val  输出参数，比例值
 * @return 0 成功，-1 读取失败
 */
int SS_get_speed_proportion(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_SPEED_PROPORTION, val);
}

/**
 * @brief 设置速度环比例系数
 * @param addr 从机地址
 * @param val  比例值
 * @return 0 成功，非0 写入失败
 */
int SS_set_speed_proportion(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_SPEED_PROPORTION, val);
}

/**
 * @brief 获取速度环积分系数
 * @param addr 从机地址
 * @param val  输出参数，积分值
 * @return 0 成功，-1 读取失败
 */
int SS_get_speed_integral(uint8_t addr, uint16_t *val)
{
    return SS_read_register(addr, REG_SPEED_INTEGRAL, val);
}

/**
 * @brief 设置速度环积分系数
 * @param addr 从机地址
 * @param val  积分值
 * @return 0 成功，非0 写入失败
 */
int SS_set_speed_integral(uint8_t addr, uint16_t val)
{
    return SS_write_register(addr, REG_SPEED_INTEGRAL, val);
}

/**
 * @brief 设置位置提醒 X17（32位绝对位置）
 * @param addr 从机地址
 * @param val  位置值（步数）
 * @return 0 成功，非0 写入失败
 */
int SS_set_pos_remind_x17(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_POS_REMIND_X17, val);
}

/* ====================================================================
 *  控制命令（包含各种运行/停止/回零/点动等）
 * ==================================================================== */

/**
 * @brief 发送运行/停止命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_run_stop(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_RUN_STOP, cmd);
}

/**
 * @brief 发送执行回原点命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_exec_home(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_EXEC_HOME, cmd);
}

/**
 * @brief 发送点动命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_jog(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_JOG, cmd);
}

/**
 * @brief 发送力矩模式命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_torque_mode(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_TORQUE_MODE, cmd);
}

/**
 * @brief 发送运行持续时间命令（32位）
 * @param addr 从机地址
 * @param val  持续时间值
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_run_duration(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_CMD_RUN_DURATION, val);
}

/**
 * @brief 发送以脉冲方式停止命令（32位）
 * @param addr 从机地址
 * @param val  停止脉冲数
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_run_pulse_stop(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_CMD_RUN_PULSE_STOP, val);
}

/**
 * @brief 发送以绝对位置方式停止命令（32位）
 * @param addr 从机地址
 * @param val  绝对位置值
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_run_abs_stop(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_CMD_RUN_ABS_STOP, val);
}

/**
 * @brief 设置当前绝对位置（32位）
 * @param addr 从机地址
 * @param val  绝对位置值
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_set_curr_abs(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_CMD_SET_CURR_ABS, val);
}

/**
 * @brief 发送离线使能/复位命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_offline_ena_rst(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_OFFLINE_ENA_RST, cmd);
}

/**
 * @brief 发送执行程序命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_exec_prog(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_EXEC_PROG, cmd);
}

/**
 * @brief 发送保存掉电命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_save_power(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_SAVE_POWER, cmd);
}

/**
 * @brief 发送执行表格命令（16位）
 * @param addr 从机地址
 * @param cmd  命令字
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_exec_table(uint8_t addr, uint16_t cmd)
{
    return SS_write_register(addr, REG_CMD_EXEC_TABLE, cmd);
}

/**
 * @brief 发送任意脉冲运行命令（32位）
 * @param addr 从机地址
 * @param val  脉冲数
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_run_pulse_any(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_CMD_RUN_PULSE_ANY, val);
}

/**
 * @brief 发送任意绝对位置运行命令（32位）
 * @param addr 从机地址
 * @param val  绝对位置值
 * @return 0 成功，非0 写入失败
 */
int SS_cmd_run_abs_any(uint8_t addr, int32_t val)
{
    return _write_i32(addr, REG_CMD_RUN_ABS_ANY, val);
}
