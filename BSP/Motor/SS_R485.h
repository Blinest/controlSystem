/**********************************************************
***	编写作者：blinest
***	qq：1071378062
*** date: 2026.07.24
* brief 驱动代码实现
* 功能包括：
* 向下位机发送控制指令
**********************************************************/
#ifndef CONTROLSYSTEM_SS_R485_H
#define CONTROLSYSTEM_SS_R485_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  寄存器地址定义 (Register Address Map)
 * ==================================================================== */
#define REG_HW_VERSION          0x0000
#define REG_SW_VERSION          0x0002
#define REG_MOTOR_REAL_POS      0x0004
#define REG_RUN_INPUT_STATUS    0x0006
#define REG_SERIAL_TIMEOUT      0x0008
#define REG_BAUD_RATE           0x0009
#define REG_SMOOTH_CONST        0x000A
#define REG_POS_ERR_RUN_THRESH  0x000B
#define REG_POS_ERR_STOP_THRESH 0x000C
#define REG_MOTOR_RATED_CURR    0x000D
#define REG_IDLE_RUN_MIN_CURR   0x000E
#define REG_ENCODER_RES         0x000F
#define REG_POS_OVER_ERR_WARN   0x0010
#define REG_ACTUAL_POS_OVER_ERR 0x0011
#define REG_ALARM_STATUS        0x0012
#define REG_PROG_CMD_EXEC_POS   0x0015
#define REG_MAX_RUN_SPEED       0x0016
#define REG_ENCODER_MIN_RES     0x0017
#define REG_REAL_TIME_SPEED     0x0019
#define REG_REAL_TIME_CURR      0x001A
#define REG_INPUT0_DELAY        0x001B
#define REG_INPUT1_DELAY        0x001C
#define REG_INPUT2_DELAY        0x001D
#define REG_INPUT3_DELAY        0x001E
#define REG_INPUT4_DELAY        0x001F
#define REG_INPUT5_DELAY        0x0020
#define REG_INPUT6_DELAY        0x0021
#define REG_INPUT7_DELAY        0x0022
#define REG_MOTOR_INDUCTANCE    0x0026
#define REG_MOTOR_RESISTANCE    0x0027
#define REG_POS_INTEGRAL        0x0029
#define REG_BUS_VOLTAGE_MAX     0x0044
#define REG_BOOST_CURR_MAX      0x0045
#define REG_LAG_PULSE_MAX       0x0046
#define REG_LEAD_PULSE_MAX      0x0047
#define REG_BUS_VOLTAGE_MIN     0x0048
#define REG_DRIVER_BASE_ADDR    0x0066
#define REG_MOTOR_DIR           0x006B
#define REG_REV_INPUT_LEVEL     0x006C
#define REG_SOFT_NEG_LIMIT      0x006E
#define REG_SOFT_POS_LIMIT      0x0070
#define REG_MOTOR_START_SPEED   0x0096
#define REG_MOTOR_STOP_SPEED    0x0097
#define REG_MOTOR_ACCEL_TIME    0x0098
#define REG_MOTOR_DECEL_TIME    0x0099
#define REG_MOTOR_TARGET_SPEED  0x009A
#define REG_HW_LIMIT_SET        0x009B
#define REG_ENABLE_HOME_PORT    0x009C
#define REG_SECOND_HOME_SET     0x009D
#define REG_TORQUE_MODE_SET     0x009E
#define REG_RUN_MODE_SET        0x009F
#define REG_OUTPUT_SET_ON       0x00A0
#define REG_OUTPUT_SET_OFF      0x00A1
#define REG_OUTPUT_READ_STATUS  0x00A2
#define REG_ALARM_READ_STATUS   0x00A3
#define REG_ALARM_CLEAR         0x00A4
#define REG_ALARM_OUT_ASSIGN    0x00A5
#define REG_RUN_OUT_ASSIGN      0x00A6
#define REG_IN_POS_OUT_ASSIGN   0x00A7
#define REG_POS_REMIND_X11      0x00A8
#define REG_TABLE_SIZE          0x00AA
#define REG_TABLE_PTR           0x00AB
#define REG_TABLE_START_ADDR    0x00AC
#define REG_EMERGENCY_STOP      0x00AD
#define REG_HOME_DONE_OUT       0x00AE
#define REG_QUICK_SPEED_PORT_X0 0x00AF
#define REG_QUICK_SPEED_PORT_X17 0x00B0
#define REG_QUICK_SPEED_VAL1    0x00B3
#define REG_QUICK_SPEED_VAL2    0x00B4
#define REG_TRIG_RUN_PULSE      0x00B6
#define REG_TRIG_START_RUN      0x00BA
#define REG_SYNC_DEVIATION      0x00BE
#define REG_POS_PROPORTION      0x00BF
#define REG_SPEED_PROPORTION    0x00C0
#define REG_SPEED_INTEGRAL      0x00C1
#define REG_POS_REMIND_X17      0x00C2
#define REG_CMD_RUN_STOP        0x00C8
#define REG_CMD_EXEC_HOME       0x00C9
#define REG_CMD_JOG             0x00CA
#define REG_CMD_TORQUE_MODE     0x00CB
#define REG_CMD_RUN_DURATION    0x00CC
#define REG_CMD_RUN_PULSE_STOP  0x00CE
#define REG_CMD_RUN_ABS_STOP    0x00D0
#define REG_CMD_SET_CURR_ABS    0x00D2
#define REG_CMD_OFFLINE_ENA_RST 0x00D4
#define REG_CMD_EXEC_PROG       0x00DB
#define REG_CMD_SAVE_POWER      0x00DC
#define REG_CMD_EXEC_TABLE      0x00DD
#define REG_CMD_RUN_PULSE_ANY   0x00DE
#define REG_CMD_RUN_ABS_ANY     0x00E8
/* ====================================================================
 *  Modbus 功能码
 * ==================================================================== */
#define MODBUS_FC_READ_HOLDING  0x03
#define MODBUS_FC_WRITE_SINGLE  0x06
#define MODBUS_FC_WRITE_MULTI   0x10

/* ====================================================================
 *  波特率枚举值
 * ==================================================================== */
typedef enum {
    BAUD_4800   = 3,
    BAUD_9600   = 6,
    BAUD_19200  = 7,
    BAUD_38400  = 8,
    BAUD_57600  = 9,
    BAUD_115200 = 12,
} BaudRate_t;

/* ====================================================================
 *  报警状态位定义
 * ==================================================================== */
#define ALARM_PHASE_OPEN_SHIFT  0
#define ALARM_TEST_MODE_SHIFT   3
#define ALARM_BIT(n)            (1U << (n))

/* ====================================================================
 *  运行及输入口状态位定义
 * ==================================================================== */
#define STATUS_RUNNING_BIT      0
#define STATUS_X1_INPUT_BIT     1

/* ====================================================================
 *  运行模式 / 方向
 * ==================================================================== */
typedef enum {
    RUN_MODE_IO           = 3,
    RUN_MODE_PULSE_DIR    = 2,
} RunMode_t;
#define MOTOR_DIR_CW    0
#define MOTOR_DIR_CCW   1

/* ====================================================================
 *  Modbus RTU CRC16
 * ==================================================================== */
static inline uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ====================================================================
 *  Modbus 报文结构体
 * ==================================================================== */
typedef struct {
    uint8_t  addr;
    uint8_t  func;
    uint16_t reg;
    uint16_t count;
    uint16_t crc;
} __attribute__((packed)) ModbusReadReq_t;

typedef struct {
    uint8_t  addr;
    uint8_t  func;
    uint8_t  byteCount;
    uint8_t  data[];
} __attribute__((packed)) ModbusReadResp_t;

typedef struct {
    uint8_t  addr;
    uint8_t  func;
    uint16_t reg;
    uint16_t value;
    uint16_t crc;
} __attribute__((packed)) ModbusWriteReq_t;

/* ====================================================================
 *  构建报文辅助宏
 * ==================================================================== */
#define BUILD_READ_REQ(buf, slave_addr, start_reg, count) do { \
    (buf)[0] = (slave_addr);                                    \
    (buf)[1] = MODBUS_FC_READ_HOLDING;                          \
    (buf)[2] = (uint8_t)(((start_reg) >> 8) & 0xFF);            \
    (buf)[3] = (uint8_t)((start_reg) & 0xFF);                   \
    (buf)[4] = (uint8_t)(((count) >> 8) & 0xFF);                \
    (buf)[5] = (uint8_t)((count) & 0xFF);                       \
    uint16_t _crc = modbus_crc16((buf), 6);                     \
    (buf)[6] = (uint8_t)(_crc & 0xFF);                          \
    (buf)[7] = (uint8_t)((_crc >> 8) & 0xFF);                   \
} while(0)

#define BUILD_WRITE_REQ(buf, slave_addr, reg_addr, value) do {  \
    (buf)[0] = (slave_addr);                                    \
    (buf)[1] = MODBUS_FC_WRITE_SINGLE;                          \
    (buf)[2] = (uint8_t)(((reg_addr) >> 8) & 0xFF);             \
    (buf)[3] = (uint8_t)((reg_addr) & 0xFF);                    \
    (buf)[4] = (uint8_t)(((value) >> 8) & 0xFF);                \
    (buf)[5] = (uint8_t)((value) & 0xFF);                       \
    uint16_t _crc = modbus_crc16((buf), 6);                     \
    (buf)[6] = (uint8_t)(_crc & 0xFF);                          \
    (buf)[7] = (uint8_t)((_crc >> 8) & 0xFF);                   \
} while(0)

#define BUILD_WRITE_MULTI_REQ(buf, slave_addr, reg_addr, cnt, data) do { \
    uint16_t _i;                                                         \
    (buf)[0] = (slave_addr);                                             \
    (buf)[1] = MODBUS_FC_WRITE_MULTI;                                   \
    (buf)[2] = (uint8_t)(((reg_addr) >> 8) & 0xFF);                     \
    (buf)[3] = (uint8_t)((reg_addr) & 0xFF);                             \
    (buf)[4] = (uint8_t)(((cnt) >> 8) & 0xFF);                          \
    (buf)[5] = (uint8_t)((cnt) & 0xFF);                                 \
    (buf)[6] = (uint8_t)((cnt) * 2);                                    \
    for (_i = 0; _i < (uint16_t)(cnt) * 2; _i++)                        \
        (buf)[7 + _i] = (data)[_i];                                     \
    uint16_t _crc = modbus_crc16((buf), (uint16_t)(7 + (cnt) * 2));     \
    (buf)[7 + (cnt) * 2] = (uint8_t)(_crc & 0xFF);                      \
    (buf)[8 + (cnt) * 2] = (uint8_t)((_crc >> 8) & 0xFF);               \
} while(0)

/* ====================================================================
 *  高层 API 原型
 * ==================================================================== */
int SS_read_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value);
int SS_read_registers(uint8_t slave_addr, uint16_t reg_addr, uint16_t count, uint8_t *buf);
int SS_write_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t value);
int SS_write_registers(uint8_t slave_addr, uint16_t reg_addr, uint16_t count, const uint8_t *data);

/* ====================================================================
 *  便捷读写函数
 * ==================================================================== */
/* --- 版本信息 --- */
int SS_get_hw_version(uint8_t addr, uint8_t *ver_str);
int SS_get_sw_version(uint8_t addr, uint8_t *ver_str);
/* --- 电机实时位置 --- */
int SS_get_motor_position(uint8_t addr, uint16_t *pos);
/* --- 运行状态 --- */
int SS_get_run_status(uint8_t addr, uint16_t *status);
/* --- 串口 & 波特率 --- */
int SS_set_serial_timeout(uint8_t addr, uint16_t timeout_ms);
int SS_get_serial_timeout(uint8_t addr, uint16_t *timeout);
int SS_set_baud_rate(uint8_t addr, BaudRate_t rate);
int SS_get_baud_rate(uint8_t addr, uint16_t *rate);
/* --- 平滑常数 --- */
int SS_set_smooth_const(uint8_t addr, uint16_t val);
int SS_get_smooth_const(uint8_t addr, uint16_t *val);
/* --- 位置误差报警阈值 --- */
int SS_set_pos_err_run_thresh(uint8_t addr, uint16_t steps);
int SS_get_pos_err_run_thresh(uint8_t addr, uint16_t *steps);
int SS_set_pos_err_stop_thresh(uint8_t addr, uint16_t steps);
int SS_get_pos_err_stop_thresh(uint8_t addr, uint16_t *steps);
/* --- 电机参数 --- */
int SS_set_motor_rated_curr(uint8_t addr, uint16_t curr);
int SS_get_motor_rated_curr(uint8_t addr, uint16_t *curr);
int SS_set_idle_run_min_curr(uint8_t addr, uint16_t val);
int SS_get_idle_run_min_curr(uint8_t addr, uint16_t *val);
int SS_set_encoder_res(uint8_t addr, uint16_t lines);
int SS_get_encoder_res(uint8_t addr, uint16_t *lines);
int SS_set_motor_inductance(uint8_t addr, uint16_t val);
int SS_get_motor_inductance(uint8_t addr, uint16_t *val);
int SS_set_motor_resistance(uint8_t addr, uint16_t val);
int SS_get_motor_resistance(uint8_t addr, uint16_t *val);
/* --- 位置超差 --- */
int SS_set_pos_over_err_warn(uint8_t addr, uint16_t steps);
int SS_get_pos_over_err_warn(uint8_t addr, uint16_t *steps);
int SS_get_actual_pos_over_err(uint8_t addr, uint16_t *val);
/* --- 报警状态 --- */
int SS_set_alarm_status(uint8_t addr, uint16_t mask);
int SS_get_alarm_status(uint8_t addr, uint16_t *status);
/* --- 运行监控 --- */
int SS_get_max_run_speed(uint8_t addr, uint16_t *speed);
int SS_clear_max_run_speed(uint8_t addr);
int SS_get_real_time_speed(uint8_t addr, uint16_t *speed);
int SS_get_real_time_curr(uint8_t addr, uint16_t *curr);
/* --- 输入延时 (0~7) --- */
int SS_set_input_delay(uint8_t addr, uint8_t input_idx, uint16_t delay_ms);
int SS_get_input_delay(uint8_t addr, uint8_t input_idx, uint16_t *delay);
/* --- 位置积分 --- */
int SS_set_pos_integral(uint8_t addr, uint16_t val);
int SS_get_pos_integral(uint8_t addr, uint16_t *val);
/* --- 最大值/最小值 --- */
int SS_get_bus_voltage_max(uint8_t addr, uint16_t *val);
int SS_clear_bus_voltage_max(uint8_t addr);
int SS_get_boost_curr_max(uint8_t addr, uint16_t *val);
int SS_clear_boost_curr_max(uint8_t addr);
int SS_get_lag_pulse_max(uint8_t addr, uint16_t *val);
int SS_clear_lag_pulse_max(uint8_t addr);
int SS_get_lead_pulse_max(uint8_t addr, uint16_t *val);
int SS_clear_lead_pulse_max(uint8_t addr);
int SS_get_bus_voltage_min(uint8_t addr, uint16_t *val);
int SS_clear_bus_voltage_min(uint8_t addr);
/* --- 编码器最小分辨率 --- */
int SS_set_encoder_min_res(uint8_t addr, uint16_t val);
int SS_get_encoder_min_res(uint8_t addr, uint16_t *val);

/* ====== 新增便捷函数 ====== */
int SS_get_motor_direction(uint8_t addr, uint16_t *dir);
int SS_set_motor_direction(uint8_t addr, uint16_t dir);
int SS_set_rev_input_level(uint8_t addr, uint16_t val);
int SS_get_soft_neg_limit(uint8_t addr, int32_t *val);
int SS_set_soft_neg_limit(uint8_t addr, int32_t val);
int SS_get_soft_pos_limit(uint8_t addr, int32_t *val);
int SS_set_soft_pos_limit(uint8_t addr, int32_t val);
int SS_get_motor_start_speed(uint8_t addr, uint16_t *speed);
int SS_set_motor_start_speed(uint8_t addr, uint16_t speed);
int SS_get_motor_stop_speed(uint8_t addr, uint16_t *speed);
int SS_set_motor_stop_speed(uint8_t addr, uint16_t speed);
int SS_get_motor_accel_time(uint8_t addr, uint16_t *t);
int SS_set_motor_accel_time(uint8_t addr, uint16_t t);
int SS_get_motor_decel_time(uint8_t addr, uint16_t *t);
int SS_set_motor_decel_time(uint8_t addr, uint16_t t);
int SS_get_motor_target_speed(uint8_t addr, uint16_t *speed);
int SS_set_motor_target_speed(uint8_t addr, uint16_t speed);
int SS_get_hw_limit(uint8_t addr, uint16_t *val);
int SS_set_hw_limit(uint8_t addr, uint16_t val);
int SS_get_enable_home_port(uint8_t addr, uint16_t *val);
int SS_set_enable_home_port(uint8_t addr, uint16_t val);
int SS_get_second_home(uint8_t addr, uint16_t *val);
int SS_set_second_home(uint8_t addr, uint16_t val);
int SS_get_torque_mode(uint8_t addr, uint16_t *val);
int SS_set_torque_mode(uint8_t addr, uint16_t val);
int SS_get_run_mode(uint8_t addr, uint16_t *mode);
int SS_set_run_mode(uint8_t addr, uint16_t mode);
int SS_output_set_on(uint8_t addr, uint16_t mask);
int SS_output_set_off(uint8_t addr, uint16_t mask);
int SS_get_output_status(uint8_t addr, uint16_t *status);
int SS_get_alarm_read(uint8_t addr, uint16_t *status);
int SS_alarm_clear(uint8_t addr);
int SS_set_alarm_out_assign(uint8_t addr, uint16_t val);
int SS_set_run_out_assign(uint8_t addr, uint16_t val);
int SS_set_in_pos_out_assign(uint8_t addr, uint16_t val);
int SS_set_pos_remind_x11(uint8_t addr, int32_t val);
int SS_set_table_size(uint8_t addr, uint16_t size);
int SS_set_table_ptr(uint8_t addr, uint16_t ptr);
int SS_set_table_start(uint8_t addr, uint16_t start_addr);
int SS_get_emergency_stop(uint8_t addr, uint16_t *val);
int SS_set_emergency_stop(uint8_t addr, uint16_t val);
int SS_get_home_done_out(uint8_t addr, uint16_t *val);
int SS_set_home_done_out(uint8_t addr, uint16_t val);
int SS_set_quick_speed_port_x0(uint8_t addr, uint16_t val);
int SS_set_quick_speed_port_x17(uint8_t addr, uint16_t val);
int SS_set_quick_speed_val1(uint8_t addr, uint16_t val);
int SS_set_quick_speed_val2(uint8_t addr, uint16_t val);
int SS_get_trig_run_pulse(uint8_t addr, int32_t *val);
int SS_set_trig_run_pulse(uint8_t addr, int32_t val);
int SS_get_trig_start_run(uint8_t addr, int32_t *val);
int SS_set_trig_start_run(uint8_t addr, int32_t val);
int SS_get_sync_deviation(uint8_t addr, int16_t *val);
int SS_set_sync_deviation(uint8_t addr, int16_t val);
int SS_get_pos_proportion(uint8_t addr, uint16_t *val);
int SS_set_pos_proportion(uint8_t addr, uint16_t val);
int SS_get_speed_proportion(uint8_t addr, uint16_t *val);
int SS_set_speed_proportion(uint8_t addr, uint16_t val);
int SS_get_speed_integral(uint8_t addr, uint16_t *val);
int SS_set_speed_integral(uint8_t addr, uint16_t val);
int SS_set_pos_remind_x17(uint8_t addr, int32_t val);
int SS_cmd_run_stop(uint8_t addr, uint16_t cmd);
int SS_cmd_exec_home(uint8_t addr, uint16_t cmd);
int SS_cmd_jog(uint8_t addr, uint16_t cmd);
int SS_cmd_torque_mode(uint8_t addr, uint16_t cmd);
int SS_cmd_run_duration(uint8_t addr, int32_t val);
int SS_cmd_run_pulse_stop(uint8_t addr, int32_t val);
int SS_cmd_run_abs_stop(uint8_t addr, int32_t val);
int SS_cmd_set_curr_abs(uint8_t addr, int32_t val);
int SS_cmd_offline_ena_rst(uint8_t addr, uint16_t cmd);
int SS_cmd_exec_prog(uint8_t addr, uint16_t cmd);
int SS_cmd_save_power(uint8_t addr, uint16_t cmd);
int SS_cmd_exec_table(uint8_t addr, uint16_t cmd);
int SS_cmd_run_pulse_any(uint8_t addr, int32_t val);
int SS_cmd_run_abs_any(uint8_t addr, int32_t val);

#ifdef __cplusplus
}
#endif

#endif