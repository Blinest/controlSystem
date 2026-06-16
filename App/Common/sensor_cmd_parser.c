/**
 * @file sensor_cmd_parser.c
 * @brief 下位机指令解析库
 * 
 * 负责解析外设（传感器）反馈的指令，更新全局结构体
 * 
 * @date 2026-04-23
 * @author blin
 */

#include "sensor_cmd_parser.h"
#include "Motor/Motor.h"
#include <string.h>
#include "usart.h"

/**
 * @brief 重置解析器状态
 */
static void SensorParser_Reset(SensorContext *sensors) {
    if (!sensors) return;
    memset(&sensors->parser, 0, sizeof(sensors->parser));
    sensors->parser.imu_parser.state = SENSOR_STATE_HEAD;
    sensors->parser_mode = SENSOR_PARSER_MODE_NONE;
}

void SensorParser_Init(SensorParser *parser) {
    if (!parser) return;
    memset(parser, 0, sizeof(SensorParser));
    parser->state = SENSOR_STATE_HEAD;
}

void SensorContext_Init(SensorContext *context) {
    if (!context) return;
    memset(&context->parser, 0, sizeof(context->parser));
    context->parser.imu_parser.state = SENSOR_STATE_HEAD;
    context->parser_mode = SENSOR_PARSER_MODE_NONE;
}

/**
 * @brief 计算校验和（累加所有字节的低8位）
 */
static uint8_t calc_checksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 解析传感器反馈数据并更新结构体，然后触发上报
 */
static void process_IMU_sensor_frame(const uint8_t *buf,
								 uint8_t len,
                                 SensorContext *sensors,
                                 const MotorContext *motors,
                                 const ContinuumRobot *CR) {
    // buf[0] = 地址(SENSOR_ID), buf[1] = 功能码, buf[2] = 数据长度L, buf[3] = 传感器子ID
    if (len < 10) return;  // 最小长度：地址+功能码+L(1)+子ID(1)+x(2)+y(2)+z(2)+校验(1) = 11？实际len不包含校验？这里重新计算
    // 根据协议：帧格式为 [addr][func][L][子ID][x高][x低][y高][y低][z高][z低][cs]
    // buf 中存储了 addr .. data .. cs，在我们的调用中，buf 包含了地址到校验的所有字节，长度 = 1+1+1+1+6+1 = 10？
    uint8_t func = buf[1];
    uint8_t data_len = buf[2];
    uint8_t sensor_id = buf[3];

    if (func == FUNC_SENSOR_FEEDBACK && data_len >= 7 && sensor_id >= 1 && sensor_id <= SENSOR_NUM) {
        int16_t x = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t y = (int16_t)((buf[6] << 8) | buf[7]);
        int16_t z = (int16_t)((buf[8] << 8) | buf[9]);

        // 更新传感器数据
        sensors[sensor_id - 1].global_sensor.imu.pitch = (uint16_t)x;
        sensors[sensor_id - 1].global_sensor.imu.roll = (uint16_t)y;
        sensors[sensor_id - 1].global_sensor.imu.yaw = (uint16_t)z;

        // 触发系统状态上报（需传入电机、传感器、和系统状态）
        // 这里系统状态暂时使用 CR->state，若需其他状态可调整
        cmd_packer_send_status_frame(motors, sensors, CR, CR->state);
    }
}

void SensorParser_IMU_Feed(uint8_t byte,
                       SensorContext *sensors,
                       const MotorContext *motors,
                       const ContinuumRobot *CR) {
    if (!sensors) return;

    switch (sensors->parser.imu_parser.state) {
        case SENSOR_STATE_HEAD:
            if (byte == SENSOR_ID) {
                sensors->parser_mode = SENSOR_PARSER_MODE_IMU;
                sensors->parser.imu_parser.buf[0] = byte;
                sensors->parser.imu_parser.idx = 1;
                sensors->parser.imu_parser.state = SENSOR_STATE_FUNC;
            }
            break;

        case SENSOR_STATE_FUNC:
            sensors->parser.imu_parser.buf[1] = byte;
            sensors->parser.imu_parser.idx = 2;
            if (byte == FUNC_SENSOR_FEEDBACK) {
                sensors->parser.imu_parser.state = SENSOR_STATE_LEN;
            } else {
                SensorParser_Reset(sensors);
            }
            break;

        case SENSOR_STATE_LEN:
            sensors->parser.imu_parser.data_len = byte;
            sensors->parser.imu_parser.buf[2] = byte;
            // 数据长度不能超过缓冲区剩余空间（总容量 - 帧头(1) - 功能码(1) - L(1) - 校验(1)）
            if (sensors->parser.imu_parser.data_len > (sizeof(sensors->parser.imu_parser.buf) - 4) || sensors->parser.imu_parser.data_len == 0) {
                SensorParser_Reset(sensors);
                break;
            }
            sensors->parser.imu_parser.idx = 3;
            sensors->parser.imu_parser.state = SENSOR_STATE_DATA;
            break;

        case SENSOR_STATE_DATA:
            if (sensors->parser.imu_parser.idx < sizeof(sensors->parser.imu_parser.buf)) {
                sensors->parser.imu_parser.buf[sensors->parser.imu_parser.idx++] = byte;
            }
            // 数据长度+起始偏移3（地址、功能码、L）等于当前索引时，数据接收完毕
            if (sensors->parser.imu_parser.idx >= (uint8_t)(3 + sensors->parser.imu_parser.data_len)) {
                sensors->parser.imu_parser.state = SENSOR_STATE_CHECK;
            }
            break;

        case SENSOR_STATE_CHECK:
	        {
		        // 计算已接收字节的校验和（不含本校验字节）
        		uint8_t sum = calc_checksum(sensors->parser.imu_parser.buf, sensors->parser.imu_parser.idx);
        		if (sum == byte) {
        			// 校验通过，处理帧（buf包含地址到数据的所有字节，总长度为 sensors->parser.imu_parser.idx）
        			process_IMU_sensor_frame(sensors->parser.imu_parser.buf, sensors->parser.imu_parser.idx, sensors, motors, CR);
        		}
        		SensorParser_Reset(sensors);
        		break;
	        }
        default:
            SensorParser_Reset(sensors);
            break;
    }
}

// ==================== CMCU-06 压力传感器解析 ====================
// 协议：Modbus RTU
// 读保持寄存器响应帧格式：
//   [addr] [0x03] [data_len] [Data0] [Data1] [Data2] [Data3] [CRC_L] [CRC_H]
// 其中 data_len 通常为 4（因为读取 2 个寄存器，4 字节）
// 数据部分为 32 位有符号整数（大端序，高位在前）

// ------------------------------------------------------------------
// CRC16-Modbus 计算函数
static uint16_t CRC16_Modbus(const uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 重置解析器
static void CMCU_Parser_Reset(SensorContext *sensors) {
    CMCU_Parser *parser = &sensors->parser.press_parser;
    parser->state = CMCU_STATE_HEAD;
    parser->idx = 0;
    parser->data_len = 0;
    parser->crc_calc = 0;
    parser->crc_recv = 0;
    memset(parser->buf, 0, sizeof(parser->buf));
    sensors->parser_mode = SENSOR_PARSER_MODE_NONE;
}

// 处理完整接收到的压力数据帧
static void process_CMCU_frame(const uint8_t *buf, uint8_t len,
                               SensorContext* sensors,
                               const MotorContext *motors,
                               const ContinuumRobot * CR) {
    // 最小长度：addr+func+len+至少1字节数据+2字节CRC = 3+1+2=6，实际 len 包含所有
    if (len < 6) return;

    uint8_t addr = buf[0];
    uint8_t func = buf[1];
    uint8_t data_len = buf[2];

    // 检查功能码（正常响应）
    if (func == CMCU_FUNC_READ && data_len >= 4) {
        // 提取 4 字节数据（大端序）
        int32_t raw = (int32_t)((buf[5] << 24) | (buf[6] << 16) | (buf[3] << 8) | buf[4]);
        sensors->global_sensor.press_sensor.id = addr;
    	sensors->global_sensor.press_sensor.raw_val = raw;
        // 假设传感器量程为 ±10kg，原始值范围 -32768~32767（根据实际确定），此处示例直接显示原始值
        sensors->global_sensor.press_sensor.val = (float)raw /10.0f;   // 示例转换因子

        // 可在此触发上报、存储等操作
        cmd_packer_send_status_frame(motors, sensors, CR, CR->state);
    }
    // 处理写入响应（回显）
    else if (func == CMCU_FUNC_WRITE && data_len == 4) {
        // 写入操作成功，可选回调
        // 例如：写入保护、去皮置零等成功标志
    }
    else if ((func & 0x80)) {
        // 异常响应：功能码最高位为 1，错误码在数据字节中
        uint8_t exception_code = buf[2];
        // 处理错误
    }
}

// 主解析函数（字节流输入）
void CMCU_Parser_Feed(uint8_t byte,
                      SensorContext *sensors,
                      const MotorContext *motors,
                      ContinuumRobot* CR) {
    if (!sensors) return;
    switch (sensors->parser.press_parser.state) {
        case CMCU_STATE_HEAD:
            // 仅在匹配压力传感器地址时启动解析器，避免电机地址误判
            if (byte == CMCU_ADDR_DEFAULT) {
                sensors->parser_mode = SENSOR_PARSER_MODE_CMCU;
                sensors->parser.press_parser.buf[0] = byte;
                sensors->parser.press_parser.idx = 1;
                sensors->parser.press_parser.state = CMCU_STATE_FUNC;
            }
            break;

        case CMCU_STATE_FUNC:
            sensors->parser.press_parser.buf[1] = byte;
            sensors->parser.press_parser.idx = 2;

            // 仅处理读或写响应的功能码（正常响应 0x03/0x06，异常响应码最高位为1）
            if (byte == CMCU_FUNC_READ || byte == CMCU_FUNC_WRITE || (byte & 0x80)) {
                sensors->parser.press_parser.state = CMCU_STATE_LEN;

            } else {
                CMCU_Parser_Reset(sensors);

            }
            break;

        case CMCU_STATE_LEN:
            sensors->parser.press_parser.data_len = byte;
            sensors->parser.press_parser.buf[2] = byte;

            // 数据长度不能超过缓冲区剩余空间（最多接收有效数据后还有2字节CRC）
            if (sensors->parser.press_parser.data_len > (sizeof(sensors->parser.press_parser.buf) - 4) || sensors->parser.press_parser.data_len == 0) {
                CMCU_Parser_Reset(sensors);
                break;
            }

            sensors->parser.press_parser.idx = 3;
            sensors->parser.press_parser.state = CMCU_STATE_DATA;
            break;

        case CMCU_STATE_DATA:
            if (sensors->parser.press_parser.idx < sizeof(sensors->parser.press_parser.buf)) {
                sensors->parser.press_parser.buf[sensors->parser.press_parser.idx++] = byte;
            }
            // 数据接收完成（数据域长度所指定的字节数）
            if (sensors->parser.press_parser.idx >= (uint8_t)(3 + sensors->parser.press_parser.data_len)) {
                sensors->parser.press_parser.state = CMCU_STATE_CRC1;
            }
            break;

        case CMCU_STATE_CRC1:
            sensors->parser.press_parser.crc_recv = byte;
            sensors->parser.press_parser.state = CMCU_STATE_CRC2;
            break;

        case CMCU_STATE_CRC2:
            sensors->parser.press_parser.crc_recv |= (uint16_t)byte << 8;
            // 计算已接收字节的 CRC（地址、功能码、长度、数据域）
            sensors->parser.press_parser.crc_calc = CRC16_Modbus(sensors->parser.press_parser.buf, 3 + sensors->parser.press_parser.data_len);
            if (sensors->parser.press_parser.crc_calc == sensors->parser.press_parser.crc_recv) {
                // 校验通过，处理完整帧（总字节数 = 3 + data_len + 2）
                process_CMCU_frame(sensors->parser.press_parser.buf, 3 + sensors->parser.press_parser.data_len + 2, sensors, motors, CR);
            }
            CMCU_Parser_Reset(sensors);
            break;

        default:
            CMCU_Parser_Reset(sensors);
            break;
    }
}