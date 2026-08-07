/**********************************************************
***	编写作者：blinest
***	qq：1071378062
*** date: 2026.07.24
* brief AQMD2405NS-MT直流电机驱动器通信实现骨架
* 功能包括：
* 向下位机发送控制指令
**********************************************************/

#include "AQMD245NS.h"
#include "usart.h"   /* uart1_bus_lock/unlock, platform_uart_send/recv, platform_uart_recv */

/* ==================== CRC-16/Modbus 双查表法 ==================== */
static unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

static unsigned char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

uint16_t aqm_crc16(const uint8_t *data, uint16_t len)
{
    unsigned char uchCRCHi = 0xFF; /* CRC 高字节初始化 */
    unsigned char uchCRCLo = 0xFF; /* CRC 低字节初始化 */
    unsigned int uIndex;

    while (len--)
    {
        uIndex = uchCRCLo ^ *data++; /* 计算 CRC */
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }

    return (unsigned short)((uchCRCHi << 8) | uchCRCLo);
}

/* ==================== 内部辅助 ==================== */
static void append_crc(aqm_frame_t *frame)
{
    uint16_t crc = aqm_crc16(frame->buf, frame->len);
    frame->buf[frame->len++] = (uint8_t)(crc & 0xFF);
    frame->buf[frame->len++] = (uint8_t)(crc >> 8);
}

static bool check_crc(const aqm_frame_t *frame)
{
    if (frame->len < 2) return false;
    uint16_t calc = aqm_crc16(frame->buf, frame->len - 2);
    uint16_t recv = (uint16_t)frame->buf[frame->len - 2] |
                    ((uint16_t)frame->buf[frame->len - 1] << 8);
    return calc == recv;
}

/* ==================== 帧构建 ==================== */

uint16_t aqm_build_read(aqm_frame_t *frame, uint8_t slave,
                        uint16_t reg, uint16_t count)
{
    frame->len = 0;
    frame->buf[frame->len++] = slave;
    frame->buf[frame->len++] = MB_FUNC_READ_HOLDING;
    frame->buf[frame->len++] = (uint8_t)(reg >> 8);
    frame->buf[frame->len++] = (uint8_t)(reg & 0xFF);
    frame->buf[frame->len++] = (uint8_t)(count >> 8);
    frame->buf[frame->len++] = (uint8_t)(count & 0xFF);
    append_crc(frame);
    return frame->len;
}

uint16_t aqm_build_write(aqm_frame_t *frame, uint8_t slave,
                         uint16_t reg, uint16_t value)
{
    frame->len = 0;
    frame->buf[frame->len++] = slave;
    frame->buf[frame->len++] = MB_FUNC_WRITE_SINGLE;
    frame->buf[frame->len++] = (uint8_t)(reg >> 8);
    frame->buf[frame->len++] = (uint8_t)(reg & 0xFF);
    frame->buf[frame->len++] = (uint8_t)(value >> 8);
    frame->buf[frame->len++] = (uint8_t)(value & 0xFF);
    append_crc(frame);
    return frame->len;
}

uint16_t aqm_build_write_multi(aqm_frame_t *frame, uint8_t slave,
                               uint16_t reg, uint16_t count,
                               const uint8_t *data)
{
    frame->len = 0;
    frame->buf[frame->len++] = slave;
    frame->buf[frame->len++] = MB_FUNC_WRITE_MULTI;
    frame->buf[frame->len++] = (uint8_t)(reg >> 8);
    frame->buf[frame->len++] = (uint8_t)(reg & 0xFF);
    frame->buf[frame->len++] = (uint8_t)(count >> 8);
    frame->buf[frame->len++] = (uint8_t)(count & 0xFF);
    frame->buf[frame->len++] = (uint8_t)(count * 2U);
    for (uint16_t i = 0; i < (uint16_t)(count * 2U); i++) {
        frame->buf[frame->len++] = data[i];
    }
    append_crc(frame);
    return frame->len;
}

/* ==================== 帧解析 ==================== */

bool aqm_parse_read(const aqm_frame_t *frame, uint16_t *values, uint16_t count)
{
    if (!frame || !values || frame->len < 5) return false;
    if (!check_crc(frame)) return false;

    uint8_t func = frame->buf[1];
    if (func == (MB_FUNC_READ_HOLDING | 0x80)) return false; /* 异常 */
    if (func != MB_FUNC_READ_HOLDING) return false;

    uint8_t byte_cnt = frame->buf[2];
    if (byte_cnt != count * 2) return false;
    if (frame->len < (uint16_t)(3 + byte_cnt + 2)) return false;

    for (uint16_t i = 0; i < count; i++) {
        values[i] = ((uint16_t)frame->buf[3 + i * 2] << 8) |
                     (uint16_t)frame->buf[4 + i * 2];
    }
    return true;
}

bool aqm_parse_write(const aqm_frame_t *frame, uint16_t reg, uint16_t value)
{
    if (!frame || frame->len < 8) return false;
    if (!check_crc(frame)) return false;

    uint8_t func = frame->buf[1];
    if (func == (MB_FUNC_WRITE_SINGLE | 0x80)) return false;
    if (func != MB_FUNC_WRITE_SINGLE) return false;

    uint16_t echo_reg   = ((uint16_t)frame->buf[2] << 8) | frame->buf[3];
    uint16_t echo_value = ((uint16_t)frame->buf[4] << 8) | frame->buf[5];

    return (echo_reg == reg && echo_value == value);
}

bool aqm_parse_position_feedback(const aqm_frame_t *frame,
                                 int32_t *pulse,
                                 float *position_mm)
{
    if (!frame || !pulse || !position_mm || frame->len < 9) return false;
    if (!check_crc(frame)) return false;                 /* 接收帧校验 */
    if (frame->buf[1] == (MB_FUNC_READ_HOLDING | 0x80)) return false;
    if (frame->buf[1] != MB_FUNC_READ_HOLDING) return false;
    if (frame->buf[2] != 4U) return false;

    /* 4 字节脉冲为高位在前(大端)，如 FF FF FF F8 -> 0xFFFFFFF8 = -8
     * 与 aqm_parse_read 的 16 位寄存器解析习惯一致。 */
    uint32_t value = ((uint32_t)frame->buf[3] << 24) |
                    ((uint32_t)frame->buf[4] << 16) |
                    ((uint32_t)frame->buf[5] << 8)  |
                    ((uint32_t)frame->buf[6]);

    *pulse = (int32_t)value;
    *position_mm = (float)(*pulse) / AQM_POSITION_PULSE_PER_MM;
    return true;
}
/* ==================== 电机控制 API ==================== */

uint16_t aqm_set_speed(aqm_frame_t *frame, uint8_t slave, int16_t duty)
{
	uint16_t val;

	if (duty >= 0) {
		/* 正转: 直接写入 duty (0 ~ 1000) */
		val = (uint16_t)duty;
	} else {
		/* 反转: 写入 16 位有符号补码，如 -500 → 0xFE0C */
		val = (uint16_t)duty;
	}

	return aqm_build_write(frame, slave, AQM_REG_SET_SPEED, val);
}

uint16_t aqm_set_position(aqm_frame_t *frame, uint8_t slave,
                          uint16_t speed, uint16_t mode,
                          int32_t pulse)
{
    uint8_t data[8];

    data[0] = (uint8_t)(speed >> 8);
    data[1] = (uint8_t)(speed & 0xFF);
    data[2] = (uint8_t)(mode & 0xFF);
    data[3] = (uint8_t)(mode >> 8);
    data[4] = (uint8_t)(pulse >> 24);
    data[5] = (uint8_t)(pulse >> 16);
    data[6] = (uint8_t)(pulse >> 8);
    data[7] = (uint8_t)(pulse & 0xFF);

    return aqm_build_write_multi(frame, slave, AQM_REG_POSITION_CTRL, 4, data);
}

uint16_t aqm_read_current_pulse(aqm_frame_t *frame, uint8_t slave)
{
    /* 直接由 aqm_build_read 组帧，末尾由 append_crc 按标准
     * CRC-16/Modbus 自动计算校验字节。此前对 slave=0x03 硬编码
     * 0x44 0x36 会覆盖掉正确 CRC，导致从机校验失败而拒不回包。 */
    return aqm_build_read(frame, slave, AQM_REG_CURRENT_PULSE, 2);
}

int aqm_get_current_position(uint8_t slave, int32_t *pulse, float *position_mm)
{
    aqm_frame_t tx;
    aqm_frame_t rx;
    int ret;

    uart1_bus_lock();   /* 保护「发请求→读回复」整条事务，防止 CmdCtrl 写入插入 */

    /* 读取前清掉 ring 里积压的残留字节(如下发电机写命令时从机未消费的回显、
     * 其他设备主动上报数据)，否则这些脏数据占位会让按定长取的帧错位、CRC 失败。 */
    uart1_drain_stale_bytes();

    aqm_read_current_pulse(&tx, slave);
    if (platform_uart_send(tx.buf, tx.len) != 0)
    {
        ret = -1;
        goto out;
    }

    rx.len = 9;
    if (platform_uart_recv(rx.buf, rx.len, 50) != 0)
    {
        ret = -2;
        goto out;
    }

    if (!aqm_parse_position_feedback(&rx, pulse, position_mm))
    {
        ret = -3;
        goto out;
    }
    ret = 0;

out:
    uart1_bus_unlock();
    return ret;
}

uint16_t aqm_read_pwm(aqm_frame_t *frame, uint8_t slave)
{
	return aqm_build_read(frame, slave, AQM_REG_REAL_PWM, 1);
}

uint16_t aqm_read_error(aqm_frame_t *frame, uint8_t slave)
{
	return aqm_build_read(frame, slave, AQM_REG_ERROR_STATUS, 1);
}

uint16_t aqm_read_dev_id(aqm_frame_t *frame, uint8_t slave)
{
	return aqm_build_read(frame, slave, AQM_REG_DEVICE_ID, 1);
}