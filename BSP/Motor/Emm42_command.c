/**
 * Emm42_V5.0 指令解析器 — Demo (Motor结构体版)
 * 编译: gcc -o demo main.c -Wall -Wextra -std=c11
 */
#include <stdio.h>
#include "Emm42_command.h"

/* ================================================================
 *  解析: 收到的帧 → Motor
 *  自动区分命令帧(带magic)和回复帧(无magic)
 * ================================================================ */

emm42_result_t emm42_parse(GlobalMotor *m, const uint8_t *buf, int len, bool verify_cs)
{
    if (!m || !buf || len < 3) return EMM42_ERR_TOO_SHORT;
    if (verify_cs && buf[len - 1] != 0x6B) return EMM42_ERR_CHECKSUM;

    uint8_t addr = buf[0];
    uint8_t fc   = buf[1];
    const uint8_t *p = &buf[2];
    int n = verify_cs ? (len - 3) : (len - 2);

    m->id = addr;

    switch (fc) {

    /* 回复帧: [addr F3 status cs] → n=1, 只更新状态
     * 命令帧: [addr F3 AB en sync cs] → n=3 */
    case EMM42_FC_MOTOR_ENABLE:
        if (n >= 3) m->state = (p[1] != 0);
        break;

    case EMM42_FC_SPEED_MODE:
        if (n < 5) return EMM42_ERR_LENGTH;
        m->target_vel = emm42_be16(&p[1]);
        m->target_acc = p[3];
        break;

    case EMM42_FC_POSITION_MODE:
        if (n < 10) return EMM42_ERR_LENGTH;
        m->target_vel = emm42_be16(&p[1]);
        m->target_acc = p[3];
        m->target_pos = (uint16_t)(emm42_be32(&p[4]) & 0xFFFF);
        break;

    case EMM42_FC_IMMEDIATE_STOP:
    case EMM42_FC_SYNC_MOTION:
    case EMM42_FC_SET_HOME:
    case EMM42_FC_ABORT_HOMING:
    case EMM42_FC_TRIGGER_CALIB:
    case EMM42_FC_CLEAR_POSITION:
    case EMM42_FC_RELEASE_STALL:
    case EMM42_FC_FACTORY_RESET:
        break;

    case EMM42_FC_TRIGGER_HOMING:
        if (n >= 2) { /* cmd: [mode sync] */ }
        break;

    /* 回复: [addr 22 homing_params cs] → 长帧, 跳过
     * 命令: [addr 22 cs] → n=0 (读取) */
    case EMM42_FC_READ_HOMING_PARAM:
    case EMM42_FC_WRITE_HOMING_PARAM:
    case EMM42_FC_READ_HOMING_STATUS:
        break;

    /* 回复: [addr 1F fw hw cs] */
    case EMM42_FC_READ_FW_VERSION:
        break;

    case EMM42_FC_READ_PHASE_RL:
    case EMM42_FC_READ_POS_PID:
    case EMM42_FC_READ_BUS_VOLTAGE:
    case EMM42_FC_READ_PHASE_CURRENT:
    case EMM42_FC_READ_ENCODER:
    case EMM42_FC_READ_INPUT_PULSES:
    case EMM42_FC_READ_POS_ERROR:
    case EMM42_FC_READ_DRV_CONFIG:
    case EMM42_FC_READ_SYS_STATUS:
        break;

    /* 回复: [addr 33 sign pos(4) cs] → n=5
     * 命令: [addr 33 cs] → n=0 */
    case EMM42_FC_READ_TARGET_POS:
    case EMM42_FC_READ_REALTIME_TPOS:
    case EMM42_FC_READ_REALTIME_POS:
        if (n >= 5) {
            int32_t raw = (int32_t)emm42_be32(&p[1]);
            m->current_pos = (uint16_t)(raw & 0xFFFF);
        }
        break;

    /* 回复: [addr 35 sign speed(2) cs] → n=3
     * 命令: [addr 35 cs] → n=0 */
    case EMM42_FC_READ_REALTIME_SPD:
        if (n >= 3) m->current_vel = emm42_be16(&p[1]);
        break;

    /* 回复: [addr 3A flags cs] → n=1
     * 命令: [addr 3A cs] → n=0 */
    case EMM42_FC_READ_MOTOR_STATUS:
        if (n >= 1) m->state = (p[0] & 0x01) != 0;
        break;

    /* 回复: [addr 84 status cs] → n=1
     * 命令: [addr 84 8A save val cs] → n=3 */
    case EMM42_FC_WRITE_MICROSTEP:
        if (n >= 3) m->stepper_motor.xifen = p[2];
        break;

    /* 回复: [addr AE status cs] → n=1
     * 命令: [addr AE 4B save id cs] → n=3 */
    case EMM42_FC_WRITE_ID_ADDR:
        if (n >= 3) m->id = p[2];
        break;

    case EMM42_FC_WRITE_LOOP_MODE:
    case EMM42_FC_WRITE_OPEN_CURRENT:
    case EMM42_FC_WRITE_DRV_CONFIG:
    case EMM42_FC_WRITE_POS_PID:
    case EMM42_FC_STORE_SPEED_PARAM:
    case EMM42_FC_WRITE_VEL_SCALE:
        break;

    default:
        return EMM42_ERR_UNKNOWN_FC;
    }

    return EMM42_OK;
}


/* ================================================================
 *  构建: Motor → 发送帧写入 Motor.cmd
 * ================================================================ */

emm42_result_t emm42_build(GlobalMotor *m, uint8_t fc)
{
    if (!m) return EMM42_ERR_TOO_SHORT;
    uint8_t *c = m->cmd;
    uint8_t addr = (uint8_t)m->id;
	// 进制转换
	uint32_t pos = (uint16_t) m->target_pos;
	uint16_t vel = (uint16_t) m->target_vel;
	uint8_t acc = (uint16_t) m->target_acc;
    switch (fc) {

    case EMM42_FC_MOTOR_ENABLE:
        c[0]=addr; c[1]=0xF3; c[2]=0xAB;
        c[3]=m->state?0x01:0x00; c[4]=0x00; c[5]=0x6B;
        m->size=6; break;

    case EMM42_FC_SPEED_MODE:
        c[0]=addr; c[1]=0xF6;
        c[2]=EMM42_DIR_CW;
        c[3]=(uint8_t)(vel>>8);
        c[4]=(uint8_t)(vel&0xFF);
        c[5]=(uint8_t)acc; c[6]=0x00; c[7]=0x6B;
        m->size=8; break;

    case EMM42_FC_POSITION_MODE:
        c[0]=addr; c[1]=0xFD; c[2]=EMM42_DIR_CW;
        c[3]=(uint8_t)(vel >> 8);
        c[4]=(uint8_t)(vel & 0xFF);
        c[5]=acc;
        c[6]=(uint8_t)((pos>>24)&0xFF);
        c[7]=(uint8_t)((pos>>16)&0xFF);
        c[8]=(uint8_t)((pos>>8)&0xFF);
        c[9]=(uint8_t)(pos&0xFF);
        c[10]=EMM42_POS_REL; c[11]=0x00; c[12]=0x6B;
        m->size=13; break;

    case EMM42_FC_IMMEDIATE_STOP:
        c[0]=addr; c[1]=0xFE; c[2]=0x98; c[3]=0x00; c[4]=0x6B;
        m->size=5; break;

    case EMM42_FC_SYNC_MOTION:
        c[0]=addr; c[1]=0xFF; c[2]=0x66; c[3]=0x6B;
        m->size=4; break;

    case EMM42_FC_SET_HOME:
        c[0]=addr; c[1]=0x93; c[2]=0x88; c[3]=0x01; c[4]=0x6B;
        m->size=5; break;

    case EMM42_FC_TRIGGER_HOMING:
        c[0]=addr; c[1]=0x9A; c[2]=0x00; c[3]=0x00; c[4]=0x6B;
        m->size=5; break;

    case EMM42_FC_ABORT_HOMING:
        c[0]=addr; c[1]=0x9C; c[2]=0x48; c[3]=0x6B;
        m->size=4; break;

    case EMM42_FC_TRIGGER_CALIB:
        c[0]=addr; c[1]=0x06; c[2]=0x45; c[3]=0x6B;
        m->size=4; break;

    case EMM42_FC_CLEAR_POSITION:
        c[0]=addr; c[1]=0x0A; c[2]=0x6D; c[3]=0x6B;
        m->size=4; break;

    case EMM42_FC_RELEASE_STALL:
        c[0]=addr; c[1]=0x0E; c[2]=0x52; c[3]=0x6B;
        m->size=4; break;

    case EMM42_FC_FACTORY_RESET:
        c[0]=addr; c[1]=0x0F; c[2]=0x5F; c[3]=0x6B;
        m->size=4; break;

    /* 读取: 3字节 [addr fc cs] */
    case EMM42_FC_READ_FW_VERSION:
    case EMM42_FC_READ_PHASE_RL:
    case EMM42_FC_READ_POS_PID:
    case EMM42_FC_READ_BUS_VOLTAGE:
    case EMM42_FC_READ_PHASE_CURRENT:
    case EMM42_FC_READ_ENCODER:
    case EMM42_FC_READ_INPUT_PULSES:
    case EMM42_FC_READ_TARGET_POS:
    case EMM42_FC_READ_REALTIME_TPOS:
    case EMM42_FC_READ_REALTIME_SPD:
    case EMM42_FC_READ_REALTIME_POS:
    case EMM42_FC_READ_POS_ERROR:
    case EMM42_FC_READ_MOTOR_STATUS:
    case EMM42_FC_READ_HOMING_STATUS:
        c[0]=addr; c[1]=fc; c[2]=0x6B;
        m->size=3; break;

    /* 读取(带magic): 4字节 */
    case EMM42_FC_READ_DRV_CONFIG:
        c[0]=addr; c[1]=0x42; c[2]=0x6C; c[3]=0x6B;
        m->size=4; break;
    case EMM42_FC_READ_SYS_STATUS:
        c[0]=addr; c[1]=0x43; c[2]=0x7A; c[3]=0x6B;
        m->size=4; break;
    case EMM42_FC_READ_HOMING_PARAM:
        c[0]=addr; c[1]=0x22; c[2]=0x6B;
        m->size=3; break;

    case EMM42_FC_WRITE_MICROSTEP:
        c[0]=addr; c[1]=0x84; c[2]=0x8A;
        c[3]=0x01; c[4]=m->stepper_motor.xifen; c[5]=0x6B;
        m->size=6; break;

    case EMM42_FC_WRITE_ID_ADDR:
        c[0]=addr; c[1]=0xAE; c[2]=0x4B;
        c[3]=0x01; c[4]=(uint8_t)m->id; c[5]=0x6B;
        m->size=6; break;

    case EMM42_FC_WRITE_LOOP_MODE:
        c[0]=addr; c[1]=0x46; c[2]=0x69;
        c[3]=0x01; c[4]=0x02; c[5]=0x6B;
        m->size=6; break;

    default:
        return EMM42_ERR_UNKNOWN_FC;
    }

    return EMM42_OK;
}


const char* emm42_cmd_name(uint8_t fc) {
    switch (fc) {
        case EMM42_FC_MOTOR_ENABLE:       return "电机使能控制";
        case EMM42_FC_SPEED_MODE:         return "速度模式控制";
        case EMM42_FC_POSITION_MODE:      return "位置模式控制";
        case EMM42_FC_IMMEDIATE_STOP:     return "立即停止";
        case EMM42_FC_SYNC_MOTION:        return "多机同步运动";
        case EMM42_FC_SET_HOME:           return "设置回零零点";
        case EMM42_FC_TRIGGER_HOMING:     return "触发回零";
        case EMM42_FC_ABORT_HOMING:       return "中断回零";
        case EMM42_FC_READ_HOMING_PARAM:  return "读回零参数";
        case EMM42_FC_WRITE_HOMING_PARAM: return "写回零参数";
        case EMM42_FC_READ_HOMING_STATUS: return "读回零状态";
        case EMM42_FC_TRIGGER_CALIB:      return "编码器校准";
        case EMM42_FC_CLEAR_POSITION:     return "位置清零";
        case EMM42_FC_RELEASE_STALL:      return "解除堵转";
        case EMM42_FC_FACTORY_RESET:      return "恢复出厂";
        case EMM42_FC_READ_FW_VERSION:    return "读固件版本";
        case EMM42_FC_READ_PHASE_RL:      return "读相电阻电感";
        case EMM42_FC_READ_POS_PID:       return "读位置环PID";
        case EMM42_FC_READ_BUS_VOLTAGE:   return "读总线电压";
        case EMM42_FC_READ_PHASE_CURRENT: return "读相电流";
        case EMM42_FC_READ_ENCODER:       return "读编码器值";
        case EMM42_FC_READ_INPUT_PULSES:  return "读输入脉冲";
        case EMM42_FC_READ_TARGET_POS:    return "读目标位置";
        case EMM42_FC_READ_REALTIME_TPOS: return "读实时目标";
        case EMM42_FC_READ_REALTIME_SPD:  return "读实时转速";
        case EMM42_FC_READ_REALTIME_POS:  return "读实时位置";
        case EMM42_FC_READ_POS_ERROR:     return "读位置误差";
        case EMM42_FC_READ_MOTOR_STATUS:  return "读电机状态";
        case EMM42_FC_READ_DRV_CONFIG:    return "读驱动配置";
        case EMM42_FC_READ_SYS_STATUS:    return "读系统状态";
        case EMM42_FC_WRITE_MICROSTEP:    return "修改细分";
        case EMM42_FC_WRITE_ID_ADDR:      return "修改地址";
        case EMM42_FC_WRITE_LOOP_MODE:    return "切换开闭环";
        case EMM42_FC_WRITE_OPEN_CURRENT: return "修改开环电流";
        case EMM42_FC_WRITE_DRV_CONFIG:   return "修改驱动配置";
        case EMM42_FC_WRITE_POS_PID:      return "修改位置环PID";
        case EMM42_FC_STORE_SPEED_PARAM:  return "存储速度参数";
        case EMM42_FC_WRITE_VEL_SCALE:    return "修改速度缩放";
        default:                          return "未知";
    }
}

static void hex(const uint8_t *d, int n) {
    printf("  帧: ");
    for (int i = 0; i < n; i++) printf("%02X ", d[i]);
    printf("\n");
}

void demo_parse(const char *desc, GlobalMotor *m, const uint8_t *buf, int len) {
    printf("\n── %s ──\n", desc);
    hex(buf, len);

    emm42_result_t ret = emm42_parse(m, buf, len, true);
    if (ret != EMM42_OK) { printf("  ✗ 错误: %d\n", ret); return; }

    printf("  ✓ %s → id=%d state=%s\n",
           emm42_cmd_name(buf[1]), m->id, m->state ? "ON" : "OFF");
}

void demo_build(const char *desc, GlobalMotor *m, uint8_t fc) {
    emm42_result_t ret = emm42_build(m, fc);
    if (ret != EMM42_OK) { printf("  ✗ 构建失败: %d\n", ret); return; }
    printf("\n── 构建: %s ──\n", desc);
    hex(m->cmd, m->size);
}