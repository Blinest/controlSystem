/**
 * @file pc_cmd_parser.c
 * @brief 上位机指令解析库
 * 
 * 负责解析上位机发送的指令，更新全局结构体，并分发给下位机
 * 
 * @date 2026-03-30
 * @author Psyduck
 */

#include "pc_cmd_parser.h"
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "CR/kinematic.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"
#include <stdio.h>



// 引用外部队列
//extern osMessageQueueId_t CmdCtrlQueueHandle;

/* 帧头定义 */
#define FRAME_HEAD_MOTOR      0xAA    /* 电机指令帧头 */
#define FRAME_HEAD_SENSOR     0xBB    /* 传感器指令帧头 */

/* 电机功能码定义 */
typedef enum
{
 FUNC_MOTOR_ENABLE     = 0x01,   /* 电机使能 */
 FUNC_MOTOR_STOP       = 0x02,   /* 多电机停止指令 */
 FUNC_MOTOR_SINGLE     = 0x03,   /* 单电机控制：addr + direction + distance */
 FUNC_MOTOR_SYNC       = 0x04,   /* 多电机同步：count + start_addr + distances... */
 FUNC_MOTOR_KINEMATIC  = 0x05,   /* 基于运动学的协同控制（无数据段） */
 FUNC_MOTOR_CUSTOM     = 0x06,   /* 自定义多电机控制：count + [addr, direction, distance]... */
} MotorFuncCode_t;

/* 传感器功能码定义 */
typedef enum
{
 FUNC_SENSOR_INIT      = 0x00,   /* 传感器初始化（无数据段） */
 FUNC_SENSOR_SINGLE_RD = 0x01,   /* 单传感器数据读取（需要传感器ID） */
 FUNC_SENSOR_MULTI_RD  = 0x02,   /* 多传感器批量读取（无数据段） */
 FUNC_SENSOR_SELF_TEST = 0x03,   /* 传感器自检（需要传感器ID） */
} SensorFuncCode_t;

/* 帧解析状态机 */
typedef enum {
 CMD_STATE_HEAD = 0,   /* 等待帧头 0xAA / 0xBB */
 CMD_STATE_FUNC,       /* 已收到帧头，等待功能码 */
 CMD_STATE_LEN,        /* 已收到功能码，等待数据长度 L */
 CMD_STATE_DATA,       /* 已知 L，接收 L 字节数据 */
 CMD_STATE_CHECK       /* 等待校验和字节 */
} CmdParseState_t;

/* 解析缓冲区与状态 */
#define CTRL_BUF_SIZE  32
static uint8_t s_ctrlBuf[CTRL_BUF_SIZE];
static uint8_t s_ctrlLen;
static uint8_t s_ctrlIdx;
static CmdParseState_t s_ctrlState = CMD_STATE_HEAD;

/* 连接状态 */
extern bool is_connected;

/**
 * @brief 重置控制指令解析状态
 */
static void pc_cmd_parser_reset(void) {
    s_ctrlState = CMD_STATE_HEAD;
    s_ctrlLen = 0;
    s_ctrlIdx = 0;
}

/**
 * @brief 解析并执行控制指令
 */
static void pc_cmd_parse_and_execute(void)
{
    uint8_t head     = s_ctrlBuf[0];
    uint8_t func     = s_ctrlBuf[1];
    uint8_t data_len = s_ctrlLen;
    
    if (data_len > 0 && (uint8_t)(3 + data_len) > CTRL_BUF_SIZE)
        return;
        
    switch (head)
    {
        case FRAME_HEAD_MOTOR:
            switch (func)
            {
                case FUNC_MOTOR_SINGLE:
                    // 单电机控制: 地址 + 方向 + 距离
                    if (data_len >= 3) {
                        uint8_t addr = s_ctrlBuf[3];      // 电机地址
                        uint8_t direction = s_ctrlBuf[4]; // 方向 (0:负方向, 1:正方向)
                        uint16_t distance = (s_ctrlBuf[5] << 8) | s_ctrlBuf[6]; // 距离
                        // 测试用
                        printf("[CMD] 单电机控制: addr=%d, dir=%d, dist=%d\n", 
                               addr, direction, distance);
                        
                        // 调用单电机控制函数
                        motor_single_control(addr, direction, distance);
                    }
                    break;
                    
                case FUNC_MOTOR_SYNC:
                    // 多电机同步控制: 数量 + 起始地址 + 距离数组
                    if (data_len >= 2) {
                        uint8_t count = s_ctrlBuf[3];      // 电机数量
                        uint8_t start_addr = s_ctrlBuf[4]; // 起始地址
                        
                        if (data_len >= (2 + count * 2)) {
                            float distances[MOTOR_NUM];
                            printf("[CMD] 多电机同步: count=%d, start_addr=%d\n", count, start_addr);
                            
                            // 解析距离数组
                            for (int i = 0; i < count; i++) {
                                distances[i] =(float)((s_ctrlBuf[5 + i*2] << 8) | s_ctrlBuf[6 + i*2]);
                                printf("  电机%d: 距离=%f\n", start_addr + i, distances[i]);
                            }
                            
                            // 调用多电机同步控制函数
                            motor_sync_control(count, start_addr, distances);
                        }
                    }
                    break;
                    
                case FUNC_MOTOR_KINEMATIC:
                    // 基于运动学的协同控制
                    // 数据格式: [R][theta][phi][deltaL1][deltaL2][deltaL3][deltaL4]
                    if (data_len >= 7) {
                        uint8_t R = s_ctrlBuf[3]; // 半径参数
                        float theta = (float)((int16_t)((s_ctrlBuf[4] << 8) | s_ctrlBuf[5])) / 100.0f; // 角度θ
                        float phi = (float)((int16_t)((s_ctrlBuf[6] << 8) | s_ctrlBuf[7])) / 100.0f;   // 角度φ
                        
                        printf("[CMD] 运动学控制: R=%d, theta=%.2f, phi=%.2f\n", R, theta, phi);
                        
                        // 计算肌腱长度变化
                        float deltaL[4] = {0};
                    	// 调用运动学模型并执行控制指令
                    	motor_kinematic_control(calculate_L(R, theta, phi , deltaL), R, theta, phi, deltaL);
                        printf("  计算得到的肌腱长度变化: ");
                        for (int i = 0; i < 4; i++) {
                            printf("deltaL[%d]=%.2f ", i, deltaL[i]);
                        }
                        printf("\n");
                    }
                    break;
                    
                case FUNC_MOTOR_CUSTOM:
                    // 自定义多电机控制: 数量 + [地址, 方向, 距离]...
                    if (data_len >= 1) {
                        uint8_t count = s_ctrlBuf[3]; // 电机数量
                        
                        if (data_len >= (1 + count * 3)) {
                            printf("[CMD] 自定义电机控制: count=%d\n", count);
                            
                            // 解析参数数组
                            uint8_t params[32]; // 最大支持10个电机的参数
                            for (int i = 0; i < (1 + count * 3); i++) {
                                params[i] = s_ctrlBuf[3 + i];
                            }
                            
                            // 调用自定义控制函数
                            motor_custom_control(count, params);
                        }
                    }
                    break;
                    
                case FUNC_MOTOR_ENABLE: // 电机使能
                    for (int i = 0; i < MOTOR_NUM; i++)
                    {
                        motor_enable(global_motor[i].id);
                    }
                    break;
                    
                case FUNC_MOTOR_STOP: // 电机停止
                    motor_stop_all();
                    break;
                    
                default:
                    break;
            }
            break;
        case FRAME_HEAD_SENSOR:
            switch (func)
            {
                case FUNC_SENSOR_INIT:
                    // 传感器初始化
                    printf("[CMD] 传感器初始化\n");
                    sensor_init();
                    break;
                    
                case FUNC_SENSOR_SINGLE_RD:
                    // 单传感器数据读取
                    if (data_len >= 1) {
                        uint8_t sensor_id = s_ctrlBuf[3];
                        printf("[CMD] 单传感器读取: id=%d\n", sensor_id);
                        sensor_single_read(sensor_id);
                    }
                    break;
                    
                case FUNC_SENSOR_MULTI_RD:
                    // 多传感器批量读取
                    printf("[CMD] 多传感器批量读取\n");
                    sensor_multi_read();
                    break;
                    
                case FUNC_SENSOR_SELF_TEST:
                    // 传感器自检
                    if (data_len >= 1) {
                        uint8_t sensor_id = s_ctrlBuf[3];
                        printf("[CMD] 传感器自检: id=%d\n", sensor_id);
                        sensor_self_test();
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

/**
 * @brief 上位机指令解析函数 (PC -> STM32)，将电机数据存入临时数据缓冲区，用于后续执行
 * @param byte 接收到的单字节数据
 */
void pc_cmd_parser_feed_byte(uint8_t byte)
{
    uint8_t receive = byte;
    
    switch (s_ctrlState)
    {
        case CMD_STATE_HEAD:
            if (receive == FRAME_HEAD_MOTOR || receive == FRAME_HEAD_SENSOR)
            {
                // 解析下一个字节，并将帧头存入发送数据缓冲区
                s_ctrlBuf[0] = receive;
                s_ctrlIdx = 1;
                s_ctrlState = CMD_STATE_FUNC;
            }
            break;
            
        /* 根据帧头进行不同的控制操作 */
        case CMD_STATE_FUNC:
            /* 保存功能码 */
            s_ctrlBuf[1] = receive;
            s_ctrlLen = 2;
            
            if (s_ctrlBuf[0] == FRAME_HEAD_MOTOR)
            {
                // 根据功能码执行特定动作
                switch ((MotorFuncCode_t)receive)
                {
                    case FUNC_MOTOR_SINGLE:
                    case FUNC_MOTOR_KINEMATIC:
                    case FUNC_MOTOR_CUSTOM:
                        s_ctrlState = CMD_STATE_LEN;
                        break;
                        
                    case FUNC_MOTOR_ENABLE:
                    case FUNC_MOTOR_STOP:
                        s_ctrlLen = 0;
                        // 进入指令解析执行函数
                        pc_cmd_parse_and_execute();
                        pc_cmd_parser_reset();
                        break;
                        
                    default:
                        pc_cmd_parser_reset();
                        break;
                }
            }
            else if (s_ctrlBuf[0] == FRAME_HEAD_SENSOR)
            {
                switch ((SensorFuncCode_t)receive)
                {
                    case FUNC_SENSOR_INIT:
                        pc_cmd_parse_and_execute();
                        pc_cmd_parser_reset();
                        break;
                        
                    case FUNC_SENSOR_SELF_TEST:
                        s_ctrlState = CMD_STATE_LEN;
                        break;
                        
                    default:
                        pc_cmd_parser_reset();
                        break;
                }
            }
            break;
            
        case CMD_STATE_LEN:
            s_ctrlLen = receive;
            s_ctrlBuf[2] = receive;
            
            if (s_ctrlLen + 4 > CTRL_BUF_SIZE) // 帧头(1) + 功能(1) + 长度(1) + 校验(1)
            {
                pc_cmd_parser_reset();
                break;
            }
            
            s_ctrlIdx = 3;
            s_ctrlState = (s_ctrlLen == 0) ? CMD_STATE_CHECK : CMD_STATE_DATA;
            break;
            
        case CMD_STATE_DATA:
            s_ctrlBuf[s_ctrlIdx++] = receive;
            
            if (s_ctrlIdx >= (uint8_t)(3 + s_ctrlLen))
            {
                s_ctrlState = CMD_STATE_CHECK;
            }
            break;
            
        case CMD_STATE_CHECK:
            s_ctrlBuf[s_ctrlIdx] = receive; // 存入校验位
            uint16_t sum = 0;
            
            for (int i = 0; i < s_ctrlIdx; i++) 
                sum += s_ctrlBuf[i];
                
            if ((sum & 0xFF) == receive) {
                is_connected = true; // 收到暗号，解锁连接
                pc_cmd_parse_and_execute();
            }
            
            pc_cmd_parser_reset();
            break;
            
        default:
            pc_cmd_parser_reset();
            break;
    }
}

/**
 * @brief 重置上位机指令解析器
 */
void pc_cmd_parser_reset_all(void) {
    pc_cmd_parser_reset();
}