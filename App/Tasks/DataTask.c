/**
 * @file DataTask.c
 * @brief 数据任务：负责 USART1 RX (外设反馈) 和 USART2 TX (发送至 PC)
 *
 * 任务流程：
 * 1. 监听 CmdDataQueue (USART1 RX)，解析外设反馈并更新全局状态。
 * 2. 将系统状态打包并通过 SensorMessageQueue 缓冲。
 * 3. 轮询 SensorMessageQueue，将反馈字节发送至 USART2 TX (上位机)。
 */
#include "cmsis_os.h"
#include "usart.h"
#include "Common/cmd_parse.h"
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "string.h"

extern osMessageQueueId_t CmdDataQueueHandle;
extern osMessageQueueId_t SensorMessageQueueHandle;

// 引用 CmdCtrlTask.c 中的全局变量
extern float motor_pos[MOTOR_NUM][3];
extern float sensor_angle[SENSOR_NUM][3];
extern float scale_val;
extern uint8_t sys_state;

// 外设解析状态机
typedef enum {
    PERIPH_STATE_HEAD = 0,
    PERIPH_STATE_FUNC,
    PERIPH_STATE_LEN,
    PERIPH_STATE_DATA,
    PERIPH_STATE_CHECK
} PeriphParseState_t;

static PeriphParseState_t s_periphState = PERIPH_STATE_HEAD;
static uint8_t s_periphBuf[64];
static uint8_t s_periphIdx = 0;
static uint8_t s_periphLen = 0;

/**
 * @brief 打包并发送 test_frame 到 SensorMessageQueue 缓冲
 */
void send_test_frame_to_queue(void) {
    uint8_t frame[64];
    uint16_t len = cmd_pack_status_frame(frame, motor_pos, sensor_angle, scale_val, sys_state);

    // 发送到 SensorMessageQueue 缓冲，由 CmdCtrlTask 统一调度发送
    for (int i = 0; i < len; i++) {
        uint16_t msg = frame[i];
        osMessageQueuePut(SensorMessageQueueHandle, &msg, 0, 0);
    }
}

void StartDataTask(void *argument)
{
    uint8_t rx_byte;
    uint16_t feedback_msg;
    for(;;)
    {
        // ====================================
        // 0. 数据回传流: 从 SensorMessageQueue (已打包) 发送至 PC (USART2 TX)
        // ====================================
        if (osMessageQueueGet(SensorMessageQueueHandle, &feedback_msg, NULL, 0) == osOK) {
            uint8_t b = (uint8_t)feedback_msg;
            HAL_UART_Transmit(&huart2, &b, 1, 10);
        }

        // ====================================
        // 1. 数据采集流: 从外设 (USART1 RX) 接收并喂入解析模块
        // ====================================
        if (osMessageQueueGet(CmdDataQueueHandle, &rx_byte, NULL, 0) == osOK)
        {
            // 专注数据流路径，解析与逻辑交由 cmd_parse 处理
            cmd_parse_feed_periph_byte(rx_byte);
        }
        
        osDelay(1); // 释放 CPU，防止忙等
    }
}