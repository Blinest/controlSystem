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
// 引用 CmdCtrlTask.c 中的全局变量
extern float motor_pos[MOTOR_NUM][3];
extern float sensor_angle[SENSOR_NUM][3];
extern float scale_val;
extern uint8_t sys_state;

// 外设解析状态机
typedef enum {
    PERIPH_ID = 1,
    PERIPH_STATE_FUNC = 2,
    PERIPH_STATE_LEN,
    PERIPH_STATE_DATA,
    PERIPH_STATE_CHECK
} PeriphParseState_t;



static PeriphParseState_t s_periphState = PERIPH_ID;
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
    MotorFeedback motor_feedback;
    uint16_t feedback_msg;
    for(;;)
    {
        // ====================================
        // 0. 数据回传流:  SensorMessageQueue (已打包) 发送至 PC (USART2 TX)
        // ====================================

    	uint8_t test_frame[] = {
    		0xBB, 0x02, 0x34,  // 帧头、功能码、长度(52)
            0x03, 0x04,        // MOTOR_NUM=3, SENSOR_NUM=4
            // 电机1: X=10.00(0x03E8), Y=20.00(0x07D0), Z=30.00(0x0BB8), state=1
            0x03, 0xE8, 0x07, 0xD0, 0x0B, 0xB8, 0x01,

            // 电机2: X=40.00(0x0FA0), Y=50.00(0x1388), Z=60.00(0x1770), state=0
            0x0F, 0xA0, 0x13, 0x88, 0x17, 0x70, 0x00,

            // 电机3: X=70.00(0x1B58), Y=80.00(0x1F40), Z=90.00(0x2328), state=1
            0x1B, 0x58, 0x1F, 0x40, 0x23, 0x28, 0x01,

            // 传感器1: X=25.00(0x09C4), Y=25.00(0x09C4), Z=25.00(0x09C4)
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,

            // 传感器2: X=25.00(0x09C4), Y=25.00(0x09C4), Z=25.00(0x09C4)
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,

            // 传感器3: X=25.00(0x09C4), Y=25.00(0x09C4), Z=25.00(0x09C4)
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,

            // 传感器4: X=25.00(0x09C4), Y=25.00(0x09C4), Z=25.00(0x09C4)
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,

            // scale1=50.00(0x1388), scale2=0(0x0000), sys_state=1
            0x13, 0x88, 0x00, 0x20, 0x01,

            // 校验和
            0xA9
		};
    	// 计算校验和
    	uint16_t checksum = 0;
    	for (int i = 0; i < sizeof(test_frame)-1; i++) {
    		checksum += test_frame[i];
    	}
    	test_frame[sizeof(test_frame)-1] = checksum & 0xFF;

    	// 发送固定测试帧
    	HAL_UART_Transmit(&huart2, test_frame, sizeof(test_frame), 100);
    	osDelay(100);
        if (osMessageQueueGet(SensorMessageQueueHandle, &feedback_msg, NULL, 0) == osOK) {

		}

        // ====================================
        // 1. 数据采集流: 从外设 (USART1 RX) 接收并喂入解析模块
        // ====================================
        if (osMessageQueueGet(CmdDataQueueHandle, &motor_feedback, NULL, 0) == osOK)
        {
            // 处理电机反馈数据
            // 查找对应的电机索引
            for (int i = 0; i < MOTOR_NUM; i++) {
                if (motor[i].id == motor_feedback.addr) {
                    // 更新电机位置数据到 Motor 结构体
                	// 存储电机的角度信息
                    motor[i].current_pos = motor_feedback.motor_data_cur[0]; // °
                    motor[i].current_vel = motor_feedback.motor_data_cur[1]; // °/s
                    motor[i].current_acc = motor_feedback.motor_data_cur[2]; // °/s^2
                	// 转换为步进电机的距离数据
                	// TODO: 将步进电机的角度信息转换为位移信息

                    // 更新电机状态
                    motor[i].state = motor_feedback.state;
                    break;
                }
            }
        }
        
        osDelay(1); // 释放 CPU，防止忙等
    }
}