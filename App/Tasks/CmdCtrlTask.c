/**
*  @file CmdCtrlTask.c
 * @brief 指令控制任务：从队列取串口数据，交给 cmd_parse 解析并执行
 * @author blin
 *
 * 串口2 接收字节经 CmdCtrlQueueHandle 送入本任务，每字节调用 cmd_parse_feed_byte()，
 * 指令解析与电机/传感器控制逻辑在 Common/cmd_parse 中实现。
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "string.h"
#include "Common/can_driver.h"

#include "Common/cmd_parse_unified.h"
#include "Motor/Motor.h"
#include "stdio.h"
#include "usart.h"
#include "Sensor/Sensor.h"
#define RX_BUF_SIZE 256

bool is_connected = false;   // false:未连接 true:已连接

void StartCmdCtrlTask(void *argument)
{
	uint8_t rx_buffer[RX_BUF_SIZE];
	uint16_t rx_len = 0;
	float motor_pos[MOTOR_NUM][3] = {0};
	float sensor_angle[SENSOR_NUM][3] = {0};
    // 测试串口用
    uint8_t test_msg[] = "send to usart1\r\n";
    uint8_t receive;

	// 初始化所有数组为 0.0
	memset(motor_pos, 0, sizeof(motor_pos));
	memset(sensor_angle, 0, sizeof(sensor_angle));
    for (;;)
    {
        // 电机使能放在循环开始，确保每次循环都执行
        
	    // 从队列接收上位机指令
	    if (osMessageQueueGet(CmdCtrlQueueHandle, &receive, NULL, 0) == osOK) {
	        // 优先回显，提高响应性
	        if (rx_len < RX_BUF_SIZE) {
	            rx_buffer[rx_len++] = receive;
	        	// 进入指令解析函数，指令解析函数负责指令解析，而后再将解析好的指令传给电机进行解析
	        	cmd_parse_feed_byte(receive);
	        } else {
                rx_len = 0; // 缓冲区溢出重置
            }
	    }
    	static uint32_t last_send_time = 0;
    	uint32_t current_time = osKernelGetTickCount();

    	// 每100ms读取一次位置信息
    	if ((current_time - last_send_time) >= 500)
    	{
    		// 电机状态检测 (使用 static 以节省堆栈空间)
    		motor_status_check();

    		last_send_time = current_time;
    	}
        // 添加小延时，避免过度占用CPU
		osDelay(100);
	  }
}