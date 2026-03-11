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
#include "usart.h"
#include "Common/cmd_parse.h"
#include "string.h"
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#define RX_BUF_SIZE 256
uint8_t rx_buffer[RX_BUF_SIZE];
uint16_t rx_len = 0;


float motor_pos[MOTOR_NUM][3] = {0};
float sensor_angle[SENSOR_NUM][3] = {0};
float scale_val = 100.0f;

uint8_t sys_state = 0;       // 0:未启动  1:已启动
bool is_connected = false;   // false:未连接 true:已连接

// ==========================================
// 纯 C 语言实现的缓存区 (替代原本的 std::vector)
// ==========================================


// 从缓存区头部擦除已处理的数据 (替代 vector.erase)
void erase_buffer(uint16_t count) {
	if (count >= rx_len) {
		rx_len = 0;
	} else {
		memmove(rx_buffer, rx_buffer + count, rx_len - count);
		rx_len -= count;
	}
}


// 辅助函数：大端序压入 short
void push_short_be(uint8_t* buf, uint16_t* idx, int16_t val) {
	buf[(*idx)++] = (val >> 8) & 0xFF;
	buf[(*idx)++] = val & 0xFF;
}

void StartCmdCtrlTask(void *argument)
{
    // 测试串口用
    // uint8_t test_msg[] = "send to usart1\r\n";
    uint8_t receive;
	uint32_t last_send_time = HAL_GetTick(); // 替代 millis()
	uint8_t rx_byte;
    uint32_t test_counter = 0; // 测试计数器

	// 初始化所有数组为 0.0
	memset(motor_pos, 0, sizeof(motor_pos));
	memset(sensor_angle, 0, sizeof(sensor_angle));
    
    // 设置测试模式为 true 以发送测试数据
    bool test_mode = true;


    for (;;)
    {	// ====================================
	    // 1. 从队列倾听上位机指令 (替代 Serial.available())
	    // ====================================
	    // 不断从学弟写好的 CmdCtrlQueueHandle 队列中把数据抽出来，塞进我们的处理缓存
	    while (osMessageQueueGet(CmdCtrlQueueHandle, &receive, NULL, 0) == osOK) {
	        if (rx_len < RX_BUF_SIZE) {
	            rx_buffer[rx_len++] = receive;
	        	Usart_SendString(&huart1, &receive, 1);
	        	// 进入指令解析函数
	        	cmd_parse_feed_byte(receive);
	        }
	    }
	    // ====================================
	    // 2. 状态机解析 (逻辑原封不动)
	    // ====================================
    	cmd_parse_feed_byte(receive);
	    // ====================================
	    // 3. 定时向 PC 反馈状态
	    // ====================================

	    // 每次循环延时 2 毫秒，释放 CPU 控制权给学弟的其他任务
	    osDelay(2);
	  }


}