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

// 辅助函数：大端序读取 short
int16_t read_short_be(const uint8_t* buf, uint16_t index) {
	return (int16_t)((buf[index] << 8) | buf[index + 1]);
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

	// 初始化所有数组为 0.0
	memset(motor_pos, 0, sizeof(motor_pos));
	memset(sensor_angle, 0, sizeof(sensor_angle));


    for (;;)
    {	// ====================================
	    // 1. 从队列倾听上位机指令 (替代 Serial.available())
	    // ====================================
	    // 不断从学弟写好的 CmdCtrlQueueHandle 队列中把数据抽出来，塞进我们的处理缓存
	    while (osMessageQueueGet(CmdCtrlQueueHandle, &rx_byte, NULL, 0) == osOK) {
	        if (rx_len < RX_BUF_SIZE) {
	            rx_buffer[rx_len++] = rx_byte;
	        }
	    }
	    // ====================================
	    // 2. 状态机解析 (逻辑原封不动)
	    // ====================================
	    while (rx_len >= 4) {
	        if (rx_buffer[0] != 0xAA) {
	            erase_buffer(1);
	            continue;
	        }

	        uint8_t func = rx_buffer[1];
	        uint8_t d_len = rx_buffer[2];
	        uint16_t f_len = 3 + d_len + 1;

	        if (rx_len < f_len) break; // 数据不够，等下一波

	        uint16_t sum = 0;
	        for (uint16_t i = 0; i < f_len - 1; i++) {
	            sum += rx_buffer[i];
	        }
	        uint8_t checksum = sum & 0xFF;

	        if (checksum == rx_buffer[f_len - 1]) {
	            is_connected = true; // 收到暗号，解锁连接

	            if (func == 0xFE) {
	                // 握手
	            } else if (func == 0xFF) {
	                sys_state = 1;
	            } else if (func == 0x10) {
	                sys_state = 0;
	            } else if (func == 0x11) {
	                memset(motor_pos, 0, sizeof(motor_pos));
	                memset(sensor_angle, 0, sizeof(sensor_angle));
	            }

	            // 仅在启动态处理物理量
	            if (sys_state == 1) {
	                if (func == 0x12 && d_len == 2) {
	                    scale_val = read_short_be(rx_buffer, 3) / 100.0f;
	                }
	                else if (func == 0x01 && d_len == 7) {
	                    uint8_t m_id = rx_buffer[3];
	                    if (m_id >= 1 && m_id <= MOTOR_NUM) {
	                        motor_pos[m_id - 1][0] = read_short_be(rx_buffer, 4) / 100.0f;
	                        motor_pos[m_id - 1][1] = read_short_be(rx_buffer, 6) / 100.0f;
	                        motor_pos[m_id - 1][2] = read_short_be(rx_buffer, 8) / 100.0f;
	                    }
	                }
	                else if (func == 0x03 && d_len == 7) {
	                    uint8_t s_id = rx_buffer[3];
	                    if (s_id >= 1 && s_id <= SENSOR_NUM) {
	                        sensor_angle[s_id - 1][0] = read_short_be(rx_buffer, 4) / 100.0f;
	                        sensor_angle[s_id - 1][1] = read_short_be(rx_buffer, 6) / 100.0f;
	                        sensor_angle[s_id - 1][2] = read_short_be(rx_buffer, 8) / 100.0f;
	                    }
	                }
	            }
	        }
	        // 解析完一帧，从缓存中抹除
	        erase_buffer(f_len);
	    }
	    // ====================================
	    // 3. 定时向 PC 反馈状态
	    // ====================================
	    if (is_connected) {
	        uint32_t current_time = HAL_GetTick();
	        if (current_time - last_send_time >= 200) { // 5Hz 刷新率
	            last_send_time = current_time;

	            uint8_t tx_frame[256];
	            uint16_t tx_idx = 0;

	            uint8_t payload[200];
	            uint16_t p_idx = 0;

	            payload[p_idx++] = MOTOR_NUM;
	            payload[p_idx++] = SENSOR_NUM;

	            // 打包电机
	            for (int i = 0; i < MOTOR_NUM; i++) {
	                push_short_be(payload, &p_idx, (int16_t)(motor_pos[i][0] * 100));
	                push_short_be(payload, &p_idx, (int16_t)(motor_pos[i][1] * 100));
	                push_short_be(payload, &p_idx, (int16_t)(motor_pos[i][2] * 100));
	            }

	            // 打包传感器
	            for (int i = 0; i < SENSOR_NUM; i++) {
	                push_short_be(payload, &p_idx, (int16_t)(sensor_angle[i][0] * 100));
	                push_short_be(payload, &p_idx, (int16_t)(sensor_angle[i][1] * 100));
	                push_short_be(payload, &p_idx, (int16_t)(sensor_angle[i][2] * 100));
	            }

	            push_short_be(payload, &p_idx, (int16_t)(scale_val * 100));
	            payload[p_idx++] = sys_state;

	            // 组装帧头和数据
	            tx_frame[tx_idx++] = 0xBB;
	            tx_frame[tx_idx++] = 0x02;
	            tx_frame[tx_idx++] = p_idx; // 长度

	            for (uint16_t i = 0; i < p_idx; i++) {
	                tx_frame[tx_idx++] = payload[i];
	            }

	            // 计算校验和
	            uint16_t tx_sum = 0;
	            for (uint16_t i = 0; i < tx_idx; i++) {
	                tx_sum += tx_frame[i];
	            }
	            tx_frame[tx_idx++] = tx_sum & 0xFF;

	            // 调用 HAL 库发送 (替代 Serial.write)
	            HAL_UART_Transmit(&huart2, tx_frame, tx_idx, 100);
	        }
	    }
	    // 每次循环延时 2 毫秒，释放 CPU 控制权给学弟的其他任务
	    osDelay(2);
	  }

        // // 取出从串口中断2中得到的数据
        // if (osMessageQueueGet(CmdCtrlQueueHandle, &receive, 0, osWaitForever) == osOK)
        // {
        //     Usart_SendString(&huart1, &receive, 1);
        //     // 进入指令解析函数
        // 	cmd_parse_feed_byte(receive);
        // }
}