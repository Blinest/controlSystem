//
// Created by blin on 2026/4/30.
//
#include "usart.h"
#include "CMCU-06.h"
#include <string.h>   // for memcpy
#include "main.h"     // for HAL_Delay

// 内部函数：计算CRC16-Modbus
static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len);

void CMCU_06_Init(void)
{
	// 关闭写入保护 -> 复位 -> 恢复写入保护
	for (uint8_t i = 1; i < 4; i++)
	{
		CMCU_06_Write_Protect(i, false);  // 关闭写入保护
		HAL_Delay(10);
		CMCU_06_Reset(i);                 // 复位传感器
		HAL_Delay(50);
		CMCU_06_Write_Protect(i, true);   // 重新打开写入保护
	}
}

/**
 * 写入保护控制
 * 发送: [addr] [0x06] [0x00] [0x17] [0x00] [state] [CRC_L] [CRC_H]
 * state: 0x00 = 打开写入保护, 0x01 = 关闭写入保护
 */
void CMCU_06_Write_Protect(uint8_t addr, bool enable_protect)
{
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x06;
	cmd[2] = 0x00;
	cmd[3] = 0x17;
	cmd[4] = 0x00;
	cmd[5] = enable_protect ? 0x00 : 0x01;   // 写入保护值

	// 计算 CRC16-Modbus (前6字节)
	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;        // CRC 低字节
	cmd[7] = (crc >> 8) & 0xFF; // CRC 高字节

	Usart_SendString(&huart1, cmd, 8);
}

/**
 * 复位传感器（读取两个保持寄存器实现复位）
 * 发送: [addr] [0x03] [0x00] [0x00] [0x00] [0x02] [CRC_L] [CRC_H]
 */
void CMCU_06_Reset(uint8_t addr)
{
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x03;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x02;

	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;
	cmd[7] = (crc >> 8) & 0xFF;

	Usart_SendString(&huart1, cmd, 8);
}

/**
 * 去皮置零（清零当前载荷）
 * 发送: [addr] [0x06] [0x00] [0x15] [0x00] [0x01] [CRC_L] [CRC_H]
 */
void CMCU_06_ResetPins(uint8_t addr)
{
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x06;
	cmd[2] = 0x00;
	cmd[3] = 0x15;
	cmd[4] = 0x00;
	cmd[5] = 0x01;   // 去皮命令值

	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;
	cmd[7] = (crc >> 8) & 0xFF;

	Usart_SendString(&huart1, cmd, 8);
}

/**
 * 砝码校准（待实现）
 */
void CMCU_06_Cal(uint8_t addr)
{
	// TODO: 根据传感器手册实现校准命令，同样使用动态CRC
}

/**
 * 单次读取传感器数据
 * 发送读指令后，等待接收9字节（地址+功能码+数据长度+4字节数据+CRC低+CRC高）
 */
void CMCU_06_single_read(uint8_t addr)
{
	// 1. 发送读命令（与 Reset 命令结构相同，但独立发送）
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x03;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x02;

	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;
	cmd[7] = (crc >> 8) & 0xFF;

	Usart_SendString(&huart1, cmd, 8);
}

// CRC16-Modbus 计算函数（多项式0x8005，初始0xFFFF）
static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len)
{
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