//
// Created by blin on 2026/3/7.
//
// 用于IMU数据读取与处理，通过I2C实现读取，并提供数据接口供其他模块调用
#include "Sensor/IMU.h"

#include <string.h>

#include "usart.h"

void IMU_single_read(uint8_t sensor_id)
{
	// 使用串口1完成数据读取
	char read[20] = "123";
	Usart_SendString(&huart1, read, strlen(read));
}
