//
// Created by blin on 2026/3/7.
//
// 用于IMU数据读取与处理，通过I2C实现读取，并提供数据接口供其他模块调用
#include "Sensor/IMU.h"
#include "Sensor/WT_IMU.h"
#include <string.h>

#include "usart.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

void IMU_Init(void)
{
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
}

void IMU_single_read(uint8_t sensor_id)
{
	// 使用串口1完成数据读取
	uint16_t reg_start = 0x3D;   // Roll 寄存器地址（需根据实际模块调整）
	uint16_t reg_count = 3;      // 连续读取 3 个寄存器

	WitReadReg(reg_start, reg_count);

}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Usart_SendString(&huart1, p_data, uiSize);
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
	for(i = 0; i < uiRegNum; i++)
	{
		switch(uiReg)
		{
			//            case AX:
			//            case AY:
		case AZ:
			s_cDataUpdate |= ACC_UPDATE;
			break;
			//            case GX:
			//            case GY:
		case GZ:
			s_cDataUpdate |= GYRO_UPDATE;
			break;
			//            case HX:
			//            case HY:
		case HZ:
			s_cDataUpdate |= MAG_UPDATE;
			break;
			//            case Roll:
			//            case Pitch:
		case Yaw:
			s_cDataUpdate |= ANGLE_UPDATE;
			break;
		default:
			s_cDataUpdate |= READ_UPDATE;
			break;
		}
		uiReg++;
	}
}
static void AutoScanSensor(void)
{
	int i, iRetry;

	for(i = 1; i < 10; i++)
	{
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			if(s_cDataUpdate != 0)
			{

				return ;
			}
			iRetry--;
		}while(iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}
