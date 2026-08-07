/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void Usart_SendString(UART_HandleTypeDef* huart, unsigned char *str, unsigned short len);
void UART1_Receive_Start(void);
void UART2_Receive_Start(void);

/* UART1 中断环形接收缓冲
 * UART1 上交互两类数据：
 *  1) Modbus 电机(AQMD 0x03 / SS 0x01/0x02)查询后的响应，阻塞式读取；
 *  2) 其他设备主动上报的数据，中断接收、由调用方(DataTask/motor_status_check)处理。
 * 中断每收到一字节写入 ring，是 UART1 唯一的接收入口，消除与阻塞读的 RxState 冲突。
 */
#define UART1_RX_RING_SIZE 512

/** @brief 初始化 UART1 中断接收环形缓冲(复位 ring)并挂第一个中断接收 */
void UART1_RX_Init(void);

/** @brief 读取一帧主动上报的数据(已按 帧头+定长 切帧)，返回帧字节数，无则 0 */
int uart1_get_reporting_frame(uint8_t *buf, uint16_t *len);

/** @brief 丢弃 UART1 ring 中积压的残留/上报字节。
 * 在读位置、读电机回复这类按定长取帧之前调用，防止脏数据占位导致帧错位。
 * 需在已持有 uart1_bus_lock 的事务内调用。 */
void uart1_drain_stale_bytes(void);

/** @brief 平台 UART1 阻塞发送 / 从 ring 阻塞读取 */
int platform_uart_send(const uint8_t *data, uint16_t len);
int platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/* UART1 总线互斥锁：调用方在「发请求→读回复」事务外包围 */
void uart1_bus_lock(void);
void uart1_bus_unlock(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

