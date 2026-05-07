/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#include <stdbool.h>

// --- 外部队列句柄（在 main.c 或全局定义，此处仅声明）---
uint8_t uart1_rx_byte;
uint8_t uart2_rx_byte;
static SemaphoreHandle_t uart1_tx_sem = NULL;
// 函数声明
void parse_motor_feedback(uint8_t *buffer, uint8_t length);

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */
void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */
  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */
  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */
  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */
  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */
  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief  初始化 USART1 中断发送所用的信号量
 * @note   必须在开启接收中断、任务调度前调用
 */
void UART1_TxSem_Init(void)
{
	uart1_tx_sem = xSemaphoreCreateBinary();
	if (uart1_tx_sem == NULL) {
		Error_Handler();
	}
	// 初始状态为 “可用”（表示没有发送正在进行）
	xSemaphoreGive(uart1_tx_sem);
}
/**
 * @brief  发送完成中断回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		// 释放信号量，唤醒等待发送完成的任务
		xSemaphoreGiveFromISR(uart1_tx_sem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
 * @brief  通用串口发送函数
 *         - 对于 USART1，使用中断发送 + 信号量等待，不会阻塞 CPU
 *         - 对于其他串口（如 USART2），保留轮询发送（可根据需要改为 IT 方式）
 * @param  huart: 串口句柄
 * @param  str:   数据指针
 * @param  len:   发送字节数
 */
void Usart_SendString(UART_HandleTypeDef *huart, uint8_t *str, uint16_t len)
{
	if (huart->Instance == USART1) {
		if (uart1_tx_sem == NULL) return;

		// ① 获取发送权（信号量 -1）
		if (xSemaphoreTake(uart1_tx_sem, portMAX_DELAY) != pdTRUE) return;

		// ② 启动中断发送
		HAL_StatusTypeDef status = HAL_UART_Transmit_IT(huart, str, len);
		if (status != HAL_OK) {
			xSemaphoreGive(uart1_tx_sem);   // 启动失败，归还发送权
			return;
		}

		// ③ 等待发送完成（ISR 会 Give 信号量，唤醒这里）
		if (xSemaphoreTake(uart1_tx_sem, portMAX_DELAY) != pdTRUE) {
			HAL_UART_AbortTransmit_IT(huart);
			xSemaphoreGive(uart1_tx_sem);   // 异常后也归还
			return;
		}

		// ★★★ ④ 关键：发送成功，归还发送权，让下次调用能获取 ★★★
		xSemaphoreGive(uart1_tx_sem);
	}
	else {
		HAL_UART_Transmit(huart, str, len, 1000);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		// 将接收到的字节放入电机反馈队列
		osMessageQueuePut(MotorRxQueueHandle, &uart1_rx_byte, 0, 0);
		// 重新启动接收（非常重要！）
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
	else if (huart->Instance == USART2) {
		// 将接收到的字节放入命令队列
		osMessageQueuePut(CmdCtrlQueueHandle, &uart2_rx_byte, 0, 0);
		// 重新启动接收
		HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
	}
}

void UART1_Receive_Start(void) {
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

void UART2_Receive_Start(void) {
	HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	// 错误发生时，重新启动接收（可选）
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	} else if (huart->Instance == USART2) {
		HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
	}
}
/* USER CODE END 1 */