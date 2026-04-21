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
//初始化串口标志
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "FreeRTOS.h"
#include "queue.h"

#include <stdbool.h>
__IO bool rxFrameFlag1 = false;
__IO uint8_t rxCmd1[256] = {0};
__IO uint8_t rxCount1 = 0;
__IO bool rxFrameFlag2 = false;     // USART2 帧接收完成标志
__IO uint8_t rxCmd2[256] = {0}; // USART2 接收缓冲区
__IO uint8_t rxCount2 = 0;           // USART2 接收数据长度

// 电机/传感器地址缓冲区
#define MAX_BUFFER_SIZE 256
uint8_t motor_buffer[MAX_BUFFER_SIZE] = {0};
uint8_t motor_buffer_count = 0;


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
  huart1.Init.BaudRate = 9600;
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


void Usart_SendString(UART_HandleTypeDef *huart, uint8_t *str, uint16_t len)
{
  /* 强制重置错误状态，确保发送不会因之前的接收错误而阻塞 */
  if (huart->gState == HAL_UART_STATE_ERROR) {
      huart->gState = HAL_UART_STATE_READY;
      huart->ErrorCode = HAL_UART_ERROR_NONE;
  }

  // 根据数据长度计算合理的超时时间
  // 9600波特率下，每字节约1.04ms，加上一些余量
  // 计算公式：超时时间(ms) = (字节数 * 1.5ms) + 40ms（固定余量）
  uint32_t timeout = (len * 15) / 10 + 40;  // 转换为ms
  HAL_UART_Transmit(huart, str, len, timeout);
}


uint8_t rxData1 = 0, rxData2 = 0;
// 启动中断
void UART1_Receive_Start() {
  // 启用 RXNE 中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void UART2_Receive_Start() {
  // 启用 RXNE 中断
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void USART_RX_CustomHandler(UART_HandleTypeDef *huart)
{
  uint32_t sr = READ_REG(huart->Instance->SR);
  uint32_t cr1 = READ_REG(huart->Instance->CR1);
  uint8_t data;

  /* 1. 处理接收错误 (ORE, NE, FE, PE) */
  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
  {
    /* 读取 DR 寄存器是清除 F1 系列 ORE 标志的必要步骤 */
    data = (uint8_t)(huart->Instance->DR & 0xFF);
    
    /* 虽然是错误字节，但也尝试入队，防止指令流断裂 */
    if (huart->Instance == USART2) {
        osMessageQueuePut(CmdCtrlQueueHandle, &data, 0, 0);
    } else if (huart->Instance == USART1) {
        osMessageQueuePut(CmdDataQueueHandle, &data, 0, 0);
    }
    
    /* 清除标志位 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    
    /* 强制重置 HAL 状态，防止 ORE 导致 gState 永久处于 BUSY 或 ERROR */
    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* 核心修复：确保 RXNE 中断始终开启。HAL 的 IRQ 处理程序在检测到错误时可能会关闭它 */
    SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
    return;
  }

  /* 2. 正常接收处理 (RXNE) */
  if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE))
  {
    data = (uint8_t)(huart->Instance->DR & 0xFF);
    
    if (huart->Instance == USART2) {
        osMessageQueuePut(CmdCtrlQueueHandle, &data, 0, 0);
    } else if (huart->Instance == USART1) {
        osMessageQueuePut(CmdDataQueueHandle, &data, 0, 0);
    }
  }
}

/**
  * @brief 串口接收中断 (HAL 原有的保留，用于 TX 结束等)
  * @param huart: UART 句柄指针
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 这里不再需要调用 HAL_UART_Receive_IT，因为我们使用了寄存器级中断
}

/**
  * @brief 串口错误回调函数
  * @param huart: UART 句柄指针
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 处理所有可能的错误
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
      __HAL_UART_CLEAR_OREFLAG(huart);
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET)
    {
      __HAL_UART_CLEAR_FEFLAG(huart);
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET)
    {
      __HAL_UART_CLEAR_NEFLAG(huart);
    }
    
    // 重新启动自定义接收（通过清除错误标志即可，不需要调用 HAL_UART_Receive_IT）
  }
  else if (huart->Instance == USART2)
  {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
      __HAL_UART_CLEAR_OREFLAG(huart);
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET)
    {
      __HAL_UART_CLEAR_FEFLAG(huart);
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET)
    {
      __HAL_UART_CLEAR_NEFLAG(huart);
    }
  }
}



/* USER CODE END 1 */

