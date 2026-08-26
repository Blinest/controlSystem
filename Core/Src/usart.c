/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART 配置 + 寄存器级 RXNE 中断收发
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
}

/* USART2 init */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
}

/* HAL MSP Init */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9  TX, PA10 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2 TX, PA3 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

/* HAL MSP DeInit */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief 发送字符串（阻塞）
 */
void Usart_SendString(UART_HandleTypeDef *huart, uint8_t *str, uint16_t len)
{
  if (huart->gState == HAL_UART_STATE_ERROR) {
    huart->gState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
  }
  uint32_t timeout = (len * 15) / 10 + 40;
  HAL_UART_Transmit(huart, str, len, timeout);
}

/**
 * @brief 开启 USART1 RXNE 中断
 * USART1 接收字节 → CmdCtrlQueueHandle
 */
void UART1_Receive_Start(void)
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/**
 * @brief 开启 USART1 RXNE 中断
 * USART1 接收字节 → CmdCtrlQueueHandle
 */
void UART2_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**
 * @brief 自定义 RXNE/ORE 中断处理
 * 由 USART1_IRQHandler / USART2_IRQHandler 调用
 */
void USART_RX_CustomHandler(UART_HandleTypeDef *huart)
{
  uint32_t sr = READ_REG(huart->Instance->SR);
  uint32_t cr1 = READ_REG(huart->Instance->CR1);
  uint8_t data;

  /* 错误处理 */
  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
  {
    data = (uint8_t)(huart->Instance->DR & 0xFF);
    /* 错误字节也入队 */
    if (huart->Instance == USART1)
      osMessageQueuePut(CmdCtrlQueueHandle, &data, 0, 0);

    /* 清除错误标志 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* 确保 RXNE 中断始终开启 */
    SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
    return;
  }

  /* 正常接收 */
  if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE))
  {
    data = (uint8_t)(huart->Instance->DR & 0xFF);
    if (huart->Instance == USART2)
      osMessageQueuePut(CmdCtrlQueueHandle, &data, 0, 0);
  }
}

/* USER CODE END 1 */
