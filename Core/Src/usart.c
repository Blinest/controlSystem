/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART 配置 + HAL 中断接收
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

static uint8_t uart1_rx_byte;
static uint8_t uart2_rx_byte;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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
 * @brief 开启 USART1 HAL 单字节中断接收
 */
void UART1_Receive_Start(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/**
 * @brief 开启 USART2 HAL 单字节中断接收
 */
void UART2_Receive_Start(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {



    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }
  else if (huart->Instance == USART2)
  {
  	uint8_t data = uart2_rx_byte;
  	if (CmdCtrlQueueHandle != NULL)
  	{
  		osMessageQueuePut(CmdCtrlQueueHandle, &data, 0, 0);

  	}
    (void)HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }
  else if (huart->Instance == USART2)
  {
    (void)HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  }
}

int platform_uart_send(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0U) return -1;
  return (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100) == HAL_OK) ? 0 : -1;
}

int platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  if (buf == NULL || len == 0U) return -1;
  return (HAL_UART_Receive(&huart1, buf, len, timeout_ms) == HAL_OK) ? 0 : -1;
}

/* USER CODE END 1 */
