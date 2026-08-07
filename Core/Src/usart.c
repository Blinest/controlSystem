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
#include "cmsis_os2.h"

static uint8_t uart1_rx_byte;
static uint8_t uart2_rx_byte;

/* UART1 总线互斥锁：保护「发请求→读回复」整条事务的原子性。
 * 跨任务共享：DataTask(motor_status_check 读) 与 CmdCtrlTask(电机命令写)。 */
static osMutexId_t uart1_bus_mutex;

void uart1_bus_lock(void)
{
  if (uart1_bus_mutex == NULL)
  {
    uart1_bus_mutex = osMutexNew(NULL);
  }
  if (uart1_bus_mutex != NULL)
  {
    (void)osMutexAcquire(uart1_bus_mutex, osWaitForever);
  }
}

void uart1_bus_unlock(void)
{
  if (uart1_bus_mutex != NULL)
  {
    (void)osMutexRelease(uart1_bus_mutex);
  }
}

/* ==================== UART1 中断环形接收缓冲 ====================
 * UART1 是唯一接收入口：中断每收到一字节写入 ring。
 * 消除了阻塞式 HAL_UART_Receive 与 UART1 中断接收抢占 RxState 的冲突。
 * head: ISR 写入游标；tail: 任务消费游标。 */
typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t  buf[UART1_RX_RING_SIZE];
} uart1_ring_t;

static uart1_ring_t uart1_rx;

/* 记录当前 Modbus 请求的从机地址（platform_uart_send 发出时缓存帧首字节）。
 * platform_uart_recv 用它把「电机回复」与「其他设备主动上报」区分开。 */
static volatile uint8_t uart1_pending_slave = 0xFFU;

/* 被动积压的、以非目标从机地址开头的原始字节（主动上报设备数据），
 * 由 Motor 层在 motor_status_check 里取走处理。 */
static uint8_t  uart1_reporting_buf[UART1_RX_RING_SIZE];
static volatile uint16_t uart1_reporting_len;

static void uart1_ring_write(uint8_t byte)
{
    uint16_t next = (uint16_t)((uart1_rx.head + 1U) % UART1_RX_RING_SIZE);
    /* ring 满时丢弃最旧一字节，保证新数据不丢 */
    if (next == uart1_rx.tail)
    {
        uart1_rx.tail = (uint16_t)((uart1_rx.tail + 1U) % UART1_RX_RING_SIZE);
    }
    uart1_rx.buf[uart1_rx.head] = byte;
    uart1_rx.head = next;
}

static int uart1_ring_available(void)
{
    return (uart1_rx.head >= uart1_rx.tail)
           ? (int)(uart1_rx.head - uart1_rx.tail)
           : (int)(UART1_RX_RING_SIZE - uart1_rx.tail + uart1_rx.head);
}

static uint8_t uart1_ring_peek(uint16_t off)
{
    return uart1_rx.buf[(uart1_rx.tail + off) % UART1_RX_RING_SIZE];
}

static void uart1_ring_pop(uint16_t count)
{
    uart1_rx.tail = (uint16_t)((uart1_rx.tail + count) % UART1_RX_RING_SIZE);
}

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
    /* 唯一接收入口：把每字节写入 ring，再重挂中断接收 */
    uart1_ring_write(uart1_rx_byte);

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

  /* 记录本次 Modbus 请求的从机地址（帧首字节），供 platform_uart_recv 分辨回复 */
  if (data[0] != 0x00U)
  {
    uart1_pending_slave = data[0];
  }

  return (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100) == HAL_OK) ? 0 : -1;
}

/* UART1 中断接收：只由该项后续的 HAL 中断驱动，这里不再阻塞读取 */
void UART1_RX_Init(void)
{
  /* 复位 ring */
  uart1_rx.head = 0;
  uart1_rx.tail = 0;
  uart1_reporting_len = 0;
  uart1_pending_slave = 0xFFU;

  /* 挂第一个中断接收 */
  UART1_Receive_Start();
}

/* 把 ring 中「非目标从机地址」开头的原始字节切到上报缓冲，供 Motor 层处理。
 * 当 ring 中有可用字节且其首字节不是 uart1_pending_slave(即正在等待的 Modbus 回复
 * 从机)时，把整段非目标前缀搬入上报缓冲。返回搬入字节数。 */
static uint16_t uart1_stash_reporting(void)
{
  uint16_t stashed = 0;

  while (uart1_ring_available() > 0)
  {
    uint8_t head_byte = uart1_ring_peek(0);
    if (head_byte == uart1_pending_slave)
    {
      break; /* 首字节匹配目标从机，视为 Modbus 回复起点，不再搬走 */
    }

    if (uart1_reporting_len < (uint16_t)sizeof(uart1_reporting_buf))
    {
      uart1_reporting_buf[uart1_reporting_len++] = uart1_ring_peek(0);
    }
    uart1_ring_pop(1);
    stashed++;
  }

  return stashed;
}

int platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  if (buf == NULL || len == 0U) return -1;

  uint32_t start = HAL_GetTick();

  for (;;)
  {
    /* 先把非目标从机开头的上报数据切走，避免占住待读回复的位置 */
    (void)uart1_stash_reporting();

    /* 等待从 ring 取到 len 字节(可能跨多次循环累积) */
    if (uart1_ring_available() >= len)
    {
      for (uint16_t i = 0; i < len; i++)
      {
        buf[i] = uart1_ring_peek(0);
        uart1_ring_pop(1);
      }
      return 0;
    }

    /* 超时检查 */
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return -1;
    }

    osDelay(1);
  }
}

/* 取出一段主动上报的数据；无则返回 0，有则拷贝并返回字节数 */
int uart1_get_reporting_frame(uint8_t *buf, uint16_t *len)
{
  if (buf == NULL || len == NULL) return 0;

  uint16_t n = uart1_reporting_len;
  if (n == 0U) return 0;

  if (n > *len) n = *len;
  memcpy(buf, uart1_reporting_buf, n);
  *len = n;
  uart1_reporting_len = 0;
  return (int)n;
}

/* 丢弃 ring 里积压的残留/上报字节(非目标从机开头的数据)。
 * 在读位置、读电机回复这类「按定长取帧」前调用，避免脏数据占位导致帧错位、
 * CRC 校验失败。不取锁：由调用方在已经持有 uart1_bus_lock 的事务内调用。 */
void uart1_drain_stale_bytes(void)
{
  uart1_reporting_len = 0;
  uart1_rx.tail = uart1_rx.head;
}

/* USER CODE END 1 */
