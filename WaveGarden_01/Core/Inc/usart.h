/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

#define UART2_RX_BUF_SIZE 128
#define UART6_RX_BUF_SIZE 128

extern volatile uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];    // uart2 수신 버퍼
extern volatile uint8_t uart6_rx_buf[UART6_RX_BUF_SIZE];    // uart3 수신 버퍼

extern uint8_t uart_data_ready;               // 데이터 수신 완료 플래그
extern uint16_t uart_rx_length;               // 수신된 실제 길이

extern uint8_t uart6_data_ready;               // 데이터 수신 완료 플래그
extern uint16_t uart6_rx_length;               // 수신된 실제 길이

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t UART2_Print(uint8_t * pData);
extern uint8_t UART6_Print(uint8_t * pData);
extern uint8_t UART2_Process (void);
extern uint8_t UART6_Process (void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

