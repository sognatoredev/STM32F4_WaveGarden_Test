/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"

volatile uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];    // uart2 수신 버퍼
volatile uint8_t uart6_rx_buf[UART6_RX_BUF_SIZE];    // uart6 수신 버퍼

uint8_t uart_data_ready = 0;               // 데이터 수신 완료 플래그
uint16_t uart_rx_length = 0;               // 수신된 실제 길이

uint8_t uart6_data_ready = 0;               // 데이터 수신 완료 플래그
uint16_t uart6_rx_length = 0;               // 수신된 실제 길이

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

typedef enum
{
  ESP32_READY = 0,
  ESP32_IDLE = 1,
  ESP32_BUSY = 2,
  ESP32_RESET = 3,
} ESP32_State;

ESP32_State ESP32_CurrentState;

typedef struct 
{
  const uint8_t * cmd;
  void (*func)(uint8_t* pData);
  uint8_t cmp_size;
  const uint8_t * message;
} Command;

Command cmd_table[] = 
{
  { "\r\nready\r\n", UART6_Print, 9, (uint8_t *)"ESP status ready.\r\n"},
  { "LED_ON\r\n", UART6_Print, 8, (uint8_t *)"Turning LED on\r\n"},
  { "LED_OFF\r\n", UART6_Print, 9, (uint8_t *)"Turning LED off\r\n"},
  { "AT version", UART6_Print, 10, (uint8_t *)"Device Info Print Out.\r\n"},
  { NULL, NULL, NULL, NULL},
  // { "ESP AT\r\n", ESP32_Print, (uint8_t *)"AT\r\n"},
};

Command esp_cmd_table[] =
{
  { "ESP AT\r\n", ESP32_Print, 8, (uint8_t *)"AT\r\n"},
  { "ESP ATGMR\r\n", ESP32_Print, 11, (uint8_t *)"AT+GMR\r\n"},
  { NULL, NULL, NULL, NULL},
};


uint8_t UART2_Print(uint8_t * pData)
{
  if ((uart_data_ready == 1) && (uart_rx_length != strlen(pData)))
  {
    HAL_UART_Transmit(&huart2, pData, uart_rx_length, 100);
  }
  else
  {
    HAL_UART_Transmit(&huart2, pData, strlen(pData), 100);
  }
}

void UART6_Print(uint8_t * pData)
{
  // if ((uart6_data_ready == 1) && (uart6_rx_length != strlen(pData)))
  // {
  //   HAL_UART_Transmit(&huart2, pData, uart6_rx_length, 100);
  // }
  // else
  // {
  if (ESP32_CurrentState == ESP32_RESET)
  {
    ESP32_CurrentState = ESP32_IDLE;
    
    if (HAL_UART_Transmit(&huart2, pData, strlen((uint8_t *) cmd_table[0].message), 1000) != HAL_OK)
    {
      memset((uint8_t *) uart6_rx_buf, NULL, strlen(uart6_rx_buf));
    }
  }
  else
  {
    if (HAL_UART_Transmit(&huart2, pData, strlen(pData), 1000) != HAL_OK)
    {
      // memset((uint8_t *) uart6_rx_buf, NULL, strlen(uart6_rx_buf));
    }
  }
  
  // HAL_UART_Transmit(&huart2, pData, uart6_rx_length, 100);
  // }
}

void ESP32_Print(uint8_t * pData)
{
  // if ((uart6_data_ready == 1) && (uart6_rx_length != strlen(pData)))
  // {
  //   HAL_UART_Transmit(&huart2, pData, uart6_rx_length, 100);
  // }
  // else
  // {
  memset((uint8_t *) uart6_rx_buf, 0, strlen(uart6_rx_buf));
  HAL_UART_Transmit(&huart6, pData, strlen(pData), 1000);
  // }
}

#if 0
uint8_t UART6_RxDataParsing (uint8_t * pData)
{
  if (!(strncmp(pData, "\r\nready\r\n", 9))) 
  {
    UART6_Print("ESP32 status ready.\r\n");
      // led_on();
  } 
  else if (!(strncmp(pData, "LED_OFF", 7))) 
  {
    UART6_Print("LED Off.\r\n");
      // led_off();
  } 
  else if (!(strncmp(pData, "GET_TEMP", 8))) 
  {
    UART6_Print("Unknown command.\r\n");
      // get_temperature();
  } 
  else 
  {
      UART6_Print("Unknown command.\r\n");
  }
}
#else

uint8_t UART6_RxDataParsing (uint8_t * pData)
{
  for (int i = 0; cmd_table[i].cmd != NULL; i++) 
  {
    // if (!(strcmp(pData, cmd_table[i].cmd))) 
    if (!(strncmp(pData, cmd_table[i].cmd, cmd_table[i].cmp_size))) 
    {
      if (i == 0)
      {
        ESP32_CurrentState = ESP32_RESET;
        // memset((uint8_t *) uart6_rx_buf, 0, strlen(uart6_rx_buf));
      }
      cmd_table[i].func((uint8_t *)cmd_table[i].message);
      // memset((uint8_t *) uart6_rx_buf, NULL, strlen(uart6_rx_buf));
      return;
    }
  }
}

uint8_t UART2_RxDataParsing (uint8_t * pData)
{
  for (int i = 0; esp_cmd_table[i].cmd != NULL; i++) 
  {
    // if (!(strcmp(pData, esp_cmd_table[i].cmd))) 
    if (!(strncmp(pData, esp_cmd_table[i].cmd, esp_cmd_table[i].cmp_size))) 
    {
      esp_cmd_table[i].func((uint8_t *)esp_cmd_table[i].message);
      return;
    }
  }
}
#endif

uint8_t UART2_Process (void)
{
  if (prvLED2_Status != LED2_Status)
  {
    prvLED2_Status = LED2_Status;
    HAL_UART_Transmit(&huart2, "LED2 Toggle.\r\n", 14, 100);
  }

  if (uart_data_ready)
  {
      // 수신된 데이터 처리
      UART2_Print(uart2_rx_buf);    // uart2 수신 버퍼

      UART2_RxDataParsing(uart2_rx_buf);

      uart_rx_length = 0;
      uart_data_ready = 0;
      // DMA 재시작
      // HAL_UART_DMAStop(&huart2);  // 수신 중단
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf, UART2_RX_BUF_SIZE);
      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }

  return ;
}

uint8_t UART6_Process (void)
{
  if (uart6_data_ready)
  {
    UART6_RxDataParsing(uart6_rx_buf);

    // 수신된 데이터 처리
    UART6_Print(uart6_rx_buf);    // uart2 수신 버퍼

    

    
    uart6_rx_length = 0;
    uart6_data_ready = 0;

    // memset((uint8_t *) uart6_rx_buf, 0, uart6_rx_length);
    // DMA 재시작
    // HAL_UART_DMAStop(&huart2);  // 수신 중단
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *) uart6_rx_buf, UART6_RX_BUF_SIZE);
    // __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  }

  return ;
}

/* USER CODE END 1 */
