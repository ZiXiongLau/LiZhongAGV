/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "stm32f7xx_ll_dma.h"
#include <stdio.h>
#include <stdarg.h>
#include "CRC8.h"
#include "delay.h"
#include "motor_control.h"
#include <motor485_driven.h>
#include "tim.h"
#ifdef SD_RW_ENABLE
#include "rtc.h"
#include "fatfs.h"
#endif
#include "battery.h"
#include <motor_driven.h>
#include "flash_access.h"
#include "adc.h"
//#include "nav.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//#define RT_USART_PRINTF_BUFFER_LEN  128
//static char gUartPrintfBuffer[RT_USART_PRINTF_BUFFER_LEN];
int gUartPrintfFlag = 1;    //串口打印标志
uint32_t gUartNumWifiPrint; //串口打印序号
static uint8_t canOpenObjectAccessFlag = 0;
static uint8_t canOpenId = 0x00;

uint8_t gUsartWifiRevBuf[USART1_REV_BUFFER_LEN];
ST_UART_RECEIVE_DATA gStUsartWifiRevData = {UART_RECEIVE_STATE_IDLE, 0, 0, 0, gUsartWifiRevBuf};

#if defined(RT_USING_UART1)
ST_USART_DATA gStUart1Data;
DMA_HandleTypeDef gHUart1RxDma;
DMA_HandleTypeDef gHUart1TxDma;

#if (USART1_SEND_BUFFER_LEN != 0)
    uint8_t uart1_send_buffer[USART1_SEND_BUFFER_LEN];
#endif
#if (USART1_REV_BUFFER_LEN != 0)
    uint8_t uart1_rev_buffer[USART1_REV_BUFFER_LEN];
#endif
#endif


#if defined(RT_USING_UART2)
ST_USART_DATA gStUart2Data;
DMA_HandleTypeDef gHUart2RxDma;
DMA_HandleTypeDef gHUart2TxDma;

#if (USART2_SEND_BUFFER_LEN != 0)
    uint8_t uart2_send_buffer[USART2_SEND_BUFFER_LEN];
#endif
#if (USART2_REV_BUFFER_LEN != 0)
    uint8_t uart2_rev_buffer[USART2_REV_BUFFER_LEN];
#endif
#endif

#if defined(RT_USING_UART4)
ST_USART_DATA gStUart4Data;
DMA_HandleTypeDef gHUart4RxDma;

#if (USART4_SEND_BUFFER_LEN != 0)
    uint8_t uart4_send_buffer[USART4_SEND_BUFFER_LEN];
#endif
#if (USART4_REV_BUFFER_LEN != 0)
    uint8_t uart4_rev_buffer[USART4_REV_BUFFER_LEN];
#endif
#endif

#if defined(RT_USING_UART5)
ST_USART_DATA gStUart5Data;
DMA_HandleTypeDef gHUart5RxDma;
DMA_HandleTypeDef gHUart5TxDma;

#if (USART5_SEND_BUFFER_LEN != 0)
    uint8_t uart5_send_buffer[USART5_SEND_BUFFER_LEN];
#endif
#if (USART5_REV_BUFFER_LEN != 0)
    uint8_t uart5_rev_buffer[USART5_REV_BUFFER_LEN];
#endif
#endif

#if defined(RT_USING_UART6)
ST_USART_DATA gStUart6Data;
DMA_HandleTypeDef gHUart6RxDma;
DMA_HandleTypeDef gHUart6TxDma;

#if (USART6_SEND_BUFFER_LEN != 0)
    uint8_t uart6_send_buffer[USART6_SEND_BUFFER_LEN];
#endif
#if (USART6_REV_BUFFER_LEN != 0)
    uint8_t uart6_rev_buffer[USART6_REV_BUFFER_LEN];
#endif
#endif


#if defined(RT_USING_UART7)
ST_USART_DATA gStUart7Data;
DMA_HandleTypeDef gHUart7RxDma;

#if (USART7_SEND_BUFFER_LEN != 0)
    uint8_t uart7_send_buffer[USART7_SEND_BUFFER_LEN];
#endif
#if (USART7_REV_BUFFER_LEN != 0)
    uint8_t uart7_rev_buffer[USART7_REV_BUFFER_LEN];
#endif
#endif

Usart_msg rxUsart5Msg;                 //串口5接收数据结构体

static void UsartTxDmaConfiguration(ST_USART_DATA* uart);
static void UsartRxDmaConfiguration(ST_USART_DATA* usart);
static void SetUsartTo485Direction(ST_USART_DATA *uart, Direction485 direct);

extern osTimerId myMotorTestTimerHandle;

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 100000;
  huart4.Init.WordLength = UART_WORDLENGTH_9B;
  huart4.Init.StopBits = UART_STOPBITS_2;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT|UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart4.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart5.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
  huart6.Init.BaudRate = 460800;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart6.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */
#if defined(RT_USING_UART4)
    huart4.hdmarx = &gHUart4RxDma;
    gStUart4Data.uart_device = &huart4;
    gStUart4Data.flag = 0;
    gStUart4Data.tx_trans_flag = 0;
    gStUart4Data.sendPushIn = 0;
    gStUart4Data.sendPopOut = 0;
    gStUart4Data.revPopOut = 0;
    gHUart4RxDma.Instance = DMA1_Stream2;
    gHUart4RxDma.Init.Channel = DMA_CHANNEL_4;    
    gStUart4Data.sendBufSize = USART4_SEND_BUFFER_LEN;
    gStUart4Data.revBufSize = USART4_REV_BUFFER_LEN;
    gStUart4Data.revPopOut = 0;
#if (USART4_SEND_BUFFER_LEN != 0)
    gStUart4Data.sendBuf = uart4_send_buffer;
#endif
#if (USART4_REV_BUFFER_LEN != 0)
    gStUart4Data.revBuf =  uart4_rev_buffer;
#endif
#endif
  /* USER CODE END UART4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA11     ------> UART4_RX
    PA12     ------> UART4_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UART4_MspInit 1 */
    UsartRxDmaConfiguration(&gStUart4Data);
  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */
#if defined(RT_USING_UART5)
      huart5.hdmarx = &gHUart5RxDma;
      huart5.hdmatx = &gHUart5TxDma;
      gStUart5Data.uart_device = &huart5;

      if (IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG))
      {
        //与工控机的串口通讯暂时不考虑开启dma发送(若开启dma发送，需要使能相应的dma流传输完成中断和串口发送完成中断，为避免占用中断资源，通讯交互速率不满足时，才考虑开启)
        gStUart5Data.flag = ENABLE_485_DIR /*| USE_SEND_DMA*/; 
      }
      else
      {
        gStUart5Data.flag = ENABLE_485_DIR;
      }
      gStUart5Data.tx_trans_flag = 0;
      gStUart5Data.sendPushIn = 0;
      gStUart5Data.sendPopOut = 0;
      gStUart5Data.revPopOut = 0;
      gStUart5Data.revLenPopOut = 0;
      gHUart5RxDma.Instance = DMA1_Stream0;
      gHUart5RxDma.Init.Channel = DMA_CHANNEL_4; 
      gHUart5TxDma.Instance = DMA1_Stream7;
      gHUart5TxDma.Init.Channel = DMA_CHANNEL_4;
      gStUart5Data.sendBufSize = USART5_SEND_BUFFER_LEN;
      gStUart5Data.revBufSize = USART5_REV_BUFFER_LEN;
      gStUart5Data.revPopOut = 0;
#if (USART5_SEND_BUFFER_LEN != 0)
      gStUart5Data.sendBuf = uart5_send_buffer;
#endif
#if (USART5_REV_BUFFER_LEN != 0)
      gStUart5Data.revBuf =  uart5_rev_buffer;
#endif
#endif
    SetUsartTo485Direction(&gStUart5Data, DIRECTION485_RECEIVE);
  /* USER CODE END UART5_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PB12     ------> UART5_RX
    PB13     ------> UART5_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_TX Init */
    hdma_uart5_tx.Instance = DMA1_Stream7;
    hdma_uart5_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_tx.Init.Mode = DMA_NORMAL;
    hdma_uart5_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart5_tx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */
    UsartRxDmaConfiguration(&gStUart5Data);

    //注意此处不能使能串口DMA的发送器，否则会导致FIFO Error Interrupt Flag;调用标准函数HAL_UART_Transmit_DMA()中会进行相应地使能
    //正规使能流程:1.配置合适的DMA数据流;2.使能DMA数据流(DMA_SxCR);3.使能相关的外设(USARTx_CR3)。详细原因见文档AN4031 4.3节
    //LL_USART_EnableDMAReq_TX(gStUart5Data.uart_device->Instance); 
    
  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
#if defined(RT_USING_UART1)
        huart1.hdmatx = &gHUart1TxDma;
        huart1.hdmarx = &gHUart1RxDma;
        gStUart1Data.uart_device = &huart1;
        gStUart1Data.flag = USE_SEND_DMA;
        gStUart1Data.tx_trans_flag = 0;
        gStUart1Data.sendPushIn = 0;
        gStUart1Data.sendPopOut = 0;
        gStUart1Data.revPopOut = 0;
        gHUart1TxDma.Instance = DMA2_Stream7;
        gHUart1TxDma.Init.Channel = DMA_CHANNEL_4;
        gHUart1RxDma.Instance = DMA2_Stream2;
        gHUart1RxDma.Init.Channel = DMA_CHANNEL_4;    
        gStUart1Data.sendBufSize = USART1_SEND_BUFFER_LEN;
        gStUart1Data.revBufSize = USART1_REV_BUFFER_LEN;
        gStUart1Data.revPopOut = 0;
#if (USART1_SEND_BUFFER_LEN != 0)
        gStUart1Data.sendBuf = uart1_send_buffer;
#endif
#if (USART1_REV_BUFFER_LEN != 0)
        gStUart1Data.revBuf =  uart1_rev_buffer;
#endif
#endif
  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
    UsartTxDmaConfiguration(&gStUart1Data);
    UsartRxDmaConfiguration(&gStUart1Data);
    //gUart1InitFlag = 1;
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
#if defined(RT_USING_UART2)
      huart2.hdmatx = &gHUart2TxDma;
      huart2.hdmarx = &gHUart2RxDma;
      gStUart2Data.uart_device = &huart2;
      gStUart2Data.flag =  ENABLE_485_DIR;
      gStUart2Data.tx_trans_flag = 0;
      gStUart2Data.sendPushIn = 0;
      gStUart2Data.sendPopOut = 0;
      gStUart2Data.revPopOut = 0;
      gHUart2TxDma.Instance = DMA1_Stream6;
      gHUart2TxDma.Init.Channel = DMA_CHANNEL_4;
      gHUart2RxDma.Instance = DMA1_Stream5;
      gHUart2RxDma.Init.Channel = DMA_CHANNEL_4;    
      gStUart2Data.sendBufSize = USART2_SEND_BUFFER_LEN;
      gStUart2Data.revBufSize = USART2_REV_BUFFER_LEN;
      gStUart2Data.revPopOut = 0;
#if (USART2_SEND_BUFFER_LEN != 0)
      gStUart2Data.sendBuf = uart2_send_buffer;
#endif
#if (USART2_REV_BUFFER_LEN != 0)
      gStUart2Data.revBuf =  uart2_rev_buffer;
#endif
#endif
    SetUsartTo485Direction(&gStUart2Data, DIRECTION485_RECEIVE);
  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

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

  /* USER CODE BEGIN USART2_MspInit 1 */
    UsartRxDmaConfiguration(&gStUart2Data);
    //LL_USART_EnableDMAReq_TX(gStUart2Data.uart_device->Instance);   //使能DMA
  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */
#if defined(RT_USING_UART6)
      huart6.hdmatx = &gHUart6TxDma;
      huart6.hdmarx = &gHUart6RxDma;
      gStUart6Data.uart_device = &huart6;
      gStUart6Data.flag = USE_SEND_DMA;
      gStUart6Data.tx_trans_flag = 0;
      gStUart6Data.sendPushIn = 0;
      gStUart6Data.sendPopOut = 0;
      gStUart6Data.revPopOut = 0;
      gHUart6TxDma.Instance = DMA2_Stream6;
      gHUart6TxDma.Init.Channel = DMA_CHANNEL_5;
      gHUart6RxDma.Instance = DMA2_Stream1;
      gHUart6RxDma.Init.Channel = DMA_CHANNEL_5;    
      gStUart6Data.sendBufSize = USART6_SEND_BUFFER_LEN;
      gStUart6Data.revBufSize = USART6_REV_BUFFER_LEN;
      gStUart6Data.revPopOut = 0;
#if (USART6_SEND_BUFFER_LEN != 0)
      gStUart6Data.sendBuf = uart6_send_buffer;
#endif
#if (USART6_REV_BUFFER_LEN != 0)
      gStUart6Data.revBuf =  uart6_rev_buffer;
#endif
#endif

  /* USER CODE END USART6_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
    PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

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

  /* USER CODE BEGIN USART6_MspInit 1 */
    UsartTxDmaConfiguration(&gStUart6Data);
    UsartRxDmaConfiguration(&gStUart6Data);
  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA11     ------> UART4_RX
    PA12     ------> UART4_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PB12     ------> UART5_RX
    PB13     ------> UART5_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
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
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

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

  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*****************************************************************************
 功能描述  : 串口发送DMA初始化
 输入参数  : ST_USART_DATA* usart
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月9日
*****************************************************************************/
static void UsartTxDmaConfiguration(ST_USART_DATA* uart)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    uart->uart_device->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    uart->uart_device->hdmatx->Init.PeriphInc = DMA_PINC_DISABLE;
    uart->uart_device->hdmatx->Init.MemInc = DMA_MINC_ENABLE;
    uart->uart_device->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uart->uart_device->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uart->uart_device->hdmatx->Init.Mode = DMA_NORMAL;
    uart->uart_device->hdmatx->Init.Priority = DMA_PRIORITY_LOW;
    uart->uart_device->hdmatx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(uart->uart_device->hdmatx) != HAL_OK)
    {
      Error_Handler();
    }
}
/*****************************************************************************
 功能描述  : 串口接收DMA初始化
 输入参数  : ST_USART_DATA* usart
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月9日
*****************************************************************************/
static void UsartRxDmaConfiguration(ST_USART_DATA* uart) 
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    uart->uart_device->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    uart->uart_device->hdmarx->Init.PeriphInc = DMA_PINC_DISABLE;
    uart->uart_device->hdmarx->Init.MemInc = DMA_MINC_ENABLE;
    uart->uart_device->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uart->uart_device->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uart->uart_device->hdmarx->Init.Mode = DMA_CIRCULAR;
    uart->uart_device->hdmarx->Init.Priority = DMA_PRIORITY_LOW;
    uart->uart_device->hdmarx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(uart->uart_device->hdmarx) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_DMA_Start(uart->uart_device->hdmarx, (uint32_t)&uart->uart_device->Instance->RDR, (uint32_t)uart->revBuf, uart->revBufSize);

    LL_USART_EnableDMAReq_RX(uart->uart_device->Instance);  //使能DMA
}
/*****************************************************************************
 功能描述  : 获取串口设备指针
 输入参数  : uint32_t deviceNum  串口设备号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月9日
*****************************************************************************/
ST_USART_DATA* UsartGetSelectDevice(uint32_t deviceNum)
{
#if defined(RT_USING_UART1)
    if(USART1_DEVICE == deviceNum)
    {
        return &gStUart1Data;
    }
#endif 

#if defined(RT_USING_UART2)
    if(USART2_DEVICE == deviceNum)
    {
        return &gStUart2Data;
    }
#endif 

#if defined(RT_USING_UART4)
    if(USART4_DEVICE == deviceNum)
    {
        return &gStUart4Data;
    }
#endif 

#if defined(RT_USING_UART5)
    if(USART5_DEVICE == deviceNum)
    {
        return &gStUart5Data;
    }
#endif 

#if defined(RT_USING_UART6)
    if(USART6_DEVICE == deviceNum)
    {
        return &gStUart6Data;
    }
#endif

#if defined(RT_USING_UART7)
    if(USART7_DEVICE == deviceNum)
    {
        return &gStUart7Data;
    }
#endif

    return NULL;
}
/*****************************************************************************
 功能描述  : 用户配置功能串初始化
 输入参数  : 无
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年06月16日
*****************************************************************************/
void AppUsartInit(void)
{
    if (IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG))
    {
        rxUsart5Msg.inIndex = 0;                           //队首索引初始化
        rxUsart5Msg.outIndex = 0;                          //队尾索引初始化
        
        HAL_UART_ReceiverTimeout_Config(&huart5, 35);      //配置接收器超时值，modbus规定命令间隔为3.5字符，等于3.5*10(1个起始位，8个数据位，0个检验位，1个停止位)
        HAL_UART_EnableReceiverTimeout(&huart5);           //使能接收器超时功能
        __HAL_UART_ENABLE_IT(&huart5, UART_IT_RTO);        //使能接收器超时中断

        if (gStUart5Data.flag & USE_SEND_DMA)
        {
            //DMA流传输完成中断触发后的回调UART_DMATransmitCplt()会执行SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);此处使不使能均可以，为便于可读性，此处进行使能
            __HAL_UART_ENABLE_IT(&huart5, UART_IT_TC);     //使能发送完成中断
        }
    }
}
/*****************************************************************************
 功能描述  : 串口中断处理
 输入参数  : UART_HandleTypeDef *huart 串口句柄
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年06月16日
*****************************************************************************/
void AppUsart_ISRProc(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RTOF) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RTOF);
        
        rxUsart5Msg.rxCount[rxUsart5Msg.inIndex]= GetUsartRxDmaDataLength(USART5_DEVICE);   //获取当前中断录入dma的新数据长度

        //HAL_UART_DMAStop(&huart5);   //DMA循环模式，不需关闭再开启
        USARTRx_ENQUEUE(rxUsart5Msg);        // 串口5接收数据入队
    }
    /*else if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) != RESET)
    {
        SetUsartTo485Direction(&gStUart5Data, DIRECTION485_RECEIVE);
        //注意此处不能清除发送完成标志，否则会导致无法进入HAL_UART_IRQHandler()中的UART_EndTransmit_IT()执行restore huart->gState to Ready，从而导致dma只能发送一次
        //__HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_TC);    
    }*/
}
/*****************************************************************************
 功能描述  : 串口发送完成中断回调函数
 输入参数  : UART_HandleTypeDef *huart  串口句柄
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年06月21日
*****************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);                      //清除发送串口完成中断标志

    if(gStUart5Data.flag & ENABLE_485_DIR) //使用485方向
    {
        SetUsartTo485Direction(&gStUart5Data, DIRECTION485_RECEIVE);     //发送完成后，转换RS485的方向
    }
}

/*****************************************************************************
 功能描述  : 判断接收缓冲区是否为空
 输入参数  : Usart_msg msg    消息
 输出参数  : 0:缓冲区非空
             1:缓冲区空
 作    者  : 田忠
 日    期  : 2023年06月16日
*****************************************************************************/
uint8_t IsRxBuffEmpty(Usart_msg msg)
{
	return (msg.inIndex== msg.outIndex);
}

/*****************************************************************************
 功能描述  : 判断dma发送完成
 输入参数  : ST_USART_DATA *uart  串口设备
 输出参数  : uint32_t   1表示发送完成
 作    者  : 刘鹏
 日    期  : 2020年12月21日
*****************************************************************************/
uint32_t isDmaSendFinish(ST_USART_DATA *uart)
{
    if(NULL == uart)
    {
        return 0;
    }
    
    if(uart->flag & USE_SEND_DMA)   //使用dma发送
    {
        if((HAL_DMA_STATE_RESET == HAL_DMA_GetState(uart->uart_device->hdmatx))
            || (HAL_DMA_STATE_READY == HAL_DMA_GetState(uart->uart_device->hdmatx)))
        {
            return 1;
        }
        else if(DMA2_Stream6 == uart->uart_device->hdmatx->Instance)
        {
            if(LL_DMA_IsActiveFlag_TC6(DMA2))
            {
                return 1;
            }
        }
        else if(DMA2_Stream7 == uart->uart_device->hdmatx->Instance)
        {
            if(LL_DMA_IsActiveFlag_TC7(DMA2))
            {
                return 1;
            }
        }
    }
    else
    {
        return 1;
    }
    
    return 0;
}
/*****************************************************************************
 功能描述  : 读取串口DMA当前中断接收的数据长度
 输入参数  : uint32_t deviceNum 串口号
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年6月19日
*****************************************************************************/
uint32_t GetUsartRxDmaDataLength(uint32_t deviceNum)
{
    ST_USART_DATA* uart = UsartGetSelectDevice(deviceNum);
    
    uint32_t lPushIn;

    uint32_t lValidSize = 0;
 
    lPushIn = READ_BIT(uart->uart_device->hdmarx->Instance->NDTR, DMA_SxNDT);  // DMA未填充数目
 
    if(lPushIn > uart->revBufSize)
    {
        lPushIn = uart->revBufSize;
    }
    
    lPushIn = (uart->revBufSize - lPushIn) % uart->revBufSize;                 // DMA已填充数目

    if(lPushIn >= uart->revLenPopOut)                                          //获取当前缓冲区未处理数据量
    {
        lValidSize = lPushIn - uart->revLenPopOut;
    }
    else
    {
        lValidSize = lPushIn + uart->revBufSize - uart->revLenPopOut;
    }
    uart->revLenPopOut += lValidSize;
    uart->revLenPopOut %= uart->revBufSize;                                    // 更新队尾索引
    
    return lValidSize;
}

/*****************************************************************************
 功能描述  : 串口读取
 输入参数  : uint32_t deviceNum  设备号
             void *buffer        缓冲区
             uint32_t size       读取大小
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月9日
*****************************************************************************/
uint32_t UsartDeviceRead(uint32_t deviceNum, void *buffer, uint32_t size)
{
    ST_USART_DATA* uart = UsartGetSelectDevice(deviceNum);
    
    uint32_t lSizeTemp = 0, lPushIn;

    uint32_t lValidSize = 0;
 
    if((NULL == uart) || (uart->revBufSize == 0) || (NULL == uart->revBuf))
    {
        return 0;
    }

    lPushIn = READ_BIT(uart->uart_device->hdmarx->Instance->NDTR, DMA_SxNDT);  // DMA未填充数目
 
	lSizeTemp = uart->revBufSize - uart->revPopOut;                            // 接收缓冲区剩余空间

    //接收故障判断，DMA传输错误，则重新初始化
    /*if(SET == DMA_GetFlagStatus(uart->rx_trans_error_flag))
    {
        UsartDmaConfiguration(uart);
        return 0;
    }*/

    if(lPushIn > uart->revBufSize)
    {
        lPushIn = uart->revBufSize;
    }
    
    lPushIn = (uart->revBufSize - lPushIn) % uart->revBufSize;                 // DMA已填充数目

    if((NULL == buffer) || (0 == size)) //清空缓冲区
    {
        uart->revPopOut = lPushIn;
        return 0;
    }

    if(lPushIn >= uart->revPopOut)                                             //获取当前缓冲区未处理数据量
    {
        lValidSize = lPushIn - uart->revPopOut;
    }
    else
    {
        lValidSize = lPushIn + uart->revBufSize - uart->revPopOut;
    }

    if(lValidSize > 0)
    {
        if(lValidSize > size)
        {
            lValidSize = size;
        }
        if(lSizeTemp >= lValidSize)
        {
            memcpy((uint8_t*)buffer, &(uart->revBuf[uart->revPopOut]), lValidSize);      // 剩余空间大于有效字节数，直接复制
        }
        else
        {
            memcpy((uint8_t*)buffer, &(uart->revBuf[uart->revPopOut]), lSizeTemp);
            memcpy((uint8_t*)buffer + lSizeTemp, uart->revBuf, lValidSize - lSizeTemp);  // 剩余空间小于有效字节数，环形复制
        }
        uart->revPopOut += lValidSize;
        uart->revPopOut %= uart->revBufSize;                                             // 更新队尾索引
    }    

    return lValidSize;
}
/*****************************************************************************
 功能描述  : 建立dma串口传输，发送串口数据
 输入参数  : ST_USART_DATA *uart  串口设备
             uint32_t memoryBaseAddr   内存地址
             uint32_t bufferSize       内存大小
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月21日
*****************************************************************************/
static void UsartDmaSendData(ST_USART_DATA *uart, uint32_t memoryBaseAddr, uint32_t bufferSize)
{ 
    if(HAL_DMA_GetState(uart->uart_device->hdmatx) == HAL_DMA_STATE_BUSY)
    {
        HAL_DMA_Abort(uart->uart_device->hdmatx);
    }
        
    HAL_DMA_Start(uart->uart_device->hdmatx, memoryBaseAddr, (uint32_t)&uart->uart_device->Instance->TDR, bufferSize);

    LL_USART_EnableDMAReq_TX(uart->uart_device->Instance);
}
/*****************************************************************************
 功能描述  : 向串口写数据
 输入参数  : uint32_t deviceNum       串口设备号
             const void *buffer  待写数据
             uint32_t size            待写长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月21日
*****************************************************************************/
uint32_t UsartDeviceWrite(uint32_t deviceNum, const void *buffer, uint32_t size)
{
    uint32_t lSizeTemp = 0, breakFlag = 0;
    uint32_t tickstart, curTick;
    HAL_StatusTypeDef lStatus = HAL_OK;
    ST_USART_DATA *uart = UsartGetSelectDevice(deviceNum);

    if((NULL == uart) || (0 == size))
    {
        return 0;
    }

    if(uart->flag & ENABLE_485_DIR) //使用485方向
    {
        if(uart->dir != DIRECTION485_TRANSMISSION)
        {
            SetUsartTo485Direction(uart, DIRECTION485_TRANSMISSION);
            delay_us(10);
        }
    }

    if(uart->flag & USE_SEND_DMA)   //使用dma发送
    {
        if((size >= uart->sendBufSize) || (NULL == uart->sendBuf))
        {
            return 0;
        }
        
        /*if(uart->config.useSendFifoFlag)//先存入缓冲区，然后轮询从缓冲区通过dma发送
        {
            lEmptySize = uart->sendBufSize - UsartSendBufferValidSize(uart);
            lSizeTemp = uart->sendBufSize - uart->sendPushIn;
            if(size < lEmptySize)
            {
                if(lSizeTemp >= size)
                {
                    memcpy(&(uart->sendBuf[uart->sendPushIn]), (uint8_t*)buffer, size);        
                }
                else
                {
                    memcpy(&(uart->sendBuf[uart->sendPushIn]), (uint8_t*)buffer, lSizeTemp);
                    memcpy(uart->sendBuf, (uint8_t*)buffer + lSizeTemp, size - lSizeTemp);
                }
                uart->sendPushIn += size;
                uart->sendPushIn %= uart->sendBufSize;
            }
        }
        else*///不写缓冲区，直接通过DMA发送
        {
            if ((IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG)) && (USART5_DEVICE == deviceNum))
            {
                tickstart = HAL_GetTick();
                while(HAL_DMA_GetState(&hdma_uart5_tx) != HAL_DMA_STATE_READY)
                {
                    /* Check for the Timeout */
                    curTick = HAL_GetTick() - tickstart;
                    if(curTick >= 3)
                    {
                        break;
                    }
                }
                lStatus = HAL_UART_Transmit_DMA(uart->uart_device, (uint8_t*)buffer, size);
                if(DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_87)
                {
                    if (HAL_OK != lStatus)
                    {
                        rt_kprintf("HAL_UART_Transmit_DMA return status:%d.\r\n", lStatus);
                    }
                }
            }
            else
            {
                tickstart = HAL_GetTick();
                while(0 == isDmaSendFinish(uart))
                {
                    /* Check for the Timeout */
                    curTick = HAL_GetTick() - tickstart;
                    if(curTick >= 5)
                    {
                        break;
                    }
                }
                memcpy((uint8_t*)(uart->sendBuf), (uint8_t*)buffer, size);
                UsartDmaSendData(uart, (uint32_t)(uart->sendBuf), size);
            }
            

            //为保证数据完整发送，485方向放置串口发送完成中断里转换
            /*if(uart->flag & ENABLE_485_DIR) //使用485方向
            {
                if(uart->dir != DIRECTION485_RECEIVE)
                {
                    while (gStUart5Data.tx_trans_flag)
                    {
                        ;
                    }
                    {
                        tickstart = HAL_GetTick();
                        while (!(LL_USART_IsActiveFlag_TC(uart->uart_device->Instance)))
                        {
                            // Check for the Timeout
                            curTick = HAL_GetTick() - tickstart;
                            if(curTick >= 5)
                            {
                                break;
                            }
                        }
                        delay_us(10);
                        SetUsartTo485Direction(uart, DIRECTION485_RECEIVE);
                        UsartDeviceRead(deviceNum, NULL, 0);    //清空缓冲区
                    }
                }
            }*/
        }
    }
    else//不使用dma发送
    {
        lSizeTemp = 0; 
        while(lSizeTemp < size)
        {        
            uart->uart_device->Instance->TDR = *((uint8_t*)buffer + lSizeTemp);
            /* Get tick */ 
            tickstart = HAL_GetTick();
            while (!(LL_USART_IsActiveFlag_TXE(uart->uart_device->Instance)))
            {
                /* Check for the Timeout */
                curTick = HAL_GetTick() - tickstart;
                if(curTick > 4)
                {
                    breakFlag = 1;
                    break;
                }
            }
            if(breakFlag)
            {
                break;
            }
            lSizeTemp++;
        }
        if(uart->flag & ENABLE_485_DIR) //使用485方向
        {
            if(uart->dir != DIRECTION485_RECEIVE)
            {
                tickstart = HAL_GetTick();
                while (!(LL_USART_IsActiveFlag_TC(uart->uart_device->Instance)))
                {
                    /* Check for the Timeout */
                    curTick = HAL_GetTick() - tickstart;
                    if(curTick >= 5)
                    {
                        break;
                    }
                }
                delay_us(10);
                SetUsartTo485Direction(uart, DIRECTION485_RECEIVE);
                if (!((IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG)) && (USART5_DEVICE == deviceNum))) //与工控机通信的串口号不进行缓冲区清空处理
                {
                    UsartDeviceRead(deviceNum, NULL, 0);    //清空缓冲区
                }
            }
        }
    }
    
    return lSizeTemp;
}
/*****************************************************************************
 功能描述  : 设定串口转485数据传输方向
 输入参数  : uint32_t deviceNum  串口序号
             Direction485 direct    传输方向
 输出参数  : 无
 作    者  :
 日    期  : 2020年4月17日
*****************************************************************************/
static void SetUsartTo485Direction(ST_USART_DATA *uart, Direction485 direct)
{
    uart->dir = direct;
    
    if(USART2 == uart->uart_device->Instance)
    {
        if(gStUfoData.flag & UFO_KINCO_NEW_PCB) //步科驱动器使用新pcb控制
        {
            if(direct)
            {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
            }
            else
            { 
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
            }
        }
        else
        {
            if(direct)
            {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
            }
            else
            { 
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
            }
        }
    }
    else if(UART5 == uart->uart_device->Instance)
    {
        if(direct)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
        else
        { 
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
    }
}

/*****************************************************************************
 功能描述  : 调试数据打印
 输入参数  : const char *fmt
             ...              
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月24日
*****************************************************************************/
/*void rt_kprintf(const char *fmt, ...)
{
    va_list args;
    uint32_t length;

    if(!gUart1InitFlag)
    {
        return;
    }

    LockThread();

    va_start(args, fmt);
    
    length = vsnprintf(gUartPrintfBuffer, RT_USART_PRINTF_BUFFER_LEN - 1, fmt, args);
    
    if (length > RT_USART_PRINTF_BUFFER_LEN - 1)
    {
        length = RT_USART_PRINTF_BUFFER_LEN - 1;
    }

    UsartDeviceWrite(USART1_DEVICE, gUartPrintfBuffer, length);

    va_end(args);

    UnLockThread();
}*/
/*****************************************************************************
 功能描述  : 将一串数组打印出来
 输入参数  : void* array     数组
             uint16_t size   长度
             uint8_t format  格式
             uint8_t width   宽度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月7日
*****************************************************************************/
void rt_kprintfArray(void* array, uint16_t size, uint8_t format, uint8_t width)
{
    uint16_t i;
    
    if(1 == width)
    {
        for(i = 0; i < size; i++)
        {
            if(16 == format)
            {
                rt_kprintf("num%d: 0x%x\r\n", i, *((uint8_t*)array + i));
            }
            else
            {
                rt_kprintf("num%d: %d\r\n", i, *((uint8_t*)array + i));
            }
        }
    }
    else if(2 == width)
    {
        for(i = 0; i < size; i++)
        {
            if(16 == format)
            {
                rt_kprintf("num%d: 0x%x\r\n", i, *((uint16_t*)array + i));
            }
            else
            {
                rt_kprintf("num%d: %d\r\n", i, *((uint16_t*)array + i));
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 初始化接收结构体
 输入参数  : ST_UART_RECEIVE_DATA* usartDevice
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月24日
*****************************************************************************/
void UsartMsgInit(ST_UART_RECEIVE_DATA* usartDevice)
{
    usartDevice->flag = UART_RECEIVE_STATE_IDLE;
    usartDevice->lastTick = HAL_GetTick();
    usartDevice->length = 0;
    usartDevice->index = 0;
}
/*****************************************************************************
 功能描述  : WIFI接收命令据解析
 输入参数  : u8 cmd       命令字
             u8* cmdData  命令数据
             u8 size      命令长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年4月8日
*****************************************************************************/
void WifiRevCmdAnalysis(uint8_t* cmdData, uint16_t size)
{
    if(DEBUG_DATA_TYPE_93)
    {
        rt_kprintfArray(cmdData, size, 16, 1);
        rt_kprintf("\r\n\r\n");
    }
    else if(DEBUG_DATA_TYPE_95)
    {
        usart_kprintf("Rev cmd:%d,%d.\r\n", cmdData[2], HAL_GetTick());
    }

    LockThread();
    
    if('*' == cmdData[0])//短包数据
    {
        switch(cmdData[2])
        {
            case 0x01:  //电机控制
                TestMotorControl(&cmdData[3], size - 3);            
                break;
            case 0x02:  //获取程序版本
                PrintfHardwareType(RT_FALSE);            
                break;
            case 0x03: //复位从机
                LockThread();
                while(1);//复位芯片
            case 0x04: //读或清除故障
                if(cmdData[3])
                {
                    ClearErrorCode(M_TOTAL_NUM);
                }
                else if(IS_WARNING_CODE(gErrorMotorNum))
                {
                    rt_kprintf("M%d warningcode: 0x%04x.\r\n", gErrorMotorNum - WARNING_MOTOR_NUM_OFFSET, gErrorResult);
                }
                else
                {
                    rt_kprintf("M%d errorcode: 0x%04x.\r\n", gErrorMotorNum, gErrorResult);
                }
                break;
            case 0x05: //读电池信息
                if(gStBatteryState.validFlag || gStBatteryState.driverVolValidFlag)
                {
                    if(!gStBatteryState.validFlag)
                    {
                        rt_kprintf("Driver battery info...\r\n");
                    }
                    BatterPrintInfo();
                }
                else
                {
                    rt_kprintf("No battery info!\r\n\r\n");
                }
                rt_kprintf("TMain:%d,TSwitch:%d,TPowerB:%d.\r\n", (int8_t)(gRespondState.moduleTmp[0]- 40), 
                    (int8_t)(gRespondState.moduleTmp[1]- 40), (int8_t)(gRespondState.moduleTmp[2]- 40));
                rt_kprintf("M1:%d,M2:%d,M5:%d,M6:%d.\r\n", (int8_t)(gRespondState.motorTmp[0]- 40), 
                    (int8_t)(gRespondState.motorTmp[1]- 40), (int8_t)(gRespondState.motorTmp[4]- 40), 
                    (int8_t)(gRespondState.motorTmp[5]- 40));
                rt_kprintf("D1:%d,D2:%d,D3:%d,D5:%d,D6:%d.\r\n", (int8_t)(gRespondState.driverTmp[0]- 40), 
                    (int8_t)(gRespondState.driverTmp[1]- 40), (int8_t)(gRespondState.driverTmp[2]- 40), 
                    (int8_t)(gRespondState.driverTmp[5]- 40), (int8_t)(gRespondState.driverTmp[6]- 40));
                break;
#ifdef SD_RW_ENABLE
            case 0x14:
                if(0x04 == cmdData[3])
                {
                    if(HAL_OK == SetCurDateTime(cmdData))
                    {
                        rt_kprintf("Time calibration: %s %s.\r\n", gCurDateStr, gCurTimeStr);
                    }
                    else
                    {
                        rt_kprintf("Time calibration failed!\r\n");
                    }
                }
                else
                {
                    if(HAL_OK == UpdateCurDateTime())
                    {
                        rt_kprintf("Cur time: %s %s.\r\n", gCurDateStr, gCurTimeStr);
                    }
                    else
                    {
                        rt_kprintf("Get time failed!\r\n");
                    }
                }
                break;
             case 0x15:
                SdCmdDeal(&cmdData[3]);
                break;
#endif
             case 0x30:  //修改flash配置参数
                UsartWriteFlashConfigureParas(&cmdData[3], size - 3);
                break;
             case 0x31:  //上发主机命令内容
                PrintfMotorStaus();
                if(DEBUG_DATA_TYPE_8A) gPrintfTestDataShowFlag++;
                break;
             case 0x33:  //模拟故障
                SetErrorCode(cmdData[3], cmdData[3], (cmdData[3] >= WARNING_MOTOR_NUM_OFFSET) ? ERROR_NONE : ERROR_L_NORMAL);
                break;
             case 0x35:  //老化测试指令内容
                OldTestRevCmdAnalysis(&cmdData[3], size - 3);
                break;   
             case 0x40:
                Motor485Test(cmdData[3], &cmdData[4]);
                break;
             case 0x41:
                TestUfoControl(&cmdData[3], size - 3);
                break;
             case 0x42: //读ad值
                ReadAdcSensorValue(&cmdData[3], size - 3);
                break;
             case 0x43: //导航控制
                //NavControlCmd(&cmdData[3], size - 3);
                break;
             case  0x72:  
                canOpenObjectAccessFlag = cmdData[3];
                canOpenId = cmdData[4];
                rt_kprintf("Access canopen object(0-read): %d, id-%d.\r\n", 
                    canOpenObjectAccessFlag, canOpenId);            
                break;
             case  0x73:  
                if(canOpenObjectAccessFlag)
                {
                    MotorCanSdoSet(canOpenId, &cmdData[3], size - 4);
                }
                else
                {
                    MotorCanSdoGet(canOpenId, &cmdData[3], size - 4);
                }
                break;
             case 0x80:
                UdpRespondReved(cmdData, size);
                break;
             case 0x81:
                if(0x00 == cmdData[3])
                {
                    gMotorTestDataUploadFlag = RT_FALSE;
                    rt_kprintf("End upload.\r\n");
                }
                else
                {
                    gMotorTestDataUploadFlag = RT_TRUE;
                    gMotorTestDataUploadStartTime = HAL_GetTick();
                    if((*(uint16_t*)(&cmdData[6]) >= 50) && (*(uint16_t*)(&cmdData[6]) <= 1000))
                    {
                        gStMotorData[M_LEFT].profileAcc = *(uint16_t*)(&cmdData[6]);
                    }
                    rt_kprintf("Begin upload: acc:%d, time:%d.\r\n", gStMotorData[M_LEFT].profileAcc, gMotorTestDataUploadStartTime);
                }
                break;   
             case 0xCE:
                if(size > 4)
                {
                    if(cmdData[size - 2] == CRC8_Table(&cmdData[3], size - 5))//CRC校验
                    {
                        UsartWriteFlashConfigureParas(&cmdData[3], size - 5);
                    }
                }
                break;
             case DOWNLOAD_BOOTLOADER: //Bootloader升级
             case DOWNLOAD_APP:
                if(size > 5)
                {
                    if(cmdData[size - 2] == CRC8_Table(&cmdData[3], size - 5))//CRC校验
                    {
                        gU8DownloadType = cmdData[2];
                        IAPCmdMSgDeal(cmdData[3], &cmdData[4], size - 6);
                    }
                }
                break;
        }
    }
    else if('L' == cmdData[0])  //长包数据
    {
        switch(cmdData[3])
        {
            //程序下载相关命令
            case DOWNLOAD_BOOTLOADER:
            case DOWNLOAD_APP:
                if(size > 6)
                {
                    gU8DownloadType = cmdData[3];
                    if(cmdData[size - 2] == CRC8_Table(&cmdData[4], size - 6))//CRC校验
                    {
                        IAPCmdMSgDeal(cmdData[4], &cmdData[5], size - 7);
                    }
                }
            break;
        }
    }
    UnLockThread();
}
/*****************************************************************************
 功能描述  : WIFI数据接收
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月8日
*****************************************************************************/
void WifiRevMsgProcess(void)
{
    uint32_t l_receive_len;
    uint32_t i;
    uint32_t lTickTemp = HAL_GetTick();
    uint8_t lRevBuf[32];
    uint32_t lUartNumWifiPrint = USART6_DEVICE;

    if(gStUsartWifiRevData.index >= USART1_REV_BUFFER_LEN)
    {
        UsartMsgInit(&gStUsartWifiRevData);
        return;
    }
    //同时查询串口1和串口6的缓冲区，如果有数据就作为打印口，以便进行新旧电路板兼容
    l_receive_len = UsartDeviceRead(USART6_DEVICE, lRevBuf, sizeof(lRevBuf));
    if(0 == l_receive_len)
    {
        l_receive_len = UsartDeviceRead(USART1_DEVICE, lRevBuf, sizeof(lRevBuf));
        if(0 != l_receive_len)
        {
            lUartNumWifiPrint = USART1_DEVICE;
        }
    }

    if(0 == l_receive_len)
    {
        if((UART_RECEIVE_STATE_RECEIVING == gStUsartWifiRevData.flag) || (UART_RECEIVE_STATE_RECEIVING_LONG == gStUsartWifiRevData.flag))
        {
            lTickTemp -= gStUsartWifiRevData.lastTick;
            if(lTickTemp >= 500)
            {
                UsartMsgInit(&gStUsartWifiRevData);
            }
        }
    }
    else
    {
        if(l_receive_len > sizeof(lRevBuf)) l_receive_len = sizeof(lRevBuf);
        for(i = 0; i < l_receive_len; i++)
        {
            if(UART_RECEIVE_STATE_IDLE == gStUsartWifiRevData.flag)
            {
                if('*' == lRevBuf[i])//短包数据，小于256字节
                {
                    UsartMsgInit(&gStUsartWifiRevData);
                    gStUsartWifiRevData.flag = UART_RECEIVE_STATE_RECEIVING;
                }
                else if('L' == lRevBuf[i])//长包数据
                {
                    UsartMsgInit(&gStUsartWifiRevData);
                    gStUsartWifiRevData.flag = UART_RECEIVE_STATE_RECEIVING_LONG;
                }
                gStUsartWifiRevData.data[gStUsartWifiRevData.index++] = lRevBuf[i];
            }
            else if(UART_RECEIVE_STATE_RECEIVING == gStUsartWifiRevData.flag)//短包数据，小于256字节
            {
                gStUsartWifiRevData.data[gStUsartWifiRevData.index++] = lRevBuf[i];
                if(0 == gStUsartWifiRevData.length)
                {
                    gStUsartWifiRevData.length = lRevBuf[i];
                    if(gStUsartWifiRevData.length <= 3)
                    {
                        UsartMsgInit(&gStUsartWifiRevData);
                    }
                }
                else if(gStUsartWifiRevData.index == gStUsartWifiRevData.length)//接收完成
                {
                    if('#' == lRevBuf[i])
                    {
                        gUartPrintfFlag = 1;    //收到串口命令，串口打印
                        gUartNumWifiPrint = lUartNumWifiPrint;
                        WifiRevCmdAnalysis(gStUsartWifiRevData.data, gStUsartWifiRevData.length);
                        UsartMsgInit(&gStUsartWifiRevData);
                    }
                    else
                    {
                        UsartMsgInit(&gStUsartWifiRevData);
                    }
                }
            }
            else//长包数据
            {
                gStUsartWifiRevData.data[gStUsartWifiRevData.index++] = lRevBuf[i];
                if(0 == gStUsartWifiRevData.length)
                {
                    if(gStUsartWifiRevData.index > 2)
                    {
                        gStUsartWifiRevData.length = *(uint16_t*)(gStUsartWifiRevData.data + 1);
                        if((gStUsartWifiRevData.length <= 4) || (gStUsartWifiRevData.length > USART1_REV_BUFFER_LEN))
                        {
                            UsartMsgInit(&gStUsartWifiRevData);
                        }
                    }
                }
                else
                {
                    if(gStUsartWifiRevData.index > gStUsartWifiRevData.length)
                    {
                        UsartMsgInit(&gStUsartWifiRevData);
                    }
                    else if(gStUsartWifiRevData.index == gStUsartWifiRevData.length)
                    {
                        if('#' == lRevBuf[i])
                        {
                            gUartPrintfFlag = 1;    //收到串口命令，串口打印
                            gUartNumWifiPrint = lUartNumWifiPrint;
                            WifiRevCmdAnalysis(gStUsartWifiRevData.data, gStUsartWifiRevData.index);
                            UsartMsgInit(&gStUsartWifiRevData);
                        }
                        else
                        {
                            UsartMsgInit(&gStUsartWifiRevData);
                        }
                    }
                }
            }
        }
    }
}

/*****************************************************************************
 功能描述  : 解析调试串口的数据
 输入参数  : void
 输出参数  : 无
 作    者  : 刘子雄
 日    期  : 2024年12月2日
*****************************************************************************/
void DebugUartParse(void)
{
	uint32_t l_receive_len;
    uint8_t lRevBuf[32];
    uint32_t i = 0;

	l_receive_len = UsartDeviceRead(USART1_DEVICE, lRevBuf, sizeof(lRevBuf));
    if(0 == l_receive_len)
    {
		//没有收到数据
    	return;
    }

    if(l_receive_len > sizeof(lRevBuf))
		l_receive_len = sizeof(lRevBuf);

    while(i < l_receive_len)
    {
    	if('G' == lRevBuf[i])
		{
			//开始
			osTimerStart(myMotorTestTimerHandle, 20);//周期性定时器,周期为20ms
    		rt_kprintf("myMotorTestTimer Start!\r\n");
		}
		else if('S' == lRevBuf[i])
		{
			//停止
			rt_kprintf("S");
		}
    	i++;
    }


/*    for(i = 0; i < l_receive_len; i++)
	{
		if('G' == lRevBuf[i])
		{
			//开始
		}
		else if('S' == lRevBuf[i])
		{
			//停止
		}
	}*/
}


/*****************************************************************************
 功能描述  : VoFa调参代码,解析出DataBuff中的数据
 输入参数  : void
 输出参数  : 返回解析得到的数据
 作    者  : 刘子雄
 日    期  : 2024年12月2日
*****************************************************************************/
uint8_t pid_rx_buff[100];
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    for(uint8_t i=0;i<100;i++) // 查找等号和感叹号的位置
    {
        if(pid_rx_buff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(pid_rx_buff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(pid_rx_buff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (pid_rx_buff[data_Start_Num]-48)  + (pid_rx_buff[data_Start_Num+2]-48)*0.1f +
                      (pid_rx_buff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (pid_rx_buff[data_Start_Num]-48)*10 + (pid_rx_buff[data_Start_Num+1]-48) + (pid_rx_buff[data_Start_Num+3]-48)*0.1f +
                      (pid_rx_buff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (pid_rx_buff[data_Start_Num]-48)*100 + (pid_rx_buff[data_Start_Num+1]-48)*10 + (pid_rx_buff[data_Start_Num+2]-48) +
                      (pid_rx_buff[data_Start_Num+4]-48)*0.1f + (pid_rx_buff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
//    printf("data=%.2f\r\n",data_return);
    return data_return;
}


/* USER CODE END 1 */
