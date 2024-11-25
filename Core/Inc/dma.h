/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define DMA_RxSERIES               (10u)           /* 接收FIFO级数 */
#define DMARXBUFF_NUMBER           (256u)          /* 接收缓冲区大小 */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

//自定义DMA传送数据结构体
typedef struct {
    volatile uint8_t inIndex;                    //待处理接收报文(循环队列队尾)
    volatile uint8_t outIndex;                   //已处理接收报文(循环队列队头)
    uint8_t          dataBuf[DMARXBUFF_NUMBER];  //缓冲数据场
    uint8_t          rxCount[DMA_RxSERIES];      //缓冲长度
} DMA_msg;

#define DMARx_ENQUEUE(msg)   (((msg).inIndex  >= (DMA_RxSERIES - 1)) ? ((msg).inIndex  = 0) : ((msg).inIndex ++))    /* 数据入队 */
#define DMARx_DEQUEUE(msg)   (((msg).outIndex >= (DMA_RxSERIES - 1)) ? ((msg).outIndex = 0) : ((msg).outIndex++))    /* 数据出队 */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

