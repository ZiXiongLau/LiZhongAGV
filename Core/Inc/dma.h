/**
  ******************************************************************************
  * File Name          : dma.h
  * Description        : This file contains all the function prototypes for
  *                      the dma.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __dma_H
#define __dma_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define DMA_RxSERIES               (10u)           /* ����FIFO���� */
#define DMARXBUFF_NUMBER           (256u)          /* ���ջ�������С */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

//�Զ���DMA�������ݽṹ��
typedef struct {
    volatile uint8_t inIndex;                    //��������ձ���(ѭ�����ж�β)
    volatile uint8_t outIndex;                   //�Ѵ�����ձ���(ѭ�����ж�ͷ)
    uint8_t          dataBuf[DMARXBUFF_NUMBER];  //�������ݳ�
    uint8_t          rxCount[DMA_RxSERIES];      //���峤��
} DMA_msg;

#define DMARx_ENQUEUE(msg)   (((msg).inIndex  >= (DMA_RxSERIES - 1)) ? ((msg).inIndex  = 0) : ((msg).inIndex ++))    /* ������� */
#define DMARx_DEQUEUE(msg)   (((msg).outIndex >= (DMA_RxSERIES - 1)) ? ((msg).outIndex = 0) : ((msg).outIndex++))    /* ���ݳ��� */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __dma_H */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
