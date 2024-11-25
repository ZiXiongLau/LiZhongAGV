/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

extern CAN_HandleTypeDef hcan3;

/* USER CODE BEGIN Private defines */

#define USE_CAN2_DEVICE
#define USE_CAN3_DEVICE

#define CAN_RX_MSG_MAX_NUM      10

#define ENABLE_CAN1_RX_INTERRUPT            //ʹ��can1�����ж�
#define ENABLE_CAN2_RX_INTERRUPT



//CAN��Ŷ���?
enum{
    CAN1_DEVICE = 1,
    CAN2_DEVICE = 2,
    CAN3_DEVICE = 3,
};

/* Symbolic names for formats of CAN message                                 */
typedef enum {
    STANDARD_FORMAT = 0, 
    EXTENDED_FORMAT
} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {
    DATA_FRAME = 0,
    DATA_FME1,
    REMOTE_FRAME 
} CAN_FRAME;

//�Զ���can���ݽṹ��
typedef struct {
    uint32_t id;                /* 29 bit identifier                               */
    uint8_t data[8];            /* Data field                                      */
    uint8_t len;                /* Length of data field in bytes                   */
    uint32_t ch;                /* Object channel                                  */  //����ͨ��
    uint8_t format;             /* 0 - STANDARD,   1 - EXTENDED IDENTIFIER         */  //0-��׼  1-��չ��ʶ
    uint8_t type;               /* 0 - DATA FRAME, 1 - REMOTE FRAME                */  //0-����֡  1��Զ��֡
    uint8_t dataValid;          //������Ч��־
} CAN_msg;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void MX_CAN3_Init(void);

/* USER CODE BEGIN Prototypes */
void CanTest(void);
void CanStart(uint8_t deviceNum);
HAL_StatusTypeDef CanDeviceWrite(uint8_t deviceNum, const CAN_msg *buffer, uint32_t timeout); //��������
HAL_StatusTypeDef CanDeviceRead(uint8_t deviceNum, CAN_msg *buffer, uint32_t timeout);		//��������
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

