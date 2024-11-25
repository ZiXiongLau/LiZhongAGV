/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#ifndef __can_H
#define __can_H
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

#define ENABLE_CAN1_RX_INTERRUPT            //使能can1接收中断

//CAN序号定义
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

//自定义can数据结构体
typedef struct {
    uint32_t id;                /* 29 bit identifier                               */
    uint8_t data[8];            /* Data field                                      */
    uint8_t len;                /* Length of data field in bytes                   */
    uint32_t ch;                /* Object channel                                  */  //对象通道
    uint8_t format;             /* 0 - STANDARD,   1 - EXTENDED IDENTIFIER         */  //0-标准  1-扩展标识
    uint8_t type;               /* 0 - DATA FRAME, 1 - REMOTE FRAME                */  //0-数据帧  1—远程帧
    uint8_t dataValid;          //数据有效标志
} CAN_msg;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void MX_CAN3_Init(void);

/* USER CODE BEGIN Prototypes */
void CanTest(void);
void CanStart(uint8_t deviceNum);
HAL_StatusTypeDef CanDeviceWrite(uint8_t deviceNum, const CAN_msg *buffer, uint32_t timeout); //发送数据
HAL_StatusTypeDef CanDeviceRead(uint8_t deviceNum, CAN_msg *buffer, uint32_t timeout);		//接收数据
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
