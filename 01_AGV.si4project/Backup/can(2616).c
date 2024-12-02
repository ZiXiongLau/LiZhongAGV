/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "predef.h"
#include "delay.h"

CAN_msg rxCanMsg[CAN_RX_MSG_MAX_NUM] = {0};
uint8_t rxCanMsgIndex = 0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}
/* CAN3 init function */
void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 6;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = ENABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = ENABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */
    if(gStUfoData.flag & UFO_CAN_RX_PI9)  //更换can引脚
    {
        __HAL_RCC_CAN1_CLK_ENABLE();

        __HAL_RCC_GPIOH_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();
        /**CAN1 GPIO Configuration
        PH13     ------> CAN1_TX
        PI9     ------> CAN1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
#ifdef ENABLE_CAN1_RX_INTERRUPT
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
#endif

        return;
    }
  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOH_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PH13     ------> CAN1_TX
    PH14     ------> CAN1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
#ifndef ENABLE_CAN1_RX_INTERRUPT
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
#endif
  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
  else if(canHandle->Instance==CAN3)
  {
  /* USER CODE BEGIN CAN3_MspInit 0 */

  /* USER CODE END CAN3_MspInit 0 */
    /* CAN3 clock enable */
    __HAL_RCC_CAN3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN3 GPIO Configuration
    PB3     ------> CAN3_RX
    PB4     ------> CAN3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN3_MspInit 1 */

  /* USER CODE END CAN3_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */
    if(gStUfoData.flag & UFO_CAN_RX_PI9)  //更换can引脚
    {
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN1 GPIO Configuration
        PH13     ------> CAN1_TX
        PI9     ------> CAN1_RX
        */
        HAL_GPIO_DeInit(GPIOH, GPIO_PIN_13);
        HAL_GPIO_DeInit(GPIOI, GPIO_PIN_9);

		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
        return;
    }

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PH13     ------> CAN1_TX
    PH14     ------> CAN1_RX
    */
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_13|GPIO_PIN_14);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */
    
  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN3)
  {
  /* USER CODE BEGIN CAN3_MspDeInit 0 */

  /* USER CODE END CAN3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN3_CLK_DISABLE();

    /**CAN3 GPIO Configuration
    PB3     ------> CAN3_RX
    PB4     ------> CAN3_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);

  /* USER CODE BEGIN CAN3_MspDeInit 1 */

  /* USER CODE END CAN3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CanTest(void)
{
    CAN_msg msg = {1 , {0, 0x81, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};

    if(HAL_OK == CanDeviceRead(CAN2_DEVICE, &msg, 10))
    {
        CanDeviceWrite(CAN2_DEVICE, &msg, 10);
    }
		
		CanDeviceWrite(CAN2_DEVICE, &msg, 10);

    //CanDeviceWrite(CAN1_DEVICE, &msg, 10);
}
/*****************************************************************************
 功能描述  : 启动can传输
 输入参数  : uint8_t deviceNum  can设备号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年11月17日
*****************************************************************************/
void CanStart(uint8_t deviceNum)
{
    CAN_FilterTypeDef sFilterConfig;
	
	/*Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if(CAN1_DEVICE == deviceNum)
    {
        if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        {
            /* Filter configuration Error */
            Error_Handler();
        }
        
        //开启CAN
        if(HAL_CAN_Start(&hcan1) != HAL_OK)
        {
            //            PRINTF("开启CAN失败\r\n");
        }
#ifdef ENABLE_CAN1_RX_INTERRUPT
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // 使能FIFO0接收中断
#endif
    }
#ifdef USE_CAN2_DEVICE
    else if(CAN2_DEVICE == deviceNum)
    {
        sFilterConfig.FilterBank = 14;
        if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
        {
            /* Filter configuration Error */
            Error_Handler();
        }
        
        //开启CAN
        if(HAL_CAN_Start(&hcan2) != HAL_OK)
        {
            //            PRINTF("开启CAN失败\r\n");
        }
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);  // 使能FIFO0接收中断
    }

#endif
#ifdef USE_CAN3_DEVICE
    else if(CAN3_DEVICE == deviceNum)
    {
        if (HAL_CAN_ConfigFilter(&hcan3, &sFilterConfig) != HAL_OK)
        {
            /* Filter configuration Error */
            Error_Handler();
        }
        
        //开启CAN
        if(HAL_CAN_Start(&hcan3) != HAL_OK)
        {
            //            PRINTF("开启CAN失败\r\n");
        }
    }
#endif
}
/*****************************************************************************
 功能描述  : 自定义can格式转stm32定义can格式
 输入参数  : CAN_msg* devptr   自定义can格式
             CanTxMsgTypeDef* stmptr  stm32定义can格式
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年1月3日
*****************************************************************************/
static void CanDev2StmCan(const CAN_msg* devptr, CAN_TxHeaderTypeDef* stmptr)
{
    stmptr->DLC = devptr->len;
    if(devptr->format == STANDARD_FORMAT)
    {
        stmptr->IDE = CAN_ID_STD;
    }
    else
    {
        stmptr->IDE = CAN_ID_EXT;
    }
    if(REMOTE_FRAME == devptr->type)
    {
        stmptr->RTR = CAN_RTR_REMOTE;
    }
    else
    {
        stmptr->RTR = CAN_RTR_DATA;
    }
    stmptr->StdId = devptr->id;
    stmptr->ExtId = devptr->id;
    stmptr->TransmitGlobalTime = DISABLE;
}
/*****************************************************************************
 功能描述  : stm32定义can格式转自定义can格式
 输入参数  : CanRxMsgTypeDef* stmptr  stm32定义can格式
             CAN_msg* devptr   自定义can格式
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年1月3日
*****************************************************************************/
static void CanStmCan2Dev(const CAN_RxHeaderTypeDef* stmptr, CAN_msg* devptr)
{
    devptr->len = stmptr->DLC;   
    if(CAN_RTR_REMOTE == stmptr->RTR)
    {
        devptr->type = REMOTE_FRAME;
    }
    else
    {
        devptr->type = DATA_FRAME;
    }
    if(stmptr->IDE == CAN_ID_STD)
    {
        devptr->id = stmptr->StdId;
        devptr->format = STANDARD_FORMAT;
    }
    else
    {
        devptr->id = stmptr->ExtId ;
        devptr->format = EXTENDED_FORMAT;
    }
}
/*****************************************************************************
 功能描述  : canFIFO0中断接收回调函数
 输入参数  : CAN_HandleTypeDef *hcan
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年6月13日
*****************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef pHeader;
    int waitCnt = 0;
    
    if(hcan->Instance == CAN2)
    {
        while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0))
        {
            if(rxCanMsgIndex >= CAN_RX_MSG_MAX_NUM) rxCanMsgIndex = 0;
            if(rxCanMsg[rxCanMsgIndex].dataValid)   //已存满，则清除最开始的数据有效标志
            {
                rxCanMsg[rxCanMsgIndex].dataValid = 0;
            }
            if(HAL_OK == HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pHeader, rxCanMsg[rxCanMsgIndex].data))
            {
                CanStmCan2Dev(&pHeader, &rxCanMsg[rxCanMsgIndex]);
                rxCanMsg[rxCanMsgIndex].dataValid = 1;
                rxCanMsgIndex++;
                if(rxCanMsgIndex >= CAN_RX_MSG_MAX_NUM) rxCanMsgIndex = 0;
            }

            waitCnt++;
            if(waitCnt >= 100) break;
        }

        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);    // 再次使能FIFO0接收中断
    }
}
/*****************************************************************************
 功能描述  : 获取can接收缓冲区中一帧数据
 输入参数  : uint8_t deviceNum  can设备序号
             CAN_msg *buffer    数据输出地址
 输出参数  : HAL_StatusTypeDef  ok或错误
 作    者  : 刘鹏
 日    期  : 2023年6月13日
*****************************************************************************/
HAL_StatusTypeDef CanGetSingleRxBufferData(uint8_t deviceNum, CAN_msg *buffer)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t i = 0;

    if(deviceNum == CAN2_DEVICE)
    {
        if(rxCanMsgIndex < CAN_RX_MSG_MAX_NUM)
        {
            i = rxCanMsgIndex;
        }
        do
        {
            if(rxCanMsg[i].dataValid)   //有数据
            {
                *buffer = rxCanMsg[i];
                rxCanMsg[i].dataValid = 0;
                lResult = HAL_OK;
                break;
            }
            i++;
            if(i >= CAN_RX_MSG_MAX_NUM) i = 0;
        } while(i != rxCanMsgIndex);
    }

    return lResult;
}
/*****************************************************************************
 功能描述  : CAN设备发送
 输入参数  : uint8_t deviceNum           设备序号
             const CAN_msg *buffer  发送数据
             uint32_t timeout            等待发送超时时间ms
 输出参数  : HAL_StatusTypeDef      发送状态
 作    者  : 刘鹏
 日    期  : 2020年11月12日
*****************************************************************************/
HAL_StatusTypeDef CanDeviceWrite(uint8_t deviceNum, const CAN_msg *buffer, uint32_t timeout)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    CAN_HandleTypeDef* canHandle;
    CAN_TxHeaderTypeDef pHeader;
    uint32_t u32Temp;
    uint32_t tickstart;

    if(deviceNum == CAN1_DEVICE)
    {
        canHandle = &hcan1;
    }
#ifdef USE_CAN2_DEVICE
    else if(deviceNum == CAN2_DEVICE)
    {
        canHandle = &hcan2;
    }
#endif
#ifdef USE_CAN3_DEVICE
    else if(deviceNum == CAN3_DEVICE)
    {
        canHandle = &hcan3;
    }
#endif
    else
    {
        return lResult;
    }

    if(buffer == NULL)
    {
        return lResult;
    }

    if(DEBUG_DATA_TYPE_85 || ((DEBUG_DATA_TYPE_8C) && (CAN2_DEVICE == deviceNum))
        || ((DEBUG_DATA_TYPE_94) && (CAN3_DEVICE == deviceNum)))
    {
//        rt_kprintf("CAN%d-S,0x%x,%x,%x,%x,%x,%x,%x,%x,%x,%d. ESR:%#x.\r\n", deviceNum, buffer->id, buffer->data[0], buffer->data[1], 
//            buffer->data[2], buffer->data[3], buffer->data[4], buffer->data[5], buffer->data[6], buffer->data[7], HAL_GetTick(), canHandle->Instance->ESR);
    }
    
    CanDev2StmCan(buffer, &pHeader);
    /* Get tick */ 
    tickstart = HAL_GetTick();

    while(1)
	{
        u32Temp = HAL_CAN_GetTxMailboxesFreeLevel(canHandle);
        
		if(u32Temp)
		{
            lResult = HAL_CAN_AddTxMessage(canHandle, &pHeader, (uint8_t*)(buffer->data), &u32Temp);
            break;
		}
        else
        {
            delay_us(10);
        }

        /* Check for the Timeout */
        if(timeout != HAL_MAX_DELAY)
        {
            if((timeout == 0) || ((HAL_GetTick() - tickstart ) > timeout))
            {
                 lResult = HAL_TIMEOUT;
                 break;
            }
        }
	}
	
	return lResult;
}
/*****************************************************************************
 功能描述  : CAN设备接收
 输入参数  : uint8_t deviceNum     设备序号
             CAN_msg *buffer  接收数据
             uint32_t timeout      超时时间
 输出参数  : HAL_StatusTypeDef      接收状态
 作    者  : 刘鹏
 日    期  : 2020年11月12日
*****************************************************************************/
HAL_StatusTypeDef CanDeviceRead(uint8_t deviceNum, CAN_msg *buffer, uint32_t timeout)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
	CAN_HandleTypeDef* canHandle;
    CAN_RxHeaderTypeDef pHeader;
    uint32_t FIFONumber;
    uint32_t fillLevel;
    uint32_t tickstart;
    uint32_t interruptFlag = 0;

    if(deviceNum == CAN1_DEVICE)
    {
        canHandle = &hcan1;
        FIFONumber = CAN_RX_FIFO0;
#ifdef ENABLE_CAN1_RX_INTERRUPT
        interruptFlag = 1;
#endif
    }
#ifdef USE_CAN2_DEVICE
    else if(deviceNum == CAN2_DEVICE)
    {
        canHandle = &hcan2;
        FIFONumber = CAN_RX_FIFO0;
		#ifdef ENABLE_CAN2_RX_INTERRUPT
        interruptFlag = 1;
		#endif
    }
#endif
#ifdef USE_CAN3_DEVICE
    else if(deviceNum == CAN3_DEVICE)
    {
        canHandle = &hcan3;
        FIFONumber = CAN_RX_FIFO0;
    }
#endif
    else
    {
        return lResult;
    }

    if(buffer == NULL)
    {
        return lResult;
    }

    /* Get tick */ 
    tickstart = HAL_GetTick();

    while(1)
    {
        if(interruptFlag)
        {
            lResult = CanGetSingleRxBufferData(deviceNum, buffer);
            if(HAL_OK == lResult)
            {
                break;
            }
        }
        else
        {
            fillLevel = HAL_CAN_GetRxFifoFillLevel(canHandle, FIFONumber);
            if(fillLevel)
            {
                lResult = HAL_CAN_GetRxMessage(canHandle, FIFONumber, &pHeader, buffer->data);
                if(HAL_OK == lResult)
                {
                    CanStmCan2Dev(&pHeader, buffer);
                }
                break;            
            }
        }

        /* Check for the Timeout */
        if(timeout != HAL_MAX_DELAY)
        {
            if((timeout == 0) || ((HAL_GetTick() - tickstart ) > timeout))
            {
                 lResult = HAL_TIMEOUT;
                 break;
            }
        }
    }

    if(HAL_OK == lResult)
    {
        if(DEBUG_DATA_TYPE_85 || ((DEBUG_DATA_TYPE_8C) && (CAN2_DEVICE == deviceNum))
            || ((DEBUG_DATA_TYPE_94) && (CAN3_DEVICE == deviceNum)))
        {
            rt_kprintf("CAN%d-R,0x%x,%x,%x,%x,%x,%x,%x,%x,%x,%d.\r\n", deviceNum, buffer->id, buffer->data[0], buffer->data[1], 
                buffer->data[2], buffer->data[3], buffer->data[4], buffer->data[5], buffer->data[6], buffer->data[7], HAL_GetTick());
        }
    }

	return lResult;
}

/* USER CODE END 1 */
