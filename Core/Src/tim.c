/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "stm32f7xx_ll_tim.h"
#include "motor_control.h"
#include "dma.h"

uint8_t gTimeStartFlag[M_TOTAL_NUM] = {0};
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

DMA_msg rxTim5DmaMsg[TOTAL_CH];

#if defined(RT_USING_TIM5)
ST_TIM_DATA gStTim5Data;

#if (TIM5_SEND_BUFFER_LEN != 0)
    uint8_t tim5_send_buffer[TIM5_SEND_BUFFER_LEN];
#endif
#if (TIM5_REV_BUFFER_LEN != 0)
    uint8_t tim5_rev_buffer[TIM5_REV_BUFFER_LEN];
#endif
#endif


/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 107;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM12 init function */
void MX_TIM12_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim12, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspInit 0 */

  /* USER CODE END TIM12_MspInit 0 */
    /* TIM12 clock enable */
    __HAL_RCC_TIM12_CLK_ENABLE();

    /* TIM12 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  /* USER CODE BEGIN TIM12_MspInit 1 */

  /* USER CODE END TIM12_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_icHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */
#if defined(RT_USING_TIM5)
      uint8_t i;
      gStTim5Data.device = &htim5;
      gStTim5Data.timClk = 108;    //��ͨ��STM32CubeMX��Clock Configuration�鿴��Ӧ��Timer clocks(MHz)
      gStTim5Data.flag = 0;
      for(i=0; i<TOTAL_CH; i++)
      {
          gStTim5Data.captureCounts[i] = 0;
      }
      gStTim5Data.overFlowCounts = 0;
      gStTim5Data.tx_trans_flag = 0;
      gStTim5Data.sendPushIn = 0;
      gStTim5Data.sendPopOut = 0;
      gStTim5Data.revPopOut = 0;
      gStTim5Data.sendBufSize = TIM5_SEND_BUFFER_LEN;
      gStTim5Data.revBufSize = TIM5_REV_BUFFER_LEN;
      gStTim5Data.revLenPopOut = 0;
#if (TIM5_SEND_BUFFER_LEN != 0)
      gStTim5Data.sendBuf = tim5_send_buffer;
#endif
#if (TIM5_REV_BUFFER_LEN != 0)
      gStTim5Data.revBuf =  tim5_rev_buffer;
#endif
#endif

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOH_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PH11     ------> TIM5_CH2
    PH12     ------> TIM5_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB0     ------> TIM3_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */
    
  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspDeInit 0 */

  /* USER CODE END TIM12_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM12_CLK_DISABLE();

    /* TIM12 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
  /* USER CODE BEGIN TIM12_MspDeInit 1 */

  /* USER CODE END TIM12_MspDeInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /**TIM5 GPIO Configuration
    PH11     ------> TIM5_CH2
    PH12     ------> TIM5_CH3
    */
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_11|GPIO_PIN_12);

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*****************************************************************************
 ��������  : ��ʱ��5������ʱ��8����
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��7��14��
*****************************************************************************/
void TIM5TRIGTOTIME8Init(void)
{
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }

    //�ж�����
    /* TIM8 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* TIM8 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger = TIM_TS_ITR3;
    if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
/* TIM5 init function */
void PWMTIM5Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(DRIVER_TYPE_LEADSHINE == gStMotorData[M_BRAKE].driverType)  //ɲ�������������
  {
      htim5.Instance = TIM5;
      htim5.Init.Prescaler = 49;
      htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim5.Init.Period = 65535;
      htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  }
  else if(DRIVER_TYPE_DUOJI_GDW == gStMotorData[M_BRAKE].driverType)  //���
  {
      htim5.Instance = TIM5;
      htim5.Init.Prescaler = 4;
      htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim5.Init.Period = 64864;
      htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  }
  else
  {
      //������1ֵ���㷽��:t1 = (Prescaler + 1) * ClockDivision / 108 us = (1079 + 1) * 1 / 108 = 10 us
      //��ʱ����㷽��: t2 = t1 * Period = 10 * 4000000000 us = 40000s = 11.11h
      htim5.Instance = TIM5;
      htim5.Init.Prescaler = 1079;  //��Χ0~65534
      htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim5.Init.Period = 4000000000;   //��Χ32λ1~4294967296
      htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  }

  __HAL_RCC_TIM5_CLK_ENABLE();
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if(DRIVER_TYPE_LEADSHINE == gStMotorData[M_BRAKE].driverType)  //ɲ�������������
  {
      if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
      {
        Error_Handler();
      }
      __HAL_RCC_GPIOH_CLK_ENABLE();
      /**TIM5 GPIO Configuration
      PH11
      ------> TIM5_CH2
      */
      GPIO_InitStruct.Pin = GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
      HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  }
  else if(DRIVER_TYPE_DUOJI_GDW == gStMotorData[M_BRAKE].driverType)  //���
  {
      if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
      {
        Error_Handler();
      }
      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**TIM5 GPIO Configuration
      PA0/WKUP     ------> TIM5_CH1
      PA3     ------> TIM5_CH4
      */
      GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}
/*****************************************************************************
 ��������  : PWM��ʼ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��22��
*****************************************************************************/
void PWMInit(void)
{
    uint32_t value;
    
    PWMSetMotorOutputState(M_LEFT, ENABLE);
    PWMSetMotorOutputState(M_RIGHT, ENABLE);
    //PWMSetMotorOutputState(M_BRAKE, ENABLE);
    value = gStMotorData[M_TURN].homingSpeed * gStMotorData[M_TURN].ratio * gStMotorData[M_TURN].counts / 60;
    if(value > 0)
    {
        value = 2160000 / value;    //��Ԥ���ص�ֵ
        if(value > 0) value -= 1;   //��һ
    }
    LL_TIM_OC_SetCompareCH3(TIM3, (value >> 1));
    LL_TIM_SetAutoReload(TIM3, value);

    PWMTIM5Init();

    //PWMSetMotorDir(M_BRAKE, DIR_STOP);
    if(DRIVER_TYPE_LEADSHINE == gStMotorData[M_BRAKE].driverType)  //ɲ�������������
    {
        //TIM5TRIGTOTIME8Init();
        value = gStMotorData[M_BRAKE].homingSpeed * gStMotorData[M_BRAKE].ratio * gStMotorData[M_BRAKE].counts / 60;
        if(value > 0)
        {
            value = 2160000 / value;    //��Ԥ���ص�ֵ
            if(value > 0) value -= 1;   //��һ
        }
        if(DEBUG_DATA_TYPE_91)
        {
            rt_kprintf("Brake value:%d.\r\n", value);
        }
        LL_TIM_OC_SetCompareCH2(TIM5, value >> 1);
        LL_TIM_SetAutoReload(TIM5, value);
    }
    else if(DRIVER_TYPE_DUOJI_GDW == gStMotorData[M_BRAKE].driverType)  //���
    {
        PWMSetMotorValue(M_BRAKE, 0);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	//����PWM���
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);	//����PWM���
    }
}
/*****************************************************************************
 ��������  : ʹ�ܸ����pwm���
 �������  : uint32_t motorNum           
             FunctionalState state  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void PWMSetMotorOutputState(uint32_t motorNum, FunctionalState state)
{
    /*if(M_LEFT == motorNum)
    {
        if(ENABLE == state)
        {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	//����PWM���
        }
        else
        {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	//����PWM���
        }
    }
    else if(M_RIGHT == motorNum)
    {
        if(ENABLE == state)
        {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	//����PWM���
        }
        else
        {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);	//����PWM���
        }
    }
    else */
    if(M_TURN == motorNum)
    {
        if(ENABLE == state)
        {
            if(!gTimeStartFlag[motorNum])
            {
                gTimeStartFlag[motorNum] = 1;
                __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
                __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
                HAL_TIM_Base_Start_IT(&htim2);
                //__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
                //__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   //����PWM���
            }
        }
        else
        {
            if(gTimeStartFlag[motorNum])
            {
                gTimeStartFlag[motorNum] = 0;
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);    //�ر�PWM���
                HAL_TIM_Base_Stop_IT(&htim2);
                __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
            }
        }
    }
    else if(M_BRAKE == motorNum)   //����32831�м�ֵ
    {
        if(ENABLE == state)
        {
            if(!gTimeStartFlag[motorNum])
            {
                gTimeStartFlag[motorNum] = 1;
                __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);
                __HAL_TIM_ENABLE_IT(&htim12, TIM_IT_UPDATE);
                HAL_TIM_Base_Start_IT(&htim12);
                //__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
                //__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);   //����PWM���
            }
        }
        else
        {
            if(gTimeStartFlag[motorNum])
            {
                gTimeStartFlag[motorNum] = 0;
                HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);    //�ر�PWM���
                HAL_TIM_Base_Stop_IT(&htim12);
                __HAL_TIM_DISABLE_IT(&htim12, TIM_IT_UPDATE);
            }
        }
    }
}
/*****************************************************************************
 ��������  : ���ø����pwm���
 �������  : uint32_t motorNum  ������
             int value     ֵ��������
             ����ɲ�����:�Ƕ�ֵ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void PWMSetMotorValue(uint32_t motorNum, int32_t value)
{
    if(DEBUG_DATA_TYPE_91)
    {
        rt_kprintf("M%d Set value:%d.\r\n", motorNum, value);
    }
    if(M_LEFT == motorNum)
    {
        value = value * gStMotorData[M_LEFT].ratio * gStMotorData[M_LEFT].counts / 60;  //���ӦƵ��0, 42~125000
        if(value > 0)
        {
            value = 2160000 / value;    //��Ԥ���ص�ֵ
            if(value > 0) value -= 1;   //��һ
        }
        LL_TIM_OC_SetCompareCH4(TIM4, (value >> 1));
        if(0 != value)
        {
            LL_TIM_SetAutoReload(TIM4, value);
        }
    }
    else if(M_RIGHT == motorNum)
    {
        value = value * gStMotorData[M_LEFT].ratio * gStMotorData[M_LEFT].counts / 60;  //���ӦƵ��0, 42~125000
        if(value > 0)
        {
            value = 2160000 / value;    //��Ԥ���ص�ֵ
            if(value > 0) value -= 1;   //��һ
        }
        LL_TIM_OC_SetCompareCH3(TIM4, (value >> 1));
        if(0 != value)
        {
            LL_TIM_SetAutoReload(TIM4, value);
        }
    }
    else if(M_TURN == motorNum)
    {
        PWMSetMotorPulse(motorNum);
    }
    else if(M_BRAKE == motorNum)   //����32831�м�ֵ
    {
        if(DRIVER_TYPE_LEADSHINE == gStMotorData[M_BRAKE].driverType)  //ɲ�������������
        {
            PWMSetMotorPulse(motorNum);
        }
        else if(DRIVER_TYPE_DUOJI_GDW == gStMotorData[M_BRAKE].driverType)  //���
        {
            LL_TIM_OC_SetCompareCH1(TIM5, BRAKE_INIT_PULSE_LEFT + value * BRAKE_ONE_DEG_PULSE_LEFT);
            LL_TIM_OC_SetCompareCH4(TIM5, BRAKE_INIT_PULSE_RIGHT + value * BRAKE_ONE_DEG_PULSE_RIGHT);
        }
    }
}
/*****************************************************************************
 ��������  : ����ת�������������
 �������  : uint32_t motorNum       ������
             int interruptFlag  �жϱ�־
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
void PWMUpdateMotorPulse(uint32_t motorNum, rt_bool_t interruptFlag)
{
    int32_t curPulse;

    if(M_TURN == motorNum)
    {
        if(gTimeStartFlag[motorNum])
        {
            if(interruptFlag)
            {
                curPulse = LL_TIM_GetAutoReload(TIM2);
            }
            else
            {
                curPulse = LL_TIM_GetCounter(TIM2);
            }
            
            if(DIR_CW == gStMotorRunState[motorNum].dirFlag)
            {
                gStMotorRunState[motorNum].curPos += curPulse;
            }
            else
            {
                gStMotorRunState[motorNum].curPos -= curPulse;
            }
            LL_TIM_SetCounter(TIM2, 0);
        }
    }
    else if(M_BRAKE == motorNum)
    {
        if(gTimeStartFlag[motorNum])
        {
            if(interruptFlag)
            {
                curPulse = LL_TIM_GetAutoReload(TIM12);
            }
            else
            {
                curPulse = LL_TIM_GetCounter(TIM12);
            }
            
            if(DIR_CW == gStMotorRunState[motorNum].dirFlag)
            {
                gStMotorRunState[motorNum].curPos += curPulse;
            }
            else
            {
                gStMotorRunState[motorNum].curPos -= curPulse;
            }
            LL_TIM_SetCounter(TIM12, 0);
        }
    }
}
/*****************************************************************************
 ��������  : �趨������������
 �������  : uint32_t motorNum   ������  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
void PWMSetMotorPulse(uint32_t motorNum)
{
    int32_t posErr;

    if(motorNum < M_TOTAL_NUM)
    {
        posErr = ABS_VALUE(gStMotorRunState[motorNum].targetPos - gStMotorRunState[motorNum].curPos);
        if(DEBUG_DATA_TYPE_91)
        {
            rt_kprintf("M%d tPos:%d,cPos:%d,pErr:%d.\r\n", motorNum, gStMotorRunState[motorNum].targetPos, gStMotorRunState[motorNum].curPos,
                posErr);
        }
        if(0 == posErr)
        {
            return;
        }
        else if(M_TURN == motorNum)
        {
            LL_TIM_SetAutoReload(TIM2, posErr > 65535 ? 65535 : posErr);
        }
        else if(M_BRAKE == motorNum)
        {
            LL_TIM_SetAutoReload(TIM12, posErr > 65535 ? 65535 : posErr);
        }
        else
        {
            return;
        }
        
        PWMSetMotorOutputState(motorNum, ENABLE); //ʹ��pwm���
    }
}
/*****************************************************************************
 ��������  : �趨������з���
 �������  : uint32_t motorNum  ������
             ENUM_DIR dir  ���з���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void PWMSetMotorDir(uint32_t motorNum, ENUM_DIR dir)
{
    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRunState[motorNum].dirFlag = dir;   //��ǰ�����־
    }
    if(M_BRAKE == motorNum)
    {
        if(DIR_STOP == dir)
        {
            LL_TIM_OC_SetCompareCH1(TIM5, 32831);
            LL_TIM_OC_SetCompareCH4(TIM5, 32831);
        }
        else if(DIR_CW == dir)
        {
            LL_TIM_OC_SetCompareCH1(TIM5, 34822);
            LL_TIM_OC_SetCompareCH4(TIM5, 29960);
        }
        else
        {
            LL_TIM_OC_SetCompareCH1(TIM5, 29960);
            LL_TIM_OC_SetCompareCH4(TIM5, 34822);
        }
    }
}

/*****************************************************************************
 ��������  : ���벶���ʼ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��10��13��
*****************************************************************************/
void InputCaptureInit(ST_TIM_DATA* tim)
{
    uint32_t luTemp = 0;
   
    tim->cntFre = tim->timClk * 1000000 /(tim->device->Init.Prescaler + 1);    //��ʱ������Ƶ��(��λHz)

    tim->whlCounts = 1;
    if (gStMotorData[M_LEFT].pos_limit1 > 0)
    {
        tim->whlCounts = gStMotorData[M_LEFT].pos_limit1;                      //��ȡ���ٱ������ֱ���               
    }

    luTemp = 1000 / (tim->whlCounts);
    if (luTemp < gStMotorData[M_LEFT].currentAdjustPeriod)
    {
        luTemp = gStMotorData[M_LEFT].currentAdjustPeriod;
    }
    tim->whlSpdCalcPeriod = luTemp;                                            //���ټ�������(��λms)�����Ϊ1s����СΪ������������

    //���ö�ʱ��DMA
    //��Ϊ��ȡTIMx_CCRx�Ĵ����Ὣ��Ӧ��CCxIF���㣬�ᵼ���޷�׼ȷ�ļ�¼�����������������������Բ�ʹ��DMA��������ģʽ
    /*
    HAL_TIM_DMABurst_ReadStart(tim->device, TIM_DMABASE_CCR2, TIM_DMA_CC2, (uint32_t *)tim->revBuf, TIM_DMABURSTLENGTH_2TRANSFERS);
    __HAL_TIM_ENABLE_IT(tim->device, TIM_IT_CC2);
    HAL_TIM_IC_Start(tim->device,TIM_CHANNEL_2);

    HAL_TIM_DMABurst_ReadStart(tim->device, TIM_DMABASE_CCR2, TIM_DMA_CC3, (uint32_t *)tim->revBuf, TIM_DMABURSTLENGTH_2TRANSFERS);
    __HAL_TIM_ENABLE_IT(tim->device, TIM_IT_CC3);
    HAL_TIM_IC_Start(tim->device,TIM_CHANNEL_3);
    
    __HAL_TIM_ENABLE_IT(tim->device, TIM_IT_UPDATE);  //���������ж�,�Ա�����������
    */

    //���ö�ʱ���ж�
    HAL_TIM_IC_Start_IT(tim->device, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(tim->device, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(tim->device, TIM_IT_UPDATE);  //���������ж�,�Ա�����������
    
}
/*****************************************************************************
 ��������  : ������ڼ���
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��10��13��
*****************************************************************************/
void IncPeriodElapsed(ST_TIM_DATA* tim)
{
    tim->overFlowCounts++;
}

/*****************************************************************************
 ��������  : ��ʱ���жϴ���
 �������  : TIM_HandleTypeDef *htim  ��ʱ�����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��10��16��
*****************************************************************************/
void AppTim_ISRProc(TIM_HandleTypeDef *htim)
{
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
        
        gStTim5Data.CCRRecord[CH2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        gStTim5Data.captureCounts[CH2]++;
    }

    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);
        
        gStTim5Data.CCRRecord[CH3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        gStTim5Data.captureCounts[CH3]++;
    }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
