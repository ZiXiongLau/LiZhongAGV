/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "motor_control.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 PG11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
/*****************************************************************************
 ��������  : �趨������з���
 �������  : int motorNum  ������
             ENUM_DIR dir  ���з���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void GPIOSetMotorDir(int motorNum, ENUM_DIR dir)
{
    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRunState[motorNum].dirFlag = dir;   //��ǰ�����־
    }
    if(M_LEFT == motorNum)
    {
        /*if(DIR_CW == dir)
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
        }*/
    }
    else if(M_RIGHT == motorNum)
    {
        if(DIR_CW == dir)
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
        }
    }
    else if(M_TURN == motorNum)
    {
        if(DIR_CW == dir)
        {
            if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            }
        }
        else
        {
            if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            }
        }
    }
    else if(M_BRAKE == motorNum)
    {
        if(DIR_CW == dir)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
        }
    }
}
/*****************************************************************************
 ��������  : ʹ�ܵ��
 �������  : int motorNum           ������
             FunctionalState state  ʹ��״̬
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void GPIOEnableMotor(int motorNum, FunctionalState state)
{
    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRunState[motorNum].enableFlag = state;  //��ǰʹ�ܱ�־
    }
    if((M_LEFT == motorNum) || (M_RIGHT == motorNum))
    {
        if(gStUfoData.flag & UFO_KINCO_NEW_PCB) //����������ʹ����pcb����
        {
            if(ENABLE == state)
            {
                HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_RESET);
            }
        }
        else
        {
            if(ENABLE == state)
            {
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);
            }
        }
    }
    else if(M_TURN == motorNum)
    {
        if(ENABLE == state)
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
        }
    }
    else if(M_BRAKE == motorNum)
    {
        if(ENABLE == state)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
        }
    }
}
/*****************************************************************************
 ��������  : ��ȡ���״̬
 �������  : int motorNum  ������
 �������  : ���״̬
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
ErrorStatus GPIOGetMotorState(int motorNum)
{
    if(M_TURN == motorNum)
    {
        if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10))
        {
            return ERROR;
        }
    }

    return SUCCESS;
}
/*****************************************************************************
 ��������  : ��ȡ�����λ����1����״̬
 �������  : int motorNum  ������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��31��
*****************************************************************************/
GPIO_PinState GetMotorLimit1State(int motorNum)
{
    if(M_TURN == motorNum)
    {
        return HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9); //0���ڵ���1δ���ڵ�
    }

    return GPIO_PIN_RESET;
}
/*****************************************************************************
 ��������  : �򿪻�ر�led��
 �������  : int ledNum            led���
             ENUM_LED_STATE state  led״̬
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void GPIOSetLedState(int ledNum, ENUM_LED_STATE state)
{
    GPIO_PinState pinState = GPIO_PIN_SET;

    if(LED_ON == state)
    {
        pinState = GPIO_PIN_RESET;
    }
    
    if(LED0 == ledNum)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, pinState);
    }
    else if(LED1 == ledNum)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, pinState);
    }
    else if(LED2 == ledNum)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, pinState);
    }
    else if(LED3 == ledNum)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, pinState);
    }
}
/*****************************************************************************
 ��������  : �趨wifiоƬ��λ״̬
 �������  : rt_bool_t bResetFlag  true��λ��false����λ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��3��22��
*****************************************************************************/
void GPIOSetWifiResetState(rt_bool_t bResetFlag)
{
    if(RT_TRUE == bResetFlag)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    }
}
/*****************************************************************************
 ��������  : ��ʼ��io״̬
 �������  : ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��8��
*****************************************************************************/
void GPIOInitState(void)
{
    uint32_t i;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    //��Ϊɲ���������ʹ��IO��
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //wifiģ���ϵ縴λIO
    GPIOSetWifiResetState(RT_FALSE);
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    //�����µ�·��485ͨ�ŷ�����Ƽ�������ʹ��io��
    if(gStUfoData.flag & UFO_KINCO_NEW_PCB) //����������ʹ����pcb����
    {
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    }
    
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        if(gStMotorData[i].flag & MOTOR_ENABLE_FLAG)
        {
            GPIOEnableMotor(i, DISABLE);
        }
        if(gStMotorData[i].flag & MOTOR_DIR_FLAG)
        {
            GPIOSetMotorDir(i, DIR_CW);
        }
    }
}
/* USER CODE END 2 */
