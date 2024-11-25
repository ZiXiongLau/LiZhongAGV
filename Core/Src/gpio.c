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
 功能描述  : 设定电机运行方向
 输入参数  : int motorNum  电机序号
             ENUM_DIR dir  运行方向
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月8日
*****************************************************************************/
void GPIOSetMotorDir(int motorNum, ENUM_DIR dir)
{
    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRunState[motorNum].dirFlag = dir;   //当前方向标志
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
 功能描述  : 使能电机
 输入参数  : int motorNum           电机序号
             FunctionalState state  使能状态
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月8日
*****************************************************************************/
void GPIOEnableMotor(int motorNum, FunctionalState state)
{
    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRunState[motorNum].enableFlag = state;  //当前使能标志
    }
    if((M_LEFT == motorNum) || (M_RIGHT == motorNum))
    {
        if(gStUfoData.flag & UFO_KINCO_NEW_PCB) //步科驱动器使用新pcb控制
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
 功能描述  : 获取电机状态
 输入参数  : int motorNum  电机序号
 输出参数  : 电机状态
 作    者  : 刘鹏
 日    期  : 2020年12月8日
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
 功能描述  : 获取电机限位开关1引脚状态
 输入参数  : int motorNum  电机序号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月31日
*****************************************************************************/
GPIO_PinState GetMotorLimit1State(int motorNum)
{
    if(M_TURN == motorNum)
    {
        return HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9); //0被遮挡，1未被遮挡
    }

    return GPIO_PIN_RESET;
}
/*****************************************************************************
 功能描述  : 打开或关闭led灯
 输入参数  : int ledNum            led序号
             ENUM_LED_STATE state  led状态
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月8日
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
 功能描述  : 设定wifi芯片复位状态
 输入参数  : rt_bool_t bResetFlag  true复位，false不复位
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年3月22日
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
 功能描述  : 初始化io状态
 输入参数  : 无
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月8日
*****************************************************************************/
void GPIOInitState(void)
{
    uint32_t i;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    //华为刹车电机方向及使能IO口
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //wifi模块上电复位IO
    GPIOSetWifiResetState(RT_FALSE);
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    //步科新电路板485通信方向控制及驱动器使能io口
    if(gStUfoData.flag & UFO_KINCO_NEW_PCB) //步科驱动器使用新pcb控制
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
