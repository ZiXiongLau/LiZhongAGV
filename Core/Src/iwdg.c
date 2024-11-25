/**
  ******************************************************************************
  * File Name          : IWDG.c
  * Description        : This file provides code for the configuration
  *                      of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 4000;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN 1 */
/*****************************************************************************
 功能描述  : 将看门狗初始化成1s，上电时是16s
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月24日
*****************************************************************************/
void MX_IWDG_Init1S(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Window = 500;
    hiwdg.Init.Reload = 500;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
    Error_Handler();
    }
}
//喂独立看门狗
void IWDG_Feed(void)
{   
    HAL_IWDG_Refresh(&hiwdg); //重装载
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
