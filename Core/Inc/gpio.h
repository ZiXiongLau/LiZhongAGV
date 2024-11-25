/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "predef.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
void GPIOSetMotorDir(int motorNum, ENUM_DIR dir);
void GPIOEnableMotor(int motorNum, FunctionalState state);
ErrorStatus GPIOGetMotorState(int motorNum);
GPIO_PinState GetMotorLimit1State(int motorNum);
void GPIOSetLedState(int ledNum, ENUM_LED_STATE state);
void GPIOSetWifiResetState(rt_bool_t bResetFlag);
void GPIOInitState(void);
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

