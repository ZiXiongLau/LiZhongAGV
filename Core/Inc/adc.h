/**
  ******************************************************************************
  * File Name          : ADC.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32f7xx_ll_adc.h"
#include "motor_control.h"

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

#define ADC_FILTER_TIMES 			36  //滤波次数
enum
{
    ADC1_IN4 = 0,
    ADC1_IN5,
    ADC1_IN6,
    ADCA_MAX_CHANNELS               //ADC最大通道数量 
};
enum
{
    ADC3_IN15 = 0,          //0主控卡  
    ADC3_IN8,               //1交换机
    ADC3_IN14,              //2电源板 
    ADC3_IN7,               //3预留
    ADC3_IN13,              //4电机5/6
    ADC3_IN6,               //5电机5/6
    ADC3_IN12,              //6电机1/2
    ADC3_IN5,               //7电机1/2
    ADC3_IN9,               //8预留
    ADC3_IN4,               //9预留
    ADCTMP_MAX_CHANNELS     //ADC温度采集最大通道数量 
};

#define CVT_RATIO                   0.0008f     //12位精度adc数字值转电压的系数 3.3V / 2的12次方

#define ADC1_FRONT_BRAKE_CHANNEL    (gStUfoData.oilPressStand & 0x03)   //前轮刹车对应ADC通道序号
#define ADC1_OIL_RATIO              ((gStUfoData.oilPressStand >> 8) & 0xFF)
#define ADC1_OIL_OFFSET             ((int32_t)(-0.6f * ((gStUfoData.oilPressStand >> 8) & 0xFF)))
#define ADC1_BRAKE_TEST_THRETHOLDS  ((gStUfoData.oilPressStand >> 16) & 0xFF)
#define ADC1_BRAKE_MAX_THRETHOLDS   ((gStUfoData.oilPressStand >> 24) & 0xFF)  //刹车提供的最大油压值(单位0.1Mpa)
#define ADC_AIR_PRESSURE_RATIO      66      //1.6Mpa气压传感器对应比例系数
#define ADC_AIR_PRESSURE_OFFSET     -40     //1.6Mpa气压传感器对应偏移值
#define ADC1_IN4_LIMIT1             0 
#define ADC1_IN4_LIMIT2             24 

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

void AdcStart(void);
float AdcGetSensorValue(uint8_t channel, int32_t offset, float ratio);
void ReadAdcSensorValue(uint8_t* cmdData, uint8_t size);
void ReadTmpProcess(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
