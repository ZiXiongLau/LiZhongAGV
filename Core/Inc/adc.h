/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

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

#define ADC_FILTER_TIMES 			36  //�˲�����
enum
{
    ADC1_IN4 = 0,
    ADC1_IN5,
    ADC1_IN6,
    ADCA_MAX_CHANNELS               //ADC���ͨ������ 
};
enum
{
    ADC3_IN15 = 0,          //0���ؿ�  
    ADC3_IN8,               //1������
    ADC3_IN14,              //2��Դ�� 
    ADC3_IN7,               //3Ԥ��
    ADC3_IN13,              //4���5/6
    ADC3_IN6,               //5���5/6
    ADC3_IN12,              //6���1/2
    ADC3_IN5,               //7���1/2
    ADC3_IN9,               //8Ԥ��
    ADC3_IN4,               //9Ԥ��
    ADCTMP_MAX_CHANNELS     //ADC�¶Ȳɼ����ͨ������ 
};

#define CVT_RATIO                   0.0008f     //12λ����adc����ֵת��ѹ��ϵ�� 3.3V / 2��12�η�

#define ADC1_FRONT_BRAKE_CHANNEL    (gStUfoData.oilPressStand & 0x03)   //ǰ��ɲ����ӦADCͨ�����
#define ADC1_OIL_RATIO              ((gStUfoData.oilPressStand >> 8) & 0xFF)
#define ADC1_OIL_OFFSET             ((int32_t)(-0.6f * ((gStUfoData.oilPressStand >> 8) & 0xFF)))
#define ADC1_BRAKE_TEST_THRETHOLDS  ((gStUfoData.oilPressStand >> 16) & 0xFF)
#define ADC1_BRAKE_MAX_THRETHOLDS   ((gStUfoData.oilPressStand >> 24) & 0xFF)  //ɲ���ṩ�������ѹֵ(��λ0.1Mpa)
#define ADC_AIR_PRESSURE_RATIO      66      //1.6Mpa��ѹ��������Ӧ����ϵ��
#define ADC_AIR_PRESSURE_OFFSET     -40     //1.6Mpa��ѹ��������Ӧƫ��ֵ
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

#endif /* __ADC_H__ */

