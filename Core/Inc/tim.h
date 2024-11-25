/**
  ******************************************************************************
  * File Name          : TIM.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "predef.h"
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

/* USER CODE BEGIN Private defines */
//ͨ����Ŷ���
enum
{
    CH1 = 0,
    CH2,
    CH3,
    CH4,
    TOTAL_CH
};

typedef struct
{
    TIM_HandleTypeDef *device;
    uint32_t flag;                              //��־λ
    uint32_t timClk;                            //��ʱ��ʱ��Ƶ��
    uint32_t cntFre;                            //����Ƶ��
    uint16_t whlSpdCalcPeriod;                  //���ټ�������
    uint16_t whlCounts;                         //����һת��Ӧ������
    uint32_t lastCCRRecord[TOTAL_CH];           //�ϴ�ͨ��CCR����Ĵ���
    volatile uint32_t CCRRecord[TOTAL_CH];      //ͨ��CCR����Ĵ���(�����жϺ����н���д����)
    volatile uint32_t captureCounts[TOTAL_CH];  //��׽����(�����жϺ����н���д����)
    volatile uint32_t overFlowCounts;           //��ʱ���������(�����жϺ����н���д����) 
    /* dma channel */
    uint32_t rx_trans_error_flag;               //dma������ϱ�־
    uint32_t tx_complete_flag;                  //dma������ɱ�־
    uint32_t tx_trans_flag;                     //dma���ڴ��ͱ�־
    uint32_t sendPushIn;                        //���ͻ����������ָ��
    uint32_t sendPopOut;                        //���ͻ�����������ָ��
    uint32_t revLenPopOut;                      //���ջ��������ȳ�����ָ��
    uint32_t revPopOut;                         //���ջ�����������ָ��
    uint32_t sendBufSize;                       //���ͻ�������С
    uint32_t revBufSize;                        //���ջ�������С
    uint8_t *sendBuf;
    uint8_t *revBuf;
}ST_TIM_DATA;

//ɲ����ʼλ�ö���, 64864��ʾ3003us,120�������Ӧ900us(19440)-2100us(45360)�����2us��43����1�ȣ�216��
#define BRAKE_INIT_PULSE_LEFT       20520   //��ɲ�������ʼ����֮����Ӧ5��
#define BRAKE_ONE_DEG_PULSE_LEFT    (216)   //��ɲ�����1�ȶ�Ӧ������
#define BRAKE_INIT_PULSE_RIGHT      44280   //��ɲ�������ʼ����ֵ����Ӧ5��
#define BRAKE_ONE_DEG_PULSE_RIGHT   (-216)  //��ɲ�����1�ȶ�Ӧ������

#define RT_USING_TIM5
#define TIM5_SEND_BUFFER_LEN      0
#define TIM5_REV_BUFFER_LEN       256

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern ST_TIM_DATA gStTim5Data;

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);
void MX_TIM12_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void PWMInit(void);
void PWMSetMotorOutputState(uint32_t motorNum, FunctionalState state);
void PWMSetMotorValue(uint32_t motorNum, int32_t value);
void PWMUpdateMotorPulse(uint32_t motorNum, rt_bool_t interruptFlag);
void PWMSetMotorPulse(uint32_t motorNum);
void PWMSetMotorDir(uint32_t motorNum, ENUM_DIR dir);
void IncPeriodElapsed(ST_TIM_DATA* tim);
void InputCaptureInit(ST_TIM_DATA* tim);
void AppTim_ISRProc(TIM_HandleTypeDef *htim);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
