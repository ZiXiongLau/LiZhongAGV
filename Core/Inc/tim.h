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
//通道序号定义
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
    uint32_t flag;                              //标志位
    uint32_t timClk;                            //定时器时钟频率
    uint32_t cntFre;                            //计数频率
    uint16_t whlSpdCalcPeriod;                  //轮速计算周期
    uint16_t whlCounts;                         //车轮一转对应脉冲数
    uint32_t lastCCRRecord[TOTAL_CH];           //上次通道CCR捕获寄存器
    volatile uint32_t CCRRecord[TOTAL_CH];      //通道CCR捕获寄存器(有在中断函数中进行写操作)
    volatile uint32_t captureCounts[TOTAL_CH];  //捕捉次数(有在中断函数中进行写操作)
    volatile uint32_t overFlowCounts;           //定时器溢出次数(有在中断函数中进行写操作) 
    /* dma channel */
    uint32_t rx_trans_error_flag;               //dma传输故障标志
    uint32_t tx_complete_flag;                  //dma发送完成标志
    uint32_t tx_trans_flag;                     //dma正在传送标志
    uint32_t sendPushIn;                        //发送缓冲区入队列指针
    uint32_t sendPopOut;                        //发送缓冲区出队列指针
    uint32_t revLenPopOut;                      //接收缓冲区长度出队列指针
    uint32_t revPopOut;                         //接收缓冲区出队列指针
    uint32_t sendBufSize;                       //发送缓冲区大小
    uint32_t revBufSize;                        //接收缓冲区大小
    uint8_t *sendBuf;
    uint8_t *revBuf;
}ST_TIM_DATA;

//刹车初始位置定义, 64864表示3003us,120都舵机对应900us(19440)-2100us(45360)，误差2us（43），1度（216）
#define BRAKE_INIT_PULSE_LEFT       20520   //左刹车电机初始脉冲之，对应5度
#define BRAKE_ONE_DEG_PULSE_LEFT    (216)   //左刹车舵机1度对应脉冲数
#define BRAKE_INIT_PULSE_RIGHT      44280   //右刹车电机初始脉冲值，对应5度
#define BRAKE_ONE_DEG_PULSE_RIGHT   (-216)  //右刹车舵机1度对应脉冲数

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
