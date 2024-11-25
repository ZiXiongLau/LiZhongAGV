/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "predef.h"
#include "stm32f7xx_ll_usart.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
//串口序号定义
enum{
    USART1_DEVICE,  //wifi模块
    USART2_DEVICE,  //驱动电机
    USART3_DEVICE,  //未使用
    USART4_DEVICE,  //遥控接收器
    USART5_DEVICE,  //电池通信
    USART6_DEVICE,  //新版pcb wifi模块
    USART7_DEVICE,  //电流采集
    USART8_DEVICE   //无
};

#define RT_USING_UART4
#define USART4_SEND_BUFFER_LEN      0
#define USART4_REV_BUFFER_LEN       256

#define RT_USING_UART5
#define USART5_SEND_BUFFER_LEN      512
#define USART5_REV_BUFFER_LEN       512

#define RT_USING_UART1
#define USART1_SEND_BUFFER_LEN      1536
#define USART1_REV_BUFFER_LEN       1024

#define RT_USING_UART2
#define USART2_SEND_BUFFER_LEN      32
#define USART2_REV_BUFFER_LEN       64

#define RT_USING_UART6
#define USART6_SEND_BUFFER_LEN      USART1_SEND_BUFFER_LEN
#define USART6_REV_BUFFER_LEN       USART1_REV_BUFFER_LEN


#define RT_USING_UART7
#define USART7_SEND_BUFFER_LEN      0
#define USART7_REV_BUFFER_LEN       512

#define USART_RxSERIES              (10u)        /* 串口接收FIFO级数 */
#define RXBUFF_NUMBER               (512u)       /* 串口5接收缓冲区大小 */

//自定义串口数据结构体
typedef struct {
    volatile uint8_t inIndex;                    //待处理接收报文(循环队列队尾)
    volatile uint8_t outIndex;                   //已处理接收报文(循环队列队头)
    uint8_t          dataBuf[RXBUFF_NUMBER];     //缓冲数据场
    uint8_t          rxCount[USART_RxSERIES];    //缓冲长度
} Usart_msg;

#define USARTRx_ENQUEUE(msg)   (((msg).inIndex  >= (USART_RxSERIES - 1)) ? ((msg).inIndex  = 0) : ((msg).inIndex ++))    /* 数据入队 */
#define USARTRx_DEQUEUE(msg)   (((msg).outIndex >= (USART_RxSERIES - 1)) ? ((msg).outIndex = 0) : ((msg).outIndex++))    /* 数据出队 */

extern Usart_msg rxUsart5Msg;

//485传输方向定义
typedef enum
{
	DIRECTION485_RECEIVE = 0,
	DIRECTION485_TRANSMISSION  = 1
} Direction485;

typedef struct
{
    UART_HandleTypeDef *uart_device;
    uint32_t flag;              //标志位
    Direction485 dir;           //485传输方向
    /* dma channel */
    uint32_t rx_trans_error_flag;//dma传输故障标志
    uint32_t tx_complete_flag;//dma发送完成标志
    uint32_t tx_trans_flag;  //dma正在传送标志
    uint32_t sendPushIn;//发送缓冲区入队列指针
    uint32_t sendPopOut;//发送缓冲区出队列指针
    uint32_t revLenPopOut;//接收缓冲区长度出队列指针
    uint32_t revPopOut;//接收缓冲区出队列指针
    uint32_t sendBufSize; //发送缓冲区大小
    uint32_t revBufSize;  //接收缓冲区大小
    uint8_t *sendBuf;
    uint8_t *revBuf;
}ST_USART_DATA;


//串口标志位
#define USE_SEND_DMA                0x01        //使用发送dma
#define ENABLE_485_DIR              0x02        //使能485方向

/*串口接收状态定义*/
#define     UART_RECEIVE_STATE_IDLE         0
#define     UART_RECEIVE_STATE_RECEIVING    1
#define     UART_RECEIVE_STATE_RECEIVING_LONG    2

typedef struct
{
    uint8_t             flag;   /* 接收状态 */    
    uint16_t            length; /* 接收长度 */
    uint16_t            index;  /* 寻址 */
    uint32_t            lastTick;   /* 接收状态下上次收到数据时的tick */
    uint8_t*            data;   /* 接收数据 */
}ST_UART_RECEIVE_DATA;

extern int gUartPrintfFlag;

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void AppUsartInit(void);
void AppUsart_ISRProc(UART_HandleTypeDef *huart);
uint8_t IsRxBuffEmpty(Usart_msg msg);
uint32_t GetUsartRxDmaDataLength(uint32_t deviceNum);
ST_USART_DATA* UsartGetSelectDevice(uint32_t deviceNum);
uint32_t UsartDeviceWrite(uint32_t deviceNum, const void *buffer, uint32_t size);
uint32_t UsartDeviceRead(uint32_t deviceNum, void *buffer, uint32_t size);
void WifiRevMsgProcess(void);
void WifiRevCmdAnalysis(uint8_t* cmdData, uint16_t size);
void UsartMsgInit(ST_UART_RECEIVE_DATA* usartDevice);
uint32_t isDmaSendFinish(ST_USART_DATA *uart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

