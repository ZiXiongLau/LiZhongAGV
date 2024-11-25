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
//������Ŷ���
enum{
    USART1_DEVICE,  //wifiģ��
    USART2_DEVICE,  //�������
    USART3_DEVICE,  //δʹ��
    USART4_DEVICE,  //ң�ؽ�����
    USART5_DEVICE,  //���ͨ��
    USART6_DEVICE,  //�°�pcb wifiģ��
    USART7_DEVICE,  //�����ɼ�
    USART8_DEVICE   //��
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

#define USART_RxSERIES              (10u)        /* ���ڽ���FIFO���� */
#define RXBUFF_NUMBER               (512u)       /* ����5���ջ�������С */

//�Զ��崮�����ݽṹ��
typedef struct {
    volatile uint8_t inIndex;                    //��������ձ���(ѭ�����ж�β)
    volatile uint8_t outIndex;                   //�Ѵ�����ձ���(ѭ�����ж�ͷ)
    uint8_t          dataBuf[RXBUFF_NUMBER];     //�������ݳ�
    uint8_t          rxCount[USART_RxSERIES];    //���峤��
} Usart_msg;

#define USARTRx_ENQUEUE(msg)   (((msg).inIndex  >= (USART_RxSERIES - 1)) ? ((msg).inIndex  = 0) : ((msg).inIndex ++))    /* ������� */
#define USARTRx_DEQUEUE(msg)   (((msg).outIndex >= (USART_RxSERIES - 1)) ? ((msg).outIndex = 0) : ((msg).outIndex++))    /* ���ݳ��� */

extern Usart_msg rxUsart5Msg;

//485���䷽����
typedef enum
{
	DIRECTION485_RECEIVE = 0,
	DIRECTION485_TRANSMISSION  = 1
} Direction485;

typedef struct
{
    UART_HandleTypeDef *uart_device;
    uint32_t flag;              //��־λ
    Direction485 dir;           //485���䷽��
    /* dma channel */
    uint32_t rx_trans_error_flag;//dma������ϱ�־
    uint32_t tx_complete_flag;//dma������ɱ�־
    uint32_t tx_trans_flag;  //dma���ڴ��ͱ�־
    uint32_t sendPushIn;//���ͻ����������ָ��
    uint32_t sendPopOut;//���ͻ�����������ָ��
    uint32_t revLenPopOut;//���ջ��������ȳ�����ָ��
    uint32_t revPopOut;//���ջ�����������ָ��
    uint32_t sendBufSize; //���ͻ�������С
    uint32_t revBufSize;  //���ջ�������С
    uint8_t *sendBuf;
    uint8_t *revBuf;
}ST_USART_DATA;


//���ڱ�־λ
#define USE_SEND_DMA                0x01        //ʹ�÷���dma
#define ENABLE_485_DIR              0x02        //ʹ��485����

/*���ڽ���״̬����*/
#define     UART_RECEIVE_STATE_IDLE         0
#define     UART_RECEIVE_STATE_RECEIVING    1
#define     UART_RECEIVE_STATE_RECEIVING_LONG    2

typedef struct
{
    uint8_t             flag;   /* ����״̬ */    
    uint16_t            length; /* ���ճ��� */
    uint16_t            index;  /* Ѱַ */
    uint32_t            lastTick;   /* ����״̬���ϴ��յ�����ʱ��tick */
    uint8_t*            data;   /* �������� */
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

