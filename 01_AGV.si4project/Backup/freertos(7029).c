/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "lwip.h"
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "motor_control.h"
#include "mbtcpserver.h"
#include "mb.h"
#include <stdarg.h>
#include "battery.h"
#include "motor_control.h"
#include "bootloaderDef.h"
#include "iwdg.h"
#include "flash_access.h"
#include "adc.h"
#include "can.h"
#include "CRC8.h"
#include "log_printf.h"
//#include "nav.h"
#include "periodic_timer_task.h"
#include <motor_driven.h>


#ifdef SD_RW_ENABLE
#include "fatfs.h"
#include "rtc.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//�߳�ι����־
#define WDG_FEED_MOTOR_THREAD   0x01
#define WdgFeedThread(flag)     wdgFeedFlag |= flag
#define IsWdgFeedThreadFinish   (0x01 == (wdgFeedFlag & 0x01))
#define SBUS_DATA_LEN               25


EventGroupHandle_t EventSDLog = NULL;
#ifdef SD_RW_ENABLE
uint8_t     SDLogInfoBuf[SD_INFO_BUF_SIZE];
uint16_t    SDLogInfoIndex = 0;
#endif

#define     TCP_REV_BUF_SIZE        512
uint8_t     gTcpRevBuf[TCP_REV_BUF_SIZE];
#define     PRINTF_REV_BUF_SIZE     1024
uint8_t     gTcpPrintRevBuf[PRINTF_REV_BUF_SIZE];
ST_UART_RECEIVE_DATA gStTcpRevData = {UART_RECEIVE_STATE_IDLE, 0, 0, 0, gTcpRevBuf};
ST_UART_RECEIVE_DATA gStTcpPrintRevData = {UART_RECEIVE_STATE_IDLE, 0, 0, 0, gTcpPrintRevBuf};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t wdgFeedFlag = 0;
uint8_t wdgDisableFlag = 0;                     //��ֹ���Ź�ι����־
uint8_t tastStartFlag = 0;
uint8_t tastLockCnt = 0;
uint8_t data_buffer[100];						//������յ�������Buff��СΪ100
char tcp_server_recvbuf[300];					//�������ݴ���Buff��СΪ300��Ϊ100Ҳ����ν��ֻҪ���ڵ���100�ͺã�
int sock_conn = -1;							    // ����� socked
//int sock_print_conn = -1;						// ����Ĵ�ӡ socked
int udp_sock_num = -1;
struct sockaddr_in udpClientAddr;			    //udp�ͻ��˵�ַ
int udp_print_connected_flag = -1;              //��ӡ�˿ڿͻ���udp���ӱ�־,-1��ʾδsocket����ʧ�ܣ�0��ʾ�����ɹ�����δ�յ��ͻ�����Ϣ��1��ʾ�ȶ��յ��ͻ�����Ϣ
uint8_t gSbusData[SBUS_DATA_LEN << 1];
struct sockaddr_in socketConAddr;				//���ӵ�ַ
int32_t gRemoteStopDcc = 300;                   //ң��ֹͣʱ���ٶ�

uint32_t tcpModbusConFailCnt = 0u;              //����ʧ�ܼ���
uint32_t tcpModbusSotClsCnt = 0u;               //�ر��׽��ּ���

unsigned short   usRegHoldBuf[REG_HOLD_NREGS];

SYS_PARA         *sys_para = (SYS_PARA*)usRegHoldBuf; //����ϵͳ�趨����ʵ����
RESPOND_STATE gRespondState;                    //��Ӧ״̬��Ϣ
SEND_RESPOND_STATE gSendRespondState={0XAA,0x55,sizeof(RESPOND_STATE),0x96,{0},0};
OLD_TEST gOldTest = {OLD_TEST_PAUSE | OLD_TEST_END, NONE_AGING, 20, 150, 833, -139, 10, 300, 700, 3, 200, 0, 700, 3}; //�ϻ�������ر���
#define TCP_SERVER_BUF_MAX_LEN      512
uint8_t socketRevBuf[TCP_SERVER_BUF_MAX_LEN];
uint8_t tcpServerSendBuf[TCP_SERVER_BUF_MAX_LEN];

/* USER CODE END Variables */
osThreadId myTaskMainHandle;
uint32_t myTaskMainBuffer[ 2560 ];
osStaticThreadDef_t myTaskMainControlBlock;
osThreadId myTaskMotorHandle;
uint32_t myTaskMotorBuffer[ 2560 ];
osStaticThreadDef_t myTaskMotorControlBlock;
osThreadId myTaskTcpCreateHandle;
uint32_t myTaskTcpCreateBuffer[ 512 ];
osStaticThreadDef_t myTaskTcpCreateControlBlock;
osThreadId myTaskTcpHandle;
uint32_t myTaskTcpBuffer[ 1280 ];
osStaticThreadDef_t myTaskTcpControlBlock;
osThreadId myTaskSDHandle;
uint32_t myTaskSDBuffer[ 1280 ];
osStaticThreadDef_t myTaskSDControlBlock;
osThreadId myTaskTcpPrintCHandle;
uint32_t myTaskTcpPrintCBuffer[ 512 ];
osStaticThreadDef_t myTaskTcpPrintCControlBlock;
osTimerId myMotorTestTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void AllDataInit(void);
static void closeSelectSocket(int* sockNum);
static void SBUS_ReceiveProcess(TickType_t curTime);
static void CAN_ReceiveProcess(TickType_t curTime);
static void CAN_SendProcess(rt_bool_t firstFlag);
static void USART_ReceiveProcess(uint32_t deviceNum, Usart_msg *pUsartMsg, TickType_t curTime);
static void UfoControlCmdAnalysis(TickType_t curTime);
static void TcpPrintRevMsgProcess(void);
static void TcpSocketProcess(TickType_t curTime);
static void TcpSocketProcessNew(void);
static void TcpSocketSendNew(void);
u8_t IsMxLwipNetifLinkUp(void);


/* USER CODE END FunctionPrototypes */

void StartTaskMain(void const * argument);
void StartTaskMotor(void const * argument);
void StartTaskTcpCreate(void const * argument);
void StartTaskTcp(void const * argument);
void StartTaskSD(void const * argument);
void StartTaskTcpPrint(void const * argument);
extern void MotorTestTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  EventSDLog = xEventGroupCreate();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myMotorTestTimer */
  osTimerDef(myMotorTestTimer, MotorTestTimerCallback);
  myMotorTestTimerHandle = osTimerCreate(osTimer(myMotorTestTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of myTaskMain */
  osThreadStaticDef(myTaskMain, StartTaskMain, osPriorityNormal, 0, 2560, myTaskMainBuffer, &myTaskMainControlBlock);
  myTaskMainHandle = osThreadCreate(osThread(myTaskMain), NULL);

  /* definition and creation of myTaskMotor */
  osThreadStaticDef(myTaskMotor, StartTaskMotor, osPriorityHigh, 0, 2560, myTaskMotorBuffer, &myTaskMotorControlBlock);
  myTaskMotorHandle = osThreadCreate(osThread(myTaskMotor), NULL);

  /* definition and creation of myTaskTcpCreate */
  osThreadStaticDef(myTaskTcpCreate, StartTaskTcpCreate, osPriorityLow, 0, 512, myTaskTcpCreateBuffer, &myTaskTcpCreateControlBlock);
  myTaskTcpCreateHandle = osThreadCreate(osThread(myTaskTcpCreate), NULL);

  /* definition and creation of myTaskTcp */
  osThreadStaticDef(myTaskTcp, StartTaskTcp, osPriorityBelowNormal, 0, 1280, myTaskTcpBuffer, &myTaskTcpControlBlock);
  myTaskTcpHandle = osThreadCreate(osThread(myTaskTcp), NULL);

  /* definition and creation of myTaskSD */
  osThreadStaticDef(myTaskSD, StartTaskSD, osPriorityLow, 0, 1280, myTaskSDBuffer, &myTaskSDControlBlock);
  myTaskSDHandle = osThreadCreate(osThread(myTaskSD), NULL);

  /* definition and creation of myTaskTcpPrintC */
  osThreadStaticDef(myTaskTcpPrintC, StartTaskTcpPrint, osPriorityLow, 0, 512, myTaskTcpPrintCBuffer, &myTaskTcpPrintCControlBlock);
  myTaskTcpPrintCHandle = osThreadCreate(osThread(myTaskTcpPrintC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  AllDataInit();
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTaskMain */
/**
* @brief Function implementing the myTaskMain thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMain */
void StartTaskMain(void const * argument)
{
  /* USER CODE BEGIN StartTaskMain */
  TickType_t l_cur_tick, l_can_send_tick = 0, l_reset_wifi_tick;
  rt_bool_t wifiResetFlag = RT_TRUE;
  tastStartFlag = 1;
  tastLockCnt = 0;
  wdgDisableFlag = 0;
  MX_IWDG_Init1S(); //�����Ź���ʱ��16s�ĳ�1s
  rt_kprintf("Main task start!\r\n");
  l_reset_wifi_tick = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    l_cur_tick = xTaskGetTickCount();

    //ң����Ϣ����
    SBUS_ReceiveProcess(l_cur_tick);
    //���ػ���Ϣ����
    if(gStUfoData.flag & UFO_PC_CAN_FLAG)  //���ػ�canͨ�ű�־
    {
        CAN_ReceiveProcess(l_cur_tick);
        if(l_cur_tick - l_can_send_tick >= 10)
        {
            l_can_send_tick = l_cur_tick;
            CAN_SendProcess(RT_TRUE);
        }
        else
        {
            CAN_SendProcess(RT_FALSE);
        }
    }
    if (IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG))    //���ػ��ڴ���ͨ�ű�־
    {
        USART_ReceiveProcess(USART5_DEVICE, &rxUsart5Msg, l_cur_tick);
    }
        
    //�����������
    UfoControlCmdAnalysis(l_cur_tick);

	//����1���ݽ���
	DebugUartParse();
	
    //wifi��Ϣ����
/*    WifiRevMsgProcess();*/
    //���������߳�
//    NavDataBinRev(l_cur_tick);    //�ߵ����ݽ���
//    if(DEBUG_DATA_TYPE_3)
//    {
//        NavPrintData(l_cur_tick);   //��ӡ�ߵ�����
//    }
    //NavProcess(l_cur_tick);

    if(IsWdgFeedThreadFinish && (0 == wdgDisableFlag))
    {
        wdgFeedFlag = 0;
        IWDG_Feed();    	 /*�����߳�ι����־��ɺ���ʵι��*/
    }

    if(l_cur_tick - l_reset_wifi_tick >= 10000)
    {
        l_reset_wifi_tick = l_cur_tick - 9500;
        GPIOSetWifiResetState(wifiResetFlag);
        if(wifiResetFlag)
        {
            wifiResetFlag = RT_FALSE;
        }
    }

    BatteryCollectProcess(RT_TRUE);

    WriteFlashConfigureParasProcess(l_cur_tick);

    osDelay(1);
  }
  /* USER CODE END StartTaskMain */
}

/* USER CODE BEGIN Header_StartTaskMotor */
/**
* @brief Function implementing the myTaskMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotor */
void StartTaskMotor(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotor */
  TickType_t l_cur_tick;
  float _test_val;

  struct velocity target_velocity;

  rt_kprintf("Motor task start!\r\n");

  car_create();
//  MyMotorSet();


  target_velocity.linear_x = 0.0;
  target_velocity.angular_z = 0.0;

  //SetMotorPower(M_TURN, POWER_ON);
  /* Infinite loop */
  for(;;)
  {
    l_cur_tick = xTaskGetTickCount();

    MotorControlEntry(l_cur_tick);
	

	_test_val = GetMotorTestValue();
	target_velocity.linear_x = _test_val;

//	if(!FLOAT_EQU(_test_val, 0))//���ٶȲ�Ϊ0ʱ���Ŵ�ӡ�ٶ�ֵ
//	{
//		MotorSendReadVelocity(M_LEFT);
//		ReadMotorVelocity(l_cur_tick);
//	}

	agv_velocity_set(target_velocity);
          
    osDelay(1);

    WdgFeedThread(WDG_FEED_MOTOR_THREAD);
  }
  /* USER CODE END StartTaskMotor */
}

/* USER CODE BEGIN Header_StartTaskTcpCreate */
/**
* @brief Function implementing the myTaskTcpCreate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTcpCreate */
void StartTaskTcpCreate(void const * argument)
{
  /* USER CODE BEGIN StartTaskTcpCreate */
    struct sockaddr_in server_addr;				//��������ַ
    struct sockaddr_in socketConAddrNew;		//���ӵ�ַ
	int sock_fd;				                //�������� socked
	int sock_conn_new;
	socklen_t addr_len;							// ��ַ����
	int err;
    int opt = 1;
    struct timeval timeout;
    struct linger lingerValue;

  /* Infinite loop */
  for(;;)
  {
    if(IsMxLwipNetifLinkUp())
    {
        sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);        //����һ���µ�socket����
        if (sock_fd < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("TCP socket create failed:%d!\r\n", sock_fd);
        }
 
        /*err = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(int)); //�˿ڿ�����
        if (err < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("TCP setopt failed:%d!\r\n", err);
        }*/
    
        memset(&server_addr, 0, sizeof(server_addr));               //����������ַ���
        server_addr.sin_family = AF_INET;                           //��ַ����
        server_addr.sin_addr.s_addr = (gStUfoData.ipLastAdr << 24) + (1 << 16) + (168 << 8) + 192;//ע��ת��Ϊ�����ֽ���
        server_addr.sin_port = htons(502);                          //ʹ��SERVER_PORTָ��Ϊ����ͷ�趨�Ķ˿ں�

        err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));  //������
        if (err < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("TCP bind failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //�ر��׽���
            osDelay(1000);
            continue;
        }

        err = listen(sock_fd, 3);                                   //������������
        if (err < 0)                                                //�������ʧ����ر��׽���
        {
            rt_kprintf("TCP listen failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //�ر��׽���
            osDelay(1000);
            continue;
        }

        addr_len = sizeof(struct sockaddr_in);                      //�����ӵ�ַ��ֵ��addr_len

        while(IsMxLwipNetifLinkUp())
        {
            sock_conn_new = accept(sock_fd, (struct sockaddr *)&socketConAddrNew, &addr_len);  //�Լ�����������������ӣ�״̬��ֵ��sock_conn
            if(sock_conn_new >= 0)  
            {
                closeSelectSocket(&sock_conn);                
                rt_kprintf("TCP Client connect success: %d!\r\n", sock_conn_new);
                sock_conn = sock_conn_new;
                memcpy((uint8_t*)(&socketConAddr), (uint8_t*)(&socketConAddrNew), sizeof(struct sockaddr_in));

                lingerValue.l_onoff = 1;
                lingerValue.l_linger = 0;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_LINGER, &lingerValue, sizeof(lingerValue));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP setopt1 failed:%d!\r\n", err);
                }
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));//SO_REUSEADDR|SO_RCVTIMEO  SO_KEEPALIVE
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP setopt2 failed:%d!\r\n", err);
                }
                timeout.tv_sec = 0;
                timeout.tv_usec = 200000;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));//SO_REUSEADDR|SO_RCVTIMEO
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP setopt3 failed:%d!\r\n", err);
                }
                //����Nagle�㷨��ʹTCP�����������ݶ������л���
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP setopt4 failed:%d!\r\n", err);
                }
            }
            else                                         //״̬С��0�������ӹ��ϣ���ʱ�ر��׽���
            {
                rt_kprintf("TCP Client connect failed: %d!\r\n", sock_conn_new);
                if (tcpModbusConFailCnt != 0xFFFFFFFF)
                {
                    tcpModbusConFailCnt++;
                }
                else
                {
                    tcpModbusConFailCnt = 0;//��ֹ�������
                }
            }
            osDelay(10);
        }

        rt_kprintf("TCP socket close!\r\n");
        sock_conn = -1;
        if (tcpModbusSotClsCnt != 0xFFFFFFFF)  
        {
            tcpModbusSotClsCnt++;
        }
        else
        {
            tcpModbusSotClsCnt = 0;         //��ֹ�������
        }
        closesocket(sock_fd);                           //�ر��׽���
    }
    else
    {
        sock_conn = -1;
    }
    
    osDelay(1000);
  }
  /* USER CODE END StartTaskTcpCreate */
}

/* USER CODE BEGIN Header_StartTaskTcp */
/**
* @brief Function implementing the myTaskTcp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTcp */
void StartTaskTcp(void const * argument)
{
  /* USER CODE BEGIN StartTaskTcp */
  TickType_t l_cur_tick, l_print_tick = 0;
  
  /* init code for LWIP */
  MX_LWIP_Init();
  /* Infinite loop */
  for(;;)
  {
    l_cur_tick = xTaskGetTickCount();
    
    if(!((gStUfoData.flag & UFO_PC_CAN_FLAG) && (IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG))))  //���ػ�canͨ�ű�־�򴮿�ͨ�ű�־
    {
        if(sock_conn >= 0)                                         //״̬С��0�������ӹ��ϣ���ʱ�ر��׽���
        {
            if(gStUfoData.flag & UFO_NEW_PROTOCOL)  //��ͨ��Э��
            {
                TcpSocketProcessNew();
            }
            else
            {
                TcpSocketProcess(l_cur_tick);
            }
        }
    }

    if(l_cur_tick - l_print_tick >= 5)
    {
        l_print_tick = l_cur_tick;
        //���ݴ�ӡ����1�����ݻ���100ms
        rt_kprintf_log_buf();

        //udp�����
        if(udp_print_connected_flag >= 0)
        {
            TcpPrintRevMsgProcess();
        }
    }
    
    osDelay(2);
  }
  /* USER CODE END StartTaskTcp */
}

/* USER CODE BEGIN Header_StartTaskSD */
/**
* @brief Function implementing the myTaskSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSD */
void StartTaskSD(void const * argument)
{
  /* USER CODE BEGIN StartTaskSD */
    TickType_t l_last_tick = 0, l_tmp_tick = 0, l_cur_tick;

    InitBatteryData();
    
#ifdef SD_RW_ENABLE
    uint8_t dataBuf[SD_WRITE_BLOCK_SIZE];
    EventBits_t r_event;
  
    retSD=f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);//����


    if(retSD==FR_OK)
    {
        rt_kprintf("����SD���ɹ�!\r\n");
    }
#endif
  /* Infinite loop */
  for(;;)
  {
#ifdef SD_RW_ENABLE
    //��¼��־
    r_event = xEventGroupWaitBits(EventSDLog,   /*�¼�������*/
                              EVENT_WRITE_LOG | EVENT_READ_LOG,  /*�����������Ȥ���¼�*/
                              pdTRUE,   /* �˳�ʱ����¼�λ */
                              pdFALSE,  /* �������Ȥ�������¼� */
                              1000);    /* ָ����ʱ�¼�,�ȴ�1s */

    if(r_event & EVENT_READ_LOG)
    {
        ReadInfoFromSdCard();
    }
    if (((r_event & EVENT_WRITE_LOG) && (SDLogInfoIndex >= SD_WRITE_BLOCK_SIZE))
        || ((0 == r_event) && (SDLogInfoIndex > 0)))
    {
        WriteInfoToSdCard(EVENT_WRITE_LOG, dataBuf, SDLogInfoBuf, &SDLogInfoIndex);
    }
#endif

    //�����Ϣ�ɼ�
    l_cur_tick = xTaskGetTickCount();
    if((l_cur_tick - l_last_tick >= BATTERY485_READ_PERIOD)
        || ((l_cur_tick - l_last_tick >= 20)
        && ((gStUfoData.flag & UFO_BMS_JIKONG)
        || (gStUfoData.flag & UFO_BMS_BAIWEI))))
    {
        l_last_tick = l_cur_tick;
        BatteryCollectProcess(RT_FALSE);
    }

    //�¶���Ϣ�ɼ�
    if(l_cur_tick - l_tmp_tick >= 50)
    {
        l_tmp_tick = l_cur_tick;
        ReadTmpProcess();
    }
    
    osDelay(20);
    //readTest();
  }
  /* USER CODE END StartTaskSD */
}

/* USER CODE BEGIN Header_StartTaskTcpPrint */
/**
* @brief Function implementing the myTaskTcpPrintC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTcpPrint */
void StartTaskTcpPrint(void const * argument)
{
  /* USER CODE BEGIN StartTaskTcpPrint */
  //////UDP
  struct sockaddr_in server_addr;			//��������ַ
  int err;
  /* Infinite loop */
  for(;;)
  {
    if(IsMxLwipNetifLinkUp())
    {
        udp_sock_num = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);        //����һ���µ�socket����
        if (udp_sock_num < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("udp print socket create failed:%d!\r\n", udp_sock_num);
            udp_print_connected_flag = -1;
            osDelay(1000);
            continue;
        }
    
        memset(&server_addr, 0, sizeof(server_addr));               //����������ַ���
        server_addr.sin_family = AF_INET;                           //��ַ����
        server_addr.sin_addr.s_addr = (gStUfoData.ipLastAdr << 24) + (1 << 16) + (168 << 8) + 192;//ע��ת��Ϊ�����ֽ���
        server_addr.sin_port = htons(211);                           //ʹ��SERVER_PORTָ��Ϊ����ͷ�趨�Ķ˿ں�

        err = bind(udp_sock_num, (struct sockaddr *)&server_addr, sizeof(server_addr));  //������
        if (err < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("udp print bind failed:%d!\r\n", err);
            udp_print_connected_flag = -1;
            closesocket(udp_sock_num);                                   //�ر��׽���
            udp_sock_num = -1;
            osDelay(1000);
            continue;
        }

        udp_print_connected_flag = 0;

        while(IsMxLwipNetifLinkUp())
        {
            osDelay(500);
        }

        udp_print_connected_flag = -1;
        rt_kprintf("udp print socket close!\r\n");
        closesocket(udp_sock_num);                           //�ر��׽���
        udp_sock_num = -1;
    }
    else
    {
        udp_print_connected_flag = -1;
    }
    
    osDelay(1000);
  }
  //////TCP
  /*int sock_fd;				                //�������� socked
  int sock_conn_new;
  struct sockaddr_in server_addr;			//��������ַ
  int err;
  int opt = 1;
  struct timeval timeout;
  struct linger lingerValue;
  //uint32_t keepalive_value;
  socklen_t addr_len;
  struct sockaddr_in socketConAddrNew;		//���ӵ�ַ���ͻ��˵�Э���ַ
  /# Infinite loop #/
  for(;;)
  {
    if(IsMxLwipNetifLinkUp())
    {
        sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);        //����һ���µ�socket����
        if (sock_fd < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("TCP print socket create failed:%d!\r\n", sock_fd);
        }
        /#err = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(int)); //�˿ڿ�����
        if (err < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("TCP print setopt failed:%d!\r\n", err);
        }#/
    
        memset(&server_addr, 0, sizeof(server_addr));               //����������ַ���
        server_addr.sin_family = AF_INET;                           //��ַ����
        server_addr.sin_addr.s_addr = (gStUfoData.ipLastAdr << 24) + (1 << 16) + (168 << 8) + 192;//ע��ת��Ϊ�����ֽ���
        server_addr.sin_port = htons(211);                           //ʹ��SERVER_PORTָ��Ϊ����ͷ�趨�Ķ˿ں�

        err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));  //������
        if (err < 0)                                                                //�����ʧ����ر��׽���
        {
            rt_kprintf("TCP print bind failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //�ر��׽���
            osDelay(1000);
            continue;
        }

        err = listen(sock_fd, 2);                                   //������������
        if (err < 0)                                                //�������ʧ����ر��׽���
        {
            rt_kprintf("TCP print listen failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //�ر��׽���
            osDelay(1000);
            continue;
        }

        addr_len = sizeof(struct sockaddr_in);                      //�����ӵ�ַ��ֵ��addr_len

        while(IsMxLwipNetifLinkUp())
        {
            sock_conn_new = accept(sock_fd, (struct sockaddr *)&socketConAddrNew, &addr_len);  //�Լ�����������������ӣ�״̬��ֵ��sock_conn
            if(sock_conn_new >= 0)  
            {
                closeSelectSocket(&sock_print_conn); 
                rt_kprintf("TCP print connect success: %d!\r\n", sock_conn_new);
                sock_print_conn = sock_conn_new;

                lingerValue.l_onoff = 1;
                lingerValue.l_linger = 0;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_LINGER, &lingerValue, sizeof(lingerValue));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt1 failed:%d!\r\n", err);
                }
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));//SO_REUSEADDR|SO_RCVTIMEO  SO_KEEPALIVE
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt2 failed:%d!\r\n", err);
                }
                timeout.tv_sec = 0;
                timeout.tv_usec = 500000;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt3 failed:%d!\r\n", err);
                }
                //����Nagle�㷨��ʹTCP�����������ݶ������л���
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt4 failed:%d!\r\n", err);
                }

                // set Keep-Alive options
                /#keepalive_value = 10000;    // ���ݿ���ʱ���ú��� 10 seconds
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive_value, sizeof(keepalive_value));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt5 failed:%d!\r\n", err);
                }
                keepalive_value = 5000;     // ���Լ����5 seconds
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive_value, sizeof(keepalive_value));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt6 failed:%d!\r\n", err);
                }
                keepalive_value = 3;        // ���Դ���
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_KEEPCNT, &keepalive_value, sizeof(keepalive_value));
                if (err < 0)                                                                //�����ʧ����ر��׽���
                {
                    rt_kprintf("TCP print setopt7 failed:%d!\r\n", err);
                }#/
            }
            else                                         //״̬С��0�������ӹ��ϣ���ʱ�ر��׽���
            {
                rt_kprintf("TCP print connect failed: %d!\r\n", sock_conn_new);
            }
            osDelay(10);
        }

        rt_kprintf("TCP print socket close!\r\n");
        sock_print_conn = -1;
        closesocket(sock_fd);                           //�ر��׽���
    }
    else
    {
        sock_print_conn = -1;
    }
    
    osDelay(1000);
  }*/
  /* USER CODE END StartTaskTcpPrint */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void AllDataInit(void)
{
    //��ȡflash��������
    gUartNumWifiPrint = USART1_DEVICE;
    if(0 == FlashReadWord(FLASH_USART6_USE_FLAG_ADDR))
    {
        gUartNumWifiPrint = USART6_DEVICE;
    }
    FlashReadConfigurePara();
    FlashReadMotorPara();
    gFlashData.resetCnt++;  //��λ����+1
    gU8FlashWriteFlag = 1;  //дflash

    //����������ʼ��
    MX_UART5_Init();      //����5��Ҫ���ݴ洢��flash�еĲ���������ͨѶ�����ʣ�����ڴ˴����г�ʼ��

    MX_CAN1_Init();
#ifdef USE_CAN2_DEVICE
    MX_CAN2_Init();
#endif
#ifdef USE_CAN3_DEVICE
    MX_CAN3_Init();
#endif
    AppUsartInit();        //������Ҫ�Ĵ��ڳ�ʼ������

    InitMotorPara();
    //NavDataInit();
#if (BOOTLOADER_ENABLE == 0)
    MX_IWDG_Init();       //��BootLoader��ʱ����Ҫ��ʼ�����Ź���������BootLoader��ʼ������ֹ�ڴ˳�ʼ��ʧ��
#endif

    GPIOInitState();

    PWMInit();
	
    InputCaptureInit(&gStTim5Data);

    CanStart(CAN1_DEVICE);
#ifdef USE_CAN2_DEVICE
    CanStart(CAN2_DEVICE);
#endif
#ifdef USE_CAN3_DEVICE
    CanStart(CAN3_DEVICE);
#endif
    AdcStart();
    PrintfHardwareType(RT_TRUE);
}
/*****************************************************************************
 ��������  : �ر�ָ����ŵ�socket����
 �������  : int* sockNum
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��4��20��
*****************************************************************************/
static void closeSelectSocket(int* sockNum)
{
    int sockTemp;
    if(*sockNum >= 0)
    {
        sockTemp = *sockNum;
        *sockNum = -1;
        close(sockTemp);
        if(sockNum == (&sock_conn))
        {
            rt_kprintf("TCP Client close success:%d!\r\n", sockTemp);
        }
        else
        {
            rt_kprintf("TCP print close success:%d!\r\n", sockTemp);
        }
    }
}
/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file
            when the callback is needed,
  */
    if(netif_is_link_up(netif) && !netif_is_up(netif))
    {
        netif_set_up(netif);
    }
}
/*****************************************************************************
 ��������  : SBUS ���ݽ���
 �������  : uint8_t* cmdData   ��������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��10��
*****************************************************************************/
#define hex_buffer  (sys_para->SBUS_rx.CH)
//#define YM_MAX_mini 700
static void SBUS_ENcode(uint8_t* cmdData)
{
    hex_buffer[0] = ((int16_t)cmdData[ 1] >> 0 | ((int16_t)cmdData[ 2] << 8 )) & 0x07FF;
    hex_buffer[1] = ((int16_t)cmdData[ 2] >> 3 | ((int16_t)cmdData[ 3] << 5 )) & 0x07FF;
    hex_buffer[2] = ((int16_t)cmdData[ 3] >> 6 | ((int16_t)cmdData[ 4] << 2 ) | (int16_t)cmdData[ 5] << 10 ) & 0x07FF;
    hex_buffer[3] = ((int16_t)cmdData[ 5] >> 1 | ((int16_t)cmdData[ 6] << 7 )) & 0x07FF;
    hex_buffer[4] = ((int16_t)cmdData[ 6] >> 4 | ((int16_t)cmdData[ 7] << 4 )) & 0x07FF;
    hex_buffer[5] = ((int16_t)cmdData[ 7] >> 7 | ((int16_t)cmdData[ 8] << 1 ) | (int16_t)cmdData[9] << 9 ) & 0x07FF;
    hex_buffer[6] = ((int16_t)cmdData[ 9] >> 2 | ((int16_t)cmdData[10] << 6 )) & 0x07FF;
    hex_buffer[7] = ((int16_t)cmdData[10] >> 5 | ((int16_t)cmdData[11] << 3 )) & 0x07FF;
    hex_buffer[8] = ((int16_t)cmdData[12] << 0 | ((int16_t)cmdData[13] << 8 )) & 0x07FF;
    hex_buffer[9] = ((int16_t)cmdData[13] >> 3 | ((int16_t)cmdData[14] << 5 )) & 0x07FF;
    //hex_buffer[10] = ((int16_t)cmdData[14] >> 6 | ((int16_t)cmdData[15] << 2 ) | (int16_t)cmdData[16] << 10 ) & 0x07FF;
    //hex_buffer[11] = ((int16_t)cmdData[16] >> 1 | ((int16_t)cmdData[17] << 7 )) & 0x07FF;
    //hex_buffer[12] = ((int16_t)cmdData[17] >> 4 | ((int16_t)cmdData[18] << 4 )) & 0x07FF;
    //hex_buffer[13] = ((int16_t)cmdData[18] >> 7 | ((int16_t)cmdData[19] << 1 ) | (int16_t)cmdData[20] << 9 ) & 0x07FF;
    //hex_buffer[14] = ((int16_t)cmdData[20] >> 2 | ((int16_t)cmdData[21] << 6 )) & 0x07FF;
    //hex_buffer[15] = ((int16_t)cmdData[21] >> 5 | ((int16_t)cmdData[22] << 3 )) & 0x07FF;

    if(DEBUG_DATA_TYPE_89)
    {
        rt_kprintfArray((int16_t*)hex_buffer, 10, 10, 2);
        rt_kprintf("\r\ntime:%d.\r\n\r\n", HAL_GetTick());
    }
}
/*****************************************************************************
 ��������  : subs���ݽ��մ���
 �������  : TickType_t curTime  ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��10��
*****************************************************************************/
static void SBUS_ReceiveProcess(TickType_t curTime)
{
    static uint16_t lSbusIndex = 0, lSize, lSbusStart = 0, lFilterCnt = 0;
    static TickType_t lLastRevTime = 0; //�ϴν������ݵ�ʱ��
    uint16_t lBufferLen = sizeof(gSbusData);
    uint8_t lFlag;

    //���ճ�ʱ�ж�
    if(0 == lSbusIndex)
    {
        if(curTime - lLastRevTime >= 200)   //200msδ�յ����ݣ���ң�ص��߱�־
        {
            lLastRevTime = curTime;
            GPIOSetLedState(LED0, LED_OFF);
            sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;
        }
    }
    else if(curTime - lLastRevTime >= 500)  //500msδ����һ����������ջ�����
    {
        lLastRevTime = curTime;
        lSbusStart = 0;
        lSbusIndex = 0;
    }

    //��������
    if(lSbusIndex > lBufferLen)//���ճ���
    {
        lSbusIndex = 0;
    }
    else if(lSbusIndex < lBufferLen) //��������
    {
        lSize = UsartDeviceRead(USART4_DEVICE, &gSbusData[lSbusIndex], lBufferLen - lSbusIndex);
        if(lSize > 0)
        {
            if(DEBUG_DATA_TYPE_88)
            {
                rt_kprintfArray(&gSbusData[lSbusIndex], lSize, 16, 1);
                rt_kprintf("\r\n\r\n");
            }
            lSbusIndex += lSize;
        }
    }
    while(lSbusStart + SBUS_DATA_LEN <= lSbusIndex) //������һ������������
    {
        if((0x0f == gSbusData[lSbusStart]) && (0x00 == gSbusData[lSbusStart + SBUS_DATA_LEN - 1]))    //У����ȷ
        {
            lFlag = gSbusData[lSbusStart + SBUS_DATA_LEN - 2] & 0x0c;
            if(0 == lFlag)
            {
                lFilterCnt = 0;
                SBUS_ENcode(&gSbusData[lSbusStart]);  //���ݽ���
                sys_para->CAR_RTinf.Link &= ~LINK_REMOT_OFF;//�˳�ʧ��״̬
            }
            else if(!(sys_para->CAR_RTinf.Link & LINK_REMOT_OFF))
            {
                if(lFlag & 0x08)
                {
                    lFilterCnt += 2;    //ʧ��+2
                }
                else
                {
                    lFilterCnt++;   //��֡+1
                }
                if(lFilterCnt > 5)
                {
                    sys_para->CAR_RTinf.Link |= LINK_REMOT_OFF; //ң�عرջ���ʧ��״̬
                    if(DEBUG_DATA_TYPE_2 || DEBUG_DATA_TYPE_89 || DEBUG_DATA_TYPE_88)
                    {
                        rt_kprintf("Remote lineoff!\r\n");
                    }
                }
            }
            sys_para->CAR_RTinf.Link |= LINK_REMOTE_DATA;
            lSbusStart += SBUS_DATA_LEN;
            lLastRevTime = curTime;
            GPIOSetLedState(LED0, LED_ON);
        }
        else
        {
            lSbusStart++;
        }
    }
    if(lSbusStart >= SBUS_DATA_LEN) //�����곬��һ������ݣ������ε���������ǰ���
    {
        if(lSbusStart <= lSbusIndex)
        {
            lSbusIndex = lSbusIndex - lSbusStart;
            if(lSbusIndex > 0)
            {
                memcpy(gSbusData, &gSbusData[lSbusStart], lSbusIndex);
            }
        }
        lSbusStart = 0;
    }
}
/*****************************************************************************
 ��������  : ���ػ�can��Ϣ����
 �������  : TickType_t curTime
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��5��4��
*****************************************************************************/
static void CAN_ReceiveProcess(TickType_t curTime)
{
    CAN_msg msgRead;
    unsigned char crc8Value;
    static unsigned short lastCnt = 256;
    static TickType_t lastTime = 0;
    
    //��CAN������
    if(HAL_OK == CanDeviceRead(CAN2_DEVICE, &msgRead, 0))
    {
        if(0x980 == msgRead.id)
        {
            crc8Value = CRC8_Table_1(msgRead.data, 7);
            if(crc8Value == msgRead.data[7])
            {
                if((lastCnt < 256) && (((unsigned char)(lastCnt + 1)) != msgRead.data[6])) //֡��Ŵ���
                {
                    rt_kprintf("CAN2 cnt err, last:%d, cur:%d!\r\n", lastCnt, msgRead.data[6]);
                }
                lastCnt = msgRead.data[6];
                sys_para->PC_Remote.YM = (unsigned short)(-(*((short*)msgRead.data)));
                sys_para->PC_Remote.ZX = *(unsigned short*)(&msgRead.data[2]);
                sys_para->PC_Remote.VEL = *(unsigned short*)(&msgRead.data[4]);
                sys_para->PC_Remote.max_vel = 0;
                sys_para->CAR_RTinf.Link |= LINK_PC_DATA;
                if(DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_87)
                {
                    rt_kprintf("YM:%d,ZX:%d,Vel:%d,t:%d.\r\n", (short)(sys_para->PC_Remote.YM),
                        (short)(sys_para->PC_Remote.ZX), (short)sys_para->PC_Remote.VEL, curTime - lastTime);
                    lastTime = curTime;
                }
            }
            else
            {
                rt_kprintf("CAN2 crc err!\r\n");
            }
        }
        else if(0x982 == msgRead.id)
        {
            if(sys_para->CAR_RTinf.Link & LINK_PC)
            {
                if(msgRead.data[0] & 0x08)
                {
                    if(!(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP))
                    {
                        rt_kprintf("Enter pc quickStop.\r\n");
                    }
                    sys_para->CAR_RTinf.Link |= LINK_QUICK_STOP; //����ֹͣ
                }
                else if(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP)
                {
                    rt_kprintf("Exit pc quickStop.\r\n");
                    sys_para->CAR_RTinf.Link &= ~LINK_QUICK_STOP;
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : ���ػ�can��Ϣ����
 �������  : rt_bool_t firstFlag    :��һ��id��־
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��5��4��
*****************************************************************************/
static void CAN_SendProcess(rt_bool_t firstFlag)
{
    CAN_msg msg = {0, {0x00}, 8, 1, EXTENDED_FORMAT, DATA_FRAME};
    static uint32_t lId = 0x991;
    int32_t i;

    msg.id = lId;
    
    if(firstFlag)
    {
        if(0x991 == lId)
        {
            msg.data[0] = gRespondState.batteryVoltage & 0xff;
            msg.data[1] = (gRespondState.batteryVoltage >> 8) & 0xff;
            msg.data[2] = gRespondState.batteryCurrent & 0xff;
            msg.data[3] = (gRespondState.batteryCurrent >> 8) & 0xff;
            msg.data[4] = gRespondState.batterySoc;
            msg.data[5] = gErrorMotorNum;
            msg.data[6] = gErrorResult & 0xff;
            msg.data[7] = (gErrorResult >> 8) & 0xff;
            CanDeviceWrite(CAN2_DEVICE, &msg, 2);
            lId++;
        }
    }
    else if(0x992 == lId)
    {
        lId++;
    }
    else if(0x993 == lId)
    {
        for(i = 0; i < 8; i++)
        {
            msg.data[i] = gRespondState.motorTmp[i];
        }
        CanDeviceWrite(CAN2_DEVICE, &msg, 2);
        lId++;
    }
    else if(0x994 == lId)
    {
        for(i = 0; i < 8; i++)
        {
            msg.data[i] = gRespondState.driverTmp[i];
        }
        CanDeviceWrite(CAN2_DEVICE, &msg, 2);
        lId++;
    }
    else if(0x995 == lId)
    {
        for(i = 0; i < 6; i++)
        {
            msg.data[i] = gRespondState.moduleTmp[i];
        }
        CanDeviceWrite(CAN2_DEVICE, &msg, 2);
        lId++;
    }
    else if(0x996 == lId)
    {
        msg.data[0] = gRespondState.ctrlState & 0xff;
        msg.data[1] = gRespondState.turnTargetPos & 0xff;
        msg.data[2] = (gRespondState.turnTargetPos >> 8) & 0xff;
        msg.data[3] = gRespondState.turnCurPos & 0xff;
        msg.data[4] = (gRespondState.turnCurPos >> 8) & 0xff;
        CanDeviceWrite(CAN2_DEVICE, &msg, 2);
        lId++;
    }
    else
    {
        lId = 0x991;
    }
}
/*****************************************************************************
 ��������  : ���ػ�������Ϣ����
 �������  : uint32_t deviceNum ���ں�
             Usart_msg *pUsartMsg ������Ϣָ��
             TickType_t curTime ��ǰʱ��(ms)
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��6��15��
*****************************************************************************/
static void USART_ReceiveProcess(uint32_t deviceNum, Usart_msg *pUsartMsg, TickType_t curTime)
{
    static TickType_t lLastRevTime = 0;
    uint32_t revLen = 0;
    int32_t sendLen = 0;

    ST_USART_DATA* uart = UsartGetSelectDevice(deviceNum);
  
    //��������
    if (!IsRxBuffEmpty(*pUsartMsg))      // ���ջ������Ƿ�Ϊ��
    {
        revLen = UsartDeviceRead(deviceNum, pUsartMsg->dataBuf, pUsartMsg->rxCount[pUsartMsg->outIndex]);
        if (revLen > 0)
        {
            if(DEBUG_DATA_TYPE_8A)
            {
                test_kprintf("L%dt%d\n", revLen, HAL_GetTick() / 100 % 100);
            }
            sendLen = ParsingAccessCommand(MB_RTU, pUsartMsg->dataBuf, revLen, uart->sendBuf);
            if(sendLen > 0)
            {
                sendLen = UsartDeviceWrite(deviceNum, uart->sendBuf, sendLen);//����
                if((sendLen <= 0) && DEBUG_DATA_TYPE_1)
                {
                     rt_kprintf("Usart send gSendRespondState err!\r\n");
                }
            }
            lLastRevTime = curTime;
            GPIOSetLedState(LED1, LED_ON);
        } 
        USARTRx_DEQUEUE(*pUsartMsg);        // ����5�������ݳ���
    }
    else if(curTime - lLastRevTime >= 1000) //����1sδ�յ����ݣ��ر�����ָʾ��
    {
        lLastRevTime = curTime;
        GPIOSetLedState(LED1, LED_OFF);
    }
}

/*****************************************************************************
 ��������  : ufoɲ���Զ�����
 �������  : TickType_t curTime
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��25��
*****************************************************************************/
static void UfoBrakeAutoControl(TickType_t curTime, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    //static TickType_t lastTime = 0;
    uint8_t brakeControlFlag = 0; 

    //�ȴ�ɲ���ϵ�
    if(BRAKE_WAIT == gBrakeWaitOnFlag)
    {
        if(((BRAKE_ON == gBrakeFlag) || (BRAKE_POS_CMD == gBrakeFlag))
            && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))
        {
            if((IS_NOT_CANOPEN_DRIVER(gStMotorData[M_BRAKE].driverType)   //��canopen������
                && (MOTOR_POWER_FLAG_ON == gStMotorRunState[M_BRAKE].powerFlag))
                || (OPERATION_MODE_SET_FINISH == gStMotorRunState[M_BRAKE].operModeSetStep))
            {
                gBrakeWaitOnFlag = BRAKE_WAIT_NONE;
            }
        }
        else
        {
            gBrakeWaitOnFlag = BRAKE_WAIT_NONE;
        }
    }
    
    //ɲ������
    if(BRAKE_POS_CMD == gBrakeFlag)  //ɲ��λ��ģʽ
    {
        gBrakeFlag = BRAKE_ON;
        brakeControlFlag = 1;
        if(gStMotorData[M_BRAKE].flag & MOTOR_CURRENT_ADJUST_SPEED) //��������ת��
        {
            lMotorRunMode->run_mode = MOTOR_RUN_MODE_CURRENT_SPEED;
            lMotorRunMode->posType = POS_TYPE_NULL;
        }
        else
        {
            lMotorRunMode->run_mode = MOTOR_RUN_MODE_POS;
            lMotorRunMode->posType = POS_TYPE_MULTI_ABSOLUTE;
        }

        lMotorRunMode->target_value = Limit(gBrakeValue, 0, 700);
        lMotorRunMode->homming_type = HOMMING_TYPE_NULL;
    }
    else if((BRAKE_OFF_CMD == gBrakeFlag) || (BRAKE_OFF_FORCE_CMD == gBrakeFlag))   //ɲ���ϵ�ģʽ
    {
        if(BRAKE_OFF_CMD == gBrakeFlag)
        {
            gBrakeFlag = BRAKE_OFF;
        }
        else
        {
            gBrakeFlag = BRAKE_OFF_FORCE;
        }
        brakeControlFlag = 1;
        lMotorRunMode->target_value = 0;
        lMotorRunMode->run_mode = MOTOR_RUN_MODE_POWER_OFF;
    } 

    if(brakeControlFlag)    //ɲ�������л�
    {
        LockThread();
        SetMotorRunModeData(M_BRAKE, lMotorRunMode);
        UnLockThread();
    }
}
/*****************************************************************************
 ��������  : ���ҵ���ٶ�ֵ���ơ����ٿ���
 �������  : int16_t targetValue
            int16_t Steering_Angle                    
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��9��19��
*****************************************************************************/
void UfoLeftRightControl(int16_t targetValue, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    if(gStMotorData[M_LEFT].flag & MOTOR_CURRENT_ADJUST_SPEED) //��������ת��
    {
        lMotorRunMode->run_mode = MOTOR_RUN_MODE_FLAP_FIX_CURRENT;
    }
    else
    {
        lMotorRunMode->run_mode = MOTOR_RUN_MODE_SPEED;
    }

    lMotorRunMode->target_value = targetValue;

    if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG) //���Ҳ����������Ʊ�־
    {
        SetMotorRunModeData(M_LEFT, lMotorRunMode);
        SetMotorRunModeData(M_RIGHT, lMotorRunMode);
        if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
        {
            SetMotorRunModeData(M_LEFT_ONE, lMotorRunMode);
            SetMotorRunModeData(M_RIGHT_ONE, lMotorRunMode);
        }
    }
    else
    {
        SetMotorRunModeData(M_LEFT, lMotorRunMode);
    }
}
/*****************************************************************************
 ��������  : ����ֹͣ���̴���
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��12��6��
*****************************************************************************/
static void UfoEmergencyStopProcess( void )
{
    uint32_t lTime;

    lTime = HAL_GetTick() - gErrorTime;
    //���ϻ�ң�ضϵ����ֹͣʱ���ж��˳�����
    if(sys_para->CAR_RTinf.Link & (LINK_ERROR_STOP | LINK_REMOTE_STOP))
    {
        //�ж��������pid�Ƿ����
        if((M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)
            && (lTime >= 500)) //����������500ms֮�����������ֹͣ��־
        {
            if(DEBUG_DATA_TYPE_1)
            {
                rt_kprintf("Exit emergency stop:%#x.\r\n", sys_para->CAR_RTinf.Link);
            }
            sys_para->CAR_RTinf.Link &= ~(LINK_ERROR_STOP | LINK_REMOTE_STOP);
        }
    }

    //�������ֹͣʱ�ر�բ
    if(IS_EMERGENCY_STOP_NOT_ERR)   //�ǹ��Ͻ���ֹͣ�¹ر�բ
    {
        if(LOCK_OFF != gLockFlag)
        {
            rt_kprintf("Enter stop:%#x.\r\n", sys_para->CAR_RTinf.Link);
            CLOSE_LOCK; //�رձ�բ
            CLOSE_BRAKE_FORCE;//���������ж��Ƿ�ǿ�й�ɲ��
            if((BRAKE_OFF_FORCE == gBrakeFlag) || (BRAKE_OFF_FORCE_CMD == gBrakeFlag))
            {
                gBrakeWaitOnFlag = BRAKE_WAIT_PRE;
            }
        }
    }

    //�˳�����ֹͣ�󿪱�բ
    if(!IS_EMERGENCY_STOP)
    {
        if(LOCK_OFF == gLockFlag)
        {
            //�����������������ģʽ�Ѿ��������ʱ���ſ���բ
			if((OPERATION_MODE_SET_FINISH == gStMotorRunState[M_LEFT].operModeSetStep)
            || (OPERATION_MODE_SET_FINISH == gStMotorRunState[M_LEFT_ONE].operModeSetStep)
            || (OPERATION_MODE_SET_FINISH == gStMotorRunState[M_RIGHT].operModeSetStep)
            || (OPERATION_MODE_SET_FINISH == gStMotorRunState[M_RIGHT_ONE].operModeSetStep)
              )
            {
                OPEN_LOCK;
            }
        }
    }
}
/*****************************************************************************
 ��������  : ufo�������������ch[4]ΪF��λ��δʹ��
 �������  : TickType_t curTime ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��10��
*****************************************************************************/
static void UfoControlCmdAnalysis(TickType_t curTime)
{
    static TickType_t lastTime = 0, lastRemoteDataTime = 0;
    static u8_t cntFilter = 0, cntFilter1 = 0;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode = {0};      //��ʼ���ֲ���������ֹ����ֵ���ɿأ�����쳣
    int YM_Limit, YM_Max, YM_Mid = 600, YM_Min = 240;

    UfoEmergencyStopProcess();                      //����ֹͣ���̴���

    UfoBrakeAutoControl(curTime, &lMotorRunMode);   //ɲ���Զ�����

    if(sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) //���յ�ң������
    {
        //CH[8]����1000�ϵ�(��B������)���ҷ� ң�ؿ��ƻ�A�����������ң�عرջ�ʧ��
        if((sys_para->SBUS_rx.CH[8] > 1000) && (!(((sys_para->CAR_RTinf.Link & LINK_REMOT_OFF)) && ((sys_para->CAR_RTinf.Link & LINK_REMOTE) || (sys_para->SBUS_rx.CH[9] <= 1000)))))
        {
            if(!(sys_para->CAR_RTinf.Link & LINK_POWER_ON))
            {
                sys_para->CAR_RTinf.Link |= LINK_POWER_ON;              //�·��ϵ�����
                SET_BRAKE(0);  //�ϵ����Ҫ��ɲ��λ��ģʽ���Խ���Ѱ��ģʽ����ɲ��λ����ѧϰ
                ClearErrorCode(M_TOTAL_NUM);        //ң�������ϵ�ʱ������ϱ�־
            }
            cntFilter1 = 0;
        }
        else
        {
            cntFilter1++;
            if(cntFilter1 >= 5)
            {
                cntFilter1 = 0;
                if(sys_para->CAR_RTinf.Link & LINK_POWER_ON)
                {
                    sys_para->CAR_RTinf.Link |= LINK_REMOTE_STOP; //����ң�ؽ���ֹͣ״̬
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    LockThread();
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POWER_OFF;
                    SetMotorRunModeData(M_LEFT, &lMotorRunMode);
                    if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)  //���Ҳ����������Ʊ�־
                    {
                        SetMotorRunModeData(M_RIGHT, &lMotorRunMode);
                    }
                    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
                    {
                        SetMotorRunModeData(M_LEFT_ONE, &lMotorRunMode);
                        SetMotorRunModeData(M_RIGHT_ONE, &lMotorRunMode);
                    }
                    SetMotorRunModeData(M_TURN, &lMotorRunMode);
                    UnLockThread();
                    if(DEBUG_DATA_TYPE_5)
                    {
                        rt_kprintf("Power-off:%d,%d,%d,%d.\r\n", sys_para->SBUS_rx.CH[8], sys_para->SBUS_rx.CH[2], sys_para->SBUS_rx.CH[9], sys_para->CAR_RTinf.Link);
                    }
                }
                sys_para->CAR_RTinf.Link &= ~LINK_POWER_ON;
            }
        }

        if(DEBUG_DATA_TYPE_82)
        {
            rt_kprintf("YM: %d, ZX: %d\r\n", hex_buffer[2], hex_buffer[3]);
        }
    }
    
    if(!(sys_para->CAR_RTinf.Link & LINK_POWER_ON)) //δ�ϵ粻���п���
    {
        //����ͣ��ң��δ��ʱ��ʹ�õ��ǲ��Թ��߷�ָ����ƣ����ڴ˴�ֹͣ�������
        ErrorAutoStopCmd(&lMotorRunMode);
        return;
    }

    //�л���������״̬��ң��orPC
    if(sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) //���յ�ң������
	{
        if(sys_para->SBUS_rx.CH[9] > 1000)  //�Զ�ģʽ��λ
		{
            sys_para->CAR_RTinf.Link |= LINK_HAND_OVER;  //�л����ƽ�״̬
            if(gFlashData.testMode) //�л����Զ���������
            {
                if(!(sys_para->CAR_RTinf.Link & LINK_AUTO_NAV))
                {
                    rt_kprintf("Enter auto nav control mode!\r\n");
                    sys_para->CAR_RTinf.Link |= LINK_AUTO_NAV;  //�л����Զ�����
        	        sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;   //�˳�ң�ؿ���
        	        sys_para->CAR_RTinf.Link &= ~LINK_PC;       //�˳�PC����
                }
            }
            else
            {
                if(sys_para->CAR_RTinf.Link & LINK_PC_DATA)  //���յ�PC����
            	{
                    if(!(sys_para->CAR_RTinf.Link & LINK_PC))   //�л�ʱ�˲�
            	    {
                        cntFilter++;
                        if(cntFilter >= 5)
            	        {
            	            sys_para->CAR_RTinf.Link |= LINK_PC;     //�л���PC����
                	        sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;//�˳�ң�ؿ���
                	        sys_para->CAR_RTinf.Link &= ~LINK_AUTO_NAV;//�˳��Զ�����
            	        }
            	    }
                    else
                    {
                        cntFilter = 0;
                    }
            	}
            }
		}
        else
        {
            if((sys_para->CAR_RTinf.Link & LINK_HAND_OVER)
                && (sys_para->CAR_RTinf.Link & LINK_REMOTE))  //�ƽ�״̬�л���ң�ؿ���ʱ
            {
                sys_para->CAR_RTinf.Link &= ~LINK_HAND_OVER;   //�˳��ƽ�״̬
                if(gStUfoData.flag & UFO_ENABLE_REMOTE_QUCIK_STOP)  //ʹ��ң�ؽ���ֹͣ
                {
                    if(!(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP))
                    {
                        rt_kprintf("Enter remote quickStop.\r\n");
                    }
                    sys_para->CAR_RTinf.Link |= LINK_QUICK_STOP; //����ֹͣ
                }
            }
            if(!(sys_para->CAR_RTinf.Link & LINK_REMOTE))   //�л�ʱ�˲�
            {
                cntFilter++;
                if(cntFilter >= 5)
                {
                    sys_para->CAR_RTinf.Link |= LINK_REMOTE;     //�л����ֱ�����
                    sys_para->CAR_RTinf.Link &= ~LINK_PC;        //�˳�PC����
                    sys_para->CAR_RTinf.Link &= ~LINK_AUTO_NAV;//�˳��Զ�����
                }
            }
            else
            {
                cntFilter = 0;
            }
        }
	}

    //ʧ���ж�
    if(sys_para->CAR_RTinf.Link & LINK_REMOTE)  //ң�ؿ���
    {
        if(sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) //���յ�ң������
        {
            lastTime = curTime;
        }
        else if((TickType_t)(curTime - lastTime) >= 500) //ͨ�ų�ʱ
        {
            sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;//�˳�ң�ؿ���
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_REMOTE_LINE_OFF, ERROR_L_NORMAL);
        }
    }
    if(sys_para->CAR_RTinf.Link & LINK_PC)      //PC����
    {
        if(sys_para->CAR_RTinf.Link & LINK_PC_DATA)  //���յ�PC����
        {
            lastTime = curTime;
            sys_para->CAR_RTinf.Link &= ~LINK_PC_LOST;    //�˳�PC���ݶ�ʧ״̬
        }
        else if((TickType_t)(curTime - lastTime) >= 1500) //ͨ�ų�ʱ
        {
            sys_para->CAR_RTinf.Link &= ~LINK_PC;   //�˳�PC����
            //SetErrorCode(M_TOTAL_NUM, ERROR_CODE_PC_LINE_OFF, ERROR_L_NORMAL);
        }
        else if((TickType_t)(curTime - lastTime) >= 500) //ͨ�ų�ʱ
        {
            if ((ABS_VALUE((int16_t)sys_para->PC_Remote.YM) > 560) || DEBUG_DATA_TYPE_8A)   //����ֵ��560cm/s(20.2km/h)���ߵ���ʱ
            {
                gPrintfTestDataShowFlag = 1;
                sys_para->CAR_RTinf.Link &= ~LINK_PC;   //�˳�PC����
            //SetErrorCode(M_TOTAL_NUM, ERROR_CODE_PC_LINE_OFF, ERROR_L_NORMAL);
            }
        }
        else if((TickType_t)(curTime - lastTime) >= 50) //ͨ�ſ���
        {
            sys_para->CAR_RTinf.Link |= LINK_PC_LOST;   //�л���PC���ݶ�ʧ״̬
        }
        
    }

    /*if (DEBUG_DATA_TYPE_81)
    {
        rt_kprintf("time:%d,link:%#x,mbFailCnt:%d,mbClsCnt:%d.\r\n", (TickType_t)(curTime - lastTime), sys_para->CAR_RTinf.Link,
            tcpModbusConFailCnt,tcpModbusSotClsCnt); 
    }*/
    
    //ʧ��ʱ����
    if((!(sys_para->CAR_RTinf.Link & LINK_REMOTE))
        && (!(sys_para->CAR_RTinf.Link & LINK_PC))
        && (!(sys_para->CAR_RTinf.Link & LINK_AUTO_NAV)))  //��PC���ƻ�ң�ؿ���
    {
        if(!(sys_para->CAR_RTinf.Link & LINK_OUT_OF_CTRL))
        {
            sys_para->CAR_RTinf.Link |= LINK_OUT_OF_CTRL;
            memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
            LockThread();
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
            lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
            lMotorRunMode.target_value = 0;
            SetMotorRunModeData(M_TURN, &lMotorRunMode);
            UfoLeftRightControl(0, &lMotorRunMode);
            UnLockThread();
        }
    }

    //���յ����ݣ�������������
    if(((sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) && ((TickType_t)(curTime - lastRemoteDataTime) >= 20))
        || (sys_para->CAR_RTinf.Link & LINK_PC_DATA)
        || (sys_para->CAR_RTinf.Link & LINK_AUTO_NAV)) 
    {
        lastRemoteDataTime = curTime;

        if(sys_para->CAR_RTinf.Link & LINK_PC_DATA)
        {
            sys_para->CAR_RTinf.vel = (int16_t)sys_para->PC_Remote.VEL;
            sys_para->CAR_RTinf.vel = Limit(sys_para->CAR_RTinf.vel, -CMD_VEL_VALUE_MAX, CMD_VEL_VALUE_MAX);
        }
        
        if(((sys_para->CAR_RTinf.Link & LINK_REMOTE) && (sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA))
            || ((sys_para->CAR_RTinf.Link & LINK_PC) && (sys_para->CAR_RTinf.Link & LINK_PC_DATA))
            || (sys_para->CAR_RTinf.Link & LINK_AUTO_NAV))  //PC���ƻ�ң�ؿ��ƻ��Զ�����
        {        	
            sys_para->CAR_RTinf.Link &= ~LINK_OUT_OF_CTRL;
            //ң�����ӹ�ģʽ
        	if(sys_para->CAR_RTinf.Link & LINK_REMOTE)
        	{  
                sys_para->CAR_RTinf.YM = 	hex_buffer[2];  //��λ1000����λɲ��1700����λ����300
        		sys_para->CAR_RTinf.ZX = 	hex_buffer[3];  //��λ1000����λ��ת300����λ��ת1700
                if(ABS_VALUE(sys_para->CAR_RTinf.YM - CMD_VALUE_OFFSET) <= 50)  //��λ�˲�
        		{
        		    sys_para->CAR_RTinf.YM = CMD_VALUE_OFFSET;
        		}
                if(ABS_VALUE(sys_para->CAR_RTinf.ZX - CMD_VALUE_OFFSET) <= 30)  //��λ�˲�
        		{
        		    sys_para->CAR_RTinf.ZX = CMD_VALUE_OFFSET;
        		}
                sys_para->CAR_RTinf.YM = Limit(sys_para->CAR_RTinf.YM - CMD_VALUE_OFFSET, -CMD_VALUE_MAX, CMD_VALUE_MAX);
                if((sys_para->CAR_RTinf.Link & LINK_QUICK_STOP)
                    && (0 != sys_para->CAR_RTinf.YM))
                {
                    sys_para->CAR_RTinf.Link &= ~LINK_QUICK_STOP;
                    rt_kprintf("Exit quick:%d.\r\n", sys_para->CAR_RTinf.YM);
                }
        	}
        	//PC����ģʽ
        	else if(sys_para->CAR_RTinf.Link & LINK_PC)
        	{
        		if((s16_t)sys_para->PC_Remote.ZX > 1000)  {sys_para->PC_Remote.ZX = 1000;}        		
        		if((s16_t)sys_para->PC_Remote.ZX < -1000)  {sys_para->PC_Remote.ZX = (uint16_t)(-1000);}	

                sys_para->CAR_RTinf.YM = (int16_t)sys_para->PC_Remote.YM;
        		sys_para->CAR_RTinf.YM = Limit(sys_para->CAR_RTinf.YM, -CMD_VEL_VALUE_MAX, CMD_VEL_VALUE_MAX);
                sys_para->CAR_RTinf.YM = VelCm_sToRpm(sys_para->CAR_RTinf.YM);
        		sys_para->CAR_RTinf.ZX = (s16_t)sys_para->PC_Remote.ZX * 0.7 + 1000;
                sys_para->CAR_RTinf.SetAcc = (int16_t)sys_para->PC_Remote.SetAcc;
                sys_para->CAR_RTinf.SetAcc = Limit(sys_para->CAR_RTinf.SetAcc, -CMD_ACC_VALUE_MAX, CMD_ACC_VALUE_MAX);
                sys_para->CAR_RTinf.max_vel = (int16_t)sys_para->PC_Remote.max_vel;
        		sys_para->CAR_RTinf.max_vel = Limit(sys_para->CAR_RTinf.max_vel, -CMD_VEL_VALUE_MAX, CMD_VEL_VALUE_MAX);

                if(0xFFFF == sys_para->PC_Remote.VEL)    //���ؿ�����ֹͣ����
                {
                    if(!(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP))
                    {
                        rt_kprintf("Enter pc quickStop.\r\n");
                    }
                    sys_para->CAR_RTinf.Link |= LINK_QUICK_STOP; //����ֹͣ
                }
                else if((sys_para->CAR_RTinf.Link & LINK_QUICK_STOP)
                    && (0 != sys_para->CAR_RTinf.YM))
                {
                    rt_kprintf("Exit pc quickStop.\r\n");
                    sys_para->CAR_RTinf.Link &= ~LINK_QUICK_STOP;
                }
                /*if(1 == sys_para->PC_Remote.cmd)    //�����������
                {
                    ClearErrorCode(M_TOTAL_NUM);
                }*/
        	}
            //�Զ���������ģʽ
//        	else
//        	{
//        	    sys_para->CAR_RTinf.YM = CMD_VALUE_OFFSET + Limit(gStNavControl.YM, -CMD_VALUE_MAX, CMD_VALUE_MAX);
//        		sys_para->CAR_RTinf.ZX = CMD_VALUE_OFFSET + Limit(gStNavControl.ZX, -CMD_VALUE_MAX, CMD_VALUE_MAX);
//        	}

            //���ŵ�λ����ϵ�� ��hex_buffer[6] ����
        	YM_Limit = gStMotorData[M_LEFT].limitSpeed;   //���
        	YM_Max = YM_Limit;
            if(gStMotorData[M_LEFT].initPos > 0) YM_Mid = gStMotorData[M_LEFT].initPos;
            if(gStMotorData[M_RIGHT].initPos > 0) YM_Min = gStMotorData[M_RIGHT].initPos;
            YM_Mid = VelCm_sToRpm(YM_Mid);  //��������ΪĬ��6m/s�������ļ�����
            if(sys_para->CAR_RTinf.Link & LINK_REMOTE)
            {
                if(!(gStUfoData.flag & UFO_ENABLE_REMOTE_HIGH_VEL_FLAG))   //������ң�ظ��ٱ�־
                {
                    YM_Limit = YM_Mid;
                }
            }
        	if(hex_buffer[6] > 1200){
                YM_Limit = VelCm_sToRpm(YM_Min);    //��������ΪĬ��2.4m/s���������ļ�����
            }
            else if(hex_buffer[6] > 800){YM_Limit = YM_Mid;}
            if(sys_para->CAR_RTinf.Link & LINK_REMOTE)
            {
                YM_Max = YM_Limit;
            }

            memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
            LockThread();

        	//ת�����
        	if(BRAKE_WAIT_NONE == gBrakeWaitOnFlag)
        	{
        	    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = Limit((s16_t)sys_para->CAR_RTinf.ZX - 1000, -CMD_VALUE_MAX, CMD_VALUE_MAX);
                //����ת�����
                gSteeringAngleVelDiff = lMotorRunMode.target_value;
                gSteeringAngleVelDiff = gSteeringAngleVelDiff * 18 / CMD_VALUE_MAX;    //��Ӧ-18�㵽18��
                
                if(gStMotorData[M_TURN].flag & MOTOR_DIR_INVERSE_FLAG)  //�������
                {
                    lMotorRunMode.target_value = -lMotorRunMode.target_value;
                }
                SetMotorRunModeData(M_TURN, &lMotorRunMode);
        	}
            
        	//����/ɲ��
        	if((0 == sys_para->CAR_RTinf.YM) || IS_EMERGENCY_STOP) //ֹͣɲ��
        	{
                if((0 == sys_para->CAR_RTinf.YM) && (!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)))
                {
                    sys_para->CAR_RTinf.Link |= LINK_REV_STOP_CMD;  //���յ�ֹͣ����
                }
                UfoLeftRightControl(0, &lMotorRunMode);
                if(BRAKE_WAIT_PRE == gBrakeWaitOnFlag)
                {
                    gBrakeWaitOnFlag = BRAKE_WAIT;
                }
        	}
        	else //����
        	{
                if(BRAKE_WAIT_NONE == gBrakeWaitOnFlag)
                {
                    //lMotorRunMode.target_value = 0;
                    lMotorRunMode.target_value = -sys_para->CAR_RTinf.YM;
                    if(!(sys_para->CAR_RTinf.Link & LINK_PC))   //��pc���ƣ�Ŀ��Ϊ���ת��
                    {
                        lMotorRunMode.target_value = lMotorRunMode.target_value * YM_Max / 700;  //�ֱ��ٶ�0~700��ת����ʵ��ת��
                    }
                    lMotorRunMode.target_value = Limit(lMotorRunMode.target_value, -YM_Limit, YM_Limit);
                    UfoLeftRightControl(lMotorRunMode.target_value, &lMotorRunMode);
                }
                else if(BRAKE_WAIT_PRE == gBrakeWaitOnFlag)
                {
                    gBrakeWaitOnFlag = BRAKE_WAIT;
                }
        	}
            UnLockThread();
        }
        
        sys_para->CAR_RTinf.Link &= ~LINK_REMOTE_DATA;
        sys_para->CAR_RTinf.Link &= ~LINK_PC_DATA;
    }
}
/*****************************************************************************
 ��������  : ��ȡufo����״̬(ʹ��״̬���ٶȿ��Ƶ�λ)
 �������  : ��
 �������  : uint8_t retFlg(��λ�洢��־λ��0Ϊ��Ч��1Ϊ��Ч)
                             b0:�ٶȿ������ٵ�
                             b1:�ٶȿ��Ƹ��ٵ�(���b1��b0��Ϊ1����Ϊ���ٵ������b1��b0��Ϊ0����Ϊ���ٵ�)
                             b2:Ԥ��
                             b3:Ԥ��
                             b4:���ʹ������״̬
                             b5:Ԥ��
                             b6:Ԥ��
                             b7:Ԥ��
 ��    ��  : ����
 ��    ��  : 2023��09��08��
*****************************************************************************/
uint8_t GetUfoControlStatus(void)
{
    uint8_t i;
    uint8_t retFlg = 0;
 
    for (i=0; i<M_TOTAL_NUM; i++)
    {
        if (DRIVER_TYPE_NONE != gStMotorData[i].driverType)                          //������������Ч�������
        {
            if ((OPERATION_MODE_SET_FINISH != gStMotorRunState[i].operModeSetStep)   //�л�ģʽδ���
             || ((gStMotorData[i].flag & ENABLE_AUTO_HOMMING) && (HOME_FLAG_FINISH != gStMotorRunState[i].homeFinishFlag)) //��ʹ�����Զ�Ѱ�������£�Ѱ��δ���
               )
            {
                break;
            }
        }
    }

    if (M_TOTAL_NUM == i)
    {
        retFlg |= (1 << 4);
    }

    //���ŵ�λ����ϵ�� ��hex_buffer[6] ����
	if(hex_buffer[6] > 1200)
    {
        ;
    }
    else if(hex_buffer[6] > 800)
    {
        retFlg |= (1 << 0);                   //���ٵ�λ
    }
    else
    {
        retFlg |= (1 << 1);                   //���ٵ�λ
    }

    return retFlg;
}

/*****************************************************************************
 ��������  : ����ufo״̬��Ϣ
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��11��23��
*****************************************************************************/
static void UpdateUfoState(void)
{
    if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //�˳����Ͻ���ֹͣ״̬���ٸ�ֵ����������ػ�������Ԥ�����ػ����ڵײ����ʱ��ǰ�˳����̲���ת��
    {
        //�޹��ϻ������й����г��ֹ��ϣ����߷ǵ������������ϴ�
        if(gErrorRunStateFlag || (0 == gErrorResult) || (0 == gErrorMotorNum) || (WARNING_MOTOR_NUM_OFFSET == gErrorMotorNum))
        {
            gRespondState.faultModule = gErrorMotorNum;
            gRespondState.errorCode = gErrorResult;
        }
    }
    gRespondState.linkState = sys_para->CAR_RTinf.Link;
    gRespondState.ctrlState = GetUfoControlStatus();
}
/*****************************************************************************
 ��������  : tcp���·�������
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��03��28��
*****************************************************************************/
void TcpUpdateSendData(void)
{
    uint8_t luUfoCtrlFlg;           //ufo����״̬��־λ
    
	luUfoCtrlFlg = GetUfoControlStatus();           //ufo����״̬��־λ
	
    //���ݰ�����ͳ��
    sys_para->SBUS_rx.CH[20] = gStBatteryState.voltage;    //��ѹ
    sys_para->SBUS_rx.CH[21] = (gStBatteryState.soc << 8) + (gStBatteryState.maxtemp);//�������¶�

    if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //�˳����Ͻ���ֹͣ״̬���ٸ�ֵ����������ػ�������Ԥ�����ػ����ڵײ����ʱ��ǰ�˳����̲���ת��
    {
        //�޹��ϻ������й����г��ֹ��ϣ����߷ǵ������������ϴ�
        if(gErrorRunStateFlag || (0 == gErrorResult) || (0 == gErrorMotorNum) || (WARNING_MOTOR_NUM_OFFSET == gErrorMotorNum))
        {
            sys_para->SBUS_rx.CH[22] = (0 << 8) + (gErrorMotorNum);//��ع���,������
            sys_para->SBUS_rx.CH[23] = gErrorResult;  //����
        }
    }
    sys_para->SBUS_rx.CH[24] = (1 << 8) + (luUfoCtrlFlg);  //���״̬,��ѹֵ

    sys_para->SBUS_rx.CH[25] = gStMotorRevData[M_LEFT].speed;               //�����ٶ�ֵ����λrpm,�з���2�ֽ�
    sys_para->SBUS_rx.CH[26] = gStMotorRevData[M_LEFT_ONE].speed;           //��һ����ٶ�ֵ����λrpm,�з���2�ֽ�
    sys_para->SBUS_rx.CH[27] = gStMotorRevData[M_RIGHT].speed;              //�ҵ���ٶ�ֵ����λrpm,�з���2�ֽ�
    sys_para->SBUS_rx.CH[28] = gStMotorRevData[M_RIGHT_ONE].speed;          //��һ����ٶ�ֵ����λrpm,�з���2�ֽ�
    sys_para->SBUS_rx.CH[29] = gStMotorRevData[M_LEFT].current / 100;       //��������ֵ����λ0.1A,�з���2�ֽ�
    sys_para->SBUS_rx.CH[30] = gStMotorRevData[M_LEFT_ONE].current / 100;   //��һ�������ֵ����λ0.1A,�з���2�ֽ�
    sys_para->SBUS_rx.CH[31] = gStMotorRevData[M_RIGHT].current / 100;      //�ҵ������ֵ����λ0.1A,�з���2�ֽ�
    sys_para->SBUS_rx.CH[32] = gStMotorRevData[M_RIGHT_ONE].current / 100;  //��һ�������ֵ����λ0.1A,�з���2�ֽ�
    sys_para->SBUS_rx.CH[33] = gRespondState.turnCurPos;                    //ת��ǰλ�ã���λ��ת��Ŀ��λ��ֵһ��,�з���2�ֽ�
    sys_para->SBUS_rx.CH[34] = sys_para->CAR_RTinf.Link;                    //�ײ�������������ӱ�־,�޷���2�ֽ�
    sys_para->SBUS_rx.CH[35] = (gMotorAvaPos & 0x0000FFFF);                 //���ֱ���˶����룬��λmm,�з���2�ֽ�
    sys_para->SBUS_rx.CH[36] = (gMotorAvaPos & 0xFFFF0000) >> 16;           //���ֱ���˶����룬��λmm,�з���2�ֽ�
    sys_para->SBUS_rx.CH[37] = ABS_VALUE(gMotorAvaVelFilter) * 10;          //���ֱ���˶��ٶȣ���λmm/s,�޷���2�ֽ�
    sys_para->SBUS_rx.CH[39] += 1;
    sys_para->SBUS_rx.CH[40] = socketConAddr.sin_len;
    sys_para->SBUS_rx.CH[41] = socketConAddr.sin_family;
    sys_para->SBUS_rx.CH[42] = (socketConAddr.sin_port);
    sys_para->SBUS_rx.CH[43] = (socketConAddr.sin_addr.s_addr&0xff000000)>>24;
    sys_para->SBUS_rx.CH[44] = (socketConAddr.sin_addr.s_addr&0x00ff0000)>>16;
    sys_para->SBUS_rx.CH[45] = (socketConAddr.sin_addr.s_addr&0x0000ff00)>>8;
    sys_para->SBUS_rx.CH[46] = (socketConAddr.sin_addr.s_addr&0x00ff00ff);
    sys_para->SBUS_rx.CH[47] = socketConAddr.sin_zero[0];
    sys_para->SBUS_rx.CH[48] = socketConAddr.sin_zero[1];
    sys_para->SBUS_rx.CH[49] = socketConAddr.sin_zero[2];
}
/*****************************************************************************
 ��������  : tcp��ش���
 �������  : TickType_t curTime
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��14��
*****************************************************************************/
static void TcpSocketProcess(TickType_t curTime)
{
    static TickType_t lLastRevTime = 0; //�ϴν������ݵ�ʱ��
    ssize_t revLen, lSendLen;

    /* Read in the request */
    revLen = recv(sock_conn, socketRevBuf, sizeof(socketRevBuf), MSG_DONTWAIT);
    if(revLen > 0)
    {
        if(DEBUG_DATA_TYPE_8A)
        {
            test_kprintf("S%dL%dt%d\n", sock_conn, revLen, HAL_GetTick() / 100 % 100);
        }
        lSendLen = ParsingClientAccessCommand(socketRevBuf, tcpServerSendBuf);
        if((sock_conn >= 0) && (lSendLen > 0))
        {
            lSendLen = send(sock_conn, &tcpServerSendBuf, lSendLen, MSG_DONTWAIT);//����
            if((lSendLen < 0) && DEBUG_DATA_TYPE_1)
            {
                 rt_kprintf("Tcp send gSendRespondState err!\r\n");
            }
        }
        
        lLastRevTime = curTime;
        GPIOSetLedState(LED1, LED_ON);
    } 
    else if(curTime - lLastRevTime >= 1000) //����1sδ�յ����ݣ��ر�����ָʾ��
    {
        lLastRevTime = curTime;
        GPIOSetLedState(LED1, LED_OFF);
    }
}
/*****************************************************************************
 ��������  : tcp���������
 �������  : uint8_t* cmdData
             uint8_t size      
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��6��8��
*****************************************************************************/
static void TcpRevCmdAnalysis(uint8_t* cmdData, uint8_t size)
{
    //unsigned char crc8Value;
    uint16_t cmd = *(uint16_t*)cmdData;
    static unsigned short lastCnt = 256;
    TickType_t curTime;
    static TickType_t lastTime = 0;

    if((0x00 == cmd) && (size >= 10))  //��������
    {
        /*crc8Value = CRC8_Table_1(cmdData + 1, 7);
        if(crc8Value == cmdData[8])*/
        {
            if((lastCnt < 256) && (((unsigned char)(lastCnt + 1)) != cmdData[8])) //֡��Ŵ���
            {
                if(DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_8B)
                {
                    rt_kprintf("TCP cnt err, last:%d, cur:%d!\r\n", lastCnt, cmdData[8]);
                }
            }
            lastCnt = cmdData[8];
            sys_para->CAR_RTinf.Link |= LINK_PC_DATA;
            sys_para->PC_Remote.YM = (unsigned short)(-(*((short*)(cmdData + 2))));
            sys_para->PC_Remote.ZX = *(unsigned short*)(cmdData + 4);
            sys_para->PC_Remote.VEL = *(unsigned short*)(cmdData + 6);
            if(size >= 11)  //��չ������ٶ�ֵ�����ܲ�������9�ֽ�+2�ֽڹ�����
            {
                sys_para->PC_Remote.max_vel = *(unsigned short*)(cmdData + 9);
            }
            else
            {
                sys_para->PC_Remote.max_vel = 0;
            }
            if(size >= 15)  //��չ�˾������ʱ�̼�Ŀ����ٶȴ�С�����ܲ�������11�ֽ�+4�ֽڹ�����
            {
                sys_para->PC_Remote.decRemainTime = *(unsigned short*)(cmdData + 11);
                sys_para->PC_Remote.setDec = *(short*)(cmdData + 13);
            }
            else
            {
                sys_para->PC_Remote.decRemainTime = 0;
                sys_para->PC_Remote.setDec = 0;
            }
            if(DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_8B || DEBUG_DATA_TYPE_87)
            {
                curTime = xTaskGetTickCount();
                rt_kprintf("YM:%d,ZX:%d,Vel:%d,MVel:%d,time:%d,dec:%d,cnt:%d,t:%d.\r\n", -(short)(sys_para->PC_Remote.YM),
                    (short)(sys_para->PC_Remote.ZX), (short)sys_para->PC_Remote.VEL, sys_para->PC_Remote.max_vel,
                    sys_para->PC_Remote.decRemainTime, sys_para->PC_Remote.setDec, cmdData[8], curTime - lastTime);
                lastTime = curTime;
            }
        }
        /*else
        {
            if(DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_8B)
            {
                rt_kprintf("TCP crc8 err!\r\n");
            }
        }*/

        TcpSocketSendNew();
    }
}
/*****************************************************************************
 ��������  : tcp��Э����մ���
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��1��5��
*****************************************************************************/
static void TcpSocketProcessNew(void)
{
    int32_t l_receive_len, lRetryInex = 0;
    uint32_t i;
    uint32_t lTickTemp = HAL_GetTick();
    uint8_t lRevBuf[32];
    uint16_t crc16rev, crc16cal;
    
    l_receive_len = recv(sock_conn, lRevBuf, sizeof(lRevBuf), MSG_DONTWAIT);

    if(l_receive_len <= 0)
    {
        if(UART_RECEIVE_STATE_RECEIVING == gStTcpRevData.flag)
        {
            lTickTemp -= gStTcpRevData.lastTick;
            if(lTickTemp >= 200)
            {
                lRetryInex = 2;
                if(DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_8B)
                {
                    rt_kprintf("Tcp rev timeout!\r\n");
                }
            }
        }
    }
    else if((l_receive_len > 0) && (l_receive_len <= sizeof(lRevBuf)))
    {
        if(DEBUG_DATA_TYPE_8A)
        {
            test_kprintf("S%dL%dt%d\n", sock_conn, l_receive_len, HAL_GetTick() / 100 % 100);
        }
        if(gStTcpRevData.index + l_receive_len < TCP_REV_BUF_SIZE)
        {
            memcpy(&gStTcpRevData.data[gStTcpRevData.index], lRevBuf, l_receive_len);
            gStTcpRevData.index += l_receive_len;
        }
        else
        {
            UsartMsgInit(&gStTcpRevData);
        }
    }
    if((UART_RECEIVE_STATE_IDLE == gStTcpRevData.flag) && (gStTcpRevData.index >= 4))
    {
        if((0xAA == gStTcpRevData.data[0]) && (0x55 == gStTcpRevData.data[1]))
        {
            gStTcpRevData.length = *(uint16_t*)(gStTcpRevData.data + 2);
            if(gStTcpRevData.length + 8 > TCP_REV_BUF_SIZE)
            {
                lRetryInex = 2;
                if(DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_8B)
                {
                    rt_kprintf("Tcp rev err len:%d!\r\n", gStTcpRevData.length + 8);
                }
            }
            else
            {
                if(DEBUG_DATA_TYPE_8B)
                {
                    rt_kprintf("Tcp rev len:%d!\r\n", gStTcpRevData.length + 8);
                }
                gStTcpRevData.flag = UART_RECEIVE_STATE_RECEIVING;
                gStTcpRevData.lastTick = HAL_GetTick();
            }
        }
        else
        {
            lRetryInex = 1;
        }
    }
    if(UART_RECEIVE_STATE_RECEIVING == gStTcpRevData.flag)//�̰����ݣ�С��256�ֽ�
    {
        if(gStTcpRevData.length + 8 <= gStTcpRevData.index) //����������
        {
            crc16cal = MODBUS_RTUCrc16(gStTcpRevData.data + 4, gStTcpRevData.length + 2);
            crc16rev = *(uint16_t*)(gStTcpRevData.data + 6 + gStTcpRevData.length);
            if(crc16cal == crc16rev)
            {
                TcpRevCmdAnalysis(gStTcpRevData.data + 4, gStTcpRevData.length + 2);
            }
            else
            {
                if(DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_8B)
                {
                    rt_kprintf("Tcp rev err crc:%d, %d!\r\n", crc16cal, crc16rev);
                }
            }
            if(DEBUG_DATA_TYPE_8B)
            {
                rt_kprintfArray(gStTcpRevData.data, gStTcpRevData.length + 8, 16, 1);
                rt_kprintf("\r\n\r\n");
            }
            lRetryInex = gStTcpRevData.length + 8;
        }
    }
   
    if((0 != lRetryInex) && (gStTcpRevData.index > 0))
    {
        i = lRetryInex;
        lRetryInex = gStTcpRevData.index;
        for(; i + 1 < gStTcpRevData.index; i++)
        {
            if((0xAA == gStTcpRevData.data[i]) && (0x55 == gStTcpRevData.data[i + 1]))
            {
                lRetryInex = i;
                if(DEBUG_DATA_TYPE_8B)
                {
                    rt_kprintf("Tcp rev new start:%d!\r\n", lRetryInex);
                }
                break;
            }
        }
        gStTcpRevData.index = gStTcpRevData.index - lRetryInex;
        for(i = 0; i < gStTcpRevData.index; i++)
        {
            gStTcpRevData.data[i] = gStTcpRevData.data[i + lRetryInex];
        }
        gStTcpRevData.flag = UART_RECEIVE_STATE_IDLE;
    }
}
/*****************************************************************************
 ��������  : ���緢��
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��11��8��
*****************************************************************************/
static void TcpSocketSendNew(void)
{  
    int32_t l_send_len;

    UpdateUfoState();   //����ufo״̬��Ϣ
    
    gSendRespondState.RESPOND_STATE_DATA = gRespondState;
    gSendRespondState.crc16=MODBUS_RTUCrc16((unsigned char *)&gSendRespondState.functional_code,
        gSendRespondState.functional_parameter_len + 2);//crcУ��

    if(sock_conn >= 0)
    {
        l_send_len = send(sock_conn, &gSendRespondState, sizeof(SEND_RESPOND_STATE), MSG_DONTWAIT);//����
        if(l_send_len < 0)
        {
             rt_kprintf("Tcp send gSendRespondState err!\r\n");
        }
    }
}
/*****************************************************************************
 ��������  : ��ӡ��������
 �������  : uint8_t* sendBuf
             ssize_t sendLen   
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��4��27��
*****************************************************************************/
void PrintfSendMsg(uint8_t* sendBuf, int sendLen, rt_bool_t useUdp)
{
    if(DEBUG_DATA_TYPE_95)
    {
        usart_kprintf("Udp%d,%d:%d.%d.%d.%d:%d,%d.\r\n", gUartPrintfFlag, udp_print_connected_flag, udpClientAddr.sin_addr.s_addr & 0xff,
            (udpClientAddr.sin_addr.s_addr >> 8) & 0xff,
            (udpClientAddr.sin_addr.s_addr >> 16) & 0xff,
            (udpClientAddr.sin_addr.s_addr >> 24) & 0xff, ntohs(udpClientAddr.sin_port), useUdp);
    }
    if(((0 == gUartPrintfFlag) && (udp_print_connected_flag > 0) && (sendLen > 0)) || useUdp)
    {
        sendto(udp_sock_num, (uint8_t*)sendBuf, sendLen, MSG_DONTWAIT, (struct sockaddr *)&udpClientAddr, sizeof(udpClientAddr));
        if(DEBUG_DATA_TYPE_8D)
        {
            usart_kprintf("\r\nsendlen: %d.\r\n", sendLen);
        }
    }
    else if(sendLen > 1)
    {
         UsartDeviceWrite(gUartNumWifiPrint, (uint8_t*)(&sendBuf[1]), sendLen - 1); //���ڴӵ�һ���ֽڿ�ʼ���ͣ������Ͱ����
    }
}
/*****************************************************************************
 ��������  : tcp��ӡ�˿ڽ��մ���
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��1��5��
*****************************************************************************/
static void TcpPrintRevMsgProcess(void)
{
    int32_t l_receive_len = 0;
    uint32_t i = 0, temp = 0;
    uint32_t lTickTemp = HAL_GetTick();
    uint8_t* lRevBuf = gStTcpPrintRevData.data;
    struct sockaddr_in lUdpClientAddr;			    //udp�ͻ��˵�ַ
    socklen_t lUdpClientLen;                        //udp�ͻ��˵�ַ����

    if(gStTcpPrintRevData.index >= PRINTF_REV_BUF_SIZE)
    {
        UsartMsgInit(&gStTcpPrintRevData);
        return;
    }
    
    if(PRINTF_REV_BUF_SIZE > gStTcpPrintRevData.index)
    {
        l_receive_len = recvfrom(udp_sock_num, &(lRevBuf[gStTcpPrintRevData.index]), PRINTF_REV_BUF_SIZE - gStTcpPrintRevData.index, MSG_DONTWAIT, (struct sockaddr *)&lUdpClientAddr, &lUdpClientLen);
    }
    if(l_receive_len < 0) l_receive_len = 0;
    if(0 == l_receive_len)
    {
        if((UART_RECEIVE_STATE_RECEIVING == gStTcpPrintRevData.flag) || (UART_RECEIVE_STATE_RECEIVING_LONG == gStTcpPrintRevData.flag))
        {
            lTickTemp -= gStTcpPrintRevData.lastTick;
            if(lTickTemp >= 500)
            {
                UsartMsgInit(&gStTcpPrintRevData);
            }
        }
    }    
    if((l_receive_len > 0) || ((UART_RECEIVE_STATE_IDLE == gStTcpPrintRevData.flag) && (gStTcpPrintRevData.index > 0)))
    {
        gStTcpPrintRevData.index += l_receive_len;
        if(gStTcpPrintRevData.index > PRINTF_REV_BUF_SIZE) gStTcpPrintRevData.index = PRINTF_REV_BUF_SIZE;
        if(UART_RECEIVE_STATE_IDLE == gStTcpPrintRevData.flag)
        {
            for(i = 0; i < gStTcpPrintRevData.index; i++)
            {
                if('*' == lRevBuf[i])//�̰����ݣ�С��256�ֽ�
                {
                    temp = i;
                    gStTcpPrintRevData.flag = UART_RECEIVE_STATE_RECEIVING;
                    gStTcpPrintRevData.lastTick = HAL_GetTick();
                    break;
                }
                else if('L' == lRevBuf[i])//��������
                {
                    temp = i;
                    gStTcpPrintRevData.flag = UART_RECEIVE_STATE_RECEIVING_LONG;
                    gStTcpPrintRevData.lastTick = HAL_GetTick();
                    break;
                }
            }
            if((UART_RECEIVE_STATE_IDLE != gStTcpPrintRevData.flag) && (temp > 0))  //ȥ��ǰ����Ч������
            {
                for(i = temp; i < gStTcpPrintRevData.index; i++)
                {
                    lRevBuf[i - temp] = lRevBuf[i];
                }
                gStTcpPrintRevData.index -= temp;
            }
        }
        if(UART_RECEIVE_STATE_RECEIVING == gStTcpPrintRevData.flag)//�̰����ݣ�С��256�ֽ�
        {
            if((0 == gStTcpPrintRevData.length) && (gStTcpPrintRevData.index >= 2))
            {
                gStTcpPrintRevData.length = lRevBuf[1];
                if((gStTcpPrintRevData.length <= 3) && (gStTcpPrintRevData.length >= PRINTF_REV_BUF_SIZE))
                {
                    UsartMsgInit(&gStTcpPrintRevData);
                }
            }
            if((gStTcpPrintRevData.index >= gStTcpPrintRevData.length) && (gStTcpPrintRevData.length > 3))//�������
            {
                if('#' == lRevBuf[gStTcpPrintRevData.length - 1])
                {
                    gUartPrintfFlag = 0;    //�յ�����������ڴ�ӡ
                    WifiRevCmdAnalysis(lRevBuf, gStTcpPrintRevData.length);
                    if(gStTcpPrintRevData.index > gStTcpPrintRevData.length)    //ȥ��ǰ������������
                    {
                        for(i = gStTcpPrintRevData.length; i < gStTcpPrintRevData.index; i++)
                        {
                            lRevBuf[i - gStTcpPrintRevData.length] = lRevBuf[i];
                        }
                        gStTcpPrintRevData.index -= gStTcpPrintRevData.length;
                        gStTcpPrintRevData.length = 0;
                        gStTcpPrintRevData.flag = UART_RECEIVE_STATE_IDLE;
                    }
                    else
                    {
                        UsartMsgInit(&gStTcpPrintRevData);
                    }
                }
                else
                {
                    UsartMsgInit(&gStTcpPrintRevData);
                }
            }
        }
        else if(UART_RECEIVE_STATE_RECEIVING_LONG == gStTcpPrintRevData.flag)//��������
        {
            if(0 == gStTcpPrintRevData.length)
            {
                if(gStTcpPrintRevData.index > 2)
                {
                    gStTcpPrintRevData.length = *(uint16_t*)(lRevBuf + 1);
                    if((gStTcpPrintRevData.length <= 4) || (gStTcpPrintRevData.length > PRINTF_REV_BUF_SIZE))
                    {
                        UsartMsgInit(&gStTcpPrintRevData);
                    }
                }
            }
            if((gStTcpPrintRevData.index >= gStTcpPrintRevData.length) && (gStTcpPrintRevData.length > 4))//�������
            {
                if('#' == lRevBuf[gStTcpPrintRevData.length - 1])
                {
                    gUartPrintfFlag = 0;    //�յ�����������ڴ�ӡ
                    WifiRevCmdAnalysis(lRevBuf, gStTcpPrintRevData.length);
                    if(gStTcpPrintRevData.index > gStTcpPrintRevData.length)    //ȥ��ǰ������������
                    {
                        for(i = gStTcpPrintRevData.length; i < gStTcpPrintRevData.index; i++)
                        {
                            lRevBuf[i - gStTcpPrintRevData.length] = lRevBuf[i];
                        }
                        gStTcpPrintRevData.index -= gStTcpPrintRevData.length;
                        gStTcpPrintRevData.length = 0;
                        gStTcpPrintRevData.flag = UART_RECEIVE_STATE_IDLE;
                    }
                    else
                    {
                        UsartMsgInit(&gStTcpPrintRevData);
                    }
                }
                else
                {
                    UsartMsgInit(&gStTcpPrintRevData);
                }
            }
        }
        else
        {
            UsartMsgInit(&gStTcpPrintRevData);
        }
    }
    /*else
    {
        close(sock_print_conn);
        sock_print_conn = -1;
        rt_kprintf("TCP print abnormal:%d!\r\n", l_receive_len);
    }*/
}
/*****************************************************************************
 ��������  : ��������
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��29��
*****************************************************************************/
void LockThread(void)
{
    if(!tastStartFlag)
    {
        return;
    }

    if(!tastLockCnt)
    {
        vTaskSuspendAll();         /* ���������� */
    }
    tastLockCnt++;
}

/*****************************************************************************
 ��������  : ��������
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��29��
*****************************************************************************/
void UnLockThread(void)
{
    if(!tastStartFlag)
    {
        return;
    }

    if(tastLockCnt)
    {
        tastLockCnt--;
        if(0 == tastLockCnt)
        {
            if(!xTaskResumeAll())      /* �رյ������������Ҫ�����л����˺�������pdTRUE�����򷵻�pdFALSE */
            {
                taskYIELD ();
            }
        }
    }
}


/* USER CODE END Application */

