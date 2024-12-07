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

//线程喂狗标志
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
uint8_t wdgDisableFlag = 0;                     //禁止看门狗喂狗标志
uint8_t tastStartFlag = 0;
uint8_t tastLockCnt = 0;
uint8_t data_buffer[100];						//定义接收到的数据Buff大小为100
char tcp_server_recvbuf[300];					//定义数据处理Buff大小为300（为100也无所谓，只要大于等于100就好）
int sock_conn = -1;							    // 请求的 socked
//int sock_print_conn = -1;						// 请求的打印 socked
int udp_sock_num = -1;
struct sockaddr_in udpClientAddr;			    //udp客户端地址
int udp_print_connected_flag = -1;              //打印端口客户端udp连接标志,-1表示未socket创建失败，0表示创建成功，但未收到客户端消息，1表示稳定收到客户端消息
uint8_t gSbusData[SBUS_DATA_LEN << 1];
struct sockaddr_in socketConAddr;				//连接地址
int32_t gRemoteStopDcc = 300;                   //遥控停止时减速度

uint32_t tcpModbusConFailCnt = 0u;              //连接失败计数
uint32_t tcpModbusSotClsCnt = 0u;               //关闭套接字计数

unsigned short   usRegHoldBuf[REG_HOLD_NREGS];

SYS_PARA         *sys_para = (SYS_PARA*)usRegHoldBuf; //定义系统设定参数实例化
RESPOND_STATE gRespondState;                    //响应状态信息
SEND_RESPOND_STATE gSendRespondState={0XAA,0x55,sizeof(RESPOND_STATE),0x96,{0},0};
OLD_TEST gOldTest = {OLD_TEST_PAUSE | OLD_TEST_END, NONE_AGING, 20, 150, 833, -139, 10, 300, 700, 3, 200, 0, 700, 3}; //老化测试相关变量
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
  MX_IWDG_Init1S(); //将看门狗计时从16s改成1s
  rt_kprintf("Main task start!\r\n");
  l_reset_wifi_tick = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    l_cur_tick = xTaskGetTickCount();

    //遥控消息处理
    SBUS_ReceiveProcess(l_cur_tick);
    //工控机消息处理
    if(gStUfoData.flag & UFO_PC_CAN_FLAG)  //工控机can通信标志
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
    if (IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG))    //工控机口串口通信标志
    {
        USART_ReceiveProcess(USART5_DEVICE, &rxUsart5Msg, l_cur_tick);
    }
        
    //控制命令解析
    UfoControlCmdAnalysis(l_cur_tick);

	//串口1数据解析
	DebugUartParse();
	
    //wifi消息处理
/*    WifiRevMsgProcess();*/
    //导航处理线程
//    NavDataBinRev(l_cur_tick);    //惯导数据接收
//    if(DEBUG_DATA_TYPE_3)
//    {
//        NavPrintData(l_cur_tick);   //打印惯导数据
//    }
    //NavProcess(l_cur_tick);

    if(IsWdgFeedThreadFinish && (0 == wdgDisableFlag))
    {
        wdgFeedFlag = 0;
        IWDG_Feed();    	 /*所有线程喂狗标志完成后真实喂狗*/
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

//	if(!FLOAT_EQU(_test_val, 0))//当速度不为0时，才打印速度值
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
    struct sockaddr_in server_addr;				//服务器地址
    struct sockaddr_in socketConAddrNew;		//连接地址
	int sock_fd;				                //服务器的 socked
	int sock_conn_new;
	socklen_t addr_len;							// 地址长度
	int err;
    int opt = 1;
    struct timeval timeout;
    struct linger lingerValue;

  /* Infinite loop */
  for(;;)
  {
    if(IsMxLwipNetifLinkUp())
    {
        sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);        //建立一个新的socket连接
        if (sock_fd < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("TCP socket create failed:%d!\r\n", sock_fd);
        }
 
        /*err = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(int)); //端口可重用
        if (err < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("TCP setopt failed:%d!\r\n", err);
        }*/
    
        memset(&server_addr, 0, sizeof(server_addr));               //将服务器地址清空
        server_addr.sin_family = AF_INET;                           //地址家族
        server_addr.sin_addr.s_addr = (gStUfoData.ipLastAdr << 24) + (1 << 16) + (168 << 8) + 192;//注意转化为网络字节序
        server_addr.sin_port = htons(502);                          //使用SERVER_PORT指定为程序头设定的端口号

        err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));  //建立绑定
        if (err < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("TCP bind failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //关闭套接字
            osDelay(1000);
            continue;
        }

        err = listen(sock_fd, 3);                                   //监听连接请求
        if (err < 0)                                                //如果监听失败则关闭套接字
        {
            rt_kprintf("TCP listen failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //关闭套接字
            osDelay(1000);
            continue;
        }

        addr_len = sizeof(struct sockaddr_in);                      //将链接地址赋值给addr_len

        while(IsMxLwipNetifLinkUp())
        {
            sock_conn_new = accept(sock_fd, (struct sockaddr *)&socketConAddrNew, &addr_len);  //对监听到的请求进行连接，状态赋值给sock_conn
            if(sock_conn_new >= 0)  
            {
                closeSelectSocket(&sock_conn);                
                rt_kprintf("TCP Client connect success: %d!\r\n", sock_conn_new);
                sock_conn = sock_conn_new;
                memcpy((uint8_t*)(&socketConAddr), (uint8_t*)(&socketConAddrNew), sizeof(struct sockaddr_in));

                lingerValue.l_onoff = 1;
                lingerValue.l_linger = 0;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_LINGER, &lingerValue, sizeof(lingerValue));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP setopt1 failed:%d!\r\n", err);
                }
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));//SO_REUSEADDR|SO_RCVTIMEO  SO_KEEPALIVE
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP setopt2 failed:%d!\r\n", err);
                }
                timeout.tv_sec = 0;
                timeout.tv_usec = 200000;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));//SO_REUSEADDR|SO_RCVTIMEO
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP setopt3 failed:%d!\r\n", err);
                }
                //禁用Nagle算法，使TCP立即发送数据而不进行缓冲
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP setopt4 failed:%d!\r\n", err);
                }
            }
            else                                         //状态小于0代表连接故障，此时关闭套接字
            {
                rt_kprintf("TCP Client connect failed: %d!\r\n", sock_conn_new);
                if (tcpModbusConFailCnt != 0xFFFFFFFF)
                {
                    tcpModbusConFailCnt++;
                }
                else
                {
                    tcpModbusConFailCnt = 0;//防止数据溢出
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
            tcpModbusSotClsCnt = 0;         //防止数据溢出
        }
        closesocket(sock_fd);                           //关闭套接字
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
    
    if(!((gStUfoData.flag & UFO_PC_CAN_FLAG) && (IS_UFOONE_FLAG_SET(UFOONE_PC_USART_FLAG))))  //工控机can通信标志或串口通信标志
    {
        if(sock_conn >= 0)                                         //状态小于0代表连接故障，此时关闭套接字
        {
            if(gStUfoData.flag & UFO_NEW_PROTOCOL)  //新通信协议
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
        //数据打印，满1包数据或者100ms
        rt_kprintf_log_buf();

        //udp命令处理
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
  
    retSD=f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);//挂载


    if(retSD==FR_OK)
    {
        rt_kprintf("挂载SD卡成功!\r\n");
    }
#endif
  /* Infinite loop */
  for(;;)
  {
#ifdef SD_RW_ENABLE
    //记录日志
    r_event = xEventGroupWaitBits(EventSDLog,   /*事件对象句柄*/
                              EVENT_WRITE_LOG | EVENT_READ_LOG,  /*接收任务感兴趣的事件*/
                              pdTRUE,   /* 退出时清除事件位 */
                              pdFALSE,  /* 满足感兴趣的所有事件 */
                              1000);    /* 指定超时事件,等待1s */

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

    //电池信息采集
    l_cur_tick = xTaskGetTickCount();
    if((l_cur_tick - l_last_tick >= BATTERY485_READ_PERIOD)
        || ((l_cur_tick - l_last_tick >= 20)
        && ((gStUfoData.flag & UFO_BMS_JIKONG)
        || (gStUfoData.flag & UFO_BMS_BAIWEI))))
    {
        l_last_tick = l_cur_tick;
        BatteryCollectProcess(RT_FALSE);
    }

    //温度信息采集
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
  struct sockaddr_in server_addr;			//服务器地址
  int err;
  /* Infinite loop */
  for(;;)
  {
    if(IsMxLwipNetifLinkUp())
    {
        udp_sock_num = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);        //建立一个新的socket连接
        if (udp_sock_num < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("udp print socket create failed:%d!\r\n", udp_sock_num);
            udp_print_connected_flag = -1;
            osDelay(1000);
            continue;
        }
    
        memset(&server_addr, 0, sizeof(server_addr));               //将服务器地址清空
        server_addr.sin_family = AF_INET;                           //地址家族
        server_addr.sin_addr.s_addr = (gStUfoData.ipLastAdr << 24) + (1 << 16) + (168 << 8) + 192;//注意转化为网络字节序
        server_addr.sin_port = htons(211);                           //使用SERVER_PORT指定为程序头设定的端口号

        err = bind(udp_sock_num, (struct sockaddr *)&server_addr, sizeof(server_addr));  //建立绑定
        if (err < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("udp print bind failed:%d!\r\n", err);
            udp_print_connected_flag = -1;
            closesocket(udp_sock_num);                                   //关闭套接字
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
        closesocket(udp_sock_num);                           //关闭套接字
        udp_sock_num = -1;
    }
    else
    {
        udp_print_connected_flag = -1;
    }
    
    osDelay(1000);
  }
  //////TCP
  /*int sock_fd;				                //服务器的 socked
  int sock_conn_new;
  struct sockaddr_in server_addr;			//服务器地址
  int err;
  int opt = 1;
  struct timeval timeout;
  struct linger lingerValue;
  //uint32_t keepalive_value;
  socklen_t addr_len;
  struct sockaddr_in socketConAddrNew;		//连接地址，客户端的协议地址
  /# Infinite loop #/
  for(;;)
  {
    if(IsMxLwipNetifLinkUp())
    {
        sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);        //建立一个新的socket连接
        if (sock_fd < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("TCP print socket create failed:%d!\r\n", sock_fd);
        }
        /#err = setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(int)); //端口可重用
        if (err < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("TCP print setopt failed:%d!\r\n", err);
        }#/
    
        memset(&server_addr, 0, sizeof(server_addr));               //将服务器地址清空
        server_addr.sin_family = AF_INET;                           //地址家族
        server_addr.sin_addr.s_addr = (gStUfoData.ipLastAdr << 24) + (1 << 16) + (168 << 8) + 192;//注意转化为网络字节序
        server_addr.sin_port = htons(211);                           //使用SERVER_PORT指定为程序头设定的端口号

        err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));  //建立绑定
        if (err < 0)                                                                //如果绑定失败则关闭套接字
        {
            rt_kprintf("TCP print bind failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //关闭套接字
            osDelay(1000);
            continue;
        }

        err = listen(sock_fd, 2);                                   //监听连接请求
        if (err < 0)                                                //如果监听失败则关闭套接字
        {
            rt_kprintf("TCP print listen failed:%d!\r\n", err);
            closesocket(sock_fd);                                   //关闭套接字
            osDelay(1000);
            continue;
        }

        addr_len = sizeof(struct sockaddr_in);                      //将链接地址赋值给addr_len

        while(IsMxLwipNetifLinkUp())
        {
            sock_conn_new = accept(sock_fd, (struct sockaddr *)&socketConAddrNew, &addr_len);  //对监听到的请求进行连接，状态赋值给sock_conn
            if(sock_conn_new >= 0)  
            {
                closeSelectSocket(&sock_print_conn); 
                rt_kprintf("TCP print connect success: %d!\r\n", sock_conn_new);
                sock_print_conn = sock_conn_new;

                lingerValue.l_onoff = 1;
                lingerValue.l_linger = 0;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_LINGER, &lingerValue, sizeof(lingerValue));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt1 failed:%d!\r\n", err);
                }
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));//SO_REUSEADDR|SO_RCVTIMEO  SO_KEEPALIVE
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt2 failed:%d!\r\n", err);
                }
                timeout.tv_sec = 0;
                timeout.tv_usec = 500000;
                err = setsockopt(sock_conn_new, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt3 failed:%d!\r\n", err);
                }
                //禁用Nagle算法，使TCP立即发送数据而不进行缓冲
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt4 failed:%d!\r\n", err);
                }

                // set Keep-Alive options
                /#keepalive_value = 10000;    // 数据空闲时间多久后发送 10 seconds
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive_value, sizeof(keepalive_value));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt5 failed:%d!\r\n", err);
                }
                keepalive_value = 5000;     // 重试间隔，5 seconds
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive_value, sizeof(keepalive_value));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt6 failed:%d!\r\n", err);
                }
                keepalive_value = 3;        // 重试次数
                err = setsockopt(sock_conn_new, IPPROTO_TCP, TCP_KEEPCNT, &keepalive_value, sizeof(keepalive_value));
                if (err < 0)                                                                //如果绑定失败则关闭套接字
                {
                    rt_kprintf("TCP print setopt7 failed:%d!\r\n", err);
                }#/
            }
            else                                         //状态小于0代表连接故障，此时关闭套接字
            {
                rt_kprintf("TCP print connect failed: %d!\r\n", sock_conn_new);
            }
            osDelay(10);
        }

        rt_kprintf("TCP print socket close!\r\n");
        sock_print_conn = -1;
        closesocket(sock_fd);                           //关闭套接字
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
    //读取flash配置数据
    gUartNumWifiPrint = USART1_DEVICE;
    if(0 == FlashReadWord(FLASH_USART6_USE_FLAG_ADDR))
    {
        gUartNumWifiPrint = USART6_DEVICE;
    }
    FlashReadConfigurePara();
    FlashReadMotorPara();
    gFlashData.resetCnt++;  //复位次数+1
    gU8FlashWriteFlag = 1;  //写flash

    //外设驱动初始化
    MX_UART5_Init();      //串口5需要根据存储在flash中的参数，配置通讯波特率，因此在此处进行初始化

    MX_CAN1_Init();
#ifdef USE_CAN2_DEVICE
    MX_CAN2_Init();
#endif
#ifdef USE_CAN3_DEVICE
    MX_CAN3_Init();
#endif
    AppUsartInit();        //配置需要的串口初始化功能

    InitMotorPara();
    //NavDataInit();
#if (BOOTLOADER_ENABLE == 0)
    MX_IWDG_Init();       //无BootLoader的时候需要初始化看门狗，否则由BootLoader初始化，防止在此初始化失败
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
 功能描述  : 关闭指定序号的socket连接
 输入参数  : int* sockNum
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年4月20日
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
 功能描述  : SBUS 数据解析
 输入参数  : uint8_t* cmdData   命令数据
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月10日
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
 功能描述  : subs数据接收处理
 输入参数  : TickType_t curTime  当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月10日
*****************************************************************************/
static void SBUS_ReceiveProcess(TickType_t curTime)
{
    static uint16_t lSbusIndex = 0, lSize, lSbusStart = 0, lFilterCnt = 0;
    static TickType_t lLastRevTime = 0; //上次接收数据的时刻
    uint16_t lBufferLen = sizeof(gSbusData);
    uint8_t lFlag;

    //接收超时判断
    if(0 == lSbusIndex)
    {
        if(curTime - lLastRevTime >= 200)   //200ms未收到数据，至遥控掉线标志
        {
            lLastRevTime = curTime;
            GPIOSetLedState(LED0, LED_OFF);
            sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;
        }
    }
    else if(curTime - lLastRevTime >= 500)  //500ms未收满一包数据则清空缓冲区
    {
        lLastRevTime = curTime;
        lSbusStart = 0;
        lSbusIndex = 0;
    }

    //接收数据
    if(lSbusIndex > lBufferLen)//接收出错
    {
        lSbusIndex = 0;
    }
    else if(lSbusIndex < lBufferLen) //接收数据
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
    while(lSbusStart + SBUS_DATA_LEN <= lSbusIndex) //接收完一包数量的数据
    {
        if((0x0f == gSbusData[lSbusStart]) && (0x00 == gSbusData[lSbusStart + SBUS_DATA_LEN - 1]))    //校验正确
        {
            lFlag = gSbusData[lSbusStart + SBUS_DATA_LEN - 2] & 0x0c;
            if(0 == lFlag)
            {
                lFilterCnt = 0;
                SBUS_ENcode(&gSbusData[lSbusStart]);  //数据解析
                sys_para->CAR_RTinf.Link &= ~LINK_REMOT_OFF;//退出失联状态
            }
            else if(!(sys_para->CAR_RTinf.Link & LINK_REMOT_OFF))
            {
                if(lFlag & 0x08)
                {
                    lFilterCnt += 2;    //失联+2
                }
                else
                {
                    lFilterCnt++;   //丢帧+1
                }
                if(lFilterCnt > 5)
                {
                    sys_para->CAR_RTinf.Link |= LINK_REMOT_OFF; //遥控关闭或者失联状态
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
    if(lSbusStart >= SBUS_DATA_LEN) //解析完超过一半的数据，将后半段的数据移至前半段
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
 功能描述  : 工控机can消息处理
 输入参数  : TickType_t curTime
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年5月4日
*****************************************************************************/
static void CAN_ReceiveProcess(TickType_t curTime)
{
    CAN_msg msgRead;
    unsigned char crc8Value;
    static unsigned short lastCnt = 256;
    static TickType_t lastTime = 0;
    
    //读CAN缓冲区
    if(HAL_OK == CanDeviceRead(CAN2_DEVICE, &msgRead, 0))
    {
        if(0x980 == msgRead.id)
        {
            crc8Value = CRC8_Table_1(msgRead.data, 7);
            if(crc8Value == msgRead.data[7])
            {
                if((lastCnt < 256) && (((unsigned char)(lastCnt + 1)) != msgRead.data[6])) //帧序号错误
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
                    sys_para->CAR_RTinf.Link |= LINK_QUICK_STOP; //紧急停止
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
 功能描述  : 工控机can消息处理
 输入参数  : rt_bool_t firstFlag    :第一个id标志
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年5月4日
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
 功能描述  : 工控机串口消息处理
 输入参数  : uint32_t deviceNum 串口号
             Usart_msg *pUsartMsg 串口消息指针
             TickType_t curTime 当前时间(ms)
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年6月15日
*****************************************************************************/
static void USART_ReceiveProcess(uint32_t deviceNum, Usart_msg *pUsartMsg, TickType_t curTime)
{
    static TickType_t lLastRevTime = 0;
    uint32_t revLen = 0;
    int32_t sendLen = 0;

    ST_USART_DATA* uart = UsartGetSelectDevice(deviceNum);
  
    //读缓冲区
    if (!IsRxBuffEmpty(*pUsartMsg))      // 接收缓冲区是否为空
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
                sendLen = UsartDeviceWrite(deviceNum, uart->sendBuf, sendLen);//发送
                if((sendLen <= 0) && DEBUG_DATA_TYPE_1)
                {
                     rt_kprintf("Usart send gSendRespondState err!\r\n");
                }
            }
            lLastRevTime = curTime;
            GPIOSetLedState(LED1, LED_ON);
        } 
        USARTRx_DEQUEUE(*pUsartMsg);        // 串口5接收数据出队
    }
    else if(curTime - lLastRevTime >= 1000) //超过1s未收到数据，关闭数据指示灯
    {
        lLastRevTime = curTime;
        GPIOSetLedState(LED1, LED_OFF);
    }
}

/*****************************************************************************
 功能描述  : ufo刹车自动控制
 输入参数  : TickType_t curTime
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月25日
*****************************************************************************/
static void UfoBrakeAutoControl(TickType_t curTime, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    //static TickType_t lastTime = 0;
    uint8_t brakeControlFlag = 0; 

    //等待刹车上电
    if(BRAKE_WAIT == gBrakeWaitOnFlag)
    {
        if(((BRAKE_ON == gBrakeFlag) || (BRAKE_POS_CMD == gBrakeFlag))
            && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))
        {
            if((IS_NOT_CANOPEN_DRIVER(gStMotorData[M_BRAKE].driverType)   //非canopen驱动器
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
    
    //刹车控制
    if(BRAKE_POS_CMD == gBrakeFlag)  //刹车位置模式
    {
        gBrakeFlag = BRAKE_ON;
        brakeControlFlag = 1;
        if(gStMotorData[M_BRAKE].flag & MOTOR_CURRENT_ADJUST_SPEED) //电流调节转速
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
    else if((BRAKE_OFF_CMD == gBrakeFlag) || (BRAKE_OFF_FORCE_CMD == gBrakeFlag))   //刹车断电模式
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

    if(brakeControlFlag)    //刹车命令切换
    {
        LockThread();
        SetMotorRunModeData(M_BRAKE, lMotorRunMode);
        UnLockThread();
    }
}
/*****************************************************************************
 功能描述  : 左右电机速度值控制、差速控制
 输入参数  : int16_t targetValue
            int16_t Steering_Angle                    
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年9月19日
*****************************************************************************/
void UfoLeftRightControl(int16_t targetValue, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    if(gStMotorData[M_LEFT].flag & MOTOR_CURRENT_ADJUST_SPEED) //电流调节转速
    {
        lMotorRunMode->run_mode = MOTOR_RUN_MODE_FLAP_FIX_CURRENT;
    }
    else
    {
        lMotorRunMode->run_mode = MOTOR_RUN_MODE_SPEED;
    }

    lMotorRunMode->target_value = targetValue;

    if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG) //左右侧电机独立控制标志
    {
        SetMotorRunModeData(M_LEFT, lMotorRunMode);
        SetMotorRunModeData(M_RIGHT, lMotorRunMode);
        if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
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
 功能描述  : 紧急停止过程处理
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年12月6日
*****************************************************************************/
static void UfoEmergencyStopProcess( void )
{
    uint32_t lTime;

    lTime = HAL_GetTick() - gErrorTime;
    //故障或遥控断电紧急停止时，判断退出条件
    if(sys_para->CAR_RTinf.Link & (LINK_ERROR_STOP | LINK_REMOTE_STOP))
    {
        //判断驱动电机pid是否结束
        if((M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)
            && (lTime >= 500)) //出故障至少500ms之后再清除故障停止标志
        {
            if(DEBUG_DATA_TYPE_1)
            {
                rt_kprintf("Exit emergency stop:%#x.\r\n", sys_para->CAR_RTinf.Link);
            }
            sys_para->CAR_RTinf.Link &= ~(LINK_ERROR_STOP | LINK_REMOTE_STOP);
        }
    }

    //进入紧急停止时关抱闸
    if(IS_EMERGENCY_STOP_NOT_ERR)   //非故障紧急停止下关抱闸
    {
        if(LOCK_OFF != gLockFlag)
        {
            rt_kprintf("Enter stop:%#x.\r\n", sys_para->CAR_RTinf.Link);
            CLOSE_LOCK; //关闭抱闸
            CLOSE_BRAKE_FORCE;//根据条件判断是否强行关刹车
            if((BRAKE_OFF_FORCE == gBrakeFlag) || (BRAKE_OFF_FORCE_CMD == gBrakeFlag))
            {
                gBrakeWaitOnFlag = BRAKE_WAIT_PRE;
            }
        }
    }

    //退出紧急停止后开抱闸
    if(!IS_EMERGENCY_STOP)
    {
        if(LOCK_OFF == gLockFlag)
        {
            //动力电机驱动器操作模式已经设置完成时，才开抱闸
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
 功能描述  : ufo控制命令解析，ch[4]为F挡位暂未使用
 输入参数  : TickType_t curTime 当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月10日
*****************************************************************************/
static void UfoControlCmdAnalysis(TickType_t curTime)
{
    static TickType_t lastTime = 0, lastRemoteDataTime = 0;
    static u8_t cntFilter = 0, cntFilter1 = 0;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode = {0};      //初始化局部变量，防止变量值不可控，造成异常
    int YM_Limit, YM_Max, YM_Mid = 600, YM_Min = 240;

    UfoEmergencyStopProcess();                      //紧急停止过程处理

    UfoBrakeAutoControl(curTime, &lMotorRunMode);   //刹车自动控制

    if(sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) //接收到遥控数据
    {
        //CH[8]大于1000上电(即B档拔下)，且非 遥控控制或A档拔上情况下遥控关闭或失联
        if((sys_para->SBUS_rx.CH[8] > 1000) && (!(((sys_para->CAR_RTinf.Link & LINK_REMOT_OFF)) && ((sys_para->CAR_RTinf.Link & LINK_REMOTE) || (sys_para->SBUS_rx.CH[9] <= 1000)))))
        {
            if(!(sys_para->CAR_RTinf.Link & LINK_POWER_ON))
            {
                sys_para->CAR_RTinf.Link |= LINK_POWER_ON;              //下发上电命令
                SET_BRAKE(0);  //上电后需要打开刹车位置模式，以进入寻零模式进行刹车位置自学习
                ClearErrorCode(M_TOTAL_NUM);        //遥控重新上电时清除故障标志
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
                    sys_para->CAR_RTinf.Link |= LINK_REMOTE_STOP; //进入遥控紧急停止状态
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    LockThread();
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POWER_OFF;
                    SetMotorRunModeData(M_LEFT, &lMotorRunMode);
                    if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)  //左右侧电机独立控制标志
                    {
                        SetMotorRunModeData(M_RIGHT, &lMotorRunMode);
                    }
                    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
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
    
    if(!(sys_para->CAR_RTinf.Link & LINK_POWER_ON)) //未上电不进行控制
    {
        //故障停机遥控未打开时，使用的是测试工具发指令控制，则在此处停止动力电机
        ErrorAutoStopCmd(&lMotorRunMode);
        return;
    }

    //切换控制连接状态，遥控orPC
    if(sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) //接收到遥控数据
	{
        if(sys_para->SBUS_rx.CH[9] > 1000)  //自动模式挡位
		{
            sys_para->CAR_RTinf.Link |= LINK_HAND_OVER;  //切换至移交状态
            if(gFlashData.testMode) //切换到自动导航控制
            {
                if(!(sys_para->CAR_RTinf.Link & LINK_AUTO_NAV))
                {
                    rt_kprintf("Enter auto nav control mode!\r\n");
                    sys_para->CAR_RTinf.Link |= LINK_AUTO_NAV;  //切换至自动导航
        	        sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;   //退出遥控控制
        	        sys_para->CAR_RTinf.Link &= ~LINK_PC;       //退出PC控制
                }
            }
            else
            {
                if(sys_para->CAR_RTinf.Link & LINK_PC_DATA)  //接收到PC数据
            	{
                    if(!(sys_para->CAR_RTinf.Link & LINK_PC))   //切换时滤波
            	    {
                        cntFilter++;
                        if(cntFilter >= 5)
            	        {
            	            sys_para->CAR_RTinf.Link |= LINK_PC;     //切换至PC控制
                	        sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;//退出遥控控制
                	        sys_para->CAR_RTinf.Link &= ~LINK_AUTO_NAV;//退出自动导航
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
                && (sys_para->CAR_RTinf.Link & LINK_REMOTE))  //移交状态切换至遥控控制时
            {
                sys_para->CAR_RTinf.Link &= ~LINK_HAND_OVER;   //退出移交状态
                if(gStUfoData.flag & UFO_ENABLE_REMOTE_QUCIK_STOP)  //使能遥控紧急停止
                {
                    if(!(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP))
                    {
                        rt_kprintf("Enter remote quickStop.\r\n");
                    }
                    sys_para->CAR_RTinf.Link |= LINK_QUICK_STOP; //紧急停止
                }
            }
            if(!(sys_para->CAR_RTinf.Link & LINK_REMOTE))   //切换时滤波
            {
                cntFilter++;
                if(cntFilter >= 5)
                {
                    sys_para->CAR_RTinf.Link |= LINK_REMOTE;     //切换至手柄控制
                    sys_para->CAR_RTinf.Link &= ~LINK_PC;        //退出PC控制
                    sys_para->CAR_RTinf.Link &= ~LINK_AUTO_NAV;//退出自动导航
                }
            }
            else
            {
                cntFilter = 0;
            }
        }
	}

    //失控判断
    if(sys_para->CAR_RTinf.Link & LINK_REMOTE)  //遥控控制
    {
        if(sys_para->CAR_RTinf.Link & LINK_REMOTE_DATA) //接收到遥控数据
        {
            lastTime = curTime;
        }
        else if((TickType_t)(curTime - lastTime) >= 500) //通信超时
        {
            sys_para->CAR_RTinf.Link &= ~LINK_REMOTE;//退出遥控控制
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_REMOTE_LINE_OFF, ERROR_L_NORMAL);
        }
    }
    if(sys_para->CAR_RTinf.Link & LINK_PC)      //PC控制
    {
        if(sys_para->CAR_RTinf.Link & LINK_PC_DATA)  //接收到PC数据
        {
            lastTime = curTime;
            sys_para->CAR_RTinf.Link &= ~LINK_PC_LOST;    //退出PC数据丢失状态
        }
        else if((TickType_t)(curTime - lastTime) >= 1500) //通信超时
        {
            sys_para->CAR_RTinf.Link &= ~LINK_PC;   //退出PC控制
            //SetErrorCode(M_TOTAL_NUM, ERROR_CODE_PC_LINE_OFF, ERROR_L_NORMAL);
        }
        else if((TickType_t)(curTime - lastTime) >= 500) //通信超时
        {
            if ((ABS_VALUE((int16_t)sys_para->PC_Remote.YM) > 560) || DEBUG_DATA_TYPE_8A)   //油门值≥560cm/s(20.2km/h)或者调试时
            {
                gPrintfTestDataShowFlag = 1;
                sys_para->CAR_RTinf.Link &= ~LINK_PC;   //退出PC控制
            //SetErrorCode(M_TOTAL_NUM, ERROR_CODE_PC_LINE_OFF, ERROR_L_NORMAL);
            }
        }
        else if((TickType_t)(curTime - lastTime) >= 50) //通信卡顿
        {
            sys_para->CAR_RTinf.Link |= LINK_PC_LOST;   //切换至PC数据丢失状态
        }
        
    }

    /*if (DEBUG_DATA_TYPE_81)
    {
        rt_kprintf("time:%d,link:%#x,mbFailCnt:%d,mbClsCnt:%d.\r\n", (TickType_t)(curTime - lastTime), sys_para->CAR_RTinf.Link,
            tcpModbusConFailCnt,tcpModbusSotClsCnt); 
    }*/
    
    //失控时处理
    if((!(sys_para->CAR_RTinf.Link & LINK_REMOTE))
        && (!(sys_para->CAR_RTinf.Link & LINK_PC))
        && (!(sys_para->CAR_RTinf.Link & LINK_AUTO_NAV)))  //无PC控制或遥控控制
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

    //接收到数据，解析控制命令
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
            || (sys_para->CAR_RTinf.Link & LINK_AUTO_NAV))  //PC控制或遥控控制或自动导航
        {        	
            sys_para->CAR_RTinf.Link &= ~LINK_OUT_OF_CTRL;
            //遥控器接管模式
        	if(sys_para->CAR_RTinf.Link & LINK_REMOTE)
        	{  
                sys_para->CAR_RTinf.YM = 	hex_buffer[2];  //中位1000，低位刹车1700，高位加速300
        		sys_para->CAR_RTinf.ZX = 	hex_buffer[3];  //中位1000，低位左转300，高位右转1700
                if(ABS_VALUE(sys_para->CAR_RTinf.YM - CMD_VALUE_OFFSET) <= 50)  //中位滤波
        		{
        		    sys_para->CAR_RTinf.YM = CMD_VALUE_OFFSET;
        		}
                if(ABS_VALUE(sys_para->CAR_RTinf.ZX - CMD_VALUE_OFFSET) <= 30)  //中位滤波
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
        	//PC控制模式
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

                if(0xFFFF == sys_para->PC_Remote.VEL)    //主控卡紧急停止命令
                {
                    if(!(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP))
                    {
                        rt_kprintf("Enter pc quickStop.\r\n");
                    }
                    sys_para->CAR_RTinf.Link |= LINK_QUICK_STOP; //紧急停止
                }
                else if((sys_para->CAR_RTinf.Link & LINK_QUICK_STOP)
                    && (0 != sys_para->CAR_RTinf.YM))
                {
                    rt_kprintf("Exit pc quickStop.\r\n");
                    sys_para->CAR_RTinf.Link &= ~LINK_QUICK_STOP;
                }
                /*if(1 == sys_para->PC_Remote.cmd)    //清除故障命令
                {
                    ClearErrorCode(M_TOTAL_NUM);
                }*/
        	}
            //自动导航控制模式
//        	else
//        	{
//        	    sys_para->CAR_RTinf.YM = CMD_VALUE_OFFSET + Limit(gStNavControl.YM, -CMD_VALUE_MAX, CMD_VALUE_MAX);
//        		sys_para->CAR_RTinf.ZX = CMD_VALUE_OFFSET + Limit(gStNavControl.ZX, -CMD_VALUE_MAX, CMD_VALUE_MAX);
//        	}

            //油门档位控制系数 由hex_buffer[6] 控制
        	YM_Limit = gStMotorData[M_LEFT].limitSpeed;   //最快
        	YM_Max = YM_Limit;
            if(gStMotorData[M_LEFT].initPos > 0) YM_Mid = gStMotorData[M_LEFT].initPos;
            if(gStMotorData[M_RIGHT].initPos > 0) YM_Min = gStMotorData[M_RIGHT].initPos;
            YM_Mid = VelCm_sToRpm(YM_Mid);  //中速限制为默认6m/s或配置文件配置
            if(sys_para->CAR_RTinf.Link & LINK_REMOTE)
            {
                if(!(gStUfoData.flag & UFO_ENABLE_REMOTE_HIGH_VEL_FLAG))   //不允许遥控高速标志
                {
                    YM_Limit = YM_Mid;
                }
            }
        	if(hex_buffer[6] > 1200){
                YM_Limit = VelCm_sToRpm(YM_Min);    //低速限制为默认2.4m/s或者配置文件配置
            }
            else if(hex_buffer[6] > 800){YM_Limit = YM_Mid;}
            if(sys_para->CAR_RTinf.Link & LINK_REMOTE)
            {
                YM_Max = YM_Limit;
            }

            memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
            LockThread();

        	//转向控制
        	if(BRAKE_WAIT_NONE == gBrakeWaitOnFlag)
        	{
        	    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = Limit((s16_t)sys_para->CAR_RTinf.ZX - 1000, -CMD_VALUE_MAX, CMD_VALUE_MAX);
                //差速转向计算
                gSteeringAngleVelDiff = lMotorRunMode.target_value;
                gSteeringAngleVelDiff = gSteeringAngleVelDiff * 18 / CMD_VALUE_MAX;    //对应-18°到18°
                
                if(gStMotorData[M_TURN].flag & MOTOR_DIR_INVERSE_FLAG)  //电机反向
                {
                    lMotorRunMode.target_value = -lMotorRunMode.target_value;
                }
                SetMotorRunModeData(M_TURN, &lMotorRunMode);
        	}
            
        	//油门/刹车
        	if((0 == sys_para->CAR_RTinf.YM) || IS_EMERGENCY_STOP) //停止刹车
        	{
                if((0 == sys_para->CAR_RTinf.YM) && (!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)))
                {
                    sys_para->CAR_RTinf.Link |= LINK_REV_STOP_CMD;  //接收到停止命令
                }
                UfoLeftRightControl(0, &lMotorRunMode);
                if(BRAKE_WAIT_PRE == gBrakeWaitOnFlag)
                {
                    gBrakeWaitOnFlag = BRAKE_WAIT;
                }
        	}
        	else //油门
        	{
                if(BRAKE_WAIT_NONE == gBrakeWaitOnFlag)
                {
                    //lMotorRunMode.target_value = 0;
                    lMotorRunMode.target_value = -sys_para->CAR_RTinf.YM;
                    if(!(sys_para->CAR_RTinf.Link & LINK_PC))   //非pc控制，目标为电机转速
                    {
                        lMotorRunMode.target_value = lMotorRunMode.target_value * YM_Max / 700;  //手柄速度0~700，转换成实际转速
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
 功能描述  : 获取ufo控制状态(使能状态、速度控制档位)
 输入参数  : 无
 输出参数  : uint8_t retFlg(按位存储标志位，0为无效，1为有效)
                             b0:速度控制中速档
                             b1:速度控制高速档(如果b1，b0均为1，则为高速档；如果b1，b0均为0，则为低速档)
                             b2:预留
                             b3:预留
                             b4:电机使能启动状态
                             b5:预留
                             b6:预留
                             b7:预留
 作    者  : 田忠
 日    期  : 2023年09月08日
*****************************************************************************/
uint8_t GetUfoControlStatus(void)
{
    uint8_t i;
    uint8_t retFlg = 0;
 
    for (i=0; i<M_TOTAL_NUM; i++)
    {
        if (DRIVER_TYPE_NONE != gStMotorData[i].driverType)                          //驱动器类型有效的情况下
        {
            if ((OPERATION_MODE_SET_FINISH != gStMotorRunState[i].operModeSetStep)   //切换模式未完成
             || ((gStMotorData[i].flag & ENABLE_AUTO_HOMMING) && (HOME_FLAG_FINISH != gStMotorRunState[i].homeFinishFlag)) //或使能了自动寻零的情况下，寻零未完成
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

    //油门档位控制系数 由hex_buffer[6] 控制
	if(hex_buffer[6] > 1200)
    {
        ;
    }
    else if(hex_buffer[6] > 800)
    {
        retFlg |= (1 << 0);                   //中速档位
    }
    else
    {
        retFlg |= (1 << 1);                   //高速档位
    }

    return retFlg;
}

/*****************************************************************************
 功能描述  : 更新ufo状态信息
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年11月23日
*****************************************************************************/
static void UpdateUfoState(void)
{
    if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //退出故障紧急停止状态后再赋值故障码给工控机，可以预防工控机端在底层故障时提前退出流程不控转向
    {
        //无故障或者运行过程中出现故障，或者非电机故障则进行上传
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
 功能描述  : tcp更新发送数据
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年03月28日
*****************************************************************************/
void TcpUpdateSendData(void)
{
    uint8_t luUfoCtrlFlg;           //ufo控制状态标志位
    
	luUfoCtrlFlg = GetUfoControlStatus();           //ufo控制状态标志位
	
    //数据包交换统计
    sys_para->SBUS_rx.CH[20] = gStBatteryState.voltage;    //电压
    sys_para->SBUS_rx.CH[21] = (gStBatteryState.soc << 8) + (gStBatteryState.maxtemp);//电量，温度

    if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //退出故障紧急停止状态后再赋值故障码给工控机，可以预防工控机端在底层故障时提前退出流程不控转向
    {
        //无故障或者运行过程中出现故障，或者非电机故障则进行上传
        if(gErrorRunStateFlag || (0 == gErrorResult) || (0 == gErrorMotorNum) || (WARNING_MOTOR_NUM_OFFSET == gErrorMotorNum))
        {
            sys_para->SBUS_rx.CH[22] = (0 << 8) + (gErrorMotorNum);//电池故障,电机序号
            sys_para->SBUS_rx.CH[23] = gErrorResult;  //故障
        }
    }
    sys_para->SBUS_rx.CH[24] = (1 << 8) + (luUfoCtrlFlg);  //电机状态,气压值

    sys_para->SBUS_rx.CH[25] = gStMotorRevData[M_LEFT].speed;               //左电机速度值，单位rpm,有符号2字节
    sys_para->SBUS_rx.CH[26] = gStMotorRevData[M_LEFT_ONE].speed;           //左一电机速度值，单位rpm,有符号2字节
    sys_para->SBUS_rx.CH[27] = gStMotorRevData[M_RIGHT].speed;              //右电机速度值，单位rpm,有符号2字节
    sys_para->SBUS_rx.CH[28] = gStMotorRevData[M_RIGHT_ONE].speed;          //右一电机速度值，单位rpm,有符号2字节
    sys_para->SBUS_rx.CH[29] = gStMotorRevData[M_LEFT].current / 100;       //左电机电流值，单位0.1A,有符号2字节
    sys_para->SBUS_rx.CH[30] = gStMotorRevData[M_LEFT_ONE].current / 100;   //左一电机电流值，单位0.1A,有符号2字节
    sys_para->SBUS_rx.CH[31] = gStMotorRevData[M_RIGHT].current / 100;      //右电机电流值，单位0.1A,有符号2字节
    sys_para->SBUS_rx.CH[32] = gStMotorRevData[M_RIGHT_ONE].current / 100;  //右一电机电流值，单位0.1A,有符号2字节
    sys_para->SBUS_rx.CH[33] = gRespondState.turnCurPos;                    //转向当前位置，单位和转向目标位置值一致,有符号2字节
    sys_para->SBUS_rx.CH[34] = sys_para->CAR_RTinf.Link;                    //底层控制器控制连接标志,无符号2字节
    sys_para->SBUS_rx.CH[35] = (gMotorAvaPos & 0x0000FFFF);                 //电机直线运动距离，单位mm,有符号2字节
    sys_para->SBUS_rx.CH[36] = (gMotorAvaPos & 0xFFFF0000) >> 16;           //电机直线运动距离，单位mm,有符号2字节
    sys_para->SBUS_rx.CH[37] = ABS_VALUE(gMotorAvaVelFilter) * 10;          //电机直线运动速度，单位mm/s,无符号2字节
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
 功能描述  : tcp相关处理
 输入参数  : TickType_t curTime
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月14日
*****************************************************************************/
static void TcpSocketProcess(TickType_t curTime)
{
    static TickType_t lLastRevTime = 0; //上次接收数据的时刻
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
            lSendLen = send(sock_conn, &tcpServerSendBuf, lSendLen, MSG_DONTWAIT);//发送
            if((lSendLen < 0) && DEBUG_DATA_TYPE_1)
            {
                 rt_kprintf("Tcp send gSendRespondState err!\r\n");
            }
        }
        
        lLastRevTime = curTime;
        GPIOSetLedState(LED1, LED_ON);
    } 
    else if(curTime - lLastRevTime >= 1000) //超过1s未收到数据，关闭数据指示灯
    {
        lLastRevTime = curTime;
        GPIOSetLedState(LED1, LED_OFF);
    }
}
/*****************************************************************************
 功能描述  : tcp接收命令处理
 输入参数  : uint8_t* cmdData
             uint8_t size      
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年6月8日
*****************************************************************************/
static void TcpRevCmdAnalysis(uint8_t* cmdData, uint8_t size)
{
    //unsigned char crc8Value;
    uint16_t cmd = *(uint16_t*)cmdData;
    static unsigned short lastCnt = 256;
    TickType_t curTime;
    static TickType_t lastTime = 0;

    if((0x00 == cmd) && (size >= 10))  //控制命令
    {
        /*crc8Value = CRC8_Table_1(cmdData + 1, 7);
        if(crc8Value == cmdData[8])*/
        {
            if((lastCnt < 256) && (((unsigned char)(lastCnt + 1)) != cmdData[8])) //帧序号错误
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
            if(size >= 11)  //扩展了最大速度值，功能参数长度9字节+2字节功能码
            {
                sys_para->PC_Remote.max_vel = *(unsigned short*)(cmdData + 9);
            }
            else
            {
                sys_para->PC_Remote.max_vel = 0;
            }
            if(size >= 15)  //扩展了距离减速时刻及目标减速度大小，功能参数长度11字节+4字节功能码
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
 功能描述  : tcp新协议接收处理
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年1月5日
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
    if(UART_RECEIVE_STATE_RECEIVING == gStTcpRevData.flag)//短包数据，小于256字节
    {
        if(gStTcpRevData.length + 8 <= gStTcpRevData.index) //接收完数据
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
 功能描述  : 网络发送
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年11月8日
*****************************************************************************/
static void TcpSocketSendNew(void)
{  
    int32_t l_send_len;

    UpdateUfoState();   //更新ufo状态信息
    
    gSendRespondState.RESPOND_STATE_DATA = gRespondState;
    gSendRespondState.crc16=MODBUS_RTUCrc16((unsigned char *)&gSendRespondState.functional_code,
        gSendRespondState.functional_parameter_len + 2);//crc校验

    if(sock_conn >= 0)
    {
        l_send_len = send(sock_conn, &gSendRespondState, sizeof(SEND_RESPOND_STATE), MSG_DONTWAIT);//发送
        if(l_send_len < 0)
        {
             rt_kprintf("Tcp send gSendRespondState err!\r\n");
        }
    }
}
/*****************************************************************************
 功能描述  : 打印发送数据
 输入参数  : uint8_t* sendBuf
             ssize_t sendLen   
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年4月27日
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
         UsartDeviceWrite(gUartNumWifiPrint, (uint8_t*)(&sendBuf[1]), sendLen - 1); //串口从第一个字节开始发送，不发送包序号
    }
}
/*****************************************************************************
 功能描述  : tcp打印端口接收处理
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年1月5日
*****************************************************************************/
static void TcpPrintRevMsgProcess(void)
{
    int32_t l_receive_len = 0;
    uint32_t i = 0, temp = 0;
    uint32_t lTickTemp = HAL_GetTick();
    uint8_t* lRevBuf = gStTcpPrintRevData.data;
    struct sockaddr_in lUdpClientAddr;			    //udp客户端地址
    socklen_t lUdpClientLen;                        //udp客户端地址长度

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
                if('*' == lRevBuf[i])//短包数据，小于256字节
                {
                    temp = i;
                    gStTcpPrintRevData.flag = UART_RECEIVE_STATE_RECEIVING;
                    gStTcpPrintRevData.lastTick = HAL_GetTick();
                    break;
                }
                else if('L' == lRevBuf[i])//长包数据
                {
                    temp = i;
                    gStTcpPrintRevData.flag = UART_RECEIVE_STATE_RECEIVING_LONG;
                    gStTcpPrintRevData.lastTick = HAL_GetTick();
                    break;
                }
            }
            if((UART_RECEIVE_STATE_IDLE != gStTcpPrintRevData.flag) && (temp > 0))  //去掉前面无效的数据
            {
                for(i = temp; i < gStTcpPrintRevData.index; i++)
                {
                    lRevBuf[i - temp] = lRevBuf[i];
                }
                gStTcpPrintRevData.index -= temp;
            }
        }
        if(UART_RECEIVE_STATE_RECEIVING == gStTcpPrintRevData.flag)//短包数据，小于256字节
        {
            if((0 == gStTcpPrintRevData.length) && (gStTcpPrintRevData.index >= 2))
            {
                gStTcpPrintRevData.length = lRevBuf[1];
                if((gStTcpPrintRevData.length <= 3) && (gStTcpPrintRevData.length >= PRINTF_REV_BUF_SIZE))
                {
                    UsartMsgInit(&gStTcpPrintRevData);
                }
            }
            if((gStTcpPrintRevData.index >= gStTcpPrintRevData.length) && (gStTcpPrintRevData.length > 3))//接收完成
            {
                if('#' == lRevBuf[gStTcpPrintRevData.length - 1])
                {
                    gUartPrintfFlag = 0;    //收到网口命令，网口打印
                    WifiRevCmdAnalysis(lRevBuf, gStTcpPrintRevData.length);
                    if(gStTcpPrintRevData.index > gStTcpPrintRevData.length)    //去掉前面解析完的数据
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
        else if(UART_RECEIVE_STATE_RECEIVING_LONG == gStTcpPrintRevData.flag)//长包数据
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
            if((gStTcpPrintRevData.index >= gStTcpPrintRevData.length) && (gStTcpPrintRevData.length > 4))//接收完成
            {
                if('#' == lRevBuf[gStTcpPrintRevData.length - 1])
                {
                    gUartPrintfFlag = 0;    //收到网口命令，网口打印
                    WifiRevCmdAnalysis(lRevBuf, gStTcpPrintRevData.length);
                    if(gStTcpPrintRevData.index > gStTcpPrintRevData.length)    //去掉前面解析完的数据
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
 功能描述  : 锁定任务
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月29日
*****************************************************************************/
void LockThread(void)
{
    if(!tastStartFlag)
    {
        return;
    }

    if(!tastLockCnt)
    {
        vTaskSuspendAll();         /* 开启调度锁 */
    }
    tastLockCnt++;
}

/*****************************************************************************
 功能描述  : 解锁任务
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月29日
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
            if(!xTaskResumeAll())      /* 关闭调度锁，如果需要任务切换，此函数返回pdTRUE，否则返回pdFALSE */
            {
                taskYIELD ();
            }
        }
    }
}


/* USER CODE END Application */

