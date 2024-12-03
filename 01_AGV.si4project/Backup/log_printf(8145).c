#include <preDef.h>
#include "log_printf.h"
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"
#include <lwip/sockets.h>
#include "CRC8.h"
#include <stdlib.h>
#include <string.h>


PRINT_QUENE gStPrintQuene[PRINTF_QUENE_LEN] = {0};
uint16_t gPrintQuenePushIn = 0;
uint16_t gPrintQuenePushOut = 0;
uint8_t gPrintPackageNum = 0;
uint8_t gPrintTestData[PRINT_TEST_QUENE_LEN][PRINTF_SEND_BUFFER_LEN] = {0};
uint16_t gPrintTestIndex[PRINT_TEST_QUENE_LEN] = {0};
uint8_t gPrintfTestDataShowFlag = 0;

extern struct sockaddr_in udpClientAddr;			    //udp客户端地址

/*****************************************************************************
 功能描述  : 调试数据打印
 输入参数  : const char *fmt  
             ...              
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月24日
*****************************************************************************/
/*void rt_kprintf(const char *fmt, ...)
{
    va_list args;
    uint32_t length; 

    LockThread();

    va_start(args, fmt);
    
    length = vsnprintf((char*)PrintInfoBuf, PRINTF_BUF_SIZE - 1, fmt, args);
    
    if (length > PRINTF_BUF_SIZE - 1)
    {
        length = PRINTF_BUF_SIZE - 1;
    }

    UsartDeviceWrite(gUartNumWifiPrint, PrintInfoBuf, length);

    va_end(args);

    UnLockThread();
}*/
/*****************************************************************************
 功能描述  : 将日志信息按打印格式写入缓冲区
 输入参数  : const char *fmt        打印字符串
             ...                    
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年2月5日
*****************************************************************************/
void rt_kprintf(const char *fmt, ...)
{
    va_list args;
    char buf[128];
    int16_t lSizeTemp;

    LockThread(); 

    va_start(args, fmt);
                     
    lSizeTemp = vsnprintf(buf, sizeof(buf) - 1, fmt, args);

    if(lSizeTemp > sizeof(buf))
    {
        lSizeTemp = sizeof(buf);
    }

    va_end(args);

    if(lSizeTemp > 0)
    {
        rt_ksendData((uint8_t*)buf, lSizeTemp);
    }

    UnLockThread();
}
/*****************************************************************************
 功能描述  : 将日志信息通过串口打印端口打印出去
 输入参数  : const char *fmt        打印字符串
             ...                    
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年2月5日
*****************************************************************************/
void usart_kprintf(const char *fmt, ...)
{
    va_list args;
    char buf[128];
    int16_t lSizeTemp;

    va_start(args, fmt);
                     
    lSizeTemp = vsnprintf(buf, sizeof(buf) - 1, fmt, args);

    if(lSizeTemp > sizeof(buf))
    {
        lSizeTemp = sizeof(buf);
    }

    va_end(args);

    if(lSizeTemp > 0)
    {
        LockThread(); 
        UsartDeviceWrite(gUartNumWifiPrint, (uint8_t*)buf, lSizeTemp);
        UnLockThread();
    }
}
/*****************************************************************************
 功能描述  : 测试数据存储
 输入参数  : const char *fmt        打印字符串
             ...                    
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年2月5日
*****************************************************************************/
void test_kprintf(const char *fmt, ...)
{
    va_list args;
    int16_t lSizeTemp = 0;

    va_start(args, fmt);

    if(PRINTF_SEND_BUFFER_LEN > gPrintTestIndex[0])
    {
        lSizeTemp = vsnprintf((char*)(&gPrintTestData[0][gPrintTestIndex[0]]), PRINTF_SEND_BUFFER_LEN - gPrintTestIndex[0], fmt, args);
    }
    else
    {
        gPrintTestIndex[0] = 0;
    }

    if(lSizeTemp + gPrintTestIndex[0] >= PRINTF_SEND_BUFFER_LEN)
    {
        gPrintTestIndex[0] = 0;
    }
    else if(lSizeTemp > 0)
    {
        gPrintTestIndex[0] += lSizeTemp;
        if(gPrintTestIndex[0] + 50 >= PRINTF_SEND_LEN_MAX)  //存满则放到备份区
        {
            for(lSizeTemp = PRINT_TEST_QUENE_LEN - 1; lSizeTemp > 0; lSizeTemp--)
            {
                if(gPrintTestIndex[lSizeTemp - 1] > 0)
                {
                    memcpy((uint8_t*)gPrintTestData[lSizeTemp], (uint8_t*)gPrintTestData[lSizeTemp - 1], gPrintTestIndex[lSizeTemp - 1]);
                    gPrintTestIndex[lSizeTemp] = gPrintTestIndex[lSizeTemp - 1];
                }
            }
            gPrintTestIndex[0] = 0;
        }
    }

    va_end(args);
}
/*****************************************************************************
 功能描述  : 发送测试数据
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年5月17日
*****************************************************************************/
void testSendData(void)
{
    int16_t i;

    for(i = PRINT_TEST_QUENE_LEN - 1; i >= 0; i--)
    {
        if(gPrintTestIndex[i] > 0)
        {
            rt_ksendData((uint8_t*)gPrintTestData[i], gPrintTestIndex[i]);
        }
    }
}
/*****************************************************************************
 功能描述  : 在队列尾部增加调试信息
 输入参数  : uint16_t index  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年5月6日
*****************************************************************************/
void AddTailDubugMsg(uint16_t index)
{
    int16_t lSizeTemp, lEmptySize;
    
    if(index < PRINTF_QUENE_LEN)
    {
        if(PRINTF_SEND_BUFFER_LEN > gStPrintQuene[index].dataLen)
        {
            lEmptySize = PRINTF_SEND_BUFFER_LEN - gStPrintQuene[index].dataLen;
            lSizeTemp = snprintf((char*)(&gStPrintQuene[index].data[gStPrintQuene[index].dataLen]),
                lEmptySize, "\r\n\r\nIndex:%d,len:%d,out:%d,in:%d,pack:%d,tick:%lu.\r\n\r\n", index, gStPrintQuene[index].dataLen,
                gPrintQuenePushOut, gPrintQuenePushIn, gStPrintQuene[index].data[0], HAL_GetTick());
            if(lSizeTemp > 0)
            {
                gStPrintQuene[index].dataLen += lSizeTemp;
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 从打印端口返回数据
 输入参数  : uint8_t* dataBuf  
             uint16_t dataLen   
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年3月17日
*****************************************************************************/
void rt_ksendData(uint8_t* dataBuf, uint16_t dataLen)
{
    uint16_t index, writeBufFlag, i;
    int16_t lEmptySize;

    LockThread();
    
    writeBufFlag = 0;
    //搜索空区域
    if((dataLen > 0) && (dataLen + 1 <= PRINTF_SEND_LEN_MAX))
    {
        index = gPrintQuenePushOut;
        do
        {
            if(gStPrintQuene[index].dataLen > PRINTF_SEND_BUFFER_LEN)  //检查数组越界
            {
                gStPrintQuene[index].dataLen = 0;
                gStPrintQuene[index].sendRetryCnt = 0;
            }
            if(0 == gStPrintQuene[index].sendRetryCnt)  //未发送过
            {
                if(0 == gStPrintQuene[index].dataLen)   //写入包序号
                {
                    gStPrintQuene[index].data[0] = gPrintPackageNum;
                    gPrintPackageNum++;
                    gStPrintQuene[index].dataLen++;
                }
                lEmptySize = PRINTF_SEND_LEN_MAX - gStPrintQuene[index].dataLen;
                if(lEmptySize >= dataLen)   //有空间
                {
                    writeBufFlag = 1;
                    break;
                }
                else    //已存满
                {
                    gStPrintQuene[index].sendRetryCnt = 1;  //当前缓冲区存满允许发送，不允许继续存储，防止错位
                    gStPrintQuene[index].lastSendTick = 0;  //可直接发送
                    if(index == gPrintQuenePushIn)
                    {
                        if(((gPrintQuenePushIn + 1) % PRINTF_QUENE_LEN) == gPrintQuenePushOut)    //缓冲区全满直接返回
                        {
                            //缓冲区满说明网络出现问题，未收到响应包，丢弃已存数据
                            for(i = 0; i < PRINTF_QUENE_LEN; i++)
                            {
                                gStPrintQuene[i].sendRetryCnt = 0;
                                gStPrintQuene[i].dataLen = 0;
                            }
                            gPrintQuenePushOut = gPrintQuenePushIn;
                            if(DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4)
                            {
                                usart_kprintf("Save full:%d.\r\n", HAL_GetTick());
                            }
                            break;
                        }
                        else    //指向下一缓冲区
                        {
                            gPrintQuenePushIn++;                    //入队列
                            if(gPrintQuenePushIn >= PRINTF_QUENE_LEN) gPrintQuenePushIn = 0;
                            if(DEBUG_DATA_TYPE_95)
                            {
                                usart_kprintf("Save pushIn1:%d,%d.\r\n", gPrintQuenePushIn, HAL_GetTick());
                            }
                        }
                    }
                    if((DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4) || DEBUG_DATA_TYPE_8A)  //网络调试
                    {
                        AddTailDubugMsg(index);
                    }
                }
            }
            if(index == gPrintQuenePushIn)
            {
                break;
            }
            index++;
            if(index >= PRINTF_QUENE_LEN) index = 0;
        }while(1);
    }

    //写入缓冲区
    if(writeBufFlag)
    {
        memcpy((uint8_t*)(&gStPrintQuene[index].data[gStPrintQuene[index].dataLen]), (uint8_t*)dataBuf, dataLen);
        gStPrintQuene[index].dataLen += dataLen;
        if(DEBUG_DATA_TYPE_95)
        {
            usart_kprintf("Save%d(%d,%d):%d,%d,%d,%d,%d.\r\n", index, gPrintQuenePushOut, gPrintQuenePushIn, gStPrintQuene[index].data[0], dataLen, gStPrintQuene[index].dataLen, gStPrintQuene[index].sendRetryCnt, HAL_GetTick());
        }
    }

    UnLockThread();
}
/*****************************************************************************
 功能描述  : 打印缓冲区的日志数据
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年1月5日
*****************************************************************************/
void rt_kprintf_log_buf(void)
{
    static uint32_t lLastSendTick = 0;
    uint32_t lCurTick;
    uint16_t index, sendFlag, uartSendFlag;

    //打印调试数据
    if(gPrintfTestDataShowFlag)
    {
        gPrintfTestDataShowFlag = 0;
        testSendData();
    }

    //是否是串口发送
    uartSendFlag = 1;
    if((0 == gUartPrintfFlag) && (udp_print_connected_flag > 0))
    {
        uartSendFlag = 0;
    }
    else if(0 == isDmaSendFinish(UsartGetSelectDevice(gUartNumWifiPrint)))  //上次串口dma未传输完则直接跳出
    {
        return;
    }

    lCurTick = HAL_GetTick();

    sendFlag = 0;
    index = gPrintQuenePushOut;
    do
    {
        if(gStPrintQuene[index].dataLen > PRINTF_SEND_BUFFER_LEN)  //检查数组越界
        {
            gStPrintQuene[index].dataLen = 0;
            gStPrintQuene[index].sendRetryCnt = 0;
        }
        if(((lCurTick - lLastSendTick) >= 100) || uartSendFlag) //未发送过并已存入数据但未存满，满100ms则发送，如果是串口发送则直接发送
        {
            if((gStPrintQuene[index].sendRetryCnt == 0) && (gStPrintQuene[index].dataLen > 0))
            {
                gStPrintQuene[index].sendRetryCnt = 1;
                gStPrintQuene[index].lastSendTick = 0;  //可直接发送
                if(gPrintQuenePushIn == index)  //入队列
                {
                    gPrintQuenePushIn++;
                    if(gPrintQuenePushIn >= PRINTF_QUENE_LEN) gPrintQuenePushIn = 0;
                    if(DEBUG_DATA_TYPE_95)
                    {
                        usart_kprintf("Save pushIn2:%d,%d.\r\n", gPrintQuenePushIn, lCurTick);
                    }
                }
                if((DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4) || DEBUG_DATA_TYPE_8A)  //网络调试
                {
                    AddTailDubugMsg(index);
                }
            }
        }
        if(gStPrintQuene[index].sendRetryCnt > 0)  //需要发送数据
        {
            if(lCurTick - gStPrintQuene[index].lastSendTick >= 500) //500ms重发一次
            {
                lLastSendTick = lCurTick;
                gStPrintQuene[index].lastSendTick = lCurTick;
                gStPrintQuene[index].sendRetryCnt++;
                sendFlag = 1;
                break;
            }
        }
        else if((gStPrintQuene[index].dataLen == 0) && (gPrintQuenePushOut == index) && (index != gPrintQuenePushIn))   //出队列
        {
            gPrintQuenePushOut++;
            if(gPrintQuenePushOut >= PRINTF_QUENE_LEN) gPrintQuenePushOut = 0;
            if(DEBUG_DATA_TYPE_95)
            {
                usart_kprintf("Save pushOut1:%d,%d.\r\n", gPrintQuenePushOut, lCurTick);
            }
        }
        if(index == gPrintQuenePushIn)
        {
            break;
        }
        index++;
        if(index >= PRINTF_QUENE_LEN) index = 0;
    }while(1);

    //发送数据
    if(sendFlag)
    {
        PrintfSendMsg(gStPrintQuene[index].data, gStPrintQuene[index].dataLen, RT_FALSE);
        if(DEBUG_DATA_TYPE_95)
        {
            usart_kprintf("Send%d(%d,%d):%d,%d,%d,%d.\r\n", index, gPrintQuenePushOut, gPrintQuenePushIn, gStPrintQuene[index].data[0], gStPrintQuene[index].dataLen, gStPrintQuene[index].sendRetryCnt, HAL_GetTick());
        }

        //重发次数超限，或者串口发送，清除发送标志
        if((gStPrintQuene[index].sendRetryCnt > 3) || uartSendFlag)
        {
            if((gStPrintQuene[index].sendRetryCnt > 3) 
                && (DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4))
            {
                gPrintfTestDataShowFlag = 1;
            }
            if((gPrintQuenePushOut == index) && (index != gPrintQuenePushIn))   //出队列
            {
                gPrintQuenePushOut++;
                if(gPrintQuenePushOut >= PRINTF_QUENE_LEN) gPrintQuenePushOut = 0;
                if(DEBUG_DATA_TYPE_95)
                {
                    usart_kprintf("Save pushOut2:%d,%d.\r\n", gPrintQuenePushOut, lCurTick);
                }
            }
            gStPrintQuene[index].dataLen = 0;
            gStPrintQuene[index].sendRetryCnt = 0; 
        }

        test_kprintf("I%dP%dL%dC%d(o%di%d)%d\n", index, gStPrintQuene[index].data[0], gStPrintQuene[index].dataLen, gStPrintQuene[index].sendRetryCnt, gPrintQuenePushOut, gPrintQuenePushIn, HAL_GetTick() / 100 % 100);
    }
}
/*****************************************************************************
 功能描述  : udp接收到响应数据序号
 输入参数  : uint8_t *data 数据
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年4月27日
*****************************************************************************/
void UdpRespondReved(uint8_t *data, uint16_t size)
{
    uint8_t i;
    uint8_t respondNum = data[3];   //收到响应的个数
    uint16_t index;
    uint8_t lRespondMsg[13];
    
    if(0 == respondNum) //为0表示为连接检测信号
    {
        if(data[4] > 0)    //大于1时为测试信号，对上位机发过来的ip都向外发，如果等于1为确认信号则表示ip正确
        {
            UnLockThread();
            UnLockThread();
            UnLockThread();
            index = 5;
            lRespondMsg[0] = data[0];   //数据头
            lRespondMsg[1] = sizeof(lRespondMsg);//总数据长度
            lRespondMsg[2] = data[2];   
            lRespondMsg[3] = data[3];
            lRespondMsg[4] = data[4];
            lRespondMsg[12] = '#';       //数据尾
            for(i = 0; i < data[4]; i++)
            {
                if((index + 6 + i * 6) <= size)
                {
                    //获取ip和端口
                    udpClientAddr.sin_family = AF_INET;
                    udpClientAddr.sin_len = sizeof(udpClientAddr);
                    udpClientAddr.sin_addr.s_addr = (data[index + 3 + i * 6] << 24) + (data[index + 2 + i * 6] << 16) + (data[index + 1 + i * 6] << 8) + data[index + i * 6];//注意转化为网络字节序
                    udpClientAddr.sin_port = htons((data[index + 5 + i * 6] << 8) + data[index + 4 + i * 6]);   //使用SERVER_PORT指定为程序头设定的端口号
                    lRespondMsg[5] = data[index + i * 6];
                    lRespondMsg[6] = data[index + 1 + i * 6];
                    lRespondMsg[7] = data[index + 2 + i * 6];
                    lRespondMsg[8] = data[index + 3 + i * 6];
                    lRespondMsg[9] = data[index + 4 + i * 6];
                    lRespondMsg[10] = data[index + 5 + i * 6];
                    lRespondMsg[11] = CRC8_Table(&lRespondMsg[2], 9);//CRC校验
                    PrintfSendMsg(lRespondMsg, sizeof(lRespondMsg), RT_TRUE);
                    if(data[4] > 1) //测试信号
                    {
                        udp_print_connected_flag = 0;   //重新连接时清除已连接标志
                        if(DEBUG_DATA_TYPE_95)
                        {
                            usart_kprintf("TestIp%d:%d.%d.%d.%d:%d,%d.\r\n", i, udpClientAddr.sin_addr.s_addr & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 8) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 16) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 24) & 0xff, ntohs(udpClientAddr.sin_port), HAL_GetTick());
                        }
                    }
                    else    //确认信号
                    {
                        if(DEBUG_DATA_TYPE_95)
                        {
                            usart_kprintf("ConfirmIp%d:%d.%d.%d.%d:%d,%d.\r\n", i, udpClientAddr.sin_addr.s_addr & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 8) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 16) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 24) & 0xff, ntohs(udpClientAddr.sin_port), HAL_GetTick());
                        }                        
                        udp_print_connected_flag = 1;
                        //清空缓冲区中的数据
                        gPrintPackageNum = 0;
                        for(i = 0; i < PRINTF_QUENE_LEN; i++)
                        {
                            gStPrintQuene[i].sendRetryCnt = 0;
                            gStPrintQuene[i].dataLen = 0;
                        }
                        gPrintQuenePushOut = gPrintQuenePushIn;
                        rt_kprintf("Udp connect:%d.%d.%d.%d:%d.\r\n", udpClientAddr.sin_addr.s_addr & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 8) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 16) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 24) & 0xff, ntohs(udpClientAddr.sin_port));
                        PrintfUfoVerAndSn();
                    }
                }
                else
                {
                    break;
                }
            }
        }
        
    }
    else if(respondNum <= PRINTF_QUENE_LEN)
    {
        test_kprintf("R%d", respondNum);
        for(i = 0; i < respondNum; i++) //将收到响应的包清空，不需要重试发送
        {
            test_kprintf("P%d", data[i + 4]);
            index = gPrintQuenePushOut;
            do
            {
                if(gStPrintQuene[index].sendRetryCnt > 0)  //发送过
                {
                    if(data[i + 4] == gStPrintQuene[index].data[0]) //包序号匹配
                    {                        
                        if(DEBUG_DATA_TYPE_95)
                        {
                            usart_kprintf("Rev pack%d:%d,%d.\r\n", index, data[i + 4], HAL_GetTick());
                        }
                        if((gPrintQuenePushOut == index) && (index != gPrintQuenePushIn))   //出队列
                        {
                            gPrintQuenePushOut++;
                            if(gPrintQuenePushOut >= PRINTF_QUENE_LEN) gPrintQuenePushOut = 0;
                            if(DEBUG_DATA_TYPE_95)
                            {
                                usart_kprintf("Save pushOut3:%d,%d.\r\n", gPrintQuenePushOut, HAL_GetTick());
                            }
                        }
                        test_kprintf("I%d(o%di%d)", index, gPrintQuenePushOut, gPrintQuenePushIn);
                        gStPrintQuene[index].sendRetryCnt = 0;
                        gStPrintQuene[index].dataLen = 0;
                        break;
                    }
                }
                if(index == gPrintQuenePushIn)
                {
                    break;
                }
                index++;
                if(index >= PRINTF_QUENE_LEN) index = 0;
            }while(1);
        }
        test_kprintf("t%d\n", HAL_GetTick() / 100 % 100);
    }
}


