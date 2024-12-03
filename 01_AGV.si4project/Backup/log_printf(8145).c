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

extern struct sockaddr_in udpClientAddr;			    //udp�ͻ��˵�ַ

/*****************************************************************************
 ��������  : �������ݴ�ӡ
 �������  : const char *fmt  
             ...              
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��24��
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
 ��������  : ����־��Ϣ����ӡ��ʽд�뻺����
 �������  : const char *fmt        ��ӡ�ַ���
             ...                    
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��2��5��
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
 ��������  : ����־��Ϣͨ�����ڴ�ӡ�˿ڴ�ӡ��ȥ
 �������  : const char *fmt        ��ӡ�ַ���
             ...                    
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��2��5��
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
 ��������  : �������ݴ洢
 �������  : const char *fmt        ��ӡ�ַ���
             ...                    
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��2��5��
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
        if(gPrintTestIndex[0] + 50 >= PRINTF_SEND_LEN_MAX)  //������ŵ�������
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
 ��������  : ���Ͳ�������
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��5��17��
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
 ��������  : �ڶ���β�����ӵ�����Ϣ
 �������  : uint16_t index  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��5��6��
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
 ��������  : �Ӵ�ӡ�˿ڷ�������
 �������  : uint8_t* dataBuf  
             uint16_t dataLen   
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��3��17��
*****************************************************************************/
void rt_ksendData(uint8_t* dataBuf, uint16_t dataLen)
{
    uint16_t index, writeBufFlag, i;
    int16_t lEmptySize;

    LockThread();
    
    writeBufFlag = 0;
    //����������
    if((dataLen > 0) && (dataLen + 1 <= PRINTF_SEND_LEN_MAX))
    {
        index = gPrintQuenePushOut;
        do
        {
            if(gStPrintQuene[index].dataLen > PRINTF_SEND_BUFFER_LEN)  //�������Խ��
            {
                gStPrintQuene[index].dataLen = 0;
                gStPrintQuene[index].sendRetryCnt = 0;
            }
            if(0 == gStPrintQuene[index].sendRetryCnt)  //δ���͹�
            {
                if(0 == gStPrintQuene[index].dataLen)   //д������
                {
                    gStPrintQuene[index].data[0] = gPrintPackageNum;
                    gPrintPackageNum++;
                    gStPrintQuene[index].dataLen++;
                }
                lEmptySize = PRINTF_SEND_LEN_MAX - gStPrintQuene[index].dataLen;
                if(lEmptySize >= dataLen)   //�пռ�
                {
                    writeBufFlag = 1;
                    break;
                }
                else    //�Ѵ���
                {
                    gStPrintQuene[index].sendRetryCnt = 1;  //��ǰ���������������ͣ�����������洢����ֹ��λ
                    gStPrintQuene[index].lastSendTick = 0;  //��ֱ�ӷ���
                    if(index == gPrintQuenePushIn)
                    {
                        if(((gPrintQuenePushIn + 1) % PRINTF_QUENE_LEN) == gPrintQuenePushOut)    //������ȫ��ֱ�ӷ���
                        {
                            //��������˵������������⣬δ�յ���Ӧ���������Ѵ�����
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
                        else    //ָ����һ������
                        {
                            gPrintQuenePushIn++;                    //�����
                            if(gPrintQuenePushIn >= PRINTF_QUENE_LEN) gPrintQuenePushIn = 0;
                            if(DEBUG_DATA_TYPE_95)
                            {
                                usart_kprintf("Save pushIn1:%d,%d.\r\n", gPrintQuenePushIn, HAL_GetTick());
                            }
                        }
                    }
                    if((DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4) || DEBUG_DATA_TYPE_8A)  //�������
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

    //д�뻺����
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
 ��������  : ��ӡ����������־����
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��1��5��
*****************************************************************************/
void rt_kprintf_log_buf(void)
{
    static uint32_t lLastSendTick = 0;
    uint32_t lCurTick;
    uint16_t index, sendFlag, uartSendFlag;

    //��ӡ��������
    if(gPrintfTestDataShowFlag)
    {
        gPrintfTestDataShowFlag = 0;
        testSendData();
    }

    //�Ƿ��Ǵ��ڷ���
    uartSendFlag = 1;
    if((0 == gUartPrintfFlag) && (udp_print_connected_flag > 0))
    {
        uartSendFlag = 0;
    }
    else if(0 == isDmaSendFinish(UsartGetSelectDevice(gUartNumWifiPrint)))  //�ϴδ���dmaδ��������ֱ������
    {
        return;
    }

    lCurTick = HAL_GetTick();

    sendFlag = 0;
    index = gPrintQuenePushOut;
    do
    {
        if(gStPrintQuene[index].dataLen > PRINTF_SEND_BUFFER_LEN)  //�������Խ��
        {
            gStPrintQuene[index].dataLen = 0;
            gStPrintQuene[index].sendRetryCnt = 0;
        }
        if(((lCurTick - lLastSendTick) >= 100) || uartSendFlag) //δ���͹����Ѵ������ݵ�δ��������100ms���ͣ�����Ǵ��ڷ�����ֱ�ӷ���
        {
            if((gStPrintQuene[index].sendRetryCnt == 0) && (gStPrintQuene[index].dataLen > 0))
            {
                gStPrintQuene[index].sendRetryCnt = 1;
                gStPrintQuene[index].lastSendTick = 0;  //��ֱ�ӷ���
                if(gPrintQuenePushIn == index)  //�����
                {
                    gPrintQuenePushIn++;
                    if(gPrintQuenePushIn >= PRINTF_QUENE_LEN) gPrintQuenePushIn = 0;
                    if(DEBUG_DATA_TYPE_95)
                    {
                        usart_kprintf("Save pushIn2:%d,%d.\r\n", gPrintQuenePushIn, lCurTick);
                    }
                }
                if((DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4) || DEBUG_DATA_TYPE_8A)  //�������
                {
                    AddTailDubugMsg(index);
                }
            }
        }
        if(gStPrintQuene[index].sendRetryCnt > 0)  //��Ҫ��������
        {
            if(lCurTick - gStPrintQuene[index].lastSendTick >= 500) //500ms�ط�һ��
            {
                lLastSendTick = lCurTick;
                gStPrintQuene[index].lastSendTick = lCurTick;
                gStPrintQuene[index].sendRetryCnt++;
                sendFlag = 1;
                break;
            }
        }
        else if((gStPrintQuene[index].dataLen == 0) && (gPrintQuenePushOut == index) && (index != gPrintQuenePushIn))   //������
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

    //��������
    if(sendFlag)
    {
        PrintfSendMsg(gStPrintQuene[index].data, gStPrintQuene[index].dataLen, RT_FALSE);
        if(DEBUG_DATA_TYPE_95)
        {
            usart_kprintf("Send%d(%d,%d):%d,%d,%d,%d.\r\n", index, gPrintQuenePushOut, gPrintQuenePushIn, gStPrintQuene[index].data[0], gStPrintQuene[index].dataLen, gStPrintQuene[index].sendRetryCnt, HAL_GetTick());
        }

        //�ط��������ޣ����ߴ��ڷ��ͣ�������ͱ�־
        if((gStPrintQuene[index].sendRetryCnt > 3) || uartSendFlag)
        {
            if((gStPrintQuene[index].sendRetryCnt > 3) 
                && (DEBUG_DATA_TYPE_1 && DEBUG_DATA_TYPE_2 && DEBUG_DATA_TYPE_4))
            {
                gPrintfTestDataShowFlag = 1;
            }
            if((gPrintQuenePushOut == index) && (index != gPrintQuenePushIn))   //������
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
 ��������  : udp���յ���Ӧ�������
 �������  : uint8_t *data ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��4��27��
*****************************************************************************/
void UdpRespondReved(uint8_t *data, uint16_t size)
{
    uint8_t i;
    uint8_t respondNum = data[3];   //�յ���Ӧ�ĸ���
    uint16_t index;
    uint8_t lRespondMsg[13];
    
    if(0 == respondNum) //Ϊ0��ʾΪ���Ӽ���ź�
    {
        if(data[4] > 0)    //����1ʱΪ�����źţ�����λ����������ip�����ⷢ���������1Ϊȷ���ź����ʾip��ȷ
        {
            UnLockThread();
            UnLockThread();
            UnLockThread();
            index = 5;
            lRespondMsg[0] = data[0];   //����ͷ
            lRespondMsg[1] = sizeof(lRespondMsg);//�����ݳ���
            lRespondMsg[2] = data[2];   
            lRespondMsg[3] = data[3];
            lRespondMsg[4] = data[4];
            lRespondMsg[12] = '#';       //����β
            for(i = 0; i < data[4]; i++)
            {
                if((index + 6 + i * 6) <= size)
                {
                    //��ȡip�Ͷ˿�
                    udpClientAddr.sin_family = AF_INET;
                    udpClientAddr.sin_len = sizeof(udpClientAddr);
                    udpClientAddr.sin_addr.s_addr = (data[index + 3 + i * 6] << 24) + (data[index + 2 + i * 6] << 16) + (data[index + 1 + i * 6] << 8) + data[index + i * 6];//ע��ת��Ϊ�����ֽ���
                    udpClientAddr.sin_port = htons((data[index + 5 + i * 6] << 8) + data[index + 4 + i * 6]);   //ʹ��SERVER_PORTָ��Ϊ����ͷ�趨�Ķ˿ں�
                    lRespondMsg[5] = data[index + i * 6];
                    lRespondMsg[6] = data[index + 1 + i * 6];
                    lRespondMsg[7] = data[index + 2 + i * 6];
                    lRespondMsg[8] = data[index + 3 + i * 6];
                    lRespondMsg[9] = data[index + 4 + i * 6];
                    lRespondMsg[10] = data[index + 5 + i * 6];
                    lRespondMsg[11] = CRC8_Table(&lRespondMsg[2], 9);//CRCУ��
                    PrintfSendMsg(lRespondMsg, sizeof(lRespondMsg), RT_TRUE);
                    if(data[4] > 1) //�����ź�
                    {
                        udp_print_connected_flag = 0;   //��������ʱ��������ӱ�־
                        if(DEBUG_DATA_TYPE_95)
                        {
                            usart_kprintf("TestIp%d:%d.%d.%d.%d:%d,%d.\r\n", i, udpClientAddr.sin_addr.s_addr & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 8) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 16) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 24) & 0xff, ntohs(udpClientAddr.sin_port), HAL_GetTick());
                        }
                    }
                    else    //ȷ���ź�
                    {
                        if(DEBUG_DATA_TYPE_95)
                        {
                            usart_kprintf("ConfirmIp%d:%d.%d.%d.%d:%d,%d.\r\n", i, udpClientAddr.sin_addr.s_addr & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 8) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 16) & 0xff,
                            (udpClientAddr.sin_addr.s_addr >> 24) & 0xff, ntohs(udpClientAddr.sin_port), HAL_GetTick());
                        }                        
                        udp_print_connected_flag = 1;
                        //��ջ������е�����
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
        for(i = 0; i < respondNum; i++) //���յ���Ӧ�İ���գ�����Ҫ���Է���
        {
            test_kprintf("P%d", data[i + 4]);
            index = gPrintQuenePushOut;
            do
            {
                if(gStPrintQuene[index].sendRetryCnt > 0)  //���͹�
                {
                    if(data[i + 4] == gStPrintQuene[index].data[0]) //�����ƥ��
                    {                        
                        if(DEBUG_DATA_TYPE_95)
                        {
                            usart_kprintf("Rev pack%d:%d,%d.\r\n", index, data[i + 4], HAL_GetTick());
                        }
                        if((gPrintQuenePushOut == index) && (index != gPrintQuenePushIn))   //������
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


