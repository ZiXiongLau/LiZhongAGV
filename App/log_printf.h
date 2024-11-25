#ifndef __LOG_PRINTF_H
#define __LOG_PRINTF_H

#define     PRINTF_SEND_LEN_MAX     1360    //最大单次发送数据长度
#define     PRINTF_SEND_BUFFER_LEN  (PRINTF_SEND_LEN_MAX + 100) //数组大小，留一点放调试信息
#define     PRINTF_QUENE_LEN        5       //队列长度    
#define     PRINT_TEST_QUENE_LEN    3       //测试打印队列长度

//打印数据队列结构
typedef struct
{
    uint8_t sendRetryCnt;               //发送重试次数，为0表示未发送
    uint32_t lastSendTick;              //上次发送的时间
    uint16_t dataLen;                   //数据有效长度
    uint8_t data[PRINTF_SEND_BUFFER_LEN];//数据
}PRINT_QUENE;

void rt_kprintf_log_buf(void);

#endif


