#ifndef __LOG_PRINTF_H
#define __LOG_PRINTF_H

#define     PRINTF_SEND_LEN_MAX     1360    //��󵥴η������ݳ���
#define     PRINTF_SEND_BUFFER_LEN  (PRINTF_SEND_LEN_MAX + 100) //�����С����һ��ŵ�����Ϣ
#define     PRINTF_QUENE_LEN        5       //���г���    
#define     PRINT_TEST_QUENE_LEN    3       //���Դ�ӡ���г���

//��ӡ���ݶ��нṹ
typedef struct
{
    uint8_t sendRetryCnt;               //�������Դ�����Ϊ0��ʾδ����
    uint32_t lastSendTick;              //�ϴη��͵�ʱ��
    uint16_t dataLen;                   //������Ч����
    uint8_t data[PRINTF_SEND_BUFFER_LEN];//����
}PRINT_QUENE;

void rt_kprintf_log_buf(void);

#endif


