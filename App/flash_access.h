#ifndef __FLASH_ACCESS_H__
#define __FLASH_ACCESS_H__ 

//#include "nav.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) //����0��ʼ��ַ, 32 Kbytes  ��bootloader
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) //����1��ʼ��ַ, 32 Kbytes  ��bootloader�����ڵ�ַ
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) //����2��ʼ��ַ, 32 Kbytes  ��flash����
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) //����3��ʼ��ַ, 32 Kbytes  ,�������
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) //����4��ʼ��ַ, 128 Kbytes ,�����1
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) //����5��ʼ��ַ, 256 Kbytes ,�����1
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000) //����6��ʼ��ַ, 256 Kbytes ,�����2
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000) //����7��ʼ��ַ, 256 Kbytes ,�����2
#define ADDR_FLASH_END          ((uint32_t)0x08100000) //����������ַ

#define FLASH_USART6_USE_FLAG_ADDR (ADDR_FLASH_SECTOR_2 - 4)    //����6ʹ�ñ�־дflash��ַ
#define STM32_FLASH_SAVE_ADDR       ADDR_FLASH_SECTOR_2     //flash���ݱ����ַ
#define STM32_FLASH_BYTE_MAX_NUM    512                     //��дflash�ֽ���������������4�ı�������С�ڵ��ں궨���STM32_FLASH_BLOCK_BYTE_NUM�Ĵ�С
#define STM32_FLASH_BLOCK_NUM       20                      //Flash����������������洢����flash���ݣ��Լ��ٲ�д����
#define STM32_FLASH_BLOCK_BYTE_NUM  1024                    //Flash���С
#define STM32_FLASH_MOTOR_DATA_ADDR ADDR_FLASH_SECTOR_3     //flash������ݱ����ַ
//#define NAV_TARGET_POINT_MAX_NUM        4             //Ŀ���������

#define IAP_MAX_SEND_REV_LEN    992         //���͡����յ���������󳤶ȣ�4�ı���
#define DOWNLOAD_APP            0xEE        //������������ 
#define DOWNLOAD_BOOTLOADER     0xAA        //Bootloader��������

#define FLASH_BOOTLOADER_ADDR   ADDR_FLASH_SECTOR_0     //BOOTLOADER��ʼ��ַ(�����FLASH)
#define FLASH_APP1_ADDR		    ADDR_FLASH_SECTOR_4     //��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)
#define FLASH_APP2_ADDR		    ADDR_FLASH_SECTOR_6     //�ڶ���Ӧ�ó�����ʼ��ַ(�����FLASH)

#define HISTORY_ERROR_MAX_NUM   32          //��ʷ��������¼����

//�������ظ���״̬
enum {
    IAP_STATE_IDLE,         //����״̬���ȴ�3s����app��3s������յ������µ�app�̼�����������ʼ���³���
    IAP_CONFIRM,            //ȷ�ϳ�������ģʽ
    IAP_STATE_BEGINE_UPDATE,//��ʼ��������
    IAP_STATE_UPDATE,       //������������
    IAP_STATE_UPDATE_BOOTLOADER,//����bootloader
    IAP_STATE_SUCCESS,      //�ɹ���������
    IAP_STATE_FAILED        //��������ʧ��
};

/**
 * Flash Data
 */
#pragma pack (1)  //1�ֽڶ���
typedef struct
{
    uint32_t             len;
    uint8_t              hardwareType;                   //Ӳ���汾
    uint8_t              testMode;                       //�ײ���������ģʽ��ͨ�����ڴ�������
    uint8_t              enableDebugDataSend;            //ʹ�ܵ��������Ϸ�����ͨ��rf�Ϸ���������
    uint8_t              enableBrakeFlag;                //׼��ɲ����־
    int32_t              turnInitPos;                    //ת���ʼλ��
    uint8_t              ipLastAdr;                      //ip���һ���ֽڵ�ַ
    int32_t              turnRange;                      //ת��Χ
    uint16_t             resetCnt;                       //��λ�����ۼ�(�����ϵ�����λ)
    uint8_t              reserved;                       //Ԥ��
    uint8_t              deviceSN[32];                   //�豸���к�
    uint8_t              reservedOne[64];                //Ԥ��
    uint8_t              historyErrorNum[HISTORY_ERROR_MAX_NUM];    //��ʷ�������
    uint32_t             historyErrorTime[HISTORY_ERROR_MAX_NUM];   //��ʷ����ʱ��
    uint16_t             historyErrorCode[HISTORY_ERROR_MAX_NUM];   //��ʷ������
    uint16_t             historyErrorResetCnt[HISTORY_ERROR_MAX_NUM];//��ʷ���϶�Ӧ��λ�Ĵ���
}ST_FLASH_DATA;
#pragma pack ()  //1�ֽڶ���

#define STM32_FLASH_DEFAULT_CONFIGURE_PARAS     \
{                                               \
    sizeof(ST_FLASH_DATA),                      \
    0xff,   /*Ĭ��δ����汾��*/                \
    0,      /*Ĭ�Ϸǲ���ģʽ    */              \
    0,      /*Ĭ�Ͻ�ֹ���������Ϸ�    */        \
    1,      /*Ĭ��׼��ɲ�� */                   \
    0,      /*ת���ʼλ�� */                   \
    233,    /*ip���һ���ֽڵ�ַ*/              \
    10,     /*ת��Χ*/                        \
    0,		/*��λ�����ۼ�*/                     \
}

extern ST_FLASH_DATA gFlashData;    //flash��ǰ����
extern uint8_t gU8FlashWriteFlag;   //дflash��־

uint32_t FlashReadWord(uint32_t faddr);
void FlashWriteConfigurePara(void);              //��gFlashData�ṹ��дflash
void FlashReadConfigurePara(void);               //��ȡflash����ṹ��gFlashData
void FlashWriteMotorPara(void);
void FlashReadMotorPara(void);
void UsartWriteFlashConfigureParas(uint8_t* cmdData, uint8_t size);
void PrintfHardwareType(rt_bool_t resetFlag);
void IAPCmdMSgDeal(uint8_t cmd, uint8_t* cmdData, uint16_t size);
void PrintfAllHistoryError(void);
void WriteFlashConfigureParasProcess(TickType_t curTime);
void UpdateHistoryError(uint16_t motorNum, uint16_t lResult);

#endif //__FLASH_ACCESS_H__

