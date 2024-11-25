#ifndef __FLASH_ACCESS_H__
#define __FLASH_ACCESS_H__ 

//#include "nav.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) //扇区0起始地址, 32 Kbytes  ，bootloader
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) //扇区1起始地址, 32 Kbytes  ，bootloader及串口地址
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) //扇区2起始地址, 32 Kbytes  ，flash数据
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) //扇区3起始地址, 32 Kbytes  ,电机参数
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) //扇区4起始地址, 128 Kbytes ,程序段1
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) //扇区5起始地址, 256 Kbytes ,程序段1
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000) //扇区6起始地址, 256 Kbytes ,程序段2
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000) //扇区7起始地址, 256 Kbytes ,程序段2
#define ADDR_FLASH_END          ((uint32_t)0x08100000) //扇区结束地址

#define FLASH_USART6_USE_FLAG_ADDR (ADDR_FLASH_SECTOR_2 - 4)    //串口6使用标志写flash地址
#define STM32_FLASH_SAVE_ADDR       ADDR_FLASH_SECTOR_2     //flash数据保存地址
#define STM32_FLASH_BYTE_MAX_NUM    512                     //读写flash字节最大个数，必须是4的倍数，需小于等于宏定义块STM32_FLASH_BLOCK_BYTE_NUM的大小
#define STM32_FLASH_BLOCK_NUM       20                      //Flash块个数，用于连续存储最新flash数据，以减少擦写次数
#define STM32_FLASH_BLOCK_BYTE_NUM  1024                    //Flash块大小
#define STM32_FLASH_MOTOR_DATA_ADDR ADDR_FLASH_SECTOR_3     //flash电机数据保存地址
//#define NAV_TARGET_POINT_MAX_NUM        4             //目标点最大个数

#define IAP_MAX_SEND_REV_LEN    992         //发送、接收单包数据最大长度，4的倍数
#define DOWNLOAD_APP            0xEE        //程序下载命令 
#define DOWNLOAD_BOOTLOADER     0xAA        //Bootloader下载命令

#define FLASH_BOOTLOADER_ADDR   ADDR_FLASH_SECTOR_0     //BOOTLOADER起始地址(存放在FLASH)
#define FLASH_APP1_ADDR		    ADDR_FLASH_SECTOR_4     //第一个应用程序起始地址(存放在FLASH)
#define FLASH_APP2_ADDR		    ADDR_FLASH_SECTOR_6     //第二个应用程序起始地址(存放在FLASH)

#define HISTORY_ERROR_MAX_NUM   32          //历史故障最大记录个数

//程序下载各种状态
enum {
    IAP_STATE_IDLE,         //空闲状态，等待3s进入app，3s内如果收到下载新的app固件程序的命令，则开始更新程序
    IAP_CONFIRM,            //确认程序下载模式
    IAP_STATE_BEGINE_UPDATE,//开始升级程序
    IAP_STATE_UPDATE,       //正在升级程序
    IAP_STATE_UPDATE_BOOTLOADER,//更新bootloader
    IAP_STATE_SUCCESS,      //成功升级程序
    IAP_STATE_FAILED        //升级程序失败
};

/**
 * Flash Data
 */
#pragma pack (1)  //1字节对齐
typedef struct
{
    uint32_t             len;
    uint8_t              hardwareType;                   //硬件版本
    uint8_t              testMode;                       //底层驱动测试模式，通过串口处理命令
    uint8_t              enableDebugDataSend;            //使能调试数据上发，将通过rf上发测试数据
    uint8_t              enableBrakeFlag;                //准许刹车标志
    int32_t              turnInitPos;                    //转向初始位置
    uint8_t              ipLastAdr;                      //ip最后一个字节地址
    int32_t              turnRange;                      //转向范围
    uint16_t             resetCnt;                       //复位次数累计(包含上电和命令复位)
    uint8_t              reserved;                       //预留
    uint8_t              deviceSN[32];                   //设备序列号
    uint8_t              reservedOne[64];                //预留
    uint8_t              historyErrorNum[HISTORY_ERROR_MAX_NUM];    //历史故障序号
    uint32_t             historyErrorTime[HISTORY_ERROR_MAX_NUM];   //历史故障时间
    uint16_t             historyErrorCode[HISTORY_ERROR_MAX_NUM];   //历史故障码
    uint16_t             historyErrorResetCnt[HISTORY_ERROR_MAX_NUM];//历史故障对应复位的次数
}ST_FLASH_DATA;
#pragma pack ()  //1字节对齐

#define STM32_FLASH_DEFAULT_CONFIGURE_PARAS     \
{                                               \
    sizeof(ST_FLASH_DATA),                      \
    0xff,   /*默认未定义版本鱼*/                \
    0,      /*默认非测试模式    */              \
    0,      /*默认禁止调试数据上发    */        \
    1,      /*默认准许刹车 */                   \
    0,      /*转向初始位置 */                   \
    233,    /*ip最后一个字节地址*/              \
    10,     /*转向范围*/                        \
    0,		/*复位次数累计*/                     \
}

extern ST_FLASH_DATA gFlashData;    //flash当前数据
extern uint8_t gU8FlashWriteFlag;   //写flash标志

uint32_t FlashReadWord(uint32_t faddr);
void FlashWriteConfigurePara(void);              //将gFlashData结构体写flash
void FlashReadConfigurePara(void);               //读取flash放入结构体gFlashData
void FlashWriteMotorPara(void);
void FlashReadMotorPara(void);
void UsartWriteFlashConfigureParas(uint8_t* cmdData, uint8_t size);
void PrintfHardwareType(rt_bool_t resetFlag);
void IAPCmdMSgDeal(uint8_t cmd, uint8_t* cmdData, uint16_t size);
void PrintfAllHistoryError(void);
void WriteFlashConfigureParasProcess(TickType_t curTime);
void UpdateHistoryError(uint16_t motorNum, uint16_t lResult);

#endif //__FLASH_ACCESS_H__

