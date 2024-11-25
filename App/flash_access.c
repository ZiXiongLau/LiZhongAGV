#include "motor_control.h"
#include "usart.h"
#include "iwdg.h"
#include "CRC8.h"

#define FLASH_WAITETIME  50000          //FLASH�ȴ���ʱʱ��
static uint8_t gFlashBuffer[STM32_FLASH_BYTE_MAX_NUM];//��дflash������
static const ST_FLASH_DATA gDefaultFlashData = STM32_FLASH_DEFAULT_CONFIGURE_PARAS;//Ĭ��flash����
ST_FLASH_DATA gFlashData = STM32_FLASH_DEFAULT_CONFIGURE_PARAS;   //flash��ǰ����
ST_MOTOR_DATA gStMotorData[M_TOTAL_NUM];
ST_UFO_DATA  gStUfoData = {0xcc};

uint8_t gU8FlashWriteFlag = 0;  //flashд��־

uint8_t gU8DownloadType = 0;    //������������
int gIapState = IAP_STATE_IDLE;  //�������ظ���״̬
uint32_t gAppLength;             //�����س���ĳ���
uint32_t gAppIndex;              //��ǰ�����������
uint8_t gAppHeader[4];           //app����ͷ
uint32_t gAppStartAdr = FLASH_APP1_ADDR;    //������ʼ��ַ

/*****************************************************************************
 ��������  : ��ȡָ����ַ�İ���(16λ����) 
 �������  : uint32_t faddr  ����ַ 
 �������  : uint32_t  ��Ӧ����
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
uint32_t FlashReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr;
}
/*****************************************************************************
 ��������  : ��ȡĳ����ַ���ڵ�flash����
 �������  : rt_uint32_t addr  ��ַ
 �������  : rt_uint16_t ����ֵ:0~11,��addr���ڵ�����
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
uint16_t FlashGetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	return FLASH_SECTOR_7;	
}
/*****************************************************************************
 ��������  : ��ָ����ַ��ʼд��ָ�����ȵ����ݣ�ע����Ȳ�����Ƭ����
 �������  : uint32_t WriteAddr   ��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
             uint32_t *pBuffer    ����ָ��
             uint32_t NumToWrite  ��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
HAL_StatusTypeDef FlashWrite(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)	
{ 
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t endaddr = 0, addrx = 0;
    uint32_t SectorError = 0;

    if(WriteAddr < STM32_FLASH_SAVE_ADDR || WriteAddr % 4)	//�Ƿ���ַ
    {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock();									//���� 

    addrx=WriteAddr;				        //д�����ʼ��ַ
    endaddr = WriteAddr + NumToWrite * 4;	//д��Ľ�����ַ

    if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(FlashReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
                //MX_IWDG_Init();
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     //�������ͣ��������� 
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
				FlashEraseInit.Sector = FlashGetFlashSector(addrx);
                FlashEraseInit.NbSectors = 1;                               //һ��ֻ����һ������
                status = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
                SCB_CleanInvalidateDCache();                                //�����Ч��D-Cache
                FLASH_WaitForLastOperation(FLASH_WAITETIME);                //�ȴ��ϴβ������
                //IWDG_Feed();
                //MX_IWDG_Init1S();
				if(status != HAL_OK)break;	//����������
			}else addrx+=4;
		} 
	}

    if(status == HAL_OK)
    {
        while(WriteAddr < endaddr)//д����
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer);
        	if(status != HAL_OK)//д������
        	{ 
        		break;
        	}
        	WriteAddr += 4;
        	pBuffer++;
        } 
    }

    HAL_FLASH_Lock();//����

    return status;
}
/*****************************************************************************
 ��������  : ��ָ����ַ��ʼ����ָ�����ȵ�����
 �������  : uint32_t ReadAddr   ��ʼ��ַ
             uint32_t *pBuffer   ����ָ��
             uint32_t NumToRead  ��(4λ)��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
void FlashRead(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)  	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i] = FlashReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr += 4;//ƫ��4���ֽ�.	
	}
}
/*****************************************************************************
 ��������  : �ж�һ��flash���Ƿ�Ϊ�հ�
 �������  : uint32_t blockAddr      
             uint32_t blockByteSize  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2024��3��28��
*****************************************************************************/
HAL_StatusTypeDef FlashBlockIsBland(uint32_t blockAddr, uint32_t blockByteSize)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t endaddr = 0;
    
    if(blockAddr < STM32_FLASH_SAVE_ADDR || blockAddr % 4 || (blockAddr + blockByteSize >= ADDR_FLASH_END)) //�Ƿ���ַ
    {
        return HAL_ERROR;
    }

    endaddr = blockAddr + blockByteSize;	//д��Ľ�����ַ

    while(blockAddr < endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
	{
		if(FlashReadWord(blockAddr) != 0XFFFFFFFF)  //�з�0XFFFFFFFF�ĵط�,��д������
		{
		    return HAL_ERROR;
		}
        else
        {
            blockAddr += 4;
        }
	}

    return status;
}
/*****************************************************************************
 ��������  : ��gFlashData�ṹ��дflash
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
void FlashWriteConfigurePara(void)
{
    ST_FLASH_DATA* lFlashData = &gFlashData;
    uint32_t i = 0, lTime = 0;

    lFlashData->len = sizeof(ST_FLASH_DATA);

    if(lFlashData->len >= STM32_FLASH_BYTE_MAX_NUM)
    {
        rt_kprintf("Write flash len err: %d.\r\n", lFlashData->len);
    }    
    else
    {
        //�����һ��հ�����д
        for(i = 0; i < STM32_FLASH_BLOCK_NUM; i++)
        {
            if(HAL_OK == FlashBlockIsBland(STM32_FLASH_SAVE_ADDR + i * STM32_FLASH_BLOCK_BYTE_NUM, STM32_FLASH_BYTE_MAX_NUM))
            {
                break;
            }
        }
        if(DEBUG_DATA_TYPE_97)
        {
            rt_kprintf("Empty flash num: %d.\r\n", i);
        }
        if(STM32_FLASH_BLOCK_NUM == i)  //���û�пհ�������ӵ�һ�����������ʼд
        {
            i = 0;
        }
        if(DEBUG_DATA_TYPE_97)
        {
            rt_kprintf("Write flash num: %d.\r\n", i);
            lTime = TIM5->CNT;
        }
        memset(gFlashBuffer, 0xff, STM32_FLASH_BYTE_MAX_NUM);
        memcpy(gFlashBuffer, (uint8_t*)lFlashData, lFlashData->len);
        gFlashBuffer[lFlashData->len] = CRC8_Table(gFlashBuffer, lFlashData->len);
        FlashWrite(STM32_FLASH_SAVE_ADDR + i * STM32_FLASH_BLOCK_BYTE_NUM, (uint32_t*)gFlashBuffer, STM32_FLASH_BYTE_MAX_NUM >> 2);
        if(DEBUG_DATA_TYPE_97)  //ʵ�����214ms��дflash��Ҫ1.67ms
        {
            rt_kprintf("Write flash time: %d (10us).\r\n", TIM5->CNT - lTime);
        }
    }
}
/*****************************************************************************
 ��������  : ���������дflash
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��10��22��
*****************************************************************************/
void FlashWriteMotorPara(void)
{
    uint32_t crc16;
    FlashWrite(STM32_FLASH_MOTOR_DATA_ADDR, (uint32_t*)gStMotorData, (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM) >> 2);
    FlashWrite(STM32_FLASH_MOTOR_DATA_ADDR + (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM), 
        (uint32_t*)&gStUfoData, sizeof(ST_UFO_DATA) >> 2);
    crc16 = MODBUS_RTUCrc16((uint8_t*)gStMotorData, sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM);
    crc16 += MODBUS_RTUCrc16((uint8_t*)&gStUfoData, sizeof(ST_UFO_DATA));
    FlashWrite(STM32_FLASH_MOTOR_DATA_ADDR + (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM) + sizeof(ST_UFO_DATA), &crc16, 1);
}
/*****************************************************************************
 ��������  : ��ȡflash����ṹ��gFlashData
 �������  : ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
void FlashReadConfigurePara(void)   	
{
    int lRestoreFlag = 0;
    ST_FLASH_DATA* lFlashData;
    int32_t i = 0;

    //�����һ������ʼ��
    for(i = STM32_FLASH_BLOCK_NUM - 1; i >= 0; i--)
    {        
        FlashRead(STM32_FLASH_SAVE_ADDR + i * STM32_FLASH_BLOCK_BYTE_NUM, (uint32_t*)gFlashBuffer, STM32_FLASH_BYTE_MAX_NUM >> 2);
        lFlashData = (ST_FLASH_DATA*)gFlashBuffer;
        if((lFlashData->len >= STM32_FLASH_BYTE_MAX_NUM) || (lFlashData->len > sizeof(ST_FLASH_DATA))) //���ȳ�����
        {
            continue;
        }
        else if(gFlashBuffer[lFlashData->len] != CRC8_Table(gFlashBuffer, lFlashData->len))  //crcУ��
        {
            continue;
        }
        if(DEBUG_DATA_TYPE_97)
        {
            rt_kprintf("Read flash num: %d.\r\n", i);
        }
        break;
    }

	if((lFlashData->len >= STM32_FLASH_BYTE_MAX_NUM) || (lFlashData->len > sizeof(ST_FLASH_DATA))) //���ȳ�����
	{
        lRestoreFlag = 1;
	    rt_kprintf("Read flash len err: %d.\r\n", lFlashData->len);
	}
    else if(gFlashBuffer[lFlashData->len] != CRC8_Table(gFlashBuffer, lFlashData->len))  //crcУ��
    {
        lRestoreFlag = 1;
        rt_kprintf("Read flash crc err.\r\n");
    }
    else
    {
        if(lFlashData->len != sizeof(ST_FLASH_DATA))   //����У�飬�����չ�˳��ȣ�ǰ�汣��ԭ�����ݣ�������Ĭ������
        {
            lRestoreFlag = 2;
            memcpy((uint8_t*)&gFlashData, gFlashBuffer, lFlashData->len);
            memcpy((uint8_t*)&gFlashData + lFlashData->len, (uint8_t*)&gDefaultFlashData + lFlashData->len, sizeof(ST_FLASH_DATA) - lFlashData->len);
        }
    }

    /* ��������δ���������������������ȱʡ���ã�
       ������������ȷ�������Ѵ����� */
    if(1 == lRestoreFlag)
    {
        memcpy((uint8_t*)&gFlashData, (uint8_t*)&gDefaultFlashData, sizeof(ST_FLASH_DATA));
        FlashWriteConfigurePara();
    }
    else if(2 == lRestoreFlag)  //��չ����������չ����дflash
    {
        FlashWriteConfigurePara();
    }
    else
    {
        memcpy((uint8_t*)&gFlashData, gFlashBuffer, sizeof(ST_FLASH_DATA));
    }
}
/*****************************************************************************
 ��������  : ��ȡflash����ṹ��������
 �������  : ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
void FlashReadMotorPara(void)
{
    uint32_t crc1, crc2;
    
    FlashRead(STM32_FLASH_MOTOR_DATA_ADDR, (uint32_t*)gStMotorData, (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM)  >> 2);
    FlashRead(STM32_FLASH_MOTOR_DATA_ADDR + (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM), 
        (uint32_t*)&gStUfoData, sizeof(ST_UFO_DATA) >> 2);

    crc1 = FlashReadWord(STM32_FLASH_MOTOR_DATA_ADDR + (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM) + sizeof(ST_UFO_DATA));
    crc2 = MODBUS_RTUCrc16((uint8_t*)gStMotorData, sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM);
    crc2 += MODBUS_RTUCrc16((uint8_t*)&gStUfoData, sizeof(ST_UFO_DATA));
    if(crc1 != crc2)//crcУ��
    {
        memset((uint8_t*)gStMotorData, 0x00, (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM));
        memset((uint8_t*)&gStUfoData, 0x00, sizeof(ST_UFO_DATA));
        gStUfoData.flag |= UFO_ENABLE_FLAG_ONE; //ǿ��ʹ�ܱ�־λ1����ع���
        gStUfoData.ipLastAdr = 233;
        FlashWriteMotorPara();
    }
    else if(!(gStUfoData.flag & UFO_ENABLE_FLAG_ONE))
    {
        gStUfoData.flag |= UFO_ENABLE_FLAG_ONE; //ǿ��ʹ�ܱ�־λ1����ع���
        gStUfoData.flagOne = 0; //��־λ1Ĭ������
        memset((uint8_t*)gStUfoData.reserved, 0x00, sizeof(gStUfoData.reserved));
        if(0 == gFlashData.enableBrakeFlag) //�ɰ汾gFlashData�ṹ��ɲ����������
        {
            gStMotorData[M_BRAKE].driverType = DRIVER_TYPE_NONE;
        }
        gStUfoData.ipLastAdr = gFlashData.ipLastAdr;    //�ɰ汾gFlashData�ṹ��ip��ַ����
        gStMotorData[M_TURN].initPos = gFlashData.turnInitPos;  //�ɰ汾gFlashData�ṹ��ת���������
        gStMotorData[M_TURN].pos_limit1 = gFlashData.turnRange; //�ɰ汾gFlashData�ṹ��ת���������
        FlashWriteMotorPara();
    }
    //ratioVel�ٱ��Զ�����10��
    if(gStUfoData.ratioVel < 800)
    {
        if(gStUfoData.ratioVel <= 0)
        {
            gStUfoData.ratioVel = NAV_RATIO_SCALE;
        }
        else
        {
            gStUfoData.ratioVel *= 10;
        }
        FlashWriteMotorPara();
    }
    gAutoRatioVel = gStUfoData.ratioVel;
}
/*****************************************************************************
 ��������  : ������Ӧ������
 �������  : uint8_t msgCmd  ��Ӧ��������
             uint8_t motorNum ������
             uint8_t* dataBuf ���ݻ���
             uint8_t dataLen ���ݳ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��4��8��
*****************************************************************************/
void WifiNormalRespond(uint8_t msgCmd, uint8_t motorNum, uint8_t* dataBuf, uint8_t dataLen)
{
    uint8_t lRespondMsg[200];

    lRespondMsg[0] = '*';       //����ͷ
    lRespondMsg[1] = dataLen + 7;//�����ݳ���
    lRespondMsg[2] = 0xce;       //��������
    lRespondMsg[3] = msgCmd;    //��Ӧ���յ�������
    lRespondMsg[4] = motorNum;  //������
    memcpy(&lRespondMsg[5], dataBuf, dataLen);
    lRespondMsg[dataLen + 5] = CRC8_Table(&lRespondMsg[2], dataLen + 3);//CRCУ��
    lRespondMsg[dataLen + 6] = '#';
    
    rt_ksendData(lRespondMsg, dataLen + 7);
}
/*****************************************************************************
 ��������  : ����flash
 �������  : uint8_t* cmdData  ��������
             uint8_t size      �����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��18��
*****************************************************************************/
void UsartWriteFlashConfigureParas(uint8_t* cmdData, uint8_t size)
{
    uint8_t cmd = cmdData[0];
    int32_t dataValue = *((int32_t*)(cmdData + 1));
    uint8_t lWriteFlag = 0;

    switch(cmd)
    {
        case  0x00:  //����ƽ�����к�
            if(size > 32)
            {
                *(cmdData + 32) = 0;
                memcpy(gFlashData.deviceSN, cmdData + 1, 32);
                rt_kprintf("Set Ufo SN: %s.\r\n", (char*)gFlashData.deviceSN);
                lWriteFlag = 1;
            }
            break;
        case  0x01:  //�������/�˳�����
            if(cmdData[1])
            {
                rt_kprintf("Start test mode.\r\n");
                if(0 == gFlashData.testMode)
                {
                    gFlashData.testMode = 1;
                    lWriteFlag = 1;
                }
            }
            else
            {
                rt_kprintf("Exit test mode.\r\n");
                if(1 == gFlashData.testMode)
                {
                    gFlashData.testMode = 0;
                    lWriteFlag = 1;
                }
            }
            break;
        case 0x02:
            if((cmdData[1] == 0) && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))
            {
                gStMotorData[M_BRAKE].driverType = DRIVER_TYPE_NONE;
                FlashWriteMotorPara();
            }
            rt_kprintf("BrakeFlag(0-disable): %d.\r\n", cmdData[1]);
            break;
        case 0x03:
            if(dataValue != gStMotorData[M_TURN].initPos)
            {
                gStMotorData[M_TURN].initPos = dataValue;
                if(!(gStMotorData[M_TURN].flag & ENABLE_AUTO_HOMMING))
                {
                    gStMotorRunState[M_TURN].PosInit = dataValue;
                    gStMotorRunState[M_TURN].limitPos1 = gStMotorData[M_TURN].initPos - gStMotorData[M_TURN].pos_limit1;
                    gStMotorRunState[M_TURN].limitPos2 = gStMotorData[M_TURN].initPos + gStMotorData[M_TURN].pos_limit1;
                }
                FlashWriteMotorPara();
            }
            rt_kprintf("Turnpos: %d.\r\n", gStMotorData[M_TURN].initPos);
            break;
        case 0x04:
            if(cmdData[1] != gStUfoData.ipLastAdr)
            {
                gStUfoData.ipLastAdr = cmdData[1];
                FlashWriteMotorPara();
            }
            rt_kprintf("Set IP: 192.168.1.%d.\r\n", gStUfoData.ipLastAdr);
            break;
        case 0x05:
            if(dataValue <= 0)
            {
                dataValue = 10;
            }
            if(dataValue != gStMotorData[M_TURN].pos_limit1)
            {
                gStMotorData[M_TURN].pos_limit1 = dataValue;
                if(!(gStMotorData[M_TURN].flag & ENABLE_AUTO_HOMMING))
                {
                    gStMotorRunState[M_TURN].limitPos1 = gStMotorData[M_TURN].initPos - gStMotorData[M_TURN].pos_limit1;
                    gStMotorRunState[M_TURN].limitPos2 = gStMotorData[M_TURN].initPos + gStMotorData[M_TURN].pos_limit1;
                }
                FlashWriteMotorPara();
            }
            rt_kprintf("Turnrange: %d.\r\n", gStMotorData[M_TURN].pos_limit1);
            break;
        case  0x06:  //���Ա�־
            rt_kprintf("Debug-flag: %d.\r\n", cmdData[1]);
            if(cmdData[1] != gFlashData.enableDebugDataSend)
            {
                gFlashData.enableDebugDataSend = cmdData[1];
                lWriteFlag = 1;
            }
            break;
        case  0x07:  //�������
            if(0 == cmdData[1])
            {
                rt_kprintf("Read motor%d data.\r\n", cmdData[2]);
                if(cmdData[2] < M_TOTAL_NUM)
                {
                    FlashReadMotorPara();
                    WifiNormalRespond(07, cmdData[2], (uint8_t*)(&gStMotorData[cmdData[2]]), sizeof(ST_MOTOR_DATA));
                }
                else if(cmdData[2] == M_TOTAL_NUM)
                {
                    FlashReadMotorPara();
                    WifiNormalRespond(07, cmdData[2], (uint8_t*)(&gStUfoData), sizeof(ST_UFO_DATA));
                }
            }
            else if(1 == cmdData[1])
            {
                rt_kprintf("Write motor%d data.\r\n", cmdData[2]);
                if(cmdData[2] < M_TOTAL_NUM)
                {
                    memcpy((uint8_t*)(&gStMotorData[cmdData[2]]), &cmdData[3], sizeof(ST_MOTOR_DATA));
                    WifiNormalRespond(07, cmdData[2], cmdData, 0);
                }
                else if(cmdData[2] == M_TOTAL_NUM)
                {
                    memcpy((uint8_t*)(&gStUfoData), &cmdData[3], sizeof(ST_UFO_DATA));
                    gStUfoData.flag |= UFO_ENABLE_FLAG_ONE; //ǿ��ʹ�ܱ�־λ1����ع���
                    WifiNormalRespond(07, cmdData[2], cmdData, 0);
                }
            }
            else
            {
                rt_kprintf("Save motor data.\r\n");
                FlashWriteMotorPara();
                WifiNormalRespond(07, cmdData[2], cmdData, 0);
                //ң��ֹͣ���ٶ��趨��ֻ��Чһ��
                if((gStMotorData[M_LEFT_ONE].profileAcc > 0) && (gStMotorData[M_LEFT_ONE].profileAcc < 1000))
                {
                    gRemoteStopDcc = gStMotorData[M_LEFT_ONE].profileAcc;
                    rt_kprintf("Remote dcc:%d.\r\n", gRemoteStopDcc);
                }
            }
            break;
        case  0x08:  //��ȡ�������ʷ����
            if(cmdData[1])  //�����ʷ����
            {
                rt_kprintf("Clear all history error.\r\n");
                for(dataValue = 0; dataValue < HISTORY_ERROR_MAX_NUM; dataValue++)
                {
                    if((0 != gFlashData.historyErrorCode[dataValue])
                        || (0 != gFlashData.historyErrorNum[dataValue]))
                    {
                        gFlashData.historyErrorNum[dataValue] = 0;
                        gFlashData.historyErrorTime[dataValue] = 0;
                        gFlashData.historyErrorCode[dataValue] = 0;
                        gFlashData.historyErrorResetCnt[dataValue] = 0;
                        lWriteFlag = 1;
                    }
                }
            }
            else
            {
                PrintfAllHistoryError();
            }
            break;
        /*case 0x10:  //����Ŀ���
            if(cmdData[1] < NAV_TARGET_POINT_MAX_NUM)
            {
                lWriteFlag = 1;
                gFlashData.targetValid[cmdData[1]] = 1;
                lu32Temp = *(uint32_t*)(&cmdData[2]);
                if(0 == lu32Temp)
                {
                    gFlashData.targetPos[cmdData[1]].Lon = gStNavData.pos.Lon;
                }
                else
                {
                    gFlashData.targetPos[cmdData[1]].Lon = lu32Temp;
                    gFlashData.targetPos[cmdData[1]].Lon /= 10000000;
                }
                lu32Temp = *(uint32_t*)(&cmdData[6]);
                if(0 == lu32Temp)
                {
                    gFlashData.targetPos[cmdData[1]].Lat = gStNavData.pos.Lat;
                }
                else
                {
                    gFlashData.targetPos[cmdData[1]].Lat = lu32Temp;
                    gFlashData.targetPos[cmdData[1]].Lat /= 10000000;
                }
                rt_kprintf("Set Target%d: lon:%.7lf, lat:%.7lf!\r\n", cmdData[1], gFlashData.targetPos[cmdData[1]].Lon, gFlashData.targetPos[cmdData[1]].Lat);
            }
            else
            {
                rt_kprintf("Wront target index!\r\n");
            }
            break;
        case 0x11:  //���Ŀ�����Ϣ
            rt_kprintf("Clear target info!\r\n");
            for(i = 0; i < NAV_TARGET_POINT_MAX_NUM; i++)
            {
                if(gFlashData.targetValid[i])
                {
                    gFlashData.targetValid[i] = 0;   //���Ŀ�����Ϣ
                    lWriteFlag = 1;
                }
            }
            break;
        case 0x12:  //����Ŀ����ٶ�
            if(cmdData[1] < NAV_TARGET_POINT_MAX_NUM)
            {
                lWriteFlag = 1;                
                gFlashData.targetVel[cmdData[1]] = *(uint32_t*)(&cmdData[2]);
                gFlashData.targetVel[cmdData[1]] /= 1000;
                rt_kprintf("Set Target%d: vel:%.3f!\r\n", cmdData[1], gFlashData.targetVel[cmdData[1]]);
            }
            else
            {
                rt_kprintf("Wront target index!\r\n");
            }
            break;*/
        default:
            break;
    }

    if(lWriteFlag)
    {
        FlashWriteConfigurePara();
        FlashReadConfigurePara();//д����ȡһ��
    }
}
/*****************************************************************************
 ��������  : ��ָ����ַ��ʼ����������Ѳ��������
 �������  : uint32_t WriteAddr   ��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
             uint32_t NumToWrite  ��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��3��11��
*****************************************************************************/
HAL_StatusTypeDef FlashErase(uint32_t WriteAddr, uint32_t NumToWrite)
{ 
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t endaddr = 0, addrx = 0;
    uint32_t SectorError = 0;

    if(WriteAddr < FLASH_BOOTLOADER_ADDR || WriteAddr % 4)	//�Ƿ���ַ
    {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock();									//���� 

    addrx=WriteAddr;				        //д�����ʼ��ַ
    endaddr = WriteAddr + NumToWrite * 4;	//д��Ľ�����ַ

    if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(FlashReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
                //MX_IWDG_Init();
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     //�������ͣ��������� 
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
				FlashEraseInit.Sector = FlashGetFlashSector(addrx);
                FlashEraseInit.NbSectors = 1;                               //һ��ֻ����һ������
                status = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
                SCB_CleanInvalidateDCache();                                //�����Ч��D-Cache
                FLASH_WaitForLastOperation(FLASH_WAITETIME);                //�ȴ��ϴβ������
                //IWDG_Feed();
                //MX_IWDG_Init1S();
				if(status != HAL_OK)break;	//����������
			}else addrx+=4;
		} 
	}

    HAL_FLASH_Lock();//����

    return status;
}
/*****************************************************************************
 ��������  : flash���ݶθ���
 �������  : uint32_t WriteDesAddr  д��Ŀ���ַ
             uint32_t ReadSrcAddr   ����Դ��ַ
             uint32_t NumToWrite    д�����ݳ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��4��13��
*****************************************************************************/
HAL_StatusTypeDef FlashCopy(uint32_t WriteDesAddr, uint32_t ReadSrcAddr,uint32_t NumToWrite)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t WriteAddr, ReadAddr, endaddr = 0, tempValue;

    endaddr = WriteDesAddr + (NumToWrite << 2);

    //�Ƿ���ַ
    if((WriteDesAddr < ADDR_FLASH_SECTOR_0) || (WriteDesAddr % 4)
        || (endaddr >= ADDR_FLASH_END)
        || (ReadSrcAddr + (NumToWrite << 2) > ADDR_FLASH_END))
    {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock(); //���� 
    WriteAddr = WriteDesAddr;
    ReadAddr = ReadSrcAddr;
    while(WriteAddr < endaddr)//д����
    {
        tempValue = FlashReadWord(ReadAddr);
        if(FlashReadWord(WriteAddr) != tempValue)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, tempValue);
        	if(status != HAL_OK)//д������
        	{ 
        		break;
        	}
        }
    	WriteAddr += 4;
    	ReadAddr += 4;
    }

    HAL_FLASH_Lock();   //����

    //��֤flash����
    if(status == HAL_OK)//д������
    {
        WriteAddr = WriteDesAddr;
        ReadAddr = ReadSrcAddr;
        while(WriteAddr < endaddr)//д����
        {
            if(FlashReadWord(WriteAddr) != FlashReadWord(ReadAddr))
            {
                return HAL_ERROR;
            }
        	WriteAddr += 4;
        	ReadAddr += 4;
        }
    }

    return status;
}
/*****************************************************************************
 ��������  : ��ӡ������ʷ����
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��6��30��
*****************************************************************************/
void PrintfAllHistoryError(void)
{
    int i;
    uint8_t errorNum;

    rt_kprintf("Cur reset cnt: %d.\r\n", gFlashData.resetCnt);

    for(i = 0; i < HISTORY_ERROR_MAX_NUM; i++)
    {
        errorNum = gFlashData.historyErrorNum[i] & 0x7f;
        if(0 != gFlashData.historyErrorCode[i])
        {
            if(IS_WARNING_CODE(errorNum))
            {
                rt_kprintf("History%d: Num: %d, Warn: 0x%04x, Rst:%d, Time:%dh %dm %ds %dms.",
                    i + 1, errorNum - WARNING_MOTOR_NUM_OFFSET, gFlashData.historyErrorCode[i], gFlashData.historyErrorResetCnt[i],
                    gFlashData.historyErrorTime[i] / 3600000, gFlashData.historyErrorTime[i] % 3600000 / 60000, gFlashData.historyErrorTime[i] % 60000 / 1000, gFlashData.historyErrorTime[i] % 1000);
            }
            else
            {
                rt_kprintf("History%d: Num: %d, Err: 0x%04x, Rst:%d, Time:%dh %dm %ds %dms.",
                    i + 1, errorNum, gFlashData.historyErrorCode[i], gFlashData.historyErrorResetCnt[i],
                    gFlashData.historyErrorTime[i] / 3600000, gFlashData.historyErrorTime[i] % 3600000 / 60000, gFlashData.historyErrorTime[i] % 60000 / 1000, gFlashData.historyErrorTime[i] % 1000);
            }
        }
        else
        {
            rt_kprintf("History%d: Num: %d, No err, Rst:%d, Time:%dh %dm %ds %dms.",
                i + 1, errorNum, gFlashData.historyErrorResetCnt[i],
                gFlashData.historyErrorTime[i] / 3600000, gFlashData.historyErrorTime[i] % 3600000 / 60000, gFlashData.historyErrorTime[i] % 60000 / 1000, gFlashData.historyErrorTime[i] % 1000);
        }
        if(gFlashData.historyErrorNum[i] & 0x80)
        {
            rt_kprintf("(static)\r\n");
        }
        else
        {
            rt_kprintf("\r\n");
        }
    }

    PrintErrorInfo();
}
/*****************************************************************************
 ��������  : ��ӡufo�汾�ź����к�
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��6��30��
*****************************************************************************/
void PrintfUfoVerAndSn(void)
{
    gFlashData.deviceSN[31] = 0;    //��֤�ַ�������С��32
    rt_kprintf("Ufo version: %s.\r\nUfo SN: %s.\r\n", CODE_VERSION, 
        (0 == gFlashData.deviceSN[0]) ? "Not set" : (char*)gFlashData.deviceSN);
}
/*****************************************************************************
 ��������  : ��ӡӲ���汾
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��14��
*****************************************************************************/
void PrintfHardwareType(rt_bool_t resetFlag)
{
    PrintfUfoVerAndSn();
    if(RT_FALSE == resetFlag)
    {
        rt_kprintf("debug:%d, brake:%d, turnZero:%d, turnRange:%d, ip:192.168.1.%d.\r\n", 
            gFlashData.enableDebugDataSend, gStMotorData[M_BRAKE].driverType, gStMotorData[M_TURN].initPos, gStMotorData[M_TURN].pos_limit1, gStUfoData.ipLastAdr);
    }
}
/*****************************************************************************
 ��������  : ������Ӧ�����ͣ��쳣ʱ�����Ϸ�����λ���ᳬʱ
 �������  : uint8_t msgCmd  ��Ӧ��������
             uint16_t packageNum �����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��4��8��
*****************************************************************************/
void IAPNormalRespond(uint8_t msgCmd, uint16_t packageNum)
{
    uint8_t lRespondMsg[8];

    lRespondMsg[0] = '*';       //����ͷ
    lRespondMsg[1] = 8;         //�����ݳ���
    lRespondMsg[2] = gU8DownloadType;   //���������������
    lRespondMsg[3] = msgCmd;    //��Ӧ���յ�������
    lRespondMsg[4] = packageNum & 0xff;
    lRespondMsg[5] = (packageNum >> 8) & 0xff;
    lRespondMsg[6] = CRC8_Table(&lRespondMsg[2], 4);//CRCУ��
    lRespondMsg[7] = '#';       //����β

    rt_ksendData(lRespondMsg, 8);
}
/*****************************************************************************
 ��������  : ���������������(Ӧ�ó�����������:04��01��02��03��00)
                             (����������������:04��01��02��05��03��00)
 �������  : uint8_t cmd       
             uint8_t* cmdData  
             uint8_t size      
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��4��8��
*****************************************************************************/
void IAPCmdMSgDeal(uint8_t cmd, uint8_t* cmdData, uint16_t size)
{
    static uint16_t gLastPackageNum;
    uint16_t packageNum;
    uint32_t* appDataPtr = (uint32_t*)cmdData;
    uint32_t byteToWrite = 0, startAdr;
    int i;
    
    switch(cmd)
    {
        case 0x00:
            wdgDisableFlag = 1; //��ֹ���Ź�ι��
            break;
        case 0x01://��ʼ���س��򣬻�����ݳ���            
            gAppLength = *(uint32_t*)cmdData;
            rt_kprintf("Program: %d kb.\r\n", gAppLength >> 10);
            gAppIndex = 4;//�ӵ��ĸ��ֽڿ�ʼд
            gLastPackageNum = 0xffff;
            if(IAP_CONFIRM == gIapState)
            {                
                if(HAL_OK != FlashErase(gAppStartAdr, (gAppLength + 3) >> 2))//������д���������
                {
                    break;
                }
                gIapState = IAP_STATE_BEGINE_UPDATE;
                IAPNormalRespond(cmd, 0);
            }            
            break;
        case 0x02://�������س��򣬻����������
            //����ʱ�������ݴ�С�쳣ʱ��ֱ�ӷ���
            if((IAP_STATE_IDLE == gIapState) || (IAP_STATE_FAILED == gIapState) || (size > IAP_MAX_SEND_REV_LEN + 2) || (size <= 2) || (gAppIndex > gAppLength))
            {
                break;
            }
            //��ȡ��ǰ�����
            packageNum = *(uint16_t*)cmdData;
            //��Ӧ��������
            IAPNormalRespond(cmd, packageNum);
            //�����ǰ���Ѿ�д����������������д������ѳɹ������꣬������д
            if((gLastPackageNum == packageNum) || (IAP_STATE_SUCCESS == gIapState))
            {
                break;
            }
            gLastPackageNum = packageNum;
            //��ȡд���ݼ����ݴ�С
            if(0 == packageNum)//��һ������
            {
                if(IAP_STATE_BEGINE_UPDATE == gIapState)
                {
                    if(size > 6)
                    {
                        gIapState = IAP_STATE_UPDATE;
                        gAppHeader[0] = cmdData[2];
                        gAppHeader[1] = cmdData[3];
                        gAppHeader[2] = cmdData[4];
                        gAppHeader[3] = cmdData[5];
                        appDataPtr = (uint32_t*)(&cmdData[6]);//��һ�����ݣ�ǰ4���ֽڲ�д��������ȫ������д�����д�����ڳ�������ʱ���ж��Ƿ���³ɹ�
                        byteToWrite = size - 6;
                    }
                    else
                    {
                        gIapState = IAP_STATE_IDLE;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
            else if(IAP_STATE_UPDATE == gIapState)
            {
                appDataPtr = (uint32_t*)(&cmdData[2]);
                byteToWrite = size - 2;                
            }
            else
            {
                break;
            }
            //дflash
            if(HAL_OK != FlashWrite(gAppStartAdr + gAppIndex, appDataPtr, (byteToWrite + 3) / 4))
            {
                gAppLength = 0;
                gIapState = IAP_STATE_IDLE;//дflashʧ�ܣ�תΪ����
                break;
            }
            //��֤flash����
            startAdr = gAppStartAdr + gAppIndex;
            for(i = ((byteToWrite + 3) >> 2) - 1; i >= 0; i--)
            {
                if(FlashReadWord(startAdr + (i << 2)) != appDataPtr[i])
                {
                    gAppLength = 0;
                    gIapState = IAP_STATE_IDLE;//д���ݴ�תΪ����
                    break;
                }
            }
            //д��ַ����
            gAppIndex += byteToWrite;
            //ȫ��д������д����ͷ
            if(gAppIndex == gAppLength)
            {
                gIapState = IAP_STATE_SUCCESS;
                appDataPtr = (uint32_t*)gAppHeader;
                if(HAL_OK != FlashWrite(gAppStartAdr, appDataPtr, 1))
                {                    
                    gAppLength = 0;
                    gIapState = IAP_STATE_IDLE;
                }
                else if(DOWNLOAD_BOOTLOADER == gU8DownloadType)
                {
                    gIapState = IAP_STATE_UPDATE_BOOTLOADER;    //����bootloader
                }
            }
            break;
        case 0x03://ȷ����������ɳ������
            if(IAP_STATE_SUCCESS == gIapState)
            {
                MX_IWDG_Init1S();
                wdgDisableFlag = 1; //��ֹ���Ź�ι��
                IAPNormalRespond(0x03, 0);
            }
            break;
        case 0x04://�ж������س������ʼ��ַ
            gAppStartAdr = FLASH_APP2_ADDR;
            if(DOWNLOAD_BOOTLOADER == gU8DownloadType)
            {
                gAppStartAdr += 0x04;   //bootloaderֻ�追���������ڿ�ʼλ�ã���ַƫ��4
            }
            IAPNormalRespond(0x04, 0);
            gIapState = IAP_CONFIRM;
            MX_IWDG_Init(); //���Ź����ó�16s
            break;
        case 0x05://���Ǿɵ�bootloader
            if(IAP_STATE_UPDATE_BOOTLOADER == gIapState)
            {
                rt_kprintf("Begin copy bootloader: %d.\r\n", HAL_GetTick());
                if(HAL_OK != FlashErase(FLASH_BOOTLOADER_ADDR, (gAppLength + 3) >> 2))//������д���������
                {
                    break;
                }
                if(HAL_OK != FlashCopy(FLASH_BOOTLOADER_ADDR, gAppStartAdr, (gAppLength + 3) >> 2))
                {
                    break;
                }
                rt_kprintf("End copy bootloader %d.\r\n", HAL_GetTick());
                IAPNormalRespond(0x05, 0);
                gIapState = IAP_STATE_SUCCESS;
            }
            break;
    }
}
/*****************************************************************************
 ��������  : ������ʷ������Ϣ
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��6��30��
*****************************************************************************/
void UpdateHistoryError(uint16_t motorNum, uint16_t lResult)
{
    int i;

    //��ʷ��������������
    for(i = HISTORY_ERROR_MAX_NUM - 1; i > 0; i--)
    {
        gFlashData.historyErrorNum[i] = gFlashData.historyErrorNum[i - 1];
        gFlashData.historyErrorTime[i] = gFlashData.historyErrorTime[i - 1];
        gFlashData.historyErrorCode[i] = gFlashData.historyErrorCode[i - 1];
        gFlashData.historyErrorResetCnt[i] = gFlashData.historyErrorResetCnt[i - 1];
    }
    //�������¹���
    gFlashData.historyErrorNum[0] = motorNum;
    if(!gErrorRunStateFlag)
    {
        gFlashData.historyErrorNum[0] |= 0x80;  //��ֹ״̬��־
    }
    gFlashData.historyErrorTime[0] = HAL_GetTick();
    gFlashData.historyErrorCode[0] = lResult;
    gFlashData.historyErrorResetCnt[0] = gFlashData.resetCnt;
    //��д��flash
    gU8FlashWriteFlag = 1;
}
/*****************************************************************************
 ��������  : �Զ���flash�޸ĵ����ݱ���
 �������  : TickType_t curTime  ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��6��30��
*****************************************************************************/
void WriteFlashConfigureParasProcess(TickType_t curTime)
{
    static uint32_t gu32FlashLastWriteTime = 0;    //flash�ϴ�д��ʱ��

    if(gU8FlashWriteFlag && (curTime - gu32FlashLastWriteTime >= 10000))    //10s���������дһ��flash
    {
        //ƽ��ͣ�����ſ���дflash
        if((M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag))
        {
            //���һ������Ϊ�հף����п�д����ֱ��д
            if((2 == gU8FlashWriteFlag) ||
                (HAL_OK == FlashBlockIsBland(STM32_FLASH_SAVE_ADDR + (STM32_FLASH_BLOCK_NUM - 1) * STM32_FLASH_BLOCK_BYTE_NUM, STM32_FLASH_BYTE_MAX_NUM)))
            {
                gU8FlashWriteFlag = 0;
                gu32FlashLastWriteTime = curTime;
                FlashWriteConfigurePara();
                rt_kprintf("Write history error to flash, rst:%d, tick:%d!\r\n", gFlashData.resetCnt, curTime);
            }
            else    //�����3s�ٴӵ�һ���������д���Է�ֹ�رյ�Դʱ�����ϲ�дflashʧ��
            {
                gU8FlashWriteFlag = 2;
                gu32FlashLastWriteTime = curTime - 7000;
                if(DEBUG_DATA_TYPE_97)
                {
                    rt_kprintf("Wait erase flash!\r\n");
                }
            }
        }
    }
}

