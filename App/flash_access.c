#include "motor_control.h"
#include "usart.h"
#include "iwdg.h"
#include "CRC8.h"

#define FLASH_WAITETIME  50000          //FLASH等待超时时间
static uint8_t gFlashBuffer[STM32_FLASH_BYTE_MAX_NUM];//读写flash缓冲区
static const ST_FLASH_DATA gDefaultFlashData = STM32_FLASH_DEFAULT_CONFIGURE_PARAS;//默认flash数据
ST_FLASH_DATA gFlashData = STM32_FLASH_DEFAULT_CONFIGURE_PARAS;   //flash当前数据
ST_MOTOR_DATA gStMotorData[M_TOTAL_NUM];
ST_UFO_DATA  gStUfoData = {0xcc};

uint8_t gU8FlashWriteFlag = 0;  //flash写标志

uint8_t gU8DownloadType = 0;    //程序下载类型
int gIapState = IAP_STATE_IDLE;  //程序下载各种状态
uint32_t gAppLength;             //需下载程序的长度
uint32_t gAppIndex;              //当前下载数据序号
uint8_t gAppHeader[4];           //app数据头
uint32_t gAppStartAdr = FLASH_APP1_ADDR;    //程序起始地址

/*****************************************************************************
 功能描述  : 读取指定地址的半字(16位数据) 
 输入参数  : uint32_t faddr  读地址 
 输出参数  : uint32_t  对应数据
 作    者  : 刘鹏
 日    期  : 2019年3月11日
*****************************************************************************/
uint32_t FlashReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr;
}
/*****************************************************************************
 功能描述  : 获取某个地址所在的flash扇区
 输入参数  : rt_uint32_t addr  地址
 输出参数  : rt_uint16_t 返回值:0~11,即addr所在的扇区
 作    者  : 刘鹏
 日    期  : 2019年3月11日
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
 功能描述  : 从指定地址开始写入指定长度的数据，注意会先擦出整片扇区
 输入参数  : uint32_t WriteAddr   起始地址(此地址必须为4的倍数!!)
             uint32_t *pBuffer    数据指针
             uint32_t NumToWrite  字(32位)数(就是要写入的32位数据的个数.) 
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年3月11日
*****************************************************************************/
HAL_StatusTypeDef FlashWrite(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)	
{ 
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t endaddr = 0, addrx = 0;
    uint32_t SectorError = 0;

    if(WriteAddr < STM32_FLASH_SAVE_ADDR || WriteAddr % 4)	//非法地址
    {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock();									//解锁 

    addrx=WriteAddr;				        //写入的起始地址
    endaddr = WriteAddr + NumToWrite * 4;	//写入的结束地址

    if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(FlashReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
                //MX_IWDG_Init();
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     //擦除类型，扇区擦除 
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //电压范围，VCC=2.7~3.6V之间!!
				FlashEraseInit.Sector = FlashGetFlashSector(addrx);
                FlashEraseInit.NbSectors = 1;                               //一次只擦除一个扇区
                status = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
                SCB_CleanInvalidateDCache();                                //清除无效的D-Cache
                FLASH_WaitForLastOperation(FLASH_WAITETIME);                //等待上次操作完成
                //IWDG_Feed();
                //MX_IWDG_Init1S();
				if(status != HAL_OK)break;	//发生错误了
			}else addrx+=4;
		} 
	}

    if(status == HAL_OK)
    {
        while(WriteAddr < endaddr)//写数据
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer);
        	if(status != HAL_OK)//写入数据
        	{ 
        		break;
        	}
        	WriteAddr += 4;
        	pBuffer++;
        } 
    }

    HAL_FLASH_Lock();//上锁

    return status;
}
/*****************************************************************************
 功能描述  : 从指定地址开始读出指定长度的数据
 输入参数  : uint32_t ReadAddr   起始地址
             uint32_t *pBuffer   数据指针
             uint32_t NumToRead  字(4位)数
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年3月11日
*****************************************************************************/
void FlashRead(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)  	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i] = FlashReadWord(ReadAddr);//读取4个字节.
		ReadAddr += 4;//偏移4个字节.	
	}
}
/*****************************************************************************
 功能描述  : 判断一段flash块是否为空白
 输入参数  : uint32_t blockAddr      
             uint32_t blockByteSize  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2024年3月28日
*****************************************************************************/
HAL_StatusTypeDef FlashBlockIsBland(uint32_t blockAddr, uint32_t blockByteSize)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t endaddr = 0;
    
    if(blockAddr < STM32_FLASH_SAVE_ADDR || blockAddr % 4 || (blockAddr + blockByteSize >= ADDR_FLASH_END)) //非法地址
    {
        return HAL_ERROR;
    }

    endaddr = blockAddr + blockByteSize;	//写入的结束地址

    while(blockAddr < endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
	{
		if(FlashReadWord(blockAddr) != 0XFFFFFFFF)  //有非0XFFFFFFFF的地方,则写过数据
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
 功能描述  : 将gFlashData结构体写flash
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年3月11日
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
        //往最后一块空白区域写
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
        if(STM32_FLASH_BLOCK_NUM == i)  //如果没有空白区域，则从第一块区域擦除后开始写
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
        if(DEBUG_DATA_TYPE_97)  //实测擦除214ms，写flash需要1.67ms
        {
            rt_kprintf("Write flash time: %d (10us).\r\n", TIM5->CNT - lTime);
        }
    }
}
/*****************************************************************************
 功能描述  : 将电机数据写flash
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年10月22日
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
 功能描述  : 读取flash放入结构体gFlashData
 输入参数  : 无
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年3月11日
*****************************************************************************/
void FlashReadConfigurePara(void)   	
{
    int lRestoreFlag = 0;
    ST_FLASH_DATA* lFlashData;
    int32_t i = 0;

    //从最后一块区域开始读
    for(i = STM32_FLASH_BLOCK_NUM - 1; i >= 0; i--)
    {        
        FlashRead(STM32_FLASH_SAVE_ADDR + i * STM32_FLASH_BLOCK_BYTE_NUM, (uint32_t*)gFlashBuffer, STM32_FLASH_BYTE_MAX_NUM >> 2);
        lFlashData = (ST_FLASH_DATA*)gFlashBuffer;
        if((lFlashData->len >= STM32_FLASH_BYTE_MAX_NUM) || (lFlashData->len > sizeof(ST_FLASH_DATA))) //长度超限制
        {
            continue;
        }
        else if(gFlashBuffer[lFlashData->len] != CRC8_Table(gFlashBuffer, lFlashData->len))  //crc校验
        {
            continue;
        }
        if(DEBUG_DATA_TYPE_97)
        {
            rt_kprintf("Read flash num: %d.\r\n", i);
        }
        break;
    }

	if((lFlashData->len >= STM32_FLASH_BYTE_MAX_NUM) || (lFlashData->len > sizeof(ST_FLASH_DATA))) //长度超限制
	{
        lRestoreFlag = 1;
	    rt_kprintf("Read flash len err: %d.\r\n", lFlashData->len);
	}
    else if(gFlashBuffer[lFlashData->len] != CRC8_Table(gFlashBuffer, lFlashData->len))  //crc校验
    {
        lRestoreFlag = 1;
        rt_kprintf("Read flash crc err.\r\n");
    }
    else
    {
        if(lFlashData->len != sizeof(ST_FLASH_DATA))   //长度校验，如果扩展了长度，前面保留原有数据，后面变成默认数据
        {
            lRestoreFlag = 2;
            memcpy((uint8_t*)&gFlashData, gFlashBuffer, lFlashData->len);
            memcpy((uint8_t*)&gFlashData + lFlashData->len, (uint8_t*)&gDefaultFlashData + lFlashData->len, sizeof(ST_FLASH_DATA) - lFlashData->len);
        }
    }

    /* 配置数据未存入或配置数据有误，则用缺省配置；
       若配置数据正确，则用已存配置 */
    if(1 == lRestoreFlag)
    {
        memcpy((uint8_t*)&gFlashData, (uint8_t*)&gDefaultFlashData, sizeof(ST_FLASH_DATA));
        FlashWriteConfigurePara();
    }
    else if(2 == lRestoreFlag)  //扩展了数据则将扩展数据写flash
    {
        FlashWriteConfigurePara();
    }
    else
    {
        memcpy((uint8_t*)&gFlashData, gFlashBuffer, sizeof(ST_FLASH_DATA));
    }
}
/*****************************************************************************
 功能描述  : 读取flash放入结构体电机参数
 输入参数  : 无
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年3月11日
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
    if(crc1 != crc2)//crc校验
    {
        memset((uint8_t*)gStMotorData, 0x00, (sizeof(ST_MOTOR_DATA) * M_TOTAL_NUM));
        memset((uint8_t*)&gStUfoData, 0x00, sizeof(ST_UFO_DATA));
        gStUfoData.flag |= UFO_ENABLE_FLAG_ONE; //强行使能标志位1的相关功能
        gStUfoData.ipLastAdr = 233;
        FlashWriteMotorPara();
    }
    else if(!(gStUfoData.flag & UFO_ENABLE_FLAG_ONE))
    {
        gStUfoData.flag |= UFO_ENABLE_FLAG_ONE; //强行使能标志位1的相关功能
        gStUfoData.flagOne = 0; //标志位1默认清零
        memset((uint8_t*)gStUfoData.reserved, 0x00, sizeof(gStUfoData.reserved));
        if(0 == gFlashData.enableBrakeFlag) //旧版本gFlashData结构体刹车参数兼容
        {
            gStMotorData[M_BRAKE].driverType = DRIVER_TYPE_NONE;
        }
        gStUfoData.ipLastAdr = gFlashData.ipLastAdr;    //旧版本gFlashData结构体ip地址兼容
        gStMotorData[M_TURN].initPos = gFlashData.turnInitPos;  //旧版本gFlashData结构体转向参数兼容
        gStMotorData[M_TURN].pos_limit1 = gFlashData.turnRange; //旧版本gFlashData结构体转向参数兼容
        FlashWriteMotorPara();
    }
    //ratioVel速比自动扩大10倍
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
 功能描述  : 正常响应包发送
 输入参数  : uint8_t msgCmd  响应何条命令
             uint8_t motorNum 电机序号
             uint8_t* dataBuf 数据缓存
             uint8_t dataLen 数据长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年4月8日
*****************************************************************************/
void WifiNormalRespond(uint8_t msgCmd, uint8_t motorNum, uint8_t* dataBuf, uint8_t dataLen)
{
    uint8_t lRespondMsg[200];

    lRespondMsg[0] = '*';       //数据头
    lRespondMsg[1] = dataLen + 7;//总数据长度
    lRespondMsg[2] = 0xce;       //命令类型
    lRespondMsg[3] = msgCmd;    //对应接收到的命令
    lRespondMsg[4] = motorNum;  //电机序号
    memcpy(&lRespondMsg[5], dataBuf, dataLen);
    lRespondMsg[dataLen + 5] = CRC8_Table(&lRespondMsg[2], dataLen + 3);//CRC校验
    lRespondMsg[dataLen + 6] = '#';
    
    rt_ksendData(lRespondMsg, dataLen + 7);
}
/*****************************************************************************
 功能描述  : 访问flash
 输入参数  : uint8_t* cmdData  命令数据
             uint8_t size      命令长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月18日
*****************************************************************************/
void UsartWriteFlashConfigureParas(uint8_t* cmdData, uint8_t size)
{
    uint8_t cmd = cmdData[0];
    int32_t dataValue = *((int32_t*)(cmdData + 1));
    uint8_t lWriteFlag = 0;

    switch(cmd)
    {
        case  0x00:  //设置平板序列号
            if(size > 32)
            {
                *(cmdData + 32) = 0;
                memcpy(gFlashData.deviceSN, cmdData + 1, 32);
                rt_kprintf("Set Ufo SN: %s.\r\n", (char*)gFlashData.deviceSN);
                lWriteFlag = 1;
            }
            break;
        case  0x01:  //进入测试/退出测试
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
        case  0x06:  //调试标志
            rt_kprintf("Debug-flag: %d.\r\n", cmdData[1]);
            if(cmdData[1] != gFlashData.enableDebugDataSend)
            {
                gFlashData.enableDebugDataSend = cmdData[1];
                lWriteFlag = 1;
            }
            break;
        case  0x07:  //电机数据
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
                    gStUfoData.flag |= UFO_ENABLE_FLAG_ONE; //强行使能标志位1的相关功能
                    WifiNormalRespond(07, cmdData[2], cmdData, 0);
                }
            }
            else
            {
                rt_kprintf("Save motor data.\r\n");
                FlashWriteMotorPara();
                WifiNormalRespond(07, cmdData[2], cmdData, 0);
                //遥控停止减速度设定，只生效一次
                if((gStMotorData[M_LEFT_ONE].profileAcc > 0) && (gStMotorData[M_LEFT_ONE].profileAcc < 1000))
                {
                    gRemoteStopDcc = gStMotorData[M_LEFT_ONE].profileAcc;
                    rt_kprintf("Remote dcc:%d.\r\n", gRemoteStopDcc);
                }
            }
            break;
        case  0x08:  //读取或清除历史故障
            if(cmdData[1])  //清除历史故障
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
        /*case 0x10:  //设置目标点
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
        case 0x11:  //清除目标点信息
            rt_kprintf("Clear target info!\r\n");
            for(i = 0; i < NAV_TARGET_POINT_MAX_NUM; i++)
            {
                if(gFlashData.targetValid[i])
                {
                    gFlashData.targetValid[i] = 0;   //清除目标点信息
                    lWriteFlag = 1;
                }
            }
            break;
        case 0x12:  //设置目标点速度
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
        FlashReadConfigurePara();//写完后读取一遍
    }
}
/*****************************************************************************
 功能描述  : 从指定地址开始擦除，如果已擦除则忽略
 输入参数  : uint32_t WriteAddr   起始地址(此地址必须为4的倍数!!)
             uint32_t NumToWrite  字(32位)数(就是要写入的32位数据的个数.) 
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年3月11日
*****************************************************************************/
HAL_StatusTypeDef FlashErase(uint32_t WriteAddr, uint32_t NumToWrite)
{ 
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t endaddr = 0, addrx = 0;
    uint32_t SectorError = 0;

    if(WriteAddr < FLASH_BOOTLOADER_ADDR || WriteAddr % 4)	//非法地址
    {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock();									//解锁 

    addrx=WriteAddr;				        //写入的起始地址
    endaddr = WriteAddr + NumToWrite * 4;	//写入的结束地址

    if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(FlashReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
                //MX_IWDG_Init();
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     //擦除类型，扇区擦除 
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //电压范围，VCC=2.7~3.6V之间!!
				FlashEraseInit.Sector = FlashGetFlashSector(addrx);
                FlashEraseInit.NbSectors = 1;                               //一次只擦除一个扇区
                status = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
                SCB_CleanInvalidateDCache();                                //清除无效的D-Cache
                FLASH_WaitForLastOperation(FLASH_WAITETIME);                //等待上次操作完成
                //IWDG_Feed();
                //MX_IWDG_Init1S();
				if(status != HAL_OK)break;	//发生错误了
			}else addrx+=4;
		} 
	}

    HAL_FLASH_Lock();//上锁

    return status;
}
/*****************************************************************************
 功能描述  : flash数据段复制
 输入参数  : uint32_t WriteDesAddr  写的目标地址
             uint32_t ReadSrcAddr   读的源地址
             uint32_t NumToWrite    写的数据长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年4月13日
*****************************************************************************/
HAL_StatusTypeDef FlashCopy(uint32_t WriteDesAddr, uint32_t ReadSrcAddr,uint32_t NumToWrite)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t WriteAddr, ReadAddr, endaddr = 0, tempValue;

    endaddr = WriteDesAddr + (NumToWrite << 2);

    //非法地址
    if((WriteDesAddr < ADDR_FLASH_SECTOR_0) || (WriteDesAddr % 4)
        || (endaddr >= ADDR_FLASH_END)
        || (ReadSrcAddr + (NumToWrite << 2) > ADDR_FLASH_END))
    {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock(); //解锁 
    WriteAddr = WriteDesAddr;
    ReadAddr = ReadSrcAddr;
    while(WriteAddr < endaddr)//写数据
    {
        tempValue = FlashReadWord(ReadAddr);
        if(FlashReadWord(WriteAddr) != tempValue)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, tempValue);
        	if(status != HAL_OK)//写入数据
        	{ 
        		break;
        	}
        }
    	WriteAddr += 4;
    	ReadAddr += 4;
    }

    HAL_FLASH_Lock();   //上锁

    //验证flash数据
    if(status == HAL_OK)//写入数据
    {
        WriteAddr = WriteDesAddr;
        ReadAddr = ReadSrcAddr;
        while(WriteAddr < endaddr)//写数据
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
 功能描述  : 打印所有历史故障
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年6月30日
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
 功能描述  : 打印ufo版本号和序列号
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年6月30日
*****************************************************************************/
void PrintfUfoVerAndSn(void)
{
    gFlashData.deviceSN[31] = 0;    //保证字符串长度小于32
    rt_kprintf("Ufo version: %s.\r\nUfo SN: %s.\r\n", CODE_VERSION, 
        (0 == gFlashData.deviceSN[0]) ? "Not set" : (char*)gFlashData.deviceSN);
}
/*****************************************************************************
 功能描述  : 打印硬件版本
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月14日
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
 功能描述  : 正常响应包发送，异常时不会上发，上位机会超时
 输入参数  : uint8_t msgCmd  响应何条命令
             uint16_t packageNum 包序号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年4月8日
*****************************************************************************/
void IAPNormalRespond(uint8_t msgCmd, uint16_t packageNum)
{
    uint8_t lRespondMsg[8];

    lRespondMsg[0] = '*';       //数据头
    lRespondMsg[1] = 8;         //总数据长度
    lRespondMsg[2] = gU8DownloadType;   //程序下载相关命令
    lRespondMsg[3] = msgCmd;    //对应接收到的命令
    lRespondMsg[4] = packageNum & 0xff;
    lRespondMsg[5] = (packageNum >> 8) & 0xff;
    lRespondMsg[6] = CRC8_Table(&lRespondMsg[2], 4);//CRC校验
    lRespondMsg[7] = '#';       //数据尾

    rt_ksendData(lRespondMsg, 8);
}
/*****************************************************************************
 功能描述  : 程序下载命令解析(应用程序下载流程:04，01，02，03，00)
                             (引导程序下载流程:04，01，02，05，03，00)
 输入参数  : uint8_t cmd       
             uint8_t* cmdData  
             uint8_t size      
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年4月8日
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
            wdgDisableFlag = 1; //禁止看门狗喂狗
            break;
        case 0x01://开始下载程序，获得数据长度            
            gAppLength = *(uint32_t*)cmdData;
            rt_kprintf("Program: %d kb.\r\n", gAppLength >> 10);
            gAppIndex = 4;//从第四个字节开始写
            gLastPackageNum = 0xffff;
            if(IAP_CONFIRM == gIapState)
            {                
                if(HAL_OK != FlashErase(gAppStartAdr, (gAppLength + 3) >> 2))//擦除需写程序的区域
                {
                    break;
                }
                gIapState = IAP_STATE_BEGINE_UPDATE;
                IAPNormalRespond(cmd, 0);
            }            
            break;
        case 0x02://正在下载程序，获得所有数据
            //空闲时，或数据大小异常时，直接返回
            if((IAP_STATE_IDLE == gIapState) || (IAP_STATE_FAILED == gIapState) || (size > IAP_MAX_SEND_REV_LEN + 2) || (size <= 2) || (gAppIndex > gAppLength))
            {
                break;
            }
            //获取当前包序号
            packageNum = *(uint16_t*)cmdData;
            //响应正常接收
            IAPNormalRespond(cmd, packageNum);
            //如果当前包已经写过，则无需重新再写，如果已成功升级完，无需重写
            if((gLastPackageNum == packageNum) || (IAP_STATE_SUCCESS == gIapState))
            {
                break;
            }
            gLastPackageNum = packageNum;
            //获取写数据即数据大小
            if(0 == packageNum)//第一包数据
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
                        appDataPtr = (uint32_t*)(&cmdData[6]);//第一包数据，前4个字节不写，等其余全部数据写完后再写，用于程序启动时，判断是否更新成功
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
            //写flash
            if(HAL_OK != FlashWrite(gAppStartAdr + gAppIndex, appDataPtr, (byteToWrite + 3) / 4))
            {
                gAppLength = 0;
                gIapState = IAP_STATE_IDLE;//写flash失败，转为空闲
                break;
            }
            //验证flash数据
            startAdr = gAppStartAdr + gAppIndex;
            for(i = ((byteToWrite + 3) >> 2) - 1; i >= 0; i--)
            {
                if(FlashReadWord(startAdr + (i << 2)) != appDataPtr[i])
                {
                    gAppLength = 0;
                    gIapState = IAP_STATE_IDLE;//写数据错，转为空闲
                    break;
                }
            }
            //写地址递增
            gAppIndex += byteToWrite;
            //全部写完后，最后写数据头
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
                    gIapState = IAP_STATE_UPDATE_BOOTLOADER;    //更新bootloader
                }
            }
            break;
        case 0x03://确认已正常完成程序更新
            if(IAP_STATE_SUCCESS == gIapState)
            {
                MX_IWDG_Init1S();
                wdgDisableFlag = 1; //禁止看门狗喂狗
                IAPNormalRespond(0x03, 0);
            }
            break;
        case 0x04://判断新下载程序的起始地址
            gAppStartAdr = FLASH_APP2_ADDR;
            if(DOWNLOAD_BOOTLOADER == gU8DownloadType)
            {
                gAppStartAdr += 0x04;   //bootloader只需拷贝，不放在开始位置，地址偏移4
            }
            IAPNormalRespond(0x04, 0);
            gIapState = IAP_CONFIRM;
            MX_IWDG_Init(); //看门狗设置成16s
            break;
        case 0x05://覆盖旧的bootloader
            if(IAP_STATE_UPDATE_BOOTLOADER == gIapState)
            {
                rt_kprintf("Begin copy bootloader: %d.\r\n", HAL_GetTick());
                if(HAL_OK != FlashErase(FLASH_BOOTLOADER_ADDR, (gAppLength + 3) >> 2))//擦除需写程序的区域
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
 功能描述  : 更新历史故障信息
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年6月30日
*****************************************************************************/
void UpdateHistoryError(uint16_t motorNum, uint16_t lResult)
{
    int i;

    //历史故障依次往后移
    for(i = HISTORY_ERROR_MAX_NUM - 1; i > 0; i--)
    {
        gFlashData.historyErrorNum[i] = gFlashData.historyErrorNum[i - 1];
        gFlashData.historyErrorTime[i] = gFlashData.historyErrorTime[i - 1];
        gFlashData.historyErrorCode[i] = gFlashData.historyErrorCode[i - 1];
        gFlashData.historyErrorResetCnt[i] = gFlashData.historyErrorResetCnt[i - 1];
    }
    //存入最新故障
    gFlashData.historyErrorNum[0] = motorNum;
    if(!gErrorRunStateFlag)
    {
        gFlashData.historyErrorNum[0] |= 0x80;  //静止状态标志
    }
    gFlashData.historyErrorTime[0] = HAL_GetTick();
    gFlashData.historyErrorCode[0] = lResult;
    gFlashData.historyErrorResetCnt[0] = gFlashData.resetCnt;
    //需写入flash
    gU8FlashWriteFlag = 1;
}
/*****************************************************************************
 功能描述  : 自动将flash修改的数据保存
 输入参数  : TickType_t curTime  当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年6月30日
*****************************************************************************/
void WriteFlashConfigureParasProcess(TickType_t curTime)
{
    static uint32_t gu32FlashLastWriteTime = 0;    //flash上次写的时间

    if(gU8FlashWriteFlag && (curTime - gu32FlashLastWriteTime >= 10000))    //10s间隔才允许写一次flash
    {
        //平板停下来才可以写flash
        if((M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag))
        {
            //最后一块区域为空白，则有可写区域，直接写
            if((2 == gU8FlashWriteFlag) ||
                (HAL_OK == FlashBlockIsBland(STM32_FLASH_SAVE_ADDR + (STM32_FLASH_BLOCK_NUM - 1) * STM32_FLASH_BLOCK_BYTE_NUM, STM32_FLASH_BYTE_MAX_NUM)))
            {
                gU8FlashWriteFlag = 0;
                gu32FlashLastWriteTime = curTime;
                FlashWriteConfigurePara();
                rt_kprintf("Write history error to flash, rst:%d, tick:%d!\r\n", gFlashData.resetCnt, curTime);
            }
            else    //否则等3s再从第一个区域擦除写，以防止关闭电源时出故障擦写flash失败
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

