#include "battery.h"
#include "usart.h"
#include "can.h"
Battery_status gStBatteryState = {0};
Battery_status gStBatteryStateBackup[BATTERY_MAX_NUMS]; //���ر���
uint8_t g485BatteryRevBuf[100];

/*****************************************************************************
 ��������  : ��ʼ���������
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��11��30��
*****************************************************************************/
void InitBatteryData(void)
{
    gStBatteryState.validFlag = 0;
    gStBatteryState.driverVolValidFlag = 0;
    gStBatteryState.motorRunFlag = 0;
    gStBatteryState.motorIdleTime = 1000;
    gStBatteryState.voltage = 640;
    gStBatteryState.soc = 90;
    gStBatteryState.maxtemp = 60;
}
/*****************************************************************************
 ��������  : ��װ485���������
 �������  : uint8_t *buffer  
             uint32_t size    
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��21��
*****************************************************************************/
static HAL_StatusTypeDef Battery485DeviceRead(uint8_t *sendBuffer, uint32_t sendSize, uint32_t revSize)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint32_t tickstart, lastTick = 0, curTick;
	uint32_t l_size = 0;
    uint16_t crcValue;

    if((sendSize <= 2) || (revSize > sizeof(g485BatteryRevBuf)))
    {
        return lResult;
    }

    //����У����
    crcValue = MODBUS_RTUCrc16(sendBuffer, sendSize - 2);
    sendBuffer[sendSize - 1] = (crcValue >> 8) & 0xff;
    sendBuffer[sendSize - 2] = crcValue & 0xff;

    l_size = UsartDeviceWrite(USART5_DEVICE, sendBuffer, sendSize);

    if(l_size > 0)
	{
        while(l_size > 0)   //��ջ�����
        {
            l_size = UsartDeviceRead(USART5_DEVICE, g485BatteryRevBuf, revSize);
        }
        l_size = 0;
        /* Get tick */ 
        tickstart = HAL_GetTick();
        lastTick = tickstart;

        while(1)
        {
            curTick = HAL_GetTick();
            if(curTick != lastTick)
            {
                lastTick = curTick;
                l_size += UsartDeviceRead(USART5_DEVICE, &g485BatteryRevBuf[l_size], revSize - l_size);
                if(l_size >= revSize)
                {
                    if((g485BatteryRevBuf[0] == sendBuffer[0]) && (g485BatteryRevBuf[1] == sendBuffer[1]))  //�˶�վ��
                    {
                        lResult = Motor485CheckRevData(g485BatteryRevBuf, revSize);
                    }
                    else
                    {
                        rt_kprintf("battery Id error, sid: %d, rid: %d\r\n", sendBuffer[0], g485BatteryRevBuf[0]);
                    }
                    break;
                }
            }

            /* Check for the Timeout */
            curTick -= tickstart;
            if(curTick > BATTERY485_READ_TIMEOUT_MS)
            {
                if(DEBUG_DATA_TYPE_7)
                {
                    rt_kprintf("battery id%d Rev timeout!\r\n", sendBuffer[0]);
                }
                lResult = HAL_TIMEOUT;
                break;
            }
    	}
    }
    
	return lResult;
}
/*****************************************************************************
 ��������  : ͨ��485 modbusЭ���ȡ�����Ϣ
 �������  : uint32_t infoType  ��Ϣ����
             uint16_t* dataInfo ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��10��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFrom485Modbus(uint32_t infoType, uint16_t* dataInfo)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;

    uint8_t msg[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x00,0x00};   //�����ݸ�ʽ

    //modbus����
    if(BATTERY_INFO_TYPE_SOC == infoType)
    {
        msg[3] = 0xa2;
    }
    else if(BATTERY_INFO_TYPE_TEMP == infoType)
    {
        msg[3] = 0x82;
    }
    else
    {
        return lResult;
    }

    lResult = Battery485DeviceRead(msg, sizeof(msg), 7);

    if(HAL_OK == lResult)
    {
        *dataInfo = (g485BatteryRevBuf[3] << 8) | g485BatteryRevBuf[4];
    }

    return lResult;
}
/*****************************************************************************
 ��������  : �����ݺ�
 �������  : uint8_t* buf     ���ݵ�ַ
             uint8_t buflen   ���ݳ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��30��
*****************************************************************************/
static uint16_t GetCheckSum(uint8_t* buf, uint8_t buflen)
{
    uint16_t checkSum = 0;
    uint8_t i;

    checkSum += buf[2];
    checkSum += buf[3];
    for(i = 0; ((i < buf[3]) && (i + 4 < buflen)); i++)
    {
        checkSum += buf[i + 4];
    }

    checkSum = ~checkSum;
    checkSum += 1;

    return checkSum;
}
/*****************************************************************************
 ��������  : ��ȡ�ڶ���Сufo�����Ϣ
 �������  : Battery_status* info  ������Ϣ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��30��
*****************************************************************************/
static HAL_StatusTypeDef ReadMiniUfoV2BatteryInfoFrom485(Battery_status* info)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint16_t checkSum, i;
    int16_t i16Temp;
    uint32_t tickstart, lastTick = 0, curTick;
	uint32_t l_size = 0;

    uint8_t msg[7] = {0xdd,0xa5,0x03,0x00,0x00,0x00,0x77};   //�����ݸ�ʽ

    checkSum = GetCheckSum(msg, sizeof(msg));

    msg[4] = checkSum >> 8;
    msg[5] = checkSum & 0xff;

    l_size = UsartDeviceWrite(USART5_DEVICE, msg, sizeof(msg));
    //rt_kprintfArray(msg, sizeof(msg), 16, 1);

    if(l_size > 0)
	{
        while(l_size > 0)   //��ջ�����
        {
            l_size = UsartDeviceRead(USART5_DEVICE, g485BatteryRevBuf, sizeof(g485BatteryRevBuf));
        }
        l_size = 0;
        /* Get tick */ 
        tickstart = HAL_GetTick();
        lastTick = tickstart;

        while(1)
        {
            curTick = HAL_GetTick();
            if(curTick != lastTick)
            {
                lastTick = curTick;
                l_size += UsartDeviceRead(USART5_DEVICE, &g485BatteryRevBuf[l_size], sizeof(g485BatteryRevBuf) - l_size);
                if(l_size >= 4)
                {
                    if((g485BatteryRevBuf[0] == 0xdd) && (g485BatteryRevBuf[1] == 0x03))  //�˶�����ͷ
                    {
                        if(l_size >= g485BatteryRevBuf[3] + 7)//�˶Գ���
                        {
                            //rt_kprintfArray(g485BatteryRevBuf, l_size, 16, 1);
                            if(g485BatteryRevBuf[2] == 0)   //��ȷ��Ӧ
                            {
                                //�˶�У��
                                checkSum = GetCheckSum(g485BatteryRevBuf, l_size);
                                if(checkSum== ((uint16_t)(g485BatteryRevBuf[g485BatteryRevBuf[3] + 4] << 8) 
                                    | g485BatteryRevBuf[g485BatteryRevBuf[3] + 5]))
                                {
                                    if(l_size > 26 + (g485BatteryRevBuf[26] << 1))
                                    {
                                        info->voltage = ((int16_t)((uint16_t)(g485BatteryRevBuf[4] << 8) | g485BatteryRevBuf[5])) / 10;
                                        info->current = ((int16_t)((uint16_t)(g485BatteryRevBuf[6] << 8) | g485BatteryRevBuf[7])) / 10;
                                        info->soc = g485BatteryRevBuf[23];
                                        i16Temp = ((int16_t)((uint16_t)(g485BatteryRevBuf[27] << 8) | g485BatteryRevBuf[28]) - 2731) / 10;
                                        if(i16Temp < -40) i16Temp = -40;
                                        info->maxtemp = i16Temp + 40;
                                        if(g485BatteryRevBuf[26] > 1)
                                        {
                                            for(i = 0; i < g485BatteryRevBuf[26] - 1; i++)
                                            {
                                                i16Temp = ((int16_t)(((uint16_t)(g485BatteryRevBuf[29 + (i << 1)] << 8)) | (g485BatteryRevBuf[30] + (i << 1))) - 2731) / 10;
                                                if(i16Temp < -40) i16Temp = -40;
                                                if(info->maxtemp < i16Temp + 40)
                                                {
                                                    info->maxtemp = i16Temp + 40;
                                                }
                                            }
                                        }
                                        lResult = HAL_OK;
                                    }
                                    else
                                    {
                                        rt_kprintf("battery datalen error: %d.\r\n", l_size);
                                    }
                                }
                                else
                                {
                                    rt_kprintf("battery checksum error.\r\n");
                                }
                            }
                            else
                            {
                                rt_kprintf("battery status error.\r\n");
                            }
                            break;
                        }
                    }
                    else
                    {
                        //rt_kprintfArray(g485BatteryRevBuf, l_size, 16, 1);
                        rt_kprintf("battery head error, %d, %d\r\n", g485BatteryRevBuf[0], g485BatteryRevBuf[1]);
                        break;
                    }
                }                
            }

            /* Check for the Timeout */
            curTick -= tickstart;
            if(curTick > ((l_size > 0) ? (BATTERY485_READ_TIMEOUT_MS << 1) : BATTERY485_READ_TIMEOUT_MS))
            {
                if(DEBUG_DATA_TYPE_7)
                {
                    rt_kprintf("battery Rev timeout!\r\n");
                }
                lResult = HAL_TIMEOUT;
                break;
            }
    	}
    }

    return lResult;
}
/*****************************************************************************
 ��������  : �ű��ص�ص�����ݲɼ�����
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��17��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromYBT(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int i;
    static int liFlag = 0;
    CAN_msg msgRead;
    uint8_t deviceNum = CAN3_DEVICE;

    if(gStUfoData.flag & UFO_BATTERY_CAN2_FLAG)  //���ʹ��can2ͨ�ű�־
    {
        deviceNum = CAN2_DEVICE;
    }
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(deviceNum, &msgRead, 0))
        {
            if(0x200 == msgRead.id)
            {
                /*	parse voltage	*/
            	gStBatteryState.voltage = ((msgRead.data[0]<<8) | msgRead.data[1]) / 10;
            	/*	parse current	*/
            	gStBatteryState.current = (int16_t)((msgRead.data[2]<<8) | msgRead.data[3]) / 10;
             	/*	parse minvoltage	*/
            	gStBatteryState.singleVol[0] = (msgRead.data[4]<<8) | msgRead.data[5];
            	/*	parse maxvoltage	*/
            	gStBatteryState.singleVol[1] = (msgRead.data[6]<<8) | msgRead.data[7];
                liFlag |= 0x01;
            }
            else if(0x201 == msgRead.id)
            {
                /*/#	parse soc	#/
                gStBatteryState.soc = (msgRead.data[0]<<8) | msgRead.data[1];
                /#	parse capnum	#/
                gStBatteryState.capnum = (msgRead.data[2]<<8) | msgRead.data[3];
            	/#	parse cycnum	#/
                gStBatteryState.cycnum = (msgRead.data[4]<<8) | msgRead.data[5];*/
                /*	parse soc	*/
                gStBatteryState.soc = (msgRead.data[6]<<8) | msgRead.data[7];
                liFlag |= 0x02;
            }
            else if(0x202==msgRead.id)
            {
                /*	parse ������־λ	*/
                gStBatteryState.battery_error.word[0]= (msgRead.data[0]<<8) | msgRead.data[1];
                
                /*	parse maxtempnum	*/
                gStBatteryState.maxtemp = (((msgRead.data[4]<<8) | msgRead.data[5]) - 2731) / 10 + 40;
                /*	parse mintempnum	*/
                gStBatteryState.mintemp = (((msgRead.data[6]<<8) | msgRead.data[7]) - 2731) / 10 + 40;
                liFlag |= 0x04;
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x07) == 0x07)
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}
/*****************************************************************************
 ��������  : �ű��ص��(48V30AH)������ݲɼ�����(BMSͨѶЭ���б�)
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��10��26��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromYBT_v2(void)
{
    static int liFlag = 0;
    static TickType_t l_last_tick = 0;
    static uint8_t idIndex = 0u;
    
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int i;
    CAN_msg msgRead;
    uint8_t deviceNum = CAN3_DEVICE;
    TickType_t l_cur_tick;
    CAN_msg msg = {0x100, {0x5A}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint32_t id[3] = {0x100, 0x101, 0x105};
    
    if(gStUfoData.flag & UFO_BATTERY_CAN2_FLAG)  //���ʹ��can2ͨ�ű�־
    {
        deviceNum = CAN2_DEVICE;
    }

    l_cur_tick = HAL_GetTick();
    if(l_cur_tick - l_last_tick > 250)
    {
        l_last_tick = l_cur_tick;
        msg.id = id[idIndex];
        CanDeviceWrite(deviceNum, &msg, 2);

        if (idIndex < ((sizeof(id)/sizeof(id[0]))-1u))
        {
            idIndex++;
        }
        else
        {
            idIndex = 0u;
        }
    }
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(deviceNum, &msgRead, 0))
        {
            if(0x100 == msgRead.id)
            {
                /*	parse voltage	*/
            	gStBatteryState.voltage = ((msgRead.data[0]<<8) | msgRead.data[1]) / 10;
            	/*	parse current	*/
            	gStBatteryState.current = (int16_t)((msgRead.data[2]<<8) | msgRead.data[3]) / 10;
             	/*	parse minvoltage	*/
            	//gStBatteryState.singleVol[0] = (msgRead.data[4]<<8) | msgRead.data[5];
            	/*	parse maxvoltage	*/
            	//gStBatteryState.singleVol[1] = (msgRead.data[6]<<8) | msgRead.data[7];
            	
                liFlag |= 0x01;
            }
            else if(0x101 == msgRead.id)
            {
                /*/#	parse soc	#/
                gStBatteryState.soc = (msgRead.data[0]<<8) | msgRead.data[1];
                /#	parse capnum	#/
                gStBatteryState.capnum = (msgRead.data[2]<<8) | msgRead.data[3];
            	/#	parse cycnum	#/
                gStBatteryState.cycnum = (msgRead.data[4]<<8) | msgRead.data[5];*/
                /*	parse soc	*/
                gStBatteryState.soc = (msgRead.data[4]<<8) | msgRead.data[5];
                liFlag |= 0x02;
            }
            else if(0x105==msgRead.id)
            {
                /*	parse ntc1 tempnum	*/
                gStBatteryState.maxtemp = (((msgRead.data[0]<<8) | msgRead.data[1]) - 2731) / 10 + 40;
                /*	parse ntc2 tempnum	*/
                gStBatteryState.mintemp = (((msgRead.data[2]<<8) | msgRead.data[3]) - 2731) / 10 + 40;

                liFlag |= 0x04;
            }
            else
            {
                ;
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x07) == 0x07)
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}
/*****************************************************************************
 ��������  : �ű��ص��(48V30AH)������ݲɼ�����(BMS�������ң�ͨѶЭ���б�)
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��02��10��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromYBT_v3(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    static int liFlag = 0;
    static TickType_t l_last_tick = 0;
    TickType_t l_cur_tick;
    static uint8_t idIndex = 0u;
    
    int i,j;
    CAN_msg msgRead;
    CAN_msg msg = {0x02800203, {0x00,0x03}, 2, 1, EXTENDED_FORMAT, DATA_FRAME};
    uint32_t id[3] = {0x02800203, 0x02800207, 0x02800218};
    int16_t temp[4];
    int16_t lMax,lMin;
    
    l_cur_tick = HAL_GetTick();
    if(l_cur_tick - l_last_tick > 250)
    {
        l_last_tick = l_cur_tick;
        msg.id = id[idIndex];
        CanDeviceWrite(CAN3_DEVICE, &msg, 2);

        if (idIndex < ((sizeof(id)/sizeof(id[0]))-1u))
        {
            idIndex++;
        }
        else
        {
            idIndex = 0u;
        }
    }
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(CAN3_DEVICE, &msgRead, 0))
        {
            if(id[0] == msgRead.id)//��ȡID
            {
                /*	parse voltage	*/
            	gStBatteryState.voltage = ((msgRead.data[1]<<8) | msgRead.data[2]);
            	/*	parse current	*/
            	gStBatteryState.current = (int16_t)((msgRead.data[3]<<8) | msgRead.data[4]);
                /*	parse soc	*/
                gStBatteryState.soc = (uint8_t)((msgRead.data[5]<<8) | msgRead.data[6]);

                liFlag |= 0x01;
            }
            else if(id[1] == msgRead.id)
            {
                /*	parse ������־λ	*/
                gStBatteryState.battery_error.word[0]= (msgRead.data[3]<<8) | msgRead.data[4];
                gStBatteryState.battery_error.word[1]= (msgRead.data[5]<<8) | msgRead.data[6];
                
                liFlag |= 0x02;
            }
            else if(id[2] == msgRead.id)
            {
                temp[0] = (int16_t)((msgRead.data[1]<<8) | msgRead.data[2]) + 40;
                temp[1] = (int16_t)((msgRead.data[3]<<8) | msgRead.data[4]) + 40;
                temp[2] = (int16_t)((msgRead.data[5]<<8) | msgRead.data[6]) + 40;

                lMin = 1000;
                lMax = -1000;
                
                //����С���ֵ
                for(j = 0; j < 3; j++)
                {
                    if (temp[j] < 1000)     /* �¶�1000���ʾδ���¶Ȳɼ��ߣ�������¶�û�� */
                    {
                        if(temp[j] < lMin)
                        {
                            lMin = temp[j];
                        }
                        else if(temp[j] > lMax)
                        {
                            lMax = temp[j];
                        }
                    }
                }
                /* parse maxtempnum����¶�*/
                gStBatteryState.maxtemp = lMax;//ƫ����40
                /* parse mintempnum����¶�*/
                gStBatteryState.mintemp = lMin;//ƫ����40
                liFlag |= 0x04;
            }
            else 
            {
                ;
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x07) == 0x07)
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}

/*****************************************************************************
 ��������  : ����bmsͨ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��5��25��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromJK(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int i;
    static int liFlag = 0;
    CAN_msg msgRead;
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(CAN3_DEVICE, &msgRead, 0))
        {
            if(0x2f4 == msgRead.id)
            {
                /*	parse voltage	*/
            	gStBatteryState.voltage = msgRead.data[0] | (msgRead.data[1] << 8);
            	/*	parse current	*/
            	gStBatteryState.current = (msgRead.data[2] | (msgRead.data[3] << 8)) - 4000;
             	/*	parse soc	*/
                gStBatteryState.soc = msgRead.data[4];
                liFlag |= 0x01;
            }
            else if(0x5f4==msgRead.id)
            {
                /*	parse maxtempnum	*/
                gStBatteryState.maxtemp = msgRead.data[0]  - 10;
                /*	parse mintempnum	*/
                gStBatteryState.mintemp = msgRead.data[2]  - 10;
                liFlag |= 0x02;
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x03) == 0x03)
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}
/*****************************************************************************
 ��������  : ��άbmsͨ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��5��25��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromBestWay(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    TickType_t l_cur_tick;
    static TickType_t l_last_tick = 0;
    int i;
    uint32_t lu32Temp;
    static int liFlag = 0;
    CAN_msg msg = {0x0E64090D, {0x00}, 8, 1, EXTENDED_FORMAT, DATA_FRAME};
    CAN_msg msgRead;

    l_cur_tick = HAL_GetTick();
    if(l_cur_tick - l_last_tick > 250)
    {
        l_last_tick = l_cur_tick;
        CanDeviceWrite(CAN3_DEVICE, &msg, 2);
    }
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(CAN3_DEVICE, &msgRead, 0))
        {
            if(0x0A6D0D09 == msgRead.id)//��ȡID
            {
                /* parse voltage *///��ѹ
                gStBatteryState.voltage = (msgRead.data[1] | (msgRead.data[0] << 8));          //�õ���ѹ����λ0.1V
                /* parse current *///����
                gStBatteryState.current = ((msgRead.data[3] | (msgRead.data[2] << 8))-65535);//�ŵ�Ϊ�������Ϊ������λ0.1A


                liFlag |= 0x01;
            }
            else if(0x0A6E0D09 == msgRead.id)
            {
                /* parse soc  */     //ʣ�������ٷֱ�
                gStBatteryState.soc = ((msgRead.data[1])|(msgRead.data[0] << 8))/10; //���磺0x02ED = 749 ��749/1000=74.9%����ʾSOCΪ 74.9%
                liFlag |= 0x02;
            }
            else if(0x0A700D09 == msgRead.id)
            {
                /* parse maxtempnum����¶�*/
                gStBatteryState.maxtemp = (char)msgRead.data[0] + 40;//ƫ����40
                /* parse mintempnum����¶�*/
                gStBatteryState.mintemp = (char)msgRead.data[4] + 40;//ƫ����40
                liFlag |= 0x04;
            }
            else if((0x0E640D09 <= msgRead.id) &&
                (0x0E6A0D09 >= msgRead.id))//��ȡID
            {
                lu32Temp = (msgRead.id - 0x0E640D09) >> 16;
                lu32Temp <<= 2;
                if(lu32Temp + 4 <= BATTERY_CELL_MAX_NUMS)
                {
                    gStBatteryState.singleVol[lu32Temp] = (msgRead.data[1] | (msgRead.data[0] << 8));//�õ������ѹ����λ0.001V
                    gStBatteryState.singleVol[lu32Temp + 1] = (msgRead.data[3] | (msgRead.data[2] << 8));//�õ������ѹ����λ0.001V
                    gStBatteryState.singleVol[lu32Temp + 2] = (msgRead.data[5] | (msgRead.data[4] << 8));//�õ������ѹ����λ0.001V
                    gStBatteryState.singleVol[lu32Temp + 3] = (msgRead.data[7] | (msgRead.data[6] << 8));//�õ������ѹ����λ0.001V
                }
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x07) == 0x07) //ȫ����ȡ�ŷ���OK
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}
/*****************************************************************************
 ��������  : �ű������ִ�ƽ����bmsͨ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��2��16��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromYBTThreeLarge(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int i;
    static int liFlag = 0;
    CAN_msg msgRead;
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(CAN3_DEVICE, &msgRead, 0))
        {
            if(0x18016688 == msgRead.id)
            {
                /*	parse voltage	*/
            	gStBatteryState.voltage = msgRead.data[0] | (msgRead.data[1] << 8);
            	/*	parse current	*/
            	gStBatteryState.current = (msgRead.data[2] | (msgRead.data[3] << 8));
             	/*	parse soc	*/
                gStBatteryState.soc = msgRead.data[4];
                liFlag |= 0x01;
            }
            else if(0x18026688==msgRead.id)
            {
                /*	parse minvoltage	*/
            	gStBatteryState.singleVol[0] = (msgRead.data[3]<<8) | msgRead.data[2];
            	/*	parse maxvoltage	*/
            	gStBatteryState.singleVol[1] = (msgRead.data[1]<<8) | msgRead.data[0];
                /*	parse maxtempnum	*/
                gStBatteryState.maxtemp = ((msgRead.data[5]<<8) | msgRead.data[4]) / 10;
                /*	parse mintempnum	*/
                gStBatteryState.mintemp = ((msgRead.data[7]<<8) | msgRead.data[6]) / 10;
                liFlag |= 0x02;
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x03) == 0x03)
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}
/*****************************************************************************
 ��������  : ���ִ�ƽ����յ��bmsͨ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��2��23��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromJIUPUFourLarge(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    TickType_t l_cur_tick;
    static TickType_t l_last_tick = 0;
    int i, liId, liCnt;
    static int liNum = BATTERY_MAX_NUMS, liIndex = 0;
    CAN_msg msg = {0x00, {0x00}, 8, 1, EXTENDED_FORMAT, DATA_FRAME};
    CAN_msg msgRead;

    l_cur_tick = HAL_GetTick();
    if(0 == liIndex)
    {
        liNum++;
        if(liNum >= BATTERY_MAX_NUMS)
        {
            liNum = 0;
            liCnt = 0;
            for(i = 0; i < BATTERY_MAX_NUMS; i++)
            {
                if(gStBatteryStateBackup[i].validFlag & 0x02)
                {
                    gStBatteryStateBackup[i].validFlag &= (~0x02);
                    if(0 == liCnt)
                    {
                        gStBatteryState.voltage = gStBatteryStateBackup[i].voltage;
                        gStBatteryState.current = gStBatteryStateBackup[i].current;
                        gStBatteryState.soc = gStBatteryStateBackup[i].soc;
                        gStBatteryState.singleVol[0] = gStBatteryStateBackup[i].singleVol[0];
                        gStBatteryState.singleVol[1] = gStBatteryStateBackup[i].singleVol[1];
                        gStBatteryState.maxtemp = gStBatteryStateBackup[i].maxtemp;
                        gStBatteryState.mintemp = gStBatteryStateBackup[i].mintemp;
                        lResult = HAL_OK;
                    }
                    else
                    {
                        if(gStBatteryStateBackup[i].voltage < gStBatteryState.voltage)
                        {
                            gStBatteryState.voltage = gStBatteryStateBackup[i].voltage;
                        }
                        gStBatteryState.current += gStBatteryStateBackup[i].current;
                        if(gStBatteryStateBackup[i].soc < gStBatteryState.soc)
                        {
                            gStBatteryState.soc = gStBatteryStateBackup[i].soc;
                        }
                        if(gStBatteryStateBackup[i].singleVol[0] < gStBatteryState.singleVol[0])
                        {
                            gStBatteryState.singleVol[0] = gStBatteryStateBackup[i].singleVol[0];
                        }
                        if(gStBatteryStateBackup[i].singleVol[1] > gStBatteryState.singleVol[1])
                        {
                            gStBatteryState.singleVol[1] = gStBatteryStateBackup[i].singleVol[1];
                        }
                        if(gStBatteryStateBackup[i].mintemp < gStBatteryState.mintemp)
                        {
                            gStBatteryState.mintemp = gStBatteryStateBackup[i].mintemp;
                        }
                        if(gStBatteryStateBackup[i].maxtemp > gStBatteryState.maxtemp)
                        {
                            gStBatteryState.maxtemp = gStBatteryStateBackup[i].maxtemp;
                        }
                    }
                    liCnt++;
                }
            } 
        }
        liIndex = 1;
        gStBatteryStateBackup[liNum].validFlag &= (~0x70);
    }
    else if(liNum >= BATTERY_MAX_NUMS)
    {
        liNum = 0;
    }
    else if(liIndex <= 3)
    {
        if(gStBatteryStateBackup[liNum].validFlag & 0x10)
        {
            liIndex = 4;
        }
        else if(l_cur_tick - l_last_tick > 25)
        {
            liIndex++;
            l_last_tick = l_cur_tick;
            msg.id = liNum + 1;
            msg.id = 0x18900040 | (msg.id << 8);    //��ѹ������������
            CanDeviceWrite(CAN3_DEVICE, &msg, 2);
        }
    }
    else if(liIndex <= 6)
    {
        if(gStBatteryStateBackup[liNum].validFlag & 0x20)
        {
            liIndex = 7;
        }
        else if(l_cur_tick - l_last_tick > 25)
        {
            liIndex++;
            l_last_tick = l_cur_tick;
            msg.id = liNum + 1;
            msg.id = 0x18910040 | (msg.id << 8);    //���������͵�ѹ
            CanDeviceWrite(CAN3_DEVICE, &msg, 2);
        }
    }
    else if(liIndex <= 9)
    {
        if(gStBatteryStateBackup[liNum].validFlag & 0x40)
        {
            liIndex = 10;
            l_last_tick = l_cur_tick - 70;
        }
        else if(l_cur_tick - l_last_tick > 25)
        {
            liIndex++;
            l_last_tick = l_cur_tick;
            msg.id = liNum + 1;
            msg.id = 0x18920040 | (msg.id << 8);    //�������¶�
            CanDeviceWrite(CAN3_DEVICE, &msg, 2);
        }
    }
    else if(l_cur_tick - l_last_tick > 100)
    {
        liIndex = 0;
        l_last_tick = l_cur_tick;
    }
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(CAN3_DEVICE, &msgRead, 0))
        {
            liId = msgRead.id & 0xff;
            if((liId > 0) && (liId <= 4))
            {
                if(0x18904000 == (msgRead.id & 0xffffff00))//��ȡID
                {
                    /* parse voltage *///��ѹ
                    gStBatteryStateBackup[liId - 1].voltage = (msgRead.data[1] | (msgRead.data[0] << 8));          //�õ���ѹ����λ0.1V
                    /* parse current *///����
                    gStBatteryStateBackup[liId - 1].current = (int16_t)(msgRead.data[5] | (msgRead.data[4] << 8)) - 30000;//30000ƫ�ƣ���λ0.1A
                    /* parse soc  */     //ʣ�������ٷֱ�
                    gStBatteryStateBackup[liId - 1].soc = ((msgRead.data[7]) | (msgRead.data[6] << 8)) / 10;    //���磺0x02ED = 749 ��749/1000=74.9%����ʾSOCΪ 74.9%
    
                    gStBatteryStateBackup[liId - 1].validFlag |= 0x10;
                }
                else if(0x18914000 == (msgRead.id & 0xffffff00))
                {
                    /*	parse minvoltage	*/
                	gStBatteryStateBackup[liId - 1].singleVol[0] = (msgRead.data[3]<<8) | msgRead.data[4];
                	/*	parse maxvoltage	*/
                	gStBatteryStateBackup[liId - 1].singleVol[1] = (msgRead.data[0]<<8) | msgRead.data[1];
                    gStBatteryStateBackup[liId - 1].validFlag |= 0x20;
                }
                else if(0x18924000 == (msgRead.id & 0xffffff00))
                {
                    /* parse maxtempnum����¶�*/
                    gStBatteryStateBackup[liId - 1].maxtemp = msgRead.data[0];//ƫ����40
                    /* parse mintempnum����¶�*/
                    gStBatteryStateBackup[liId - 1].mintemp = msgRead.data[2];//ƫ����40
                    gStBatteryStateBackup[liId - 1].validFlag |= 0x40;
                }
                if((gStBatteryStateBackup[liId - 1].validFlag & 0x70) == 0x70)
                {
                    gStBatteryStateBackup[liId - 1].validFlag |= 0x03;
                }
            }
        }
        else
        {
            break;
        }
    } 

    return lResult;
}
/*****************************************************************************
 ��������  : ˾��̩��������ƽ����bmsͨ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2024��7��15��
*****************************************************************************/
static HAL_StatusTypeDef ReadBatteryInfoFromSMTKFourMid(void)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    TickType_t l_cur_tick;
    static TickType_t l_last_tick = 0;
    static int liFlag = 0;
    int i, liId;
    CAN_msg msg = {0x0400FF80, {0x00}, 8, 1, EXTENDED_FORMAT, DATA_FRAME};
    CAN_msg msgRead;

    l_cur_tick = HAL_GetTick();
    
    if(l_cur_tick - l_last_tick > 2000)
    {
        l_last_tick = l_cur_tick;
        CanDeviceWrite(CAN3_DEVICE, &msg, 2);
    }
    
    //(1)������ջ�����������,û������ʱ�����صĴ�СΪ0
    for(i = 0; i < 3; i++)
    {
        if(HAL_OK == CanDeviceRead(CAN3_DEVICE, &msgRead, 0))
        {
            liId = msgRead.id & 0xff;
            if(0x01 == liId)
            {
                if(0x04028000 == (msgRead.id & 0xffffff00))//��ȡID
                {
                    /* parse voltage *///��ѹ
                    gStBatteryState.voltage = (msgRead.data[1] | (msgRead.data[0] << 8));          //�õ���ѹ����λ0.1V
                    /* parse current *///����
                    gStBatteryState.current = (int16_t)(msgRead.data[3] | (msgRead.data[2] << 8)) - 30000;//30000ƫ�ƣ���λ0.1A
                    /* parse soc  */     //ʣ�������ٷֱ�
                    gStBatteryState.soc = ((msgRead.data[5]) | (msgRead.data[4] << 8)) / 10;    //���磺0x02ED = 749 ��749/1000=74.9%����ʾSOCΪ 74.9%
    
                    liFlag |= 0x01;
                }
                else if(0x04048000 == (msgRead.id & 0xffffff00))
                {
                    /*	parse minvoltage	*/
                	gStBatteryState.singleVol[0] = (msgRead.data[3]<<8) | msgRead.data[4];
                	/*	parse maxvoltage	*/
                	gStBatteryState.singleVol[1] = (msgRead.data[0]<<8) | msgRead.data[1];
                    liFlag |= 0x02;
                }
                else if(0x04058000 == (msgRead.id & 0xffffff00))
                {
                    /* parse maxtempnum����¶�*/
                    gStBatteryState.maxtemp = msgRead.data[0];//ƫ����40
                    /* parse mintempnum����¶�*/
                    gStBatteryState.mintemp = msgRead.data[2];//ƫ����40
                    liFlag |= 0x04;
                }
            }
        }
        else
        {
            break;
        }
    }

    if((liFlag & 0x07) == 0x07)
    {
        liFlag = 0;
        lResult = HAL_OK;
    }

    return lResult;
}
/*****************************************************************************
 ��������  : ��ӡ�����ѹ��Ϣ
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��8��22��
*****************************************************************************/
void BatterPrintInfo(void)
{
    int32_t i;
    int32_t min = 0, max = 0;

    rt_kprintf("\r\nBatInfo-Vol:%.1f V,Soc:%d%%,Tmp:%d,C:%.1f A,minVol:%dmV,maxVol:%dmV,errFlg:%#x.\r\n", gStBatteryState.voltage / 10.0f, 
        gStBatteryState.soc, (int8_t)(gStBatteryState.maxtemp - 40), gStBatteryState.current / 10.0f, 
        gStBatteryState.singleVol[0], gStBatteryState.singleVol[1], gStBatteryState.battery_error.dword);

    if(gStUfoData.flag & UFO_BMS_JIUPU_FOUR_LARGE)
    {
        for(i = 0; i < BATTERY_MAX_NUMS; i++)
        {
            if(gStBatteryStateBackup[i].validFlag & 0x01)
            {
                gStBatteryStateBackup[i].validFlag &= (~0x01);
                rt_kprintf("Bat%d-Vol:%.1f V,Soc:%d%%,Tmp:%d,C:%.1f A,minVol:%dmV,maxVol:%dmV.\r\n", i + 1, gStBatteryStateBackup[i].voltage / 10.0f, 
                    gStBatteryStateBackup[i].soc, (int8_t)(gStBatteryStateBackup[i].maxtemp - 40), gStBatteryStateBackup[i].current / 10.0f,
                    gStBatteryStateBackup[i].singleVol[0], gStBatteryStateBackup[i].singleVol[1]);
            }
        }
    }

    if(gStUfoData.flag & UFO_BMS_BAIWEI)
    {
        rt_kprintf("\r\nB1:%dmv,B2:%dmv,B3:%dmv,B4:%dmv.\r\n", gStBatteryState.singleVol[0],
            gStBatteryState.singleVol[1], gStBatteryState.singleVol[2], gStBatteryState.singleVol[3]);
        rt_kprintf("B5:%dmv,B6:%dmv,B7:%dmv,B8:%dmv.\r\n", gStBatteryState.singleVol[4],
            gStBatteryState.singleVol[5], gStBatteryState.singleVol[6], gStBatteryState.singleVol[7]);
        rt_kprintf("B9:%dmv,B10:%dmv,B11:%dmv,B12:%dmv.\r\n", gStBatteryState.singleVol[8],
            gStBatteryState.singleVol[9], gStBatteryState.singleVol[10], gStBatteryState.singleVol[11]);
        rt_kprintf("B13:%dmv,B14:%dmv,B15:%dmv,B16:%dmv.\r\n", gStBatteryState.singleVol[12],
            gStBatteryState.singleVol[13], gStBatteryState.singleVol[14], gStBatteryState.singleVol[15]);
        rt_kprintf("B17:%dmv,B18:%dmv,B19:%dmv,B20:%dmv.\r\n", gStBatteryState.singleVol[16],
            gStBatteryState.singleVol[17], gStBatteryState.singleVol[18], gStBatteryState.singleVol[19]);
        
        //Ѱ����Сֵ
        for(i = BATTERY_CELL_MAX_NUMS - 1; i > 0; i--)
        {
            if(0 != gStBatteryState.singleVol[i])
            {
                if(gStBatteryState.singleVol[i] <= gStBatteryState.singleVol[min])
                {
                    min = i;
                }
                else if(gStBatteryState.singleVol[i] >= gStBatteryState.singleVol[max])
                {
                    max = i;
                }
            }
        }
    
        rt_kprintf("Bmin num:%d,vol:%dmv.\r\nBmax num:%d,vol:%dmv.\r\n\r\n", min + 1, gStBatteryState.singleVol[min],
            max + 1, gStBatteryState.singleVol[max]);
    }
}
/*****************************************************************************
 ��������  : ������ݲɼ�����
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��17��
*****************************************************************************/
void BatteryCollectProcess(rt_bool_t canFlag)
{
    static uint8_t lastSoc = 150, lastDataInfo = 0;
    uint16_t dataInfo, validFlag = 0;
    
    if((gStUfoData.flag & UFO_BMS_JBD) && (RT_FALSE == canFlag))  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadMiniUfoV2BatteryInfoFrom485(&gStBatteryState))
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_JIAYUAN) && (RT_FALSE == canFlag))  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFrom485Modbus(BATTERY_INFO_TYPE_SOC, &dataInfo))
        {
            gStBatteryState.soc = dataInfo;
            if(HAL_OK == ReadBatteryInfoFrom485Modbus(BATTERY_INFO_TYPE_TEMP, &dataInfo))
            {
                validFlag = 1;
                gStBatteryState.maxtemp = dataInfo;
                gStBatteryState.validFlag = 1;
                gStBatteryState.lastRevTime = HAL_GetTick();
            }
        }
    }
    else if((gStUfoData.flag & UFO_BMS_YBT) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromYBT())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_JIKONG) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromJK())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_BAIWEI) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromBestWay())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_YBT_v2) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromYBT_v2())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_YBT_v3) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromYBT_v3())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_YBT_THREE_LARGE) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromYBTThreeLarge())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if((gStUfoData.flag & UFO_BMS_JIUPU_FOUR_LARGE) && canFlag)  //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromJIUPUFourLarge())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }
    else if(IS_UFOONE_FLAG_SET(UFOONE_BMS_SMTK_FOUR_MID) && canFlag)    //bms��Ӧ�ͺ�
    {
        if(HAL_OK == ReadBatteryInfoFromSMTKFourMid())
        {
            validFlag = 1;
            gStBatteryState.validFlag = 1;
            gStBatteryState.lastRevTime = HAL_GetTick();
        }
    }

    //����ͨ����ѹ����
    if(IS_UFOONE_FLAG_SET(UFOONE_SOC_FROM_VOL))
    {
        if((gStBatteryState.validFlag && (0 == gStBatteryState.motorRunFlag)) || gStBatteryState.driverVolValidFlag)
        {
            if(gStUfoData.fullVol > gStUfoData.minVol / 10)
            {
                if(gStBatteryState.voltage / 10 >= gStUfoData.fullVol)
                {
                    gStBatteryState.soc = 100;  //������
                }
                else if(gStBatteryState.voltage <= gStUfoData.minVol)
                {
                    gStBatteryState.soc = 0;    //�޵���
                }
                else
                {   
                    dataInfo = (gStUfoData.fullVol * 10 - gStUfoData.minVol) / 3;   //��ѹ��Χ�ֳ�150�ȷ�
                    if(0 == dataInfo) dataInfo = 1;
                    dataInfo = (gStBatteryState.voltage - gStUfoData.minVol) * 50 / dataInfo;   //ʵ�ʵ�ѹ��ռ�ȷ�λ��
                    if(lastDataInfo != dataInfo)    //�����˲�����ֹ������������
                    {
                        if(lastDataInfo + 1 == dataInfo)    //����ֻ����һ��ֵ������ԣ������������ϵ�ֵ����Ϊ�ǳ��
                        {
                            dataInfo = lastDataInfo;
                        }
                        else
                        {
                            lastDataInfo = dataInfo;
                        }
                    }                    
                    if(dataInfo > 100)  //80%���ϵ���ռ50�ȷ�
                    {
                        gStBatteryState.soc = 80 + (dataInfo - 100) * 2 / 5;
                        if(gStBatteryState.soc > 100) gStBatteryState.soc = 100;
                    }
                    else if(dataInfo > 40)  //40%~80%ռ60�ȷ�
                    {
                        gStBatteryState.soc = 40 + (dataInfo - 40) * 33 / 50;
                    }
                    else                //40%����ռ40�ȷ�
                    {
                        gStBatteryState.soc = dataInfo;
                    }
                }
            }
            
            lastSoc = gStBatteryState.soc;
        }
        else if(lastSoc <= 100) //���й����б���֮ǰ��ֵ����ֹ�л��ص��ͨ�Ż�ȡ���ĵ���ֵ
        {
            gStBatteryState.soc = lastSoc;
        }
    }
    
    //ʵʱ��ӡ�����Ϣ
    if(validFlag)
    {
        if(DEBUG_DATA_TYPE_84 || DEBUG_DATA_TYPE_3 || DEBUG_DATA_TYPE_94)
        {
            BatterPrintInfo();
        }
        gRespondState.batteryVoltage = gStBatteryState.voltage;
        gRespondState.batteryCurrent = gStBatteryState.current;
        gRespondState.batterySoc = gStBatteryState.soc;
        gRespondState.batteryTmp[0] = gStBatteryState.maxtemp;
        gRespondState.batteryTmp[1] = gStBatteryState.mintemp;
    }
    //��ʱ��δ�յ������Ϣ����Ϊ��Ч
    if(gStBatteryState.validFlag)
    {
        if(HAL_GetTick() - gStBatteryState.lastRevTime >= (BATTERY485_READ_PERIOD << 2))
        {
            gStBatteryState.validFlag = 0;
        }
    }
}
/*****************************************************************************
 ��������  : ��ȡ�����Ϣ
 �������  : rt_uint32_t* voltage    
             int8_t* temperature  
             int8_t* current      
 �������  : uint8_t ����0��ȡ�ɹ�������1��ʾ����Ϣ
 ��    ��  : ����
 ��    ��  : 2018��12��17��
*****************************************************************************/
/*uint8_t GetBatteryInfo(uint8_t *voltage, int8_t *temperature, int8_t *current, uint8_t *error, uint8_t *powerQ)
{
    if(gStBatteryState.validFlag)
    {
        gStBatteryState.validFlag = 0;
        *voltage = gStBatteryState.voltage / 10;//�ֱ���0.1v
        *temperature = gStBatteryState.maxtemp;
        *temperature -= 40;//ƫ��-40
        *current = gStBatteryState.current / 10;//�ֱ���0.1A
        *powerQ = gStBatteryState.soc;//�ֱ���0.01A*H
        *error = gStBatteryState.error;
        return 0;
    }
    return 1;
}*/

