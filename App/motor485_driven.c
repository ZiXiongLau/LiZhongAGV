/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include <motor_control.h>
#include <motor485_driven.h>
#include "usart.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "delay.h"

uint8_t g485RevBuf[13];
uint8_t g485ErrCnt[M_TOTAL_NUM] = {0};

/*****************************************************************************
 ��������  : modbus rtu crc16У��
 �������  : uint8_t *dataBuf  ��У������
             uint16_t len      У�鳤��
 �������  : У����
 ��    ��  : ����
 ��    ��  : 2020��12��22��
*****************************************************************************/
uint16_t MODBUS_RTUCrc16(uint8_t *dataBuf, uint16_t len)
{
    int i, k;

    uint16_t crc16 = 0xffff; //��ʼֵ
    
    for(i = 0; i < len; i++)
    {
        crc16 = dataBuf[i] ^ crc16;
        for(k = 0; k < 8; k++){  /*�˴���8 -- ָÿһ��char������8bit��ÿbit��Ҫ����*/
            if(crc16 & 0x01){
                crc16 = crc16 >> 1;
                crc16 = crc16 ^ 0xa001;
            }   
            else{
                crc16 = crc16 >> 1;
            }   
        }   
    }
    
    return crc16;
}
/*****************************************************************************
 ��������  : �˶Խ�������
 �������  : uint32_t revSize  �������ݳ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
HAL_StatusTypeDef Motor485CheckRevData(uint8_t *revBuf, uint32_t revSize)
{
    uint16_t crcValue;

    if(revSize >= 2)
    {
        //����У����
        crcValue = MODBUS_RTUCrc16(revBuf, revSize - 2);
        //�˶�crc
        if((revBuf[revSize - 1] == ((crcValue >> 8) & 0xff)) 
            && (revBuf[revSize - 2] == (crcValue & 0xff)))
        {
            return HAL_OK;
        }
        else
        {
            rt_kprintf("Rev crc error!\r\n");
        }
    }
    
    return HAL_ERROR;
}
/*void readTest(void)
{
    uint32_t l_size = 0;
    
    l_size = UsartDeviceRead(USART5_DEVICE, g485RevBuf, 13);

    if(l_size > 0)
    {
        rt_kprintfArray(g485RevBuf, l_size, 16, 1);
    }
}*/
/*****************************************************************************
 ��������  : ��װ485���������
 �������  : uint8_t *buffer  
             uint32_t size    
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��12��21��
*****************************************************************************/
static HAL_StatusTypeDef Motor485DeviceRead(uint32_t motorNum, uint8_t *sendBuffer, uint32_t sendSize, uint32_t revSize)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint32_t tickstart, curTick;
	uint32_t l_size = 2;
    uint16_t crcValue;

    if((sendSize <= 4) || (revSize > sizeof(g485RevBuf)))
    {
        return lResult;
    }

    //����У����
    crcValue = MODBUS_RTUCrc16(sendBuffer, sendSize - 4);
    sendBuffer[sendSize - 3] = (crcValue >> 8) & 0xff;
    sendBuffer[sendSize - 4] = crcValue & 0xff;

    sendSize -= 2;
    l_size = UsartDeviceWrite(USART2_DEVICE, sendBuffer, sendSize);

    if(l_size == sendSize)
	{
        l_size = 0;
        /* Get tick */ 
        tickstart = HAL_GetTick();

        osDelay(1); //115200��������ʱ1ms

        while(1)
        {            
            l_size += UsartDeviceRead(USART2_DEVICE, &g485RevBuf[l_size], revSize - l_size);
            delay_us(600);
            if(l_size >= revSize)
            {
                if((g485RevBuf[0] == sendBuffer[0]))// && (g485RevBuf[1] == sendBuffer[1]))  //�˶�վ��
                {
                    lResult = Motor485CheckRevData(g485RevBuf, revSize);
                }
                else if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("Id error, sid: %d, rid: %d\r\n", sendBuffer[0], g485RevBuf[0]);
                    if(DEBUG_DATA_TYPE_7)
                    {
                        rt_kprintfArray(g485RevBuf, l_size, 16, 1);
                    }
                }
                break;
            }

            /* Check for the Timeout */
            curTick = HAL_GetTick();
            curTick -= tickstart;
            if(curTick > ((l_size > 0) ? (MOTOR485_READ_TIMEOUT_MS << 1) : MOTOR485_READ_TIMEOUT_MS))
            {
                if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("id%d Rev timeout,%d,%d!\r\n", sendBuffer[0], revSize, l_size);
                    if(DEBUG_DATA_TYPE_7)
                    {
                        rt_kprintfArray(g485RevBuf, l_size, 16, 1);
                    }
                }
                lResult = HAL_TIMEOUT;
                break;
            }
    	}
    }
    else
    {
        rt_kprintf("Send err:%d,%d!\r\n", sendSize, l_size);
    }

    if(motorNum < M_TOTAL_NUM)
    {
        if(HAL_OK == lResult)
        {
            if(g485ErrCnt[motorNum])
            {
                g485ErrCnt[motorNum]--;
            }
        }
        else
        {
            g485ErrCnt[motorNum]++;
            if(g485ErrCnt[motorNum] > 5)
            {
                g485ErrCnt[motorNum] = 0;
                SetErrorCode(motorNum, ERROR_CODE_485_LINE_OFF, ERROR_L_HIHG);
            }
        }
    }
	return lResult;
}
/*****************************************************************************
 ��������  : �趨���Ŀ���ٶ�
 �������  : uint32_t motorNum  ������
             int32_t vel        Ŀ���ٶ�
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetVelocity(uint32_t motorNum, int32_t vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x6F;
    msg[3] = 0x00;
    //��ֵ
    vel = vel * gStMotorData[motorNum].counts / 3.6621f;
    msg[7] = (vel >> 8) & 0xff;
    msg[8] = vel & 0xff;
    msg[9]= (vel >> 24) & 0xff;
    msg[10]= (vel >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : �趨�������ģʽ
 �������  : uint32_t motorNum    ������
             HXC_MOTOR_MODE mode  ����ģʽ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��8��10��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetMotorContrlmode(uint32_t motorNum, HXC_MOTOR_MODE mode)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ
    
    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0x9f;
    //��ֵ
    msg[5] = (uint8_t)mode;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : �趨����㶯
 �������  : uint32_t motorNum  ������
             ENUM_DIR moveDir   �˶�����
             int32_t vel        �ٶ�rpm
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��31��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetMotorMove(uint32_t motorNum, ENUM_DIR moveDir, int32_t vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ
    uint16_t dataMsg = 0;
    
    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0xCA;
    //��ֵ
    if(DIR_CCW == moveDir)  //����
    {
        dataMsg |= (1 << 15);
    }
    dataMsg |= (ABS_VALUE(vel) << 6);   //�㶯�ٶ�
    dataMsg |= (1 << 5);    //�㶯ֹͣ��ʽ��1Ϊ����ֹͣ��0Ϊ����ֹͣ
    if(DIR_STOP != moveDir)
    {
        dataMsg |= 1;   //����
    }
    msg[4] = (dataMsg >> 8) & 0xff;
    msg[5] = dataMsg & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : ���ݵ�ǰλ���趨����λ��
 �������  : uint32_t motorNum  ������
             int32_t pos        ����λ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��8��3��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetAbsPosAcordCurPos(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ
    int32_t lCurPos;

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }

    lResult = Motor485ReadPos(motorNum, &lCurPos);
    if(HAL_OK != lResult)
    {
        return lResult;
    }

    pos = pos - lCurPos;
    
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0xDE;
    //��ֵ
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : �趨�����������λ��
 �������  : uint32_t motorNum  ������
             int32_t pos        λ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��30��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetAbsPos(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0xD0;
    //��ֵ
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : �趨�����������λ��(ʵʱ)
 �������  : uint32_t motorNum  ������
             int32_t pos        λ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��30��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetAbsPosRealtime(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0xE8;
    //��ֵ
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : �趨���ʵ��λ��
 �������  : uint32_t motorNum  ������
             int32_t pos        λ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��31��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetActualPos(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0xD2;
    //��ֵ
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : �趨���Ŀ�����
 �������  : uint32_t motorNum  ������
             int32_t current    Ŀ�����mA
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetCurrent(uint32_t motorNum, int32_t current)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //д�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x3c;
    msg[3] = 0x00;
    //��ֵ
    current = current * 100 / gStMotorData[motorNum].normalCurrent * 10;
    msg[4] = (current >> 8) & 0xff;
    msg[5] = current & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 ��������  : ������������
 �������  : uint32_t motorNum    ���
             uint16_t* errorCode  �������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadErrorCode(uint32_t motorNum, uint16_t* errorCode)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00};   //�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x1f;
    msg[3] = 0x00;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 7);

    if(HAL_OK == lResult)
    {
        *errorCode = (g485RevBuf[3] << 8) | g485RevBuf[4];
    }

    return lResult;
}
/*****************************************************************************
 ��������  : ������ٶ�
 �������  : uint32_t motorNum    ���
             int32_t *vel         ����ٶ�
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadVelocity(uint32_t motorNum, int32_t *vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00};   //�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x3B;
    msg[3] = 0x00;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 9);

    if(HAL_OK == lResult)
    {
        *vel = (g485RevBuf[3] << 8) | g485RevBuf[4] | (g485RevBuf[5] << 24) | (g485RevBuf[6] << 16);
        *vel = *vel * 3.6621f / gStMotorData[motorNum].counts;
    }

    return lResult;
} 
/*****************************************************************************
 ��������  : �����λ��
 �������  : uint32_t motorNum  ������
             int32_t *pos       λ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��30��
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadPos(uint32_t motorNum, int32_t *pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00};   //�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0x04;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 9);

    if(HAL_OK == lResult)
    {
        *pos = (g485RevBuf[3] << 8) | g485RevBuf[4] | (g485RevBuf[5] << 24) | (g485RevBuf[6] << 16);
    }

    return lResult;
}
/*****************************************************************************
 ��������  : ���������
 �������  : uint32_t motorNum  ������
             uint16_t *error    ����      
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��7��30��
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadError(uint32_t motorNum, uint16_t *error)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00};   //�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x00;
    msg[3] = 0xA3;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 7);

    if(HAL_OK == lResult)
    {
        *error = (g485RevBuf[3] << 8) | g485RevBuf[4];
    }

    return lResult;
}
/*****************************************************************************
 ��������  : ���������
 �������  : uint32_t motorNum    ���
             int32_t *vel         �������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��6��
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadCurrent(uint32_t motorNum, int16_t *current)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00};   //�����ݸ�ʽ

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus����
    msg[2] = 0x3E;
    msg[3] = 0x00;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 7);

    if(HAL_OK == lResult)
    {
        *current = *(int16_t*)((g485RevBuf[3] << 8) | g485RevBuf[4]);
    }

    return lResult;
}

/*****************************************************************************
 ��������  : ���485ͨ�Ų���
 �������  : uint8_t motorNum  ������
             uint8_t* testData ��������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��1��25��
*****************************************************************************/
void Motor485Test(uint8_t motorNum, uint8_t* testData)
{
    uint16_t u16Temp;
    int32_t iTemp;
    int16_t i16Temp;
    
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }
    
    switch(testData[0])
    {
        case 0:
            if(HAL_OK == Motor485ReadErrorCode(motorNum, &u16Temp))
            {
                rt_kprintf("Motor %d errorcode: 0x%x\r\n", motorNum, u16Temp);
            }
        break;
        case 1:
            if(HAL_OK == Motor485ReadVelocity(motorNum, &iTemp))
            {
                rt_kprintf("Motor %d speed: %d\r\n", motorNum, iTemp);
            }
        break;
        case 2:
            iTemp = *(int32_t*)(&testData[1]);
            if(HAL_OK == Motor485SetTargetCurrent(motorNum, iTemp))
            {
                rt_kprintf("Motor %d set current: %d mA\r\n", motorNum, iTemp);
            }
        break;
        case 3:
            if(HAL_OK == Motor485ReadCurrent(motorNum, &i16Temp))
            {
                rt_kprintf("Motor %d current: %d\r\n", motorNum, i16Temp);
            }
        case 4:
            iTemp = *(int32_t*)(&testData[1]);
            if(HAL_OK == Motor485SetTargetAbsPosRealtime(motorNum, iTemp))
            {
                rt_kprintf("Motor %d set abspos: %d\r\n", motorNum, iTemp);
            }
        break;
        case 5:
            if(HAL_OK == Motor485ReadPos(motorNum, &iTemp))
            {
                rt_kprintf("Motor %d read pos: %d\r\n", motorNum, iTemp);
            }
        break;
        case 6:
            rt_kprintf("Motor %d set move: %d\r\n", motorNum, testData[1]);
            Motor485SetMotorMove(motorNum, (ENUM_DIR)testData[1], 50);
        break;
        case 7:
            Motor485ReadError(motorNum, &u16Temp);
            rt_kprintf("Motor %d read error: 0x%x\r\n", motorNum, u16Temp);
        break;
        case 10:
            rt_kprintf("Motor %d limit1 state: %d\r\n", motorNum, GetMotorLimit1State(motorNum));
        break;
    }
}
/*/#****************************************************************************
 ��������  : �趨Ŀ�����
 �������  : uint8_t idx  
             int32_t current  Ŀ�����
             rt_bool_t checkAck �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��6��8��
****************************************************************************#/
uint32_t MotorSetTargetCurrent(uint8_t idx, int32_t current, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x71, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    int16_t torque;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;  

    if(idx < M_MAX_ID)
    {
        if(gMotorIdToMotorNum[idx] < M_TOTAL_NUM)
        {
            driveType = gStMotorData[gMotorIdToMotorNum[idx]].driverType;
            torque = current * 1000 / gStMotorData[gMotorIdToMotorNum[idx]].normalCurrent;  //�������ǧ�ֱȣ�������ص�ǧ�ֱ�
            torque= Limit(torque, -1000, 1000);

            if(DRIVER_TYPE_EPOS == driveType)   //EPOS�������޷����̶�8�ֽڷ������ݣ�ֻ���ж������ݷ�����
            {
                msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
                msg.len = 6;
            }
 
            msg.id += idx;
            (*(int16_t*)&msg.data[4]) = torque;

            if(RT_TRUE == checkAck)
            {
                if(0 == MotorCanDeviceRead(&msg, &msgRead))
                {
                    return 1;
                }
            }
            else
            {
                MotorCanDeviceWrite(&msg, CAN_WRITE_TIMEOUT_MS);
            }
        }
    }

    return 0;
}
/#****************************************************************************
 ��������  : ��ȡ������
 �������  : uint8_t idx          
             uint16_t *errorCode  ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
****************************************************************************#/
uint32_t MotorReadErrorCode(uint8_t idx, uint16_t *errorCode)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x3F, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;

    if(idx < M_MAX_ID)
    {
        if(gMotorIdToMotorNum[idx] < M_TOTAL_NUM)
        {
            driveType = gStMotorData[gMotorIdToMotorNum[idx]].driverType;
        }
    }

    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        //index 0x1002 compley�������������ַ
        msg.data[1] = 0x02;
        msg.data[2] = 0x10;
    }

    msg.id += idx;
    l_size = MotorCanDeviceRead(&msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *errorCode = *(uint16_t *)&msgRead.data[4];
    if(DRIVER_TYPE_COMPLEY == driveType)//compley������������32λ��ÿλ����һ������
    {
        if(0 != *(uint16_t *)&msgRead.data[6])
        {
            //rt_kprintf("compley id-%d high error: 0x%04x!\r\n", idx, *(uint16_t *)&msgRead.data[6]);
            //log_kprintf("compley id-%d high error: 0x%04x!\r\n", idx, *(uint16_t *)&msgRead.data[6]);
        }
    }

    return 0;
}
/#****************************************************************************
 ��������  : ��������״̬,ͨ��PDOģʽ��ȡ����������������TPDO1�ĵ�ַӳ��
 �������  : uint8_t idx       
             uint16_t *status  ���״̬
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
****************************************************************************#/
uint32_t MotorReadStatus(uint8_t idx, uint16_t *status)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x41, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;

    if(idx < M_MAX_ID)
    {
        if(gMotorIdToMotorNum[idx] < M_TOTAL_NUM)
        {
            driveType = gStMotorData[gMotorIdToMotorNum[idx]].driverType;
        }
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
    {
        g_stMsgReadPDOFeedback.id = 0x180 + idx;
        l_size = MotorCanDeviceRead(&g_stMsgReadPDOFeedback, &msgRead);
    }
    else//sdoģʽ
    {
        msg.id += idx;
        l_size = MotorCanDeviceRead(&msg, &msgRead);
    }

    if(0 == l_size)
    {
        return 1;
    }

    *status = *(uint16_t *)&msgRead.data[4];

    return 0;
}
/#****************************************************************************
 ��������  : ��˲ʱ������ƽ������,ͨ��PDOģʽ��ȡ����������������TPDO4�ĵ�ַӳ��
 �������  : uint8_t idx              
             int32_t *current         ˲ʱ����
             int32_t *currentAverage  ƽ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��13��
****************************************************************************#/
uint32_t MotorReadCurrentCmd(uint8_t idx, int32_t *current, int32_t *currentAverage)
{
    uint32_t l_size;
    CAN_msg msgRead;

    g_stMsgReadPDOFeedback.id = 0x480 + idx;
    l_size = MotorCanDeviceRead(&g_stMsgReadPDOFeedback, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }
    
    *current = *(int32_t *)&msgRead.data[0];
    *currentAverage = *(int32_t *)&msgRead.data[4];    

    return 0;
}
/#****************************************************************************
 ��������  : ��ƽ������,ͨ��SDOģʽ��ȡ, index-0x30d1, subindex-0x01
 �������  : uint8_t idx              
             int32_t *currentAverage  ƽ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��14��
****************************************************************************#/
uint32_t MotorReadAvarageCurrentCmd(uint8_t idx, int32_t *currentAverage)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0xd1, 0x30, 0x01}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;

    if(idx < M_MAX_ID)
    {
        if(gMotorIdToMotorNum[idx] < M_TOTAL_NUM)
        {
            driveType = gStMotorData[gMotorIdToMotorNum[idx]].driverType;
        }
    }

    msg.id += idx;

    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        //index
        msg.data[1] = 0x1c;
        msg.data[2] = 0x22;
        //subindex
        msg.data[3] = 0x00;
    }
    else if((DRIVER_TYPE_STAND == driveType)
        || (DRIVER_TYPE_INFRANOR == driveType))
    {
        //index
        msg.data[1] = 0x78;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x00;
    }
    
    l_size = MotorCanDeviceRead(&msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *currentAverage = *(int16_t *)&msgRead.data[4] * 10;//��λ0.01A������10ת����mA
    }
    else if((DRIVER_TYPE_STAND == driveType)
        || (DRIVER_TYPE_INFRANOR == driveType))
    {
        *currentAverage = *(int16_t *)&msgRead.data[4];
        *currentAverage = *currentAverage * gStMotorData[gMotorIdToMotorNum[idx]].normalCurrent / 1000;
    }
    else 
    {
        *currentAverage = *(int32_t *)&msgRead.data[4];//��λmA
    }

    return 0;
}
/#****************************************************************************
 ��������  : ��ȡ��ǰ�ٶ�
 �������  : uint8_t idx   
             int32_t *vel  �ٶ�
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��1��19��
****************************************************************************#/
uint32_t MotorReadVelocity(uint8_t idx, int32_t *vel)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x6c, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    int32_t counts = 6;

    if(idx < M_MAX_ID)
    {
        if(gMotorIdToMotorNum[idx] < M_TOTAL_NUM)
        {
            driveType = gStMotorData[gMotorIdToMotorNum[idx]].driverType;
            counts = gStMotorData[gMotorIdToMotorNum[idx]].counts;
        }
    }

    if(DRIVER_TYPE_INFRANOR == driveType)
    {
        //index
        msg.data[1] = 0x69;
        msg.data[2] = 0x30;
    }

    msg.id += idx;
    l_size = MotorCanDeviceRead(&msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *vel = *(int32_t *)&msgRead.data[4];

    //compley��������λ0.1counts/s
    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *vel = (*vel) * 6 / counts; //0.1counts/sתrpm
    }
    else if(DRIVER_TYPE_STAND == driveType)
    {
        *vel = (*vel) * 60 / counts; //counts/sתrpm 
    }

    return 0;
}*/

