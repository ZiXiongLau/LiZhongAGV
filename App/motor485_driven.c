/*----------------------------------------------*
 * 包含头文件                                   *
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
 功能描述  : modbus rtu crc16校验
 输入参数  : uint8_t *dataBuf  待校验数据
             uint16_t len      校验长度
 输出参数  : 校验结果
 作    者  : 刘鹏
 日    期  : 2020年12月22日
*****************************************************************************/
uint16_t MODBUS_RTUCrc16(uint8_t *dataBuf, uint16_t len)
{
    int i, k;

    uint16_t crc16 = 0xffff; //初始值
    
    for(i = 0; i < len; i++)
    {
        crc16 = dataBuf[i] ^ crc16;
        for(k = 0; k < 8; k++){  /*此处的8 -- 指每一个char类型又8bit，每bit都要处理*/
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
 功能描述  : 核对接收数据
 输入参数  : uint32_t revSize  接收数据长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月6日
*****************************************************************************/
HAL_StatusTypeDef Motor485CheckRevData(uint8_t *revBuf, uint32_t revSize)
{
    uint16_t crcValue;

    if(revSize >= 2)
    {
        //计算校验码
        crcValue = MODBUS_RTUCrc16(revBuf, revSize - 2);
        //核对crc
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
 功能描述  : 封装485发送与接受
 输入参数  : uint8_t *buffer  
             uint32_t size    
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年12月21日
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

    //计算校验码
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

        osDelay(1); //115200波特率延时1ms

        while(1)
        {            
            l_size += UsartDeviceRead(USART2_DEVICE, &g485RevBuf[l_size], revSize - l_size);
            delay_us(600);
            if(l_size >= revSize)
            {
                if((g485RevBuf[0] == sendBuffer[0]))// && (g485RevBuf[1] == sendBuffer[1]))  //核对站号
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
 功能描述  : 设定电机目标速度
 输入参数  : uint32_t motorNum  电机序号
             int32_t vel        目标速度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月6日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetVelocity(uint32_t motorNum, int32_t vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x6F;
    msg[3] = 0x00;
    //赋值
    vel = vel * gStMotorData[motorNum].counts / 3.6621f;
    msg[7] = (vel >> 8) & 0xff;
    msg[8] = vel & 0xff;
    msg[9]= (vel >> 24) & 0xff;
    msg[10]= (vel >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 设定电机控制模式
 输入参数  : uint32_t motorNum    电机序号
             HXC_MOTOR_MODE mode  控制模式
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年8月10日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetMotorContrlmode(uint32_t motorNum, HXC_MOTOR_MODE mode)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式
    
    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x00;
    msg[3] = 0x9f;
    //赋值
    msg[5] = (uint8_t)mode;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 设定电机点动
 输入参数  : uint32_t motorNum  电机序号
             ENUM_DIR moveDir   运动方向
             int32_t vel        速度rpm
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月31日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetMotorMove(uint32_t motorNum, ENUM_DIR moveDir, int32_t vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式
    uint16_t dataMsg = 0;
    
    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x00;
    msg[3] = 0xCA;
    //赋值
    if(DIR_CCW == moveDir)  //反向
    {
        dataMsg |= (1 << 15);
    }
    dataMsg |= (ABS_VALUE(vel) << 6);   //点动速度
    dataMsg |= (1 << 5);    //点动停止方式，1为立即停止，0为减速停止
    if(DIR_STOP != moveDir)
    {
        dataMsg |= 1;   //运行
    }
    msg[4] = (dataMsg >> 8) & 0xff;
    msg[5] = dataMsg & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 依据当前位置设定绝对位置
 输入参数  : uint32_t motorNum  电机序号
             int32_t pos        绝对位置
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年8月3日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetAbsPosAcordCurPos(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式
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
    //modbus索引
    msg[2] = 0x00;
    msg[3] = 0xDE;
    //赋值
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 设定电机绝对运行位置
 输入参数  : uint32_t motorNum  电机序号
             int32_t pos        位置
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月30日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetAbsPos(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x00;
    msg[3] = 0xD0;
    //赋值
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 设定电机绝对运行位置(实时)
 输入参数  : uint32_t motorNum  电机序号
             int32_t pos        位置
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月30日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetAbsPosRealtime(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x00;
    msg[3] = 0xE8;
    //赋值
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 设定电机实际位置
 输入参数  : uint32_t motorNum  电机序号
             int32_t pos        位置
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月31日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetActualPos(uint32_t motorNum, int32_t pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[15] = {0x00,0x10,0x00,0x00,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x00;
    msg[3] = 0xD2;
    //赋值
    msg[7] = (pos >> 8) & 0xff;
    msg[8] = pos & 0xff;
    msg[9]= (pos >> 24) & 0xff;
    msg[10]= (pos >> 16) & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 设定电机目标电流
 输入参数  : uint32_t motorNum  电机序号
             int32_t current    目标电流mA
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月6日
*****************************************************************************/
HAL_StatusTypeDef Motor485SetTargetCurrent(uint32_t motorNum, int32_t current)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //写多数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
    msg[2] = 0x3c;
    msg[3] = 0x00;
    //赋值
    current = current * 100 / gStMotorData[motorNum].normalCurrent * 10;
    msg[4] = (current >> 8) & 0xff;
    msg[5] = current & 0xff;

    lResult = Motor485DeviceRead(motorNum, msg, sizeof(msg), 8);

    return lResult;
}
/*****************************************************************************
 功能描述  : 读驱动器故障
 输入参数  : uint32_t motorNum    序号
             uint16_t* errorCode  电机故障
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月6日
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadErrorCode(uint32_t motorNum, uint16_t* errorCode)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00};   //读数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
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
 功能描述  : 读电机速度
 输入参数  : uint32_t motorNum    序号
             int32_t *vel         电机速度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月6日
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadVelocity(uint32_t motorNum, int32_t *vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00};   //读数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
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
 功能描述  : 读电机位置
 输入参数  : uint32_t motorNum  电机序号
             int32_t *pos       位置
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月30日
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadPos(uint32_t motorNum, int32_t *pos)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00};   //读数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
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
 功能描述  : 读电机故障
 输入参数  : uint32_t motorNum  电机序号
             uint16_t *error    故障      
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年7月30日
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadError(uint32_t motorNum, uint16_t *error)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00};   //读数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
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
 功能描述  : 读电机电流
 输入参数  : uint32_t motorNum    序号
             int32_t *vel         电机电流
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月6日
*****************************************************************************/
HAL_StatusTypeDef Motor485ReadCurrent(uint32_t motorNum, int16_t *current)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint8_t msg[10] = {0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00};   //读数据格式

    if(motorNum >= M_TOTAL_NUM)
    {
        return lResult;
    }
    //id
    msg[0] = gStMotorData[motorNum].idx;
    //modbus索引
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
 功能描述  : 电机485通信测试
 输入参数  : uint8_t motorNum  电机序号
             uint8_t* testData 测试数据
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年1月25日
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
 功能描述  : 设定目标电流
 输入参数  : uint8_t idx  
             int32_t current  目标电流
             rt_bool_t checkAck 是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年6月8日
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
            torque = current * 1000 / gStMotorData[gMotorIdToMotorNum[idx]].normalCurrent;  //额定电流的千分比，即额定力矩的千分比
            torque= Limit(torque, -1000, 1000);

            if(DRIVER_TYPE_EPOS == driveType)   //EPOS驱动器无法按固定8字节发送数据，只能有多少数据发多少
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
 功能描述  : 读取故障码
 输入参数  : uint8_t idx          
             uint16_t *errorCode  故障码
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
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
        //index 0x1002 compley驱动器错误码地址
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
    if(DRIVER_TYPE_COMPLEY == driveType)//compley驱动器错误码32位，每位代表一个错误
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
 功能描述  : 读驱动器状态,通过PDO模式读取，需先配置驱动器TPDO1的地址映射
 输入参数  : uint8_t idx       
             uint16_t *status  电机状态
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
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

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        g_stMsgReadPDOFeedback.id = 0x180 + idx;
        l_size = MotorCanDeviceRead(&g_stMsgReadPDOFeedback, &msgRead);
    }
    else//sdo模式
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
 功能描述  : 读瞬时电流及平均电流,通过PDO模式读取，需先配置驱动器TPDO4的地址映射
 输入参数  : uint8_t idx              
             int32_t *current         瞬时电流
             int32_t *currentAverage  平均电流
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月13日
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
 功能描述  : 读平均电流,通过SDO模式读取, index-0x30d1, subindex-0x01
 输入参数  : uint8_t idx              
             int32_t *currentAverage  平均电流
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月14日
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
        *currentAverage = *(int16_t *)&msgRead.data[4] * 10;//单位0.01A，乘以10转换成mA
    }
    else if((DRIVER_TYPE_STAND == driveType)
        || (DRIVER_TYPE_INFRANOR == driveType))
    {
        *currentAverage = *(int16_t *)&msgRead.data[4];
        *currentAverage = *currentAverage * gStMotorData[gMotorIdToMotorNum[idx]].normalCurrent / 1000;
    }
    else 
    {
        *currentAverage = *(int32_t *)&msgRead.data[4];//单位mA
    }

    return 0;
}
/#****************************************************************************
 功能描述  : 读取当前速度
 输入参数  : uint8_t idx   
             int32_t *vel  速度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年1月19日
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

    //compley驱动器单位0.1counts/s
    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *vel = (*vel) * 6 / counts; //0.1counts/s转rpm
    }
    else if(DRIVER_TYPE_STAND == driveType)
    {
        *vel = (*vel) * 60 / counts; //counts/s转rpm 
    }

    return 0;
}*/

