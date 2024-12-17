/******************************************************************************

                  版权所有 (C), 2013-2021,刘鹏

 ******************************************************************************
  文 件 名   : motor_driven.c
  版 本 号   : 初稿
  作    者   : 刘鹏
  生成日期   : 2018年11月16日
  最近修改   :
  功能描述   : 电机驱动
  函数列表   :
              ALIGN
              can_device_read
              can_device_write
              check_read_data
              MotorDeviceControlCmd
              MotorReadAvarageCurrentCmd
              MotorReadCurrentCmd
              MotorReadCurrentVelocityAndPosition
              MotorReadErrorCode
              MotorReadPosition
              MotorReadStatus
              MotorSetHomingMethod
              MotorSetOperationMode
              MotorSetTargetVelocityAndPosition
              SetEposNMTState
  修改历史   :
  1.日    期   : 2018年11月16日
    作    者   : 刘鹏
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include <preDef.h>
#include <motor_control.h>
#include <motor_driven.h>
#include <delay.h> 
#include <can.h>

/*****************************************************************************
 功能描述  : 通过id获取电机序号
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年9月16日
*****************************************************************************/
uint32_t GetMotorNumFromId(uint8_t idx)
{
    int i;
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if((DRIVER_TYPE_NONE != gStMotorData[i].driverType)
            && (idx == gStMotorData[i].idx))
        {
            return i;
        }
    }

    return M_TOTAL_NUM;
}
/*****************************************************************************
 功能描述  : 检查接收消息是否是所需要的消息
 输入参数  : CAN_msg* revMsg   接收到的消息
             CAN_msg* sendMsd  发送请求消息的命令
 输出参数  : int    为1表示是所需消息，为0表示其它消息
 作    者  : 刘鹏
 日    期  : 2018年11月11日
*****************************************************************************/
static int MotorCheckReadData(const CAN_msg* revMsg, const CAN_msg* sendMsd)
{
    uint8_t idx = revMsg->id & 0x7f;
    uint32_t motorNum;
    
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        //检查是否是驱动器启动消息
        if((revMsg->id & 0xff80) == 0x0700)
        {
            if((1 == revMsg->len) && ((0 == revMsg->data[0]) || (0x05 == revMsg->data[0])))
            {
                SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_BOOT_UP_FLAG);
            }
        }
        //检查是否是紧急消息
        else if((revMsg->id & 0xff80) == 0x0080)
        { 
            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_EMERGENCY_FLAG);
            gStMotorRevData[motorNum].eMergencyErrorCode = *(uint32_t *)&revMsg->data;
            //rt_kprintf("EMERGENGCY:0x%x\r\n", gStMotorRevData[motorNum].eMergencyErrorCode);
        }
        else if(((0x600 == (revMsg->id & 0x600)) || (0x580 == (revMsg->id & 0x580))) && (0x40 == (revMsg->data[0] & 0x60))) //sdo模式
        {
            if(0x20 == revMsg->data[2])
            {
                if(0x02 == revMsg->data[1]) //FDK电机温度数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_TMP_FLAG);
                            gStMotorRevData[motorNum].motorTmp = *(int16_t *)&revMsg->data[4]; //单位℃
                        }
                    }
                }
                else if(0x03 == revMsg->data[1]) //母线电压数据
                {
                    if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                        gStMotorRevData[motorNum].vol = *(int16_t *)&revMsg->data[4] / 10;//单位0.1v
                    }
                }
                else if(0x04 == revMsg->data[1])
                {
                    if(0x1d == revMsg->data[3]) //麒麟速度数据
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_SPEED_FLAG);
                            gStMotorRevData[motorNum].speed = *(int16_t *)&revMsg->data[4];
                        }
                        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                        {
                            gStMotorRevData[motorNum].speed = -gStMotorRevData[motorNum].speed;
                        }
                    }
                    else if(0x1e == revMsg->data[3]) //麒麟位置数据
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_POS_FLAG);
                            gStMotorRevData[motorNum].pos = *(int32_t *)&revMsg->data[4];
                        }
                    }
                }
                else if(0x05 == revMsg->data[1])
                {
                    if(0x10 == revMsg->data[3]) //电流数据
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4];
                            gStMotorRevData[motorNum].current = gStMotorRevData[motorNum].current 
                                * gStMotorData[motorNum].normalCurrent / 100; //额定电流的百分比
                            if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                            {
                                gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                            }
                        }
                    }
                    else if(0x16 == revMsg->data[3]) //母线电压数据
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(int16_t *)&revMsg->data[4];//单位v
                        }
                    }
                    else if(0x18 == revMsg->data[3])    //麒麟温度数据
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp = (*(int16_t *)&revMsg->data[4]) / 10; //单位℃
                        }
                    }
                }
                else if(0x0b == revMsg->data[1])
                {
                    if(0x15 == revMsg->data[3]) //立迈胜母线电压数据
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                        gStMotorRevData[motorNum].vol = *(uint16_t *)&revMsg->data[4] / 10;//单位0.1v除10转换成v
                    }
                    else if(0x16 == revMsg->data[3])    //立迈胜温度数据
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                        gStMotorRevData[motorNum].tmp = (*(int16_t *)&revMsg->data[4]); //单位℃
                    }
                }
            }
            else if(0x22 == revMsg->data[2])
            {
                if(0x01 == revMsg->data[1]) //compley电压数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(int16_t *)&revMsg->data[4] / 10;//单位0.1V，除10转换成v
                        }
                    }
                }
                else if(0x02 == revMsg->data[1]) //compley温度数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp= *(int16_t *)&revMsg->data[4]; //单位℃
                        }
                    }
                }
                else if(0x1c == revMsg->data[1]) //compley电流数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4] * 10;//单位0.01A，乘以10转换成mA
                            if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                            {
                                gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                            }
                        }
                    }
                }
                else if(0xa2 == revMsg->data[1]) //elmo温度数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if((DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                            || (DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType))
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp= *(int16_t *)&revMsg->data[4]; //单位℃
                        }
                    }
                }
            }
            else if(0x30 == revMsg->data[2])
            {
                if(0x0d == revMsg->data[1]) //母线电压数据
                {
                    if(0x06 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(float *)&revMsg->data[4];//单位v
                        }
                    }
                }
                else if(0xd1 == revMsg->data[1]) //非compley驱动器、非标准驱动器、非DRIVER_TYPE_INFRANOR驱动器电流数据
                {
                    if(0x01 == revMsg->data[3])
                    {
                        if((DRIVER_TYPE_COMPLEY != gStMotorData[motorNum].driverType)
                            && (DRIVER_TYPE_STAND != gStMotorData[motorNum].driverType))
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int32_t *)&revMsg->data[4];//单位mA
                            if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                            {
                                gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                            }
                        }
                    }
                }
            }
            else if(0x60 == revMsg->data[2])     
            {
                if(0x01 == revMsg->data[1])     //谱思驱动器电机控制器状态信息
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
                        gStMotorRevData[motorNum].ctrlStatus = revMsg->data[4];
                    }
                }
                if(0x2e == revMsg->data[1])     //谱思驱动器电机状态信息
                {
                    if(0x02 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                        gStMotorRevData[motorNum].status = *(uint16_t *)&revMsg->data[4];
                    }
                }
                else if(0x41 == revMsg->data[1])     //电机状态信息
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                        gStMotorRevData[motorNum].status = *(uint16_t *)&revMsg->data[4];
                    }
                }
                else if(0x63 == revMsg->data[1])     //位置数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_POS_FLAG);
                        gStMotorRevData[motorNum].pos = *(int32_t *)&revMsg->data[4];
                    }
                }
                else if(0x6c == revMsg->data[1])    //转速数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_SPEED_FLAG);
                        gStMotorRevData[motorNum].speed = *(int32_t *)&revMsg->data[4];
                        //compley驱动器单位0.1counts/s，需转换成rpm
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            gStMotorRevData[motorNum].speed = 
                                gStMotorRevData[motorNum].speed * 6 / gStMotorData[motorNum].counts;
                        }
                        //标准驱动器单位counts/s，需转换成rpm
                        else if(DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                        {
                            gStMotorRevData[motorNum].speed = 
                                gStMotorRevData[motorNum].speed * 60 / gStMotorData[motorNum].counts;
                        }
                        else if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            /*gStMotorRevData[motorNum].speed =
                                (gStMotorRevData[motorNum].speed * 1875 / gStMotorData[motorNum].counts) >> 9;//根据编码器的数值计算出RPM，所以需要根据公式反向计算*/
                        }
                        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                        {
                            gStMotorRevData[motorNum].speed = -gStMotorRevData[motorNum].speed;
                        }
                    }
                }
                else if(0x78 == revMsg->data[1])    //标准驱动器电流数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if((DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                            || (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType))
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG); 
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4];
                            gStMotorRevData[motorNum].current = gStMotorRevData[motorNum].current 
                                * gStMotorData[motorNum].normalCurrent / 1000; //额定电流的千分比
                        }
                        /*else if(DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG); 
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4] * 10;//单位0.01A，乘以10转换成mA
                        }*/
                        else if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG); 
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4] * 100;//单位0.1A，乘以100转换成mA
                        }
                        else if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4];
                            gStMotorRevData[motorNum].current = (gStMotorRevData[motorNum].current * gStMotorData[motorNum].limitCurrent) >> 11;//单位A，乘以1000转换成mA
                        }
                        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                        {
                            gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                        }
                    }
                }
                else if(0x79 == revMsg->data[1])    //母线电压数据
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(uint32_t *)&revMsg->data[4] / 1000;//单位mv，除以1000转换成V
                        }
                    }
                }
                else if(0xf7 == revMsg->data[1])    //步科驱动器温度数据
                {
                    if(0x0b == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp= *(int16_t *)&revMsg->data[4]; //单位℃
                        }
                    }
                }
            }
        }
        else if(DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)//pdo模式
        {
            if(0x180 == (revMsg->id & 0x180))   //电机状态信息
            {
                SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                gStMotorRevData[motorNum].status = *(uint16_t *)&revMsg->data[4];
            }
        }
        
    }

    if(NULL != sendMsd)
    {
        //检查是否是PDO消息
        if((0x180 == (sendMsd->id & 0x180)) || (0x280 == (sendMsd->id & 0x280))
            || (0x380 == (sendMsd->id & 0x380)) || (0x480 == (sendMsd->id & 0x480)))
        {
            if(revMsg->id == sendMsd->id)//id相同
            {
                return 1;
            }
        }
        //检查是否是SDO消息
        if((0x600 == (sendMsd->id & 0x600)) || (0x580 == (sendMsd->id & 0x580)))
        {
            if((revMsg->id & 0x07F) == (sendMsd->id & 0x07F))//id相同
            {
                //读和读响应对应，写和写响应对应
                if(((0x40 == (sendMsd->data[0] & 0x60)) && (0x40 == (revMsg->data[0] & 0x60)))
                    || ((0x20 == (sendMsd->data[0] & 0x60)) && (0x60 == (revMsg->data[0] & 0x60))))
                {
                    //index、subindex是否相同
                    if((revMsg->data[1] == sendMsd->data[1])
                        && (revMsg->data[2] == sendMsd->data[2])
                        && (revMsg->data[3] == sendMsd->data[3]))
                    {
                        return 1;
                    }
                }
            }
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 电机can发送
 输入参数  : const CAN_msg *buffer
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年10月20日
*****************************************************************************/
static HAL_StatusTypeDef MotorCanDeviceWrite(uint32_t motorNum, const CAN_msg *buffer)
{
    HAL_StatusTypeDef lResult;
    
    if(motorNum < M_TOTAL_NUM)
    {
        while(1)
    	{
            /* Check for the Timeout */
            if((HAL_GetTick() - gStMotorRevData[motorNum].lastSendTime) > 1)  //同一电机连续发送命令报文的间隔时间须＞1ms，防止电机对命令执行响应不过来
            {
                break;
            }
            
            delay_us(100);
    	}
    }
    lResult = CanDeviceWrite(CAN2_DEVICE, buffer, CAN_WRITE_TIMEOUT_MS);

    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRevData[motorNum].lastSendTime = HAL_GetTick();
    }

    return lResult;
}
/*****************************************************************************
 功能描述  : 封装can1数据接收，增加等待超时
 输入参数  : const void *sendBuffer  发送请求命令
             uint32_t sendSize      发送请求命令大小
             void *revBuffer         接收数据目标地址
             uint32_t revSize       指定接收数据的大小
 输出参数  : 实际成功接收的大小
 作    者  : 刘鹏
 日    期  : 2018年11月11日
*****************************************************************************/
static uint32_t MotorCanDeviceRead(uint32_t motorNum, const CAN_msg *sendBuffer, CAN_msg *revBuffer)
{
    HAL_StatusTypeDef lResult;
	uint32_t l_size = 0;
    uint32_t tickstart;

    lResult = MotorCanDeviceWrite(motorNum, sendBuffer);

    if(motorNum < M_TOTAL_NUM)
    {
        gStMotorRevData[motorNum].lastSendTime = 0;
    }

    /* Get tick */ 
    tickstart = HAL_GetTick();

    if(HAL_OK == lResult)//已成功放入发送邮箱
	{
	    while(1)
    	{
    		lResult = CanDeviceRead(CAN2_DEVICE, revBuffer, CAN_READ_TIMEOUT_MS);        
    		
    		if(HAL_OK == lResult)//收到了数据
    		{
                if(0 != MotorCheckReadData((CAN_msg*)revBuffer, (CAN_msg*)sendBuffer))//检查是否是所需数据
                {
                    l_size = 1;
        			break;
                }
                else if((HAL_GetTick() - tickstart ) > CAN_READ_TIMEOUT_MS)
                {
                    break;
                }
    		}
            else//超时
            {
                break;
            }
    	}            
	}
    
	return l_size;
}
void MotorQueryInfo(void)
{
	uint8_t i;
	for(i = LEFT_MOTOR; i < TOTAL_MOTOR_NUM; i++)
    {
		MotorSendReadPosCmd(i);//发送位置信息读取指令
//		MotorSendReadVelocity(i);//发送速度信息读取指令
	}
}
/*****************************************************************************
 功能描述  : 电机读取线程，异步接收数据
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月6日
*****************************************************************************/
void MotorReadProcess(void)
{
    HAL_StatusTypeDef lResult;
    CAN_msg revBuffer;
    int cnt = 0;
        
    while(cnt++ < CAN_RX_MSG_MAX_NUM)
	{
		lResult = CanDeviceRead(CAN2_DEVICE, &revBuffer, 0);        
		
		if(HAL_OK == lResult)//收到了数据
		{
            MotorCheckReadData(&revBuffer, NULL);
		}
        else//超时或未有数据
        {
            break;
        }
	} 
}
/*****************************************************************************
 功能描述  : 设置NMT服务状态
 输入参数  : uint8_t cs   命令字  初始化状态、允许PDO传输
             uint8_t idx  指定驱动器id，如果为0表示同时设置所有驱动器
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t SetEposNMTState(uint8_t cs, uint8_t idx)
{
    CAN_msg msg = {0 , {0}, 2, 1, STANDARD_FORMAT, DATA_FRAME};

    msg.data[0] = cs;
    msg.data[1] = idx;
    CanDeviceWrite(CAN2_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}
/*****************************************************************************
 功能描述  : 不同驱动器模式值转换
 输入参数  : DRIVER_TYPE driveType    驱动器类型
             int8_t* i8OperationMode  输入模式
             int8_t inverseFlag       逆向转换标志
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月27日
*****************************************************************************/
void MotorOperationModeExchange(DRIVER_TYPE driveType, int8_t* i8OperationMode, int8_t inverseFlag)
{
    if(DRIVER_TYPE_PUSI == driveType)
    {
        if(inverseFlag)
        {
            if(5 == *i8OperationMode)
            {
                *i8OperationMode = MOTOR_OPERATION_MODE_PVM;
            }
            else if(4 == *i8OperationMode)
            {
                 *i8OperationMode = MOTOR_OPERATION_MODE_PPM;
            }
            else if(2 == *i8OperationMode)
            {
                 *i8OperationMode = MOTOR_OPERATION_MODE_PTM;
            }
        }
        else
        {
            if(MOTOR_OPERATION_MODE_PVM == *i8OperationMode)
            {
                *i8OperationMode = 5;
            }
            else if(MOTOR_OPERATION_MODE_PPM == *i8OperationMode)
            {
                 *i8OperationMode = 4;
            }
            else if(MOTOR_OPERATION_MODE_PTM == *i8OperationMode)
            {
                 *i8OperationMode = 2;
            }
        }
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        if(inverseFlag)
        {
            if(1 == *i8OperationMode)
            {
                *i8OperationMode = MOTOR_OPERATION_MODE_PVM;
            }
            else if(10 == *i8OperationMode)
            {
                 *i8OperationMode = MOTOR_OPERATION_MODE_PPM;
            }
            else if(2 == *i8OperationMode)
            {
                 *i8OperationMode = MOTOR_OPERATION_MODE_PTM;
            }
        }
        else
        {
            if(MOTOR_OPERATION_MODE_PVM == *i8OperationMode)
            {
                *i8OperationMode = 1;
            }
            else if(MOTOR_OPERATION_MODE_PPM == *i8OperationMode)
            {
                 *i8OperationMode = 10;
            }
            else if(MOTOR_OPERATION_MODE_PTM == *i8OperationMode)
            {
                 *i8OperationMode = 2;
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 设置电机运行模式
 输入参数  : uint8_t idx
             int8_t i8OperationMode  运行模式：周期性同步速度模式、文件配置速度模式、回零模式
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t MotorSetOperationMode(uint8_t idx, int8_t i8OperationMode)
{
    CAN_msg msg = {0 , {0}, 1, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
        
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        msg.id = 0x200 + idx;
        msg.data[0] = i8OperationMode;
    }
    else//sdo模式
    {
        //id
        msg.id = SDO_CLIENT_ID_BASE + idx;        
        //download
        msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD1;
        
        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x05;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x00;
        }
        else if(DRIVER_TYPE_QILING == driveType)
        {
            msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
            //index
            msg.data[1] = 0x04;
            msg.data[2] = 0x20;
            //subindex
            msg.data[3] = 0x01;
        }
        else
        {
            //index
            msg.data[1] = 0x60;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x00;
        }
        
        //data
        MotorOperationModeExchange(driveType, &i8OperationMode, 0);
        msg.data[4] = i8OperationMode;
        //len
        msg.len = 8;
    }
	  
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 获取电机运行模式
 输入参数  : uint8_t idx
             int8_t* i8OperationMode  运行模式
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年12月1日
*****************************************************************************/
uint32_t MotorGetOperationMode(uint8_t idx, int8_t* i8OperationMode)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x61, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_PUSI == driveType)
    {
        //index
        msg.data[1] = 0x05;
        msg.data[2] = 0x60;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x01;
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        //index
        msg.data[1] = 0x60;
        msg.data[2] = 0x60;
    }

    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *i8OperationMode = *(int8_t *)&msgRead.data[4];

    MotorOperationModeExchange(driveType, i8OperationMode, 1);

    return 0;
}
/*****************************************************************************
 功能描述  : 不同驱动器控制字切换
 输入参数  : DRIVER_TYPE driveType    驱动器类型
             uint16_t* usControlMode  控制字
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月27日
*****************************************************************************/
void MotorControlCmdExchange(DRIVER_TYPE driveType, uint16_t* usControlMode)
{
    if(DRIVER_TYPE_PUSI == driveType)
    {
        if(DEVICE_CTROL_ENABLE_OPERATION1 == *usControlMode)
        {
            *usControlMode = 0x7f;
        }
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        if(DEVICE_CTROL_ENABLE_OPERATION1 == *usControlMode)
        {
            *usControlMode = 0x200;
        }
    }
}
/*****************************************************************************
 功能描述  : 电机设备控制命令
 输入参数  : uint16_t usControlMode  控制模式：上电保持、运动、快速停止、断电、故障复位
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月13日
*****************************************************************************/
uint32_t MotorDeviceControlCmd(uint8_t idx, uint16_t usControlMode)
{
    CAN_msg msg = {0 , {0}, 2, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        msg.id = 0x300 + idx;
        (*(uint16_t*)&msg.data[0]) = usControlMode;
    }
    else//sdo模式
    {
        //id
        msg.id = SDO_CLIENT_ID_BASE + idx;        
        //download
        msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2e;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x01;
        }
        else
        {
            //index
            msg.data[1] = 0x40;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x00;
        }
        //data
        MotorControlCmdExchange(driveType, &usControlMode);
        (*(uint16_t*)&msg.data[4]) = usControlMode;
        //len
        msg.len = 8;
    }
	  
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 电机设备控制命令(带反馈)
 输入参数  : uint16_t usControlMode  控制模式：上电保持、运动、快速停止、断电、故障复位
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年08月08日
*****************************************************************************/
uint32_t MotorDeviceControlCmdWithJudge(uint8_t idx, uint16_t usControlMode)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD2, 0x40, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    msg.id += idx;

    if(DRIVER_TYPE_PUSI == driveType)
    {
        //index
        msg.data[1] = 0x2e;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x01;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x1c;
    }

    //data
    MotorControlCmdExchange(driveType, &usControlMode);

    (*(uint16_t*)&msg.data[4]) = usControlMode;
    
    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设定电机目标速度
 输入参数  : uint8_t idx
             int32_t vel         目标速度
             rt_bool_t checkAck     是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年11月15日
*****************************************************************************/
uint32_t MotorSetTargetVelocity(uint8_t idx, int32_t vel, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0xff, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    int32_t getVel;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    int32_t counts = 6;
    uint32_t motorNum;

            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
        counts = gStMotorData[motorNum].counts;
        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
        {
            vel = -vel;
        }
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        msg.id = 0x400 + idx;
        (*(int32_t*)&msg.data[0]) = vel;
        (*(int32_t*)&msg.data[4]) = 0;
    	  
        MotorCanDeviceWrite(motorNum, &msg);

        if(RT_TRUE == checkAck)
        {
            if(0 == MotorGetTargetVelocity(idx, &getVel, driveType, counts))
            {
                getVel = (getVel >= vel) ? (getVel - vel) : (vel - getVel);
                if(getVel > 2)
                {
                    return 1;
                }
            }
            else
            {
                return 1;
            }
        }
    }
    else//sdo模式
    {
        msg.id += idx;
        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2e;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x03;
        }
        //compley驱动器单位0.1counts/s
        if(DRIVER_TYPE_COMPLEY == driveType)
        {
            (*(int32_t*)&msg.data[4]) = vel * counts / 6;//rpm转0.1counts/s
        }
        //INFRANOR驱动器单位counts/s
        else if((DRIVER_TYPE_NIMOTION == driveType) || (DRIVER_TYPE_PUSI == driveType))
        {
            (*(int32_t*)&msg.data[4]) = vel * counts / 60;//rpm转counts/s
        }
        else if(DRIVER_TYPE_STAND == driveType)
        {
            (*(int32_t*)&msg.data[4]) = vel * counts / 60;//rpm转counts/s
        }
        else if(DRIVER_TYPE_KINCO_CAN == driveType)
        {
			//为了解决当vel足够大时，溢出的问题，uint64->uint32截断时可能会造成精度的丢失
        	(*(int32_t*)&msg.data[4]) = (int64_t)vel * 512 * counts / 1875;
//            (*(int32_t*)&msg.data[4]) = vel * 512 * counts / 1875;//rpm转指定单位
        }
        //单位转每分
        else
        {
            (*(int32_t*)&msg.data[4]) = vel;
        }
    
        if(RT_TRUE == checkAck)
        {
            if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
            {
                return 1;
            }
        }
        else
        {
            MotorCanDeviceWrite(motorNum, &msg);
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 获取设定的目标速度
 输入参数  : uint8_t idx
             int32_t* vel  
             DRIVER_TYPE driveType  驱动器类型
             int32_t counts      电机一圈脉冲数
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年12月13日
*****************************************************************************/
uint32_t MotorGetTargetVelocity(uint8_t idx, int32_t* vel, DRIVER_TYPE driveType, int32_t counts)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0xff, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *vel = *(int32_t *)&msgRead.data[4];
    //compley驱动器单位0.1counts/s，则转成转每分
    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *vel = (*vel) * 6 / counts; //0.1counts/s转rpm
    }
    //INFRANOR驱动器单位counts/s，则转成转每分
    else if(DRIVER_TYPE_NIMOTION == driveType)
    {
        *vel = (*vel) * 60 / counts; //counts/s转rpm
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设定目标位置
 输入参数  : uint8_t idx
             int32_t pos  目标位置
             rt_bool_t checkAck 是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年1月17日
*****************************************************************************/
uint32_t MotorSetTargetPosition(uint8_t idx, int32_t pos, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x7a, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    int32_t getPos;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        msg.id = 0x400 + idx;
        (*(int32_t*)&msg.data[0]) = 0;
        (*(int32_t*)&msg.data[4]) = pos;
    	  
        MotorCanDeviceWrite(motorNum, &msg);

        if(RT_TRUE == checkAck)
        {
            if(0 == MotorGetTargetPosition(idx, &getPos))
            {
                if(getPos != pos)
                {
                    return 1;
                }
            }
            else
            {
                return 1;
            }
        }
    }
    else//sdo模式
    {
        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2e;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x04;
        }
        else if(DRIVER_TYPE_QILING == driveType)
        {
            //index
            msg.data[1] = 0x04;
            msg.data[2] = 0x20;
            //subindex
            msg.data[3] = 0x13;
        }
        
        msg.id += idx;
        (*(int32_t*)&msg.data[4]) = pos;
    
        if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
        {
            return 1;
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 获取设定的目标位置
 输入参数  : uint8_t idx
             int32_t* pos  
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年12月13日
*****************************************************************************/
uint32_t MotorGetTargetPosition(uint8_t idx, int32_t* pos)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x7a, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *pos = *(int32_t *)&msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 设定目标电流
 输入参数  : uint8_t idx
             int32_t current  目标电流
             rt_bool_t checkAck 是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年6月8日
*****************************************************************************/
uint32_t MotorSetTargetCurrent(uint8_t idx, int32_t current, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD2, 0x71, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    int16_t torque;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;  
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        if(DEBUG_DATA_TYPE_86)
        {
            rt_kprintf("M%d c:%d,%d,%d.\r\n", motorNum, current, checkAck, gStMotorRunState[motorNum].operModeSetStep);
        }
        driveType = gStMotorData[motorNum].driverType;
        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
        {
            current = -current;
        }

        if(DRIVER_TYPE_EPOS == driveType)   //EPOS驱动器无法按固定8字节发送数据，只能有多少数据发多少
        {
            msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
            msg.len = 6;
        }

        msg.id += idx;
        if(DRIVER_TYPE_COMPLEY == driveType)
        {
            //index
            msg.data[1] = 0x40;
            msg.data[2] = 0x23;
            //current
            (*(int16_t*)&msg.data[4]) = current / 10; //单位0.01A
        }
        else if(DRIVER_TYPE_FDK == driveType)
        {
            //current
            (*(int16_t*)&msg.data[4]) = current / 100;//单位0.1A
        }
        else if(DRIVER_TYPE_KINCO_CAN == driveType)
        {
            //index
            msg.data[1] = 0xf6;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x08;
            current = current << 11;
            if(0 == gStMotorData[motorNum].limitCurrent)
            {
                gStMotorData[motorNum].limitCurrent = 120000;
            }
            (*(int16_t*)&msg.data[4]) = current / gStMotorData[motorNum].limitCurrent;
        }
        else
        {
            torque = current * 1000 / gStMotorData[motorNum].normalCurrent;  //额定电流的千分比，即额定力矩的千分比
            torque= Limit(torque, -4000, 4000);
            (*(int16_t*)&msg.data[4]) = torque;
        }

        if(RT_TRUE == checkAck)
        {
            if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
            {
                return 1;
            }
        }
        else
        {
            MotorCanDeviceWrite(motorNum, &msg);
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设定轮廓速度
 输入参数  : uint8_t idx
             int32_t vel     轮廓速度rpm
             rt_bool_t checkAck 是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年6月15日
*****************************************************************************/
uint32_t MotorSetProfileVelocity(uint8_t idx, uint32_t vel, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x81, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    int32_t counts = 6;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
        counts = gStMotorData[motorNum].counts; 

        msg.id += idx;

        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2e;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x03;
        }
        else if(DRIVER_TYPE_QILING == driveType)
        {
            msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
            //index
            msg.data[1] = 0x04;
            msg.data[2] = 0x20;
            //subindex
            msg.data[3] = 0x15;
        }

        if(DRIVER_TYPE_QILING == driveType)
        {
            (*(uint16_t*)&msg.data[4]) = vel;
        }
        else
        {
            (*(uint32_t*)&msg.data[4]) = vel * counts / 60;  //标准驱动器由rpm转换成couts/s
        }

        if(RT_TRUE == checkAck)
        {
            if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
            {
                return 1;
            }
        }
        else
        {
            MotorCanDeviceWrite(motorNum, &msg);
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设定轮廓加速度
 输入参数  : uint8_t idx
             int32_t acc     轮廓加速度rpm/s2
             rt_bool_t checkAck 是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年6月15日
*****************************************************************************/
uint32_t MotorSetProfileAcc(uint8_t idx, uint32_t acc, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x83, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    int32_t counts = 6;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
        counts = gStMotorData[motorNum].counts; 

        msg.id += idx;

        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2d;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x01;
        }
        else if(DRIVER_TYPE_QILING == driveType)
        {
            msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
            //index
            msg.data[1] = 0x04;
            msg.data[2] = 0x20;
            //subindex
            msg.data[3] = 0x11;
        }

        if(DRIVER_TYPE_QILING == driveType)
        {
            (*(uint16_t*)&msg.data[4]) = acc;   //加速时间ms
        }
        else
        {
            (*(uint32_t*)&msg.data[4]) = counts / 60 * acc;  //标准驱动器由rpm转换成couts/s
        }

        if(RT_TRUE == checkAck)
        {
            if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
            {
                return 1;
            }
        }
        else
        {
            MotorCanDeviceWrite(motorNum, &msg);
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设定轮廓减速度
 输入参数  : uint8_t idx
             int32_t dec     轮廓减速度rpm/s2
             rt_bool_t checkAck 是否检查响应
 输出参数  : uint32_t    0表示成功，非0表示故障
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年6月15日
*****************************************************************************/
uint32_t MotorSetProfileDec(uint8_t idx, uint32_t dec, rt_bool_t checkAck)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x84, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    int32_t counts = 6;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
        counts = gStMotorData[motorNum].counts; 

        msg.id += idx;
        
        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2d;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x02;
        }
        else if(DRIVER_TYPE_QILING == driveType)
        {
            msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;
            //index
            msg.data[1] = 0x04;
            msg.data[2] = 0x20;
            //subindex
            msg.data[3] = 0x12;
        }

        if(DRIVER_TYPE_QILING == driveType)
        {
            (*(uint16_t*)&msg.data[4]) = dec;   //减速时间ms
        }
        else
        {
            (*(uint32_t*)&msg.data[4]) = counts / 60 * dec;  //标准驱动器由rpm转换成couts/s
        }

        if(RT_TRUE == checkAck)
        {
            if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
            {
                return 1;
            }
        }
        else
        {
            MotorCanDeviceWrite(motorNum, &msg);
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设定归零模式方案
 输入参数  : uint8_t idx
             int8_t method  归零模式方案
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t MotorSetHomingMethod(uint8_t idx, int8_t method)
{
    CAN_msg msg = {0 , {0}, 1, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        msg.id = 0x500 + idx;
        msg.data[0] = method;
    }
    else//sdo模式
    {
        //id
        msg.id = SDO_CLIENT_ID_BASE + idx;        
        //download
        msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD1;
        //index
        msg.data[1] = 0x98;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x00;
        //data
        msg.data[4] = method;
        //len
        msg.len = 8;
    }
	  
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 获取归零模式方案
 输入参数  : uint8_t idx
             int8_t* method  归零模式方案
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年12月12日
*****************************************************************************/
uint32_t MotorGetHomingMethod(uint8_t idx, int8_t* method)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x98, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint32_t motorNum;
                
    motorNum = GetMotorNumFromId(idx);

    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *method = *(int8_t *)&msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 设定归零位置
 输入参数  : uint8_t idx
             int32_t pos   归零模式位置
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年12月28日
*****************************************************************************/
uint32_t MotorSetHomingPosition(uint8_t idx, int32_t pos)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0xb0, 0x30, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_STAND == driveType)
    {
        //index
        msg.data[1] = 0x7c;
        msg.data[2] = 0x60;
    }
    
    msg.id += idx;
    (*(int32_t*)&msg.data[4]) = pos;

    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }
    //CanDeviceWrite(CAN1_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}
/*****************************************************************************
 功能描述  : 设定堵转找寻原点时的检测转矩
 输入参数  : uint8_t idx
             uint16_t torq   最大转矩的百分比(单位: 0.1%)
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 田忠
 日    期  : 2022年11月16日
*****************************************************************************/
uint32_t MotorSetHomingStallTorq(uint8_t idx, uint16_t torq)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD2, 0x07, 0x20, 0x13}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    //DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    /*if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }*/

    
    msg.id += idx;
    (*(uint16_t*)&msg.data[4]) = torq;

    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }
    //CanDeviceWrite(CAN1_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}
/*****************************************************************************
 功能描述  : 设定堵转找寻原点时的检测时间
 输入参数  : uint8_t idx
             uint16_t time   (单位: ms)
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 田忠
 日    期  : 2022年11月16日
*****************************************************************************/
uint32_t MotorSetHomingStallTime(uint8_t idx, uint16_t time)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD2, 0x07, 0x20, 0x15}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    //DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    /*if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }*/

    
    msg.id += idx;
    (*(uint16_t*)&msg.data[4]) = time;

    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }
    //CanDeviceWrite(CAN1_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}


/*****************************************************************************
 功能描述  : 设定找原点速度
 输入参数  : uint8_t idx
             int32_t vel   归零模式速度(单位: 用户单位/s),停机生效
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 田忠
 日    期  : 2022年11月16日
*****************************************************************************/
uint32_t MotorSetHomingVelocity(uint8_t idx, int32_t vel)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x99, 0x60, 0x02}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    //DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    /*if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }*/

    
    msg.id += idx;
    (*(int32_t*)&msg.data[4]) = vel;

    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }
    //CanDeviceWrite(CAN1_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}
/*****************************************************************************
 功能描述  : 设定原点回归加速度
 输入参数  : uint8_t idx
             uint32_t acc   归零模式加速度(单位: 用户单位/s^2)
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 田忠
 日    期  : 2022年11月16日
*****************************************************************************/
uint32_t MotorSetHomingAcc(uint8_t idx, uint32_t acc)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x9A, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    //DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    /*if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }*/
    
    msg.id += idx;
    (*(int32_t*)&msg.data[4]) = acc;

    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }
    //CanDeviceWrite(CAN1_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}

/*****************************************************************************
 功能描述  : 设定CANopen中止连接操作码
 输入参数  : uint8_t idx
             uint32_t optionCode  0:No action
                                  1:Set Fault Signal
                                  2:Device control command:Disable voltage
                                  3:Device control command:Quick stop
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 田忠
 日    期  : 2022年12月2日
*****************************************************************************/
uint32_t MotorSetAbortConnectionOptionCode(uint8_t idx, int16_t optionCode)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD2, 0x07, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    //DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
    uint8_t errorCnt = 0;
            
    motorNum = GetMotorNumFromId(idx);

    /*if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }*/
    
    msg.id += idx;
    (*(int16_t*)&msg.data[4]) = optionCode;

    while (1)
    {
        if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
        {
            errorCnt++;
            if (errorCnt >= 3)
            {
                rt_kprintf("M%d set behavior fail.\r\n", motorNum);
                return 1;
            }
        }
        else 
        {
            break;
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 读取故障码
 输入参数  : uint8_t idx
             uint16_t *errorCode  故障码
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t MotorReadErrorCode(uint8_t idx, uint16_t *errorCode)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x3F, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        //index 0x1002 compley驱动器错误码地址
        msg.data[1] = 0x83;
        msg.data[2] = 0x21;
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        msg.data[1] = 0x01;
        msg.data[2] = 0x26;
    }

    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *errorCode = *(uint16_t *)&msgRead.data[4];
    if(DRIVER_TYPE_COMPLEY == driveType)//compley驱动器错误码32位，每位代表一个错误
    {
        if(0 != *(uint16_t *)&msgRead.data[6])
        {
            rt_kprintf("compley id-%d high error: 0x%04x!\r\n", idx, *(uint16_t *)&msgRead.data[6]);
        }
    }

    return 0;
}

CAN_msg g_stMsgReadPDOFeedback = {0 , {0}, 0, 1, STANDARD_FORMAT, REMOTE_FRAME}; //读取PDO数据的远程帧，id需指定
/*****************************************************************************
 功能描述  : 读驱动器状态,通过PDO模式读取，需先配置驱动器TPDO1的地址映射
 输入参数  : uint8_t idx
             uint16_t *status  电机状态
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t MotorReadStatus(uint8_t idx, uint16_t *status)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x41, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_PUSI == driveType)
    {
        //index
        msg.data[1] = 0x00;
        msg.data[2] = 0x60;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x08;
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        g_stMsgReadPDOFeedback.id = 0x180 + idx;
        l_size = MotorCanDeviceRead(motorNum, &g_stMsgReadPDOFeedback, &msgRead);
    }
    else//sdo模式
    {
        msg.id += idx;
        l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);
    }

    if(0 == l_size)
    {
        return 1;
    }

    *status = *(uint16_t *)&msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 读风得控驱动器工作状态
 输入参数  : uint8_t idx
             uint16_t *status  电机状态
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t MotorReadFDKWorkStatus(uint8_t idx, uint16_t *status)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x01, 0x20, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
            
    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *status = *(uint16_t *)&msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 读电机使能状态
 输入参数  : uint8_t idx
             uint16_t *enableStatus  使能状态
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月27日
*****************************************************************************/
uint32_t MotorReadEnableStatus(uint8_t idx, uint16_t *enableStatus)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x0e, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *enableStatus = msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 使能电机
 输入参数  : uint8_t idx
             uint8_t enableStatus  使能状态
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月27日
*****************************************************************************/
uint32_t MotorSetMotorEnableStatus(uint8_t idx, uint16_t enableStatus)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD1, 0x0e, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    //id
    msg.id = SDO_CLIENT_ID_BASE + idx;        
    //data
    msg.data[4] = enableStatus;
	  
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 设定控制器状态
 输入参数  : uint8_t idx
             uint8_t controlStatus  控制器状态
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月10日
*****************************************************************************/
uint32_t MotorSetMotorControlStatus(uint8_t idx, uint8_t controlStatus)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD1, 0x01, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    //id
    msg.id = SDO_CLIENT_ID_BASE + idx;        
    //data
    msg.data[4] = controlStatus;
	  
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 读瞬时电流及平均电流,通过PDO模式读取，需先配置驱动器TPDO4的地址映射
 输入参数  : uint8_t idx
             int32_t *current         瞬时电流
             int32_t *currentAverage  平均电流
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月13日
*****************************************************************************/
uint32_t MotorReadCurrentCmd(uint8_t idx, int32_t *current, int32_t *currentAverage)
{
    uint32_t l_size;
    CAN_msg msgRead;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    g_stMsgReadPDOFeedback.id = 0x480 + idx;
    l_size = MotorCanDeviceRead(motorNum, &g_stMsgReadPDOFeedback, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }
    
    *current = *(int32_t *)&msgRead.data[0];
    *currentAverage = *(int32_t *)&msgRead.data[4];    

    return 0;
}
/*****************************************************************************
 功能描述  : 读平均电流,通过SDO模式读取, index-0x30d1, subindex-0x01
 输入参数  : uint8_t idx
             int32_t *currentAverage  平均电流
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月14日
*****************************************************************************/
uint32_t MotorReadAvarageCurrentCmd(uint8_t idx, int32_t *currentAverage)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0xd1, 0x30, 0x01}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
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
        || (DRIVER_TYPE_NIMOTION == driveType)
        || (DRIVER_TYPE_FDK == driveType)
        || (DRIVER_TYPE_KINCO_CAN == driveType))
    {
        //index
        msg.data[1] = 0x78;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x00;
    }
    
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *currentAverage = *(int16_t *)&msgRead.data[4] * 10;//单位0.01A，乘以10转换成mA
    }
    else if((DRIVER_TYPE_STAND == driveType) || (DRIVER_TYPE_NIMOTION == driveType))
    {
        *currentAverage = *(int16_t *)&msgRead.data[4];
        *currentAverage = *currentAverage * gStMotorData[motorNum].normalCurrent / 1000;
    }
    else if(DRIVER_TYPE_FDK == driveType)
    {
        *currentAverage = *(int16_t *)&msgRead.data[4] * 100;//单位0.1A，乘以100转换成mA
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        *currentAverage = *(int16_t *)&msgRead.data[4];
        *currentAverage = (*currentAverage * gStMotorData[motorNum].limitCurrent) >> 11;    //转换成mA
    }
    else 
    {
        *currentAverage = *(int32_t *)&msgRead.data[4];//单位mA
    }

    if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
    {
        *currentAverage = -(*currentAverage);
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 读取当前速度
 输入参数  : uint8_t idx
             int32_t *vel  速度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年1月19日
*****************************************************************************/
uint32_t MotorReadVelocity(uint8_t idx, int32_t *vel)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x6c, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    int32_t counts = 6;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
        counts = gStMotorData[motorNum].counts;
    }

    if(DRIVER_TYPE_PUSI == driveType)
    {
        //index
        msg.data[1] = 0x30;
        msg.data[2] = 0x60;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x1d;
    }

    msg.id += idx;
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    if(DRIVER_TYPE_QILING == driveType)
    {
        *vel = *(int16_t *)&msgRead.data[4];
    }
    else
    {
        *vel = *(int32_t *)&msgRead.data[4];
    }

    //compley驱动器单位0.1counts/s
    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *vel = (*vel) * 6 / counts; //0.1counts/s转rpm
    }
    else if((DRIVER_TYPE_STAND == driveType) || (DRIVER_TYPE_PUSI == driveType))
    {
        *vel = (*vel) * 60 / counts; //counts/s转rpm
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        *vel = ((*vel) * 1875 / counts) >> 9; //转rpm
    }

    if(motorNum < M_TOTAL_NUM)
    {
        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
        {
            *vel = -(*vel);
        }
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 设置当前实际位置，compley驱动器才支持
 输入参数  : uint8_t idx
             int32_t pos  
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年1月20日
*****************************************************************************/
uint32_t MotorSetCurActualPosition(uint8_t idx, int32_t pos)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x64, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    int32_t realPos;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;
    (*(int32_t*)&msg.data[4]) = pos;

    if(0 == MotorCanDeviceRead(motorNum, &msg, &msgRead))
    {
        return 1;
    }

    delay_us(2000);

    if(0 == MotorReadPosition(idx, &realPos))
    {
        realPos -= pos;
        if(ABS_VALUE(realPos) >= 100)
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }

    return 0;
}
/*****************************************************************************
 功能描述  : 读取电机当前位置
 输入参数  : uint8_t idx
             int32_t *pos  当前位置
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2018年11月15日
*****************************************************************************/
uint32_t MotorReadPosition(uint8_t idx, int32_t *pos)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x64, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    msg.id += idx;

    if(DRIVER_TYPE_PUSI == driveType)
    {
        //index
        msg.data[1] = 0x0c;
        msg.data[2] = 0x60;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x1E;
    }
    
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *pos = *(int32_t *)&msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 读IO口状态
 输入参数  : uint8_t idx
             uint16_t *ioStatus  IO状态
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月10日
*****************************************************************************/
uint32_t MotorReadIOStatus(uint8_t idx, uint16_t *ioStatus)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x0b, 0x20, 0x05}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;
    
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    *ioStatus = *(uint16_t *)&msgRead.data[4];

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取状态命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadStatus(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x41, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdo模式
    {
        g_stMsgReadPDOFeedback.id = 0x180 + idx;
        MotorCanDeviceWrite(motorNum, &g_stMsgReadPDOFeedback);
    }
    else//sdo模式
    {
        msg.id += idx;

        if(DRIVER_TYPE_PUSI == driveType)
        {
            //index
            msg.data[1] = 0x2e;
            msg.data[2] = 0x60;
            //subindex
            msg.data[3] = 0x02;
        }
        
        MotorCanDeviceWrite(motorNum, &msg);
    } 

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取控制器状态命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadControlStatus(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x01, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;
    
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取平均电流命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadAvarageCurrentCmd(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0xd1, 0x30, 0x01}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
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
        || (DRIVER_TYPE_NIMOTION == driveType)
        || (DRIVER_TYPE_FDK == driveType)
        || (DRIVER_TYPE_KINCO_CAN == driveType))
    {
        //index
        msg.data[1] = 0x78;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x00;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x05;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x10;
    }
    
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取位置命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadPosCmd(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x63, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    msg.id += idx;

    if(DRIVER_TYPE_PUSI == driveType)
    {
        //index
        msg.data[1] = 0x0c;
        msg.data[2] = 0x60;
    }
    else if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x1E;
    }
     
    MotorCanDeviceWrite(motorNum, &msg); 

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取速度命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadVelocity(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x6c, 0x60, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME}; 
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }
    
    msg.id += idx;

    if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x04;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x1d;
    }
    
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取驱动器温度命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadDriverTmpCmd(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0xa2, 0x22, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    msg.id += idx;

    if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x05;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x18;
    }
    else if(DRIVER_TYPE_NIMOTION == driveType)
    {
        //index
        msg.data[1] = 0x0b;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x16;
    }
    else if(DRIVER_TYPE_COMPLEY == driveType)
    {
        //index
        msg.data[1] = 0x02;
        msg.data[2] = 0x22;
        //subindex
        msg.data[3] = 0x00;
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        //index
        msg.data[1] = 0xf7;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x0b;
    }
     
    MotorCanDeviceWrite(motorNum, &msg); 

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取电机温度命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadTmpCmd(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x02, 0x20, 0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    //DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    /*if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }*/

    msg.id += idx;
     
    MotorCanDeviceWrite(motorNum, &msg); 

    return 0;
}
/*****************************************************************************
 功能描述  : 发送读取电压命令（只发送，异步接收）
 输入参数  : uint8_t idx
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月4日
*****************************************************************************/
uint32_t MotorSendReadVolCmd(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_UPLOAD, 0x0d, 0x30, 0x06}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    DRIVER_TYPE driveType = DRIVER_TYPE_STAND;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        driveType = gStMotorData[motorNum].driverType;
    }

    msg.id += idx;

    if(DRIVER_TYPE_QILING == driveType)
    {
        //index
        msg.data[1] = 0x05;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x16;
    }
    else if(DRIVER_TYPE_NIMOTION == driveType)
    {
        //index
        msg.data[1] = 0x0b;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x15;
    }
    else if(DRIVER_TYPE_COMPLEY == driveType)
    {
        //index
        msg.data[1] = 0x01;
        msg.data[2] = 0x22;
        //subindex
        msg.data[3] = 0x00;
    }
    else if(DRIVER_TYPE_FDK == driveType)
    {
        //index
        msg.data[1] = 0x03;
        msg.data[2] = 0x20;
        //subindex
        msg.data[3] = 0x00;
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        //index
        msg.data[1] = 0x79;
        msg.data[2] = 0x60;
        //subindex
        msg.data[3] = 0x00;
    }
    
    MotorCanDeviceWrite(motorNum, &msg); 

    return 0;
}

/*****************************************************************************
 功能描述  : 通过SDO设置canopen字典
 输入参数  : uint8_t idx
             uint8_t *data  
             uint8_t len    
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年1月16日
*****************************************************************************/
uint32_t MotorCanSdoSet(uint8_t idx, uint8_t *data, uint8_t len)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE, {0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint16_t lIndex;

    msg.id += idx;
    if(len <= 7)
    {
        memcpy(&msg.data[1], data, len);
        lIndex = *(uint16_t *)data;
        rt_kprintf("Set Index-0x%04x, sub-0x%02x!\r\n", lIndex, data[2]);
    }
    else
    {
        rt_kprintf("Set len error!\r\n");
        return 1;
    }
    //msg.len = len + 1;
    if(4 == len){msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD1;}
    else if(5 == len){msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD2;}
    else if(6 == len){msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD3;}
    else if(7 == len){msg.data[0] = SDO_COMMAND_SPECIFIER_DOWNLOAD4;}
	  
    CanDeviceWrite(CAN2_DEVICE, &msg, CAN_WRITE_TIMEOUT_MS);

    return 0;
}
/*****************************************************************************
 功能描述  : 通过SDO获取canopen字典的内容
 输入参数  : uint8_t idx
             uint8_t *data  
             uint8_t len    
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年1月16日
*****************************************************************************/
uint32_t MotorCanSdoGet(uint8_t idx, uint8_t *data, uint8_t len)
{
    uint32_t l_size;
    CAN_msg msg = {SDO_CLIENT_ID_BASE, {0x00}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    CAN_msg msgRead;
    uint16_t lIndex;
    uint32_t lGetValue;
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;
    if(len >= 3)
    {
        memcpy(&msg.data[1], data, 3);
        lIndex = *(uint16_t *)data;
        rt_kprintf("Get Index-0x%04x, sub-0x%02x!\r\n", lIndex, data[2]);
    }
    else
    {
        rt_kprintf("Get len error!\r\n");
    }
    msg.data[0] = SDO_COMMAND_SPECIFIER_UPLOAD;
	  
    l_size = MotorCanDeviceRead(motorNum, &msg, &msgRead);

    if(0 == l_size)
    {
        return 1;
    }

    lGetValue = *(uint32_t *)&msgRead.data[4];

    rt_kprintf("Value:0x%08x\r\n", lGetValue);

    return 0;
}
/*****************************************************************************
 功能描述  : 禁止驱动器的发送pdo
 输入参数  : uint8_t idx
 输出参数  : uint32_t    0表示成功，非0表示故障
 作    者  : 刘鹏
 日    期  : 2019年1月20日
*****************************************************************************/
uint32_t MotorCanDisableTransimitPdo(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00, 0x80}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;

    msg.data[4] = 0x80 + idx;

    //index 0x1800 关闭tpdo0
    msg.data[1] = 0x00;
    msg.data[2] = 0x18;
    msg.data[5] = 0x01;
    MotorCanDeviceWrite(motorNum, &msg);
    //index 0x1801 关闭tpdo1
    msg.data[1] = 0x01;
    msg.data[2] = 0x18;
    msg.data[5] = 0x02;
    MotorCanDeviceWrite(motorNum, &msg);
    //index 0x1802 关闭tpdo2
    msg.data[1] = 0x02;
    msg.data[2] = 0x18;
    msg.data[5] = 0x03;
    MotorCanDeviceWrite(motorNum, &msg);
    //index 0x1803 关闭tpdo3
    msg.data[1] = 0x03;
    msg.data[2] = 0x18;
    msg.data[5] = 0x04;
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 功能描述  : 电机控制测试
 输入参数  : rt_uint8_t* cmdData
             rt_uint8_t size      
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月12日
*****************************************************************************/
static uint8_t gTestMotorNum = M_LEFT;
void TestMotorControl(uint8_t* cmdData, uint8_t size)
{
    uint8_t cmd;
    uint8_t *dataBuf;
    int32_t l_temp = 0;
    uint16_t l_status;
    
    if(0 == size)
    {
        return;
    }
    cmd = cmdData[0];
    dataBuf = &cmdData[1];
    if(0x01 == cmd)//切换电机序号
    {
        if(dataBuf[0] < M_TOTAL_NUM)
        {            
            gTestMotorNum = dataBuf[0];
            rt_kprintf("Set Motor num: %d.\r\n", gTestMotorNum);
        }
    }
    else if(0x02 == cmd)//切换模式
    {
        rt_kprintf("Set Motor mode: %d.\r\n", dataBuf[0]);
        ChangeMotorControlMode(gTestMotorNum, dataBuf[0]); 
    }
    else if(0x03 == cmd)//设定速度
    {
        l_temp = *(int32_t*)dataBuf;
        rt_kprintf("Set target vel: %d.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(gTestMotorNum, l_temp);
        //MotorSetTargetVelocity(gStMotorData[gTestMotorNum].idx, l_temp, RT_FALSE);
    }
    else if(0x04 == cmd)//打开/关闭驱动器电源
    {
        if(0 == dataBuf[0])
        {
            SetMotorPower(gTestMotorNum, POWER_OFF);
            gStMotorRunState[gTestMotorNum].powerFlag = MOTOR_POWER_FLAG_OFF;
            gStMotorRunState[gTestMotorNum].startRemoteFlag = 0;
        }
        else
        {
            SetMotorPower(gTestMotorNum, POWER_ON);
            gStMotorRunState[gTestMotorNum].powerFlag = MOTOR_POWER_FLAG_TIMECNT;
            gStMotorRunState[gTestMotorNum].powerTime = 0;
            gStMotorRunState[gTestMotorNum].resetFlag = 0;
        }
    }
    else if(0x05 == cmd)//打开/关闭抱闸电源
    {
        if(0 == dataBuf[0])
        {
            SetMotorLock(M_TOTAL_NUM, LOCK_OFF, RT_TRUE);   //关闭抱闸
        }
        else
        {
            SetMotorLock(M_TOTAL_NUM, LOCK_ON, RT_TRUE);    //开抱闸
        }
    }
    else if(0x06 == cmd)//切换电机控制命令
    {
        rt_kprintf("Set control cmd: %d.\r\n", *((uint16_t*)dataBuf));
        MotorDeviceControlCmd(gStMotorData[gTestMotorNum].idx, *((uint16_t*)dataBuf));
    }
    else if(0x07 == cmd)//设定位置
    {
        l_temp = *(int16_t*)dataBuf;
        rt_kprintf("Set target pos: %d.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(gTestMotorNum, ConvertAngleToCounts(gTestMotorNum, l_temp, (PosType)dataBuf[2]));
    }
    else if(0x08 == cmd)//读取速度
    {
        if(0 == MotorReadVelocity(gStMotorData[gTestMotorNum].idx, &l_temp))
        {
            rt_kprintf("Cur vel: %d.\r\n", l_temp);
        }
        else
        {
            rt_kprintf("Read vel failed!\r\n", l_temp);
        }
    }
    else if(0x09 == cmd)//读取位置
    {
        if(0 == MotorReadPosition(gStMotorData[gTestMotorNum].idx, &l_temp))
        {
            rt_kprintf("Cur pos: %d.\r\n", l_temp);
        }
        else
        {
            rt_kprintf("Read pos failed!\r\n", l_temp);
        }
    }
    else if(0x0a == cmd)//设定编码器位置
    {
        l_temp = *(int32_t*)dataBuf;
        rt_kprintf("Set encode pos: %d.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(gTestMotorNum, l_temp);
    }
    /*else if(0x0b == cmd)//读取限位1状态
    {
        if(0 != gStMotorData[gTestMotorNum].inputPin1)
        {
            if(GPIO_ReadInputDataBit(gStMotorData[gTestMotorNum].inputPort1, gStMotorData[gTestMotorNum].inputPin1))
            {
                rt_kprintf("id-%d input1 state: yes.\r\n", gStMotorData[gTestMotorNum].idx);
            }
            else
            {
                rt_kprintf("id-%d input1 state: not.\r\n", gStMotorData[gTestMotorNum].idx);
            }
        }
        else
        {
            rt_kprintf("id-%d no input1.\r\n", gStMotorData[gTestMotorNum].idx);
        }
    }
    else if(0x0c == cmd)//读取限位1状态
    {
        if(0 != gStMotorData[gTestMotorNum].inputPin2)
        {
            if(GPIO_ReadInputDataBit(gStMotorData[gTestMotorNum].inputPort2, gStMotorData[gTestMotorNum].inputPin2))
            {
                rt_kprintf("id-%d input2 state: yes.\r\n", gStMotorData[gTestMotorNum].idx);
            }
            else
            {
                rt_kprintf("id-%d input2 state: not.\r\n", gStMotorData[gTestMotorNum].idx);
            }
        }
        else
        {
            rt_kprintf("id-%d no input2.\r\n", gStMotorData[gTestMotorNum].idx);
        }
    }*/
    else if(0x0d == cmd)//读驱动器状态
    {
        if(0 == MotorReadStatus(gStMotorData[gTestMotorNum].idx, &l_status))
        {
            rt_kprintf("Cur status: 0x%x.\r\n", l_status);
        }
        else
        {
            rt_kprintf("Read status failed!\r\n", l_temp);
        }
    }
    else if(0x0e == cmd)//设定电流
    {
        l_temp = *(int32_t*)dataBuf;
        rt_kprintf("Set target current: %d mA.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(gTestMotorNum, l_temp);
    }
    else if(0x0f == cmd)//打开/关闭驱动器电源
    {
        if(0 == dataBuf[0])
        {
            SetMotorPower(gTestMotorNum, POWER_OFF);
        }
        else
        {
            SetMotorPower(gTestMotorNum, POWER_ON);
        }
    }
    else if(0x10 == cmd)//读取电流
    {
        if(0 == MotorReadAvarageCurrentCmd(gStMotorData[gTestMotorNum].idx, &l_temp))
        {
            rt_kprintf("Current: %d mA.\r\n", l_temp);
        }
        else
        {
            rt_kprintf("Read current failed!\r\n", l_temp);
        }
    }
}





