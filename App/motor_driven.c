/******************************************************************************

                  ��Ȩ���� (C), 2013-2021,����

 ******************************************************************************
  �� �� ��   : motor_driven.c
  �� �� ��   : ����
  ��    ��   : ����
  ��������   : 2018��11��16��
  ����޸�   :
  ��������   : �������
  �����б�   :
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
  �޸���ʷ   :
  1.��    ��   : 2018��11��16��
    ��    ��   : ����
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include <preDef.h>
#include <motor_control.h>
#include <motor_driven.h>
#include <delay.h> 
#include <can.h>

/*****************************************************************************
 ��������  : ͨ��id��ȡ������
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��9��16��
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
 ��������  : ��������Ϣ�Ƿ�������Ҫ����Ϣ
 �������  : CAN_msg* revMsg   ���յ�����Ϣ
             CAN_msg* sendMsd  ����������Ϣ������
 �������  : int    Ϊ1��ʾ��������Ϣ��Ϊ0��ʾ������Ϣ
 ��    ��  : ����
 ��    ��  : 2018��11��11��
*****************************************************************************/
static int MotorCheckReadData(const CAN_msg* revMsg, const CAN_msg* sendMsd)
{
    uint8_t idx = revMsg->id & 0x7f;
    uint32_t motorNum;
    
    motorNum = GetMotorNumFromId(idx);

    if(motorNum < M_TOTAL_NUM)
    {
        //����Ƿ���������������Ϣ
        if((revMsg->id & 0xff80) == 0x0700)
        {
            if((1 == revMsg->len) && ((0 == revMsg->data[0]) || (0x05 == revMsg->data[0])))
            {
                SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_BOOT_UP_FLAG);
            }
        }
        //����Ƿ��ǽ�����Ϣ
        else if((revMsg->id & 0xff80) == 0x0080)
        { 
            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_EMERGENCY_FLAG);
            gStMotorRevData[motorNum].eMergencyErrorCode = *(uint32_t *)&revMsg->data;
            //rt_kprintf("EMERGENGCY:0x%x\r\n", gStMotorRevData[motorNum].eMergencyErrorCode);
        }
        else if(((0x600 == (revMsg->id & 0x600)) || (0x580 == (revMsg->id & 0x580))) && (0x40 == (revMsg->data[0] & 0x60))) //sdoģʽ
        {
            if(0x20 == revMsg->data[2])
            {
                if(0x02 == revMsg->data[1]) //FDK����¶�����
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_TMP_FLAG);
                            gStMotorRevData[motorNum].motorTmp = *(int16_t *)&revMsg->data[4]; //��λ��
                        }
                    }
                }
                else if(0x03 == revMsg->data[1]) //ĸ�ߵ�ѹ����
                {
                    if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                        gStMotorRevData[motorNum].vol = *(int16_t *)&revMsg->data[4] / 10;//��λ0.1v
                    }
                }
                else if(0x04 == revMsg->data[1])
                {
                    if(0x1d == revMsg->data[3]) //�����ٶ�����
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_SPEED_FLAG);
                            gStMotorRevData[motorNum].speed = *(int16_t *)&revMsg->data[4];
                        }
                        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                        {
                            gStMotorRevData[motorNum].speed = -gStMotorRevData[motorNum].speed;
                        }
                    }
                    else if(0x1e == revMsg->data[3]) //����λ������
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
                    if(0x10 == revMsg->data[3]) //��������
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4];
                            gStMotorRevData[motorNum].current = gStMotorRevData[motorNum].current 
                                * gStMotorData[motorNum].normalCurrent / 100; //������İٷֱ�
                            if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                            {
                                gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                            }
                        }
                    }
                    else if(0x16 == revMsg->data[3]) //ĸ�ߵ�ѹ����
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(int16_t *)&revMsg->data[4];//��λv
                        }
                    }
                    else if(0x18 == revMsg->data[3])    //�����¶�����
                    {
                        if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp = (*(int16_t *)&revMsg->data[4]) / 10; //��λ��
                        }
                    }
                }
                else if(0x0b == revMsg->data[1])
                {
                    if(0x15 == revMsg->data[3]) //����ʤĸ�ߵ�ѹ����
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                        gStMotorRevData[motorNum].vol = *(uint16_t *)&revMsg->data[4] / 10;//��λ0.1v��10ת����v
                    }
                    else if(0x16 == revMsg->data[3])    //����ʤ�¶�����
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                        gStMotorRevData[motorNum].tmp = (*(int16_t *)&revMsg->data[4]); //��λ��
                    }
                }
            }
            else if(0x22 == revMsg->data[2])
            {
                if(0x01 == revMsg->data[1]) //compley��ѹ����
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(int16_t *)&revMsg->data[4] / 10;//��λ0.1V����10ת����v
                        }
                    }
                }
                else if(0x02 == revMsg->data[1]) //compley�¶�����
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp= *(int16_t *)&revMsg->data[4]; //��λ��
                        }
                    }
                }
                else if(0x1c == revMsg->data[1]) //compley��������
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4] * 10;//��λ0.01A������10ת����mA
                            if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                            {
                                gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                            }
                        }
                    }
                }
                else if(0xa2 == revMsg->data[1]) //elmo�¶�����
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if((DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                            || (DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType))
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp= *(int16_t *)&revMsg->data[4]; //��λ��
                        }
                    }
                }
            }
            else if(0x30 == revMsg->data[2])
            {
                if(0x0d == revMsg->data[1]) //ĸ�ߵ�ѹ����
                {
                    if(0x06 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(float *)&revMsg->data[4];//��λv
                        }
                    }
                }
                else if(0xd1 == revMsg->data[1]) //��compley���������Ǳ�׼����������DRIVER_TYPE_INFRANOR��������������
                {
                    if(0x01 == revMsg->data[3])
                    {
                        if((DRIVER_TYPE_COMPLEY != gStMotorData[motorNum].driverType)
                            && (DRIVER_TYPE_STAND != gStMotorData[motorNum].driverType))
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int32_t *)&revMsg->data[4];//��λmA
                            if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                            {
                                gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                            }
                        }
                    }
                }
            }
            else if(0x60 == revMsg->data[2])     
            {
                if(0x01 == revMsg->data[1])     //��˼���������������״̬��Ϣ
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
                        gStMotorRevData[motorNum].ctrlStatus = revMsg->data[4];
                    }
                }
                if(0x2e == revMsg->data[1])     //��˼���������״̬��Ϣ
                {
                    if(0x02 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                        gStMotorRevData[motorNum].status = *(uint16_t *)&revMsg->data[4];
                    }
                }
                else if(0x41 == revMsg->data[1])     //���״̬��Ϣ
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                        gStMotorRevData[motorNum].status = *(uint16_t *)&revMsg->data[4];
                    }
                }
                else if(0x63 == revMsg->data[1])     //λ������
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_POS_FLAG);
                        gStMotorRevData[motorNum].pos = *(int32_t *)&revMsg->data[4];
                    }
                }
                else if(0x6c == revMsg->data[1])    //ת������
                {
                    if(0x00 == revMsg->data[3])
                    {
                        SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_SPEED_FLAG);
                        gStMotorRevData[motorNum].speed = *(int32_t *)&revMsg->data[4];
                        //compley��������λ0.1counts/s����ת����rpm
                        if(DRIVER_TYPE_COMPLEY == gStMotorData[motorNum].driverType)
                        {
                            gStMotorRevData[motorNum].speed = 
                                gStMotorRevData[motorNum].speed * 6 / gStMotorData[motorNum].counts;
                        }
                        //��׼��������λcounts/s����ת����rpm
                        else if(DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                        {
                            gStMotorRevData[motorNum].speed = 
                                gStMotorRevData[motorNum].speed * 60 / gStMotorData[motorNum].counts;
                        }
                        else if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            /*gStMotorRevData[motorNum].speed =
                                (gStMotorRevData[motorNum].speed * 1875 / gStMotorData[motorNum].counts) >> 9;//���ݱ���������ֵ�����RPM��������Ҫ���ݹ�ʽ�������*/
                        }
                        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                        {
                            gStMotorRevData[motorNum].speed = -gStMotorRevData[motorNum].speed;
                        }
                    }
                }
                else if(0x78 == revMsg->data[1])    //��׼��������������
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if((DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                            || (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType))
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG); 
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4];
                            gStMotorRevData[motorNum].current = gStMotorRevData[motorNum].current 
                                * gStMotorData[motorNum].normalCurrent / 1000; //�������ǧ�ֱ�
                        }
                        /*else if(DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG); 
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4] * 10;//��λ0.01A������10ת����mA
                        }*/
                        else if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG); 
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4] * 100;//��λ0.1A������100ת����mA
                        }
                        else if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRevData[motorNum].current = *(int16_t *)&revMsg->data[4];
                            gStMotorRevData[motorNum].current = (gStMotorRevData[motorNum].current * gStMotorData[motorNum].limitCurrent) >> 11;//��λA������1000ת����mA
                        }
                        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                        {
                            gStMotorRevData[motorNum].current = -gStMotorRevData[motorNum].current;
                        }
                    }
                }
                else if(0x79 == revMsg->data[1])    //ĸ�ߵ�ѹ����
                {
                    if(0x00 == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_VOL_FLAG);
                            gStMotorRevData[motorNum].vol = *(uint32_t *)&revMsg->data[4] / 1000;//��λmv������1000ת����V
                        }
                    }
                }
                else if(0xf7 == revMsg->data[1])    //�����������¶�����
                {
                    if(0x0b == revMsg->data[3])
                    {
                        if(DRIVER_TYPE_KINCO_CAN == gStMotorData[motorNum].driverType)
                        {
                            SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_DRIVER_TMP_FLAG);
                            gStMotorRevData[motorNum].tmp= *(int16_t *)&revMsg->data[4]; //��λ��
                        }
                    }
                }
            }
        }
        else if(DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)//pdoģʽ
        {
            if(0x180 == (revMsg->id & 0x180))   //���״̬��Ϣ
            {
                SET_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                gStMotorRevData[motorNum].status = *(uint16_t *)&revMsg->data[4];
            }
        }
        
    }

    if(NULL != sendMsd)
    {
        //����Ƿ���PDO��Ϣ
        if((0x180 == (sendMsd->id & 0x180)) || (0x280 == (sendMsd->id & 0x280))
            || (0x380 == (sendMsd->id & 0x380)) || (0x480 == (sendMsd->id & 0x480)))
        {
            if(revMsg->id == sendMsd->id)//id��ͬ
            {
                return 1;
            }
        }
        //����Ƿ���SDO��Ϣ
        if((0x600 == (sendMsd->id & 0x600)) || (0x580 == (sendMsd->id & 0x580)))
        {
            if((revMsg->id & 0x07F) == (sendMsd->id & 0x07F))//id��ͬ
            {
                //���Ͷ���Ӧ��Ӧ��д��д��Ӧ��Ӧ
                if(((0x40 == (sendMsd->data[0] & 0x60)) && (0x40 == (revMsg->data[0] & 0x60)))
                    || ((0x20 == (sendMsd->data[0] & 0x60)) && (0x60 == (revMsg->data[0] & 0x60))))
                {
                    //index��subindex�Ƿ���ͬ
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
 ��������  : ���can����
 �������  : const CAN_msg *buffer
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��10��20��
*****************************************************************************/
static HAL_StatusTypeDef MotorCanDeviceWrite(uint32_t motorNum, const CAN_msg *buffer)
{
    HAL_StatusTypeDef lResult;
    
    if(motorNum < M_TOTAL_NUM)
    {
        while(1)
    	{
            /* Check for the Timeout */
            if((HAL_GetTick() - gStMotorRevData[motorNum].lastSendTime) > 1)  //ͬһ���������������ĵļ��ʱ���룾1ms����ֹ���������ִ����Ӧ������
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
 ��������  : ��װcan1���ݽ��գ����ӵȴ���ʱ
 �������  : const void *sendBuffer  ������������
             uint32_t sendSize      �������������С
             void *revBuffer         ��������Ŀ���ַ
             uint32_t revSize       ָ���������ݵĴ�С
 �������  : ʵ�ʳɹ����յĴ�С
 ��    ��  : ����
 ��    ��  : 2018��11��11��
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

    if(HAL_OK == lResult)//�ѳɹ����뷢������
	{
	    while(1)
    	{
    		lResult = CanDeviceRead(CAN2_DEVICE, revBuffer, CAN_READ_TIMEOUT_MS);        
    		
    		if(HAL_OK == lResult)//�յ�������
    		{
                if(0 != MotorCheckReadData((CAN_msg*)revBuffer, (CAN_msg*)sendBuffer))//����Ƿ�����������
                {
                    l_size = 1;
        			break;
                }
                else if((HAL_GetTick() - tickstart ) > CAN_READ_TIMEOUT_MS)
                {
                    break;
                }
    		}
            else//��ʱ
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
		MotorSendReadPosCmd(i);//����λ����Ϣ��ȡָ��
//		MotorSendReadVelocity(i);//�����ٶ���Ϣ��ȡָ��
	}
}
/*****************************************************************************
 ��������  : �����ȡ�̣߳��첽��������
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��6��
*****************************************************************************/
void MotorReadProcess(void)
{
    HAL_StatusTypeDef lResult;
    CAN_msg revBuffer;
    int cnt = 0;
        
    while(cnt++ < CAN_RX_MSG_MAX_NUM)
	{
		lResult = CanDeviceRead(CAN2_DEVICE, &revBuffer, 0);        
		
		if(HAL_OK == lResult)//�յ�������
		{
            MotorCheckReadData(&revBuffer, NULL);
		}
        else//��ʱ��δ������
        {
            break;
        }
	} 
}
/*****************************************************************************
 ��������  : ����NMT����״̬
 �������  : uint8_t cs   ������  ��ʼ��״̬������PDO����
             uint8_t idx  ָ��������id�����Ϊ0��ʾͬʱ��������������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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
 ��������  : ��ͬ������ģʽֵת��
 �������  : DRIVER_TYPE driveType    ����������
             int8_t* i8OperationMode  ����ģʽ
             int8_t inverseFlag       ����ת����־
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��27��
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
 ��������  : ���õ������ģʽ
 �������  : uint8_t idx
             int8_t i8OperationMode  ����ģʽ��������ͬ���ٶ�ģʽ���ļ������ٶ�ģʽ������ģʽ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
    {
        msg.id = 0x200 + idx;
        msg.data[0] = i8OperationMode;
    }
    else//sdoģʽ
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
 ��������  : ��ȡ�������ģʽ
 �������  : uint8_t idx
             int8_t* i8OperationMode  ����ģʽ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��1��
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
 ��������  : ��ͬ�������������л�
 �������  : DRIVER_TYPE driveType    ����������
             uint16_t* usControlMode  ������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��27��
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
 ��������  : ����豸��������
 �������  : uint16_t usControlMode  ����ģʽ���ϵ籣�֡��˶�������ֹͣ���ϵ硢���ϸ�λ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��13��
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

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
    {
        msg.id = 0x300 + idx;
        (*(uint16_t*)&msg.data[0]) = usControlMode;
    }
    else//sdoģʽ
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
 ��������  : ����豸��������(������)
 �������  : uint16_t usControlMode  ����ģʽ���ϵ籣�֡��˶�������ֹͣ���ϵ硢���ϸ�λ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��08��08��
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
 ��������  : �趨���Ŀ���ٶ�
 �������  : uint8_t idx
             int32_t vel         Ŀ���ٶ�
             rt_bool_t checkAck     �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��11��15��
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
        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
        {
            vel = -vel;
        }
    }

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
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
    else//sdoģʽ
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
        //compley��������λ0.1counts/s
        if(DRIVER_TYPE_COMPLEY == driveType)
        {
            (*(int32_t*)&msg.data[4]) = vel * counts / 6;//rpmת0.1counts/s
        }
        //INFRANOR��������λcounts/s
        else if((DRIVER_TYPE_NIMOTION == driveType) || (DRIVER_TYPE_PUSI == driveType))
        {
            (*(int32_t*)&msg.data[4]) = vel * counts / 60;//rpmתcounts/s
        }
        else if(DRIVER_TYPE_STAND == driveType)
        {
            (*(int32_t*)&msg.data[4]) = vel * counts / 60;//rpmתcounts/s
        }
        else if(DRIVER_TYPE_KINCO_CAN == driveType)
        {
			//Ϊ�˽����vel�㹻��ʱ����������⣬uint64->uint32�ض�ʱ���ܻ���ɾ��ȵĶ�ʧ
        	(*(int32_t*)&msg.data[4]) = (int64_t)vel * 512 * counts / 1875;
//            (*(int32_t*)&msg.data[4]) = vel * 512 * counts / 1875;//rpmתָ����λ
        }
        //��λתÿ��
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
 ��������  : ��ȡ�趨��Ŀ���ٶ�
 �������  : uint8_t idx
             int32_t* vel  
             DRIVER_TYPE driveType  ����������
             int32_t counts      ���һȦ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��12��13��
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
    //compley��������λ0.1counts/s����ת��תÿ��
    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *vel = (*vel) * 6 / counts; //0.1counts/sתrpm
    }
    //INFRANOR��������λcounts/s����ת��תÿ��
    else if(DRIVER_TYPE_NIMOTION == driveType)
    {
        *vel = (*vel) * 60 / counts; //counts/sתrpm
    }

    return 0;
}
/*****************************************************************************
 ��������  : �趨Ŀ��λ��
 �������  : uint8_t idx
             int32_t pos  Ŀ��λ��
             rt_bool_t checkAck �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��1��17��
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

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
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
    else//sdoģʽ
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
 ��������  : ��ȡ�趨��Ŀ��λ��
 �������  : uint8_t idx
             int32_t* pos  
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��12��13��
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
 ��������  : �趨Ŀ�����
 �������  : uint8_t idx
             int32_t current  Ŀ�����
             rt_bool_t checkAck �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��6��8��
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
        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
        {
            current = -current;
        }

        if(DRIVER_TYPE_EPOS == driveType)   //EPOS�������޷����̶�8�ֽڷ������ݣ�ֻ���ж������ݷ�����
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
            (*(int16_t*)&msg.data[4]) = current / 10; //��λ0.01A
        }
        else if(DRIVER_TYPE_FDK == driveType)
        {
            //current
            (*(int16_t*)&msg.data[4]) = current / 100;//��λ0.1A
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
            torque = current * 1000 / gStMotorData[motorNum].normalCurrent;  //�������ǧ�ֱȣ�������ص�ǧ�ֱ�
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
 ��������  : �趨�����ٶ�
 �������  : uint8_t idx
             int32_t vel     �����ٶ�rpm
             rt_bool_t checkAck �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��6��15��
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
            (*(uint32_t*)&msg.data[4]) = vel * counts / 60;  //��׼��������rpmת����couts/s
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
 ��������  : �趨�������ٶ�
 �������  : uint8_t idx
             int32_t acc     �������ٶ�rpm/s2
             rt_bool_t checkAck �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��6��15��
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
            (*(uint16_t*)&msg.data[4]) = acc;   //����ʱ��ms
        }
        else
        {
            (*(uint32_t*)&msg.data[4]) = counts / 60 * acc;  //��׼��������rpmת����couts/s
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
 ��������  : �趨�������ٶ�
 �������  : uint8_t idx
             int32_t dec     �������ٶ�rpm/s2
             rt_bool_t checkAck �Ƿ�����Ӧ
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��6��15��
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
            (*(uint16_t*)&msg.data[4]) = dec;   //����ʱ��ms
        }
        else
        {
            (*(uint32_t*)&msg.data[4]) = counts / 60 * dec;  //��׼��������rpmת����couts/s
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
 ��������  : �趨����ģʽ����
 �������  : uint8_t idx
             int8_t method  ����ģʽ����
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
    {
        msg.id = 0x500 + idx;
        msg.data[0] = method;
    }
    else//sdoģʽ
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
 ��������  : ��ȡ����ģʽ����
 �������  : uint8_t idx
             int8_t* method  ����ģʽ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��12��
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
 ��������  : �趨����λ��
 �������  : uint8_t idx
             int32_t pos   ����ģʽλ��
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��12��28��
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
 ��������  : �趨��ת��Ѱԭ��ʱ�ļ��ת��
 �������  : uint8_t idx
             uint16_t torq   ���ת�صİٷֱ�(��λ: 0.1%)
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2022��11��16��
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
 ��������  : �趨��ת��Ѱԭ��ʱ�ļ��ʱ��
 �������  : uint8_t idx
             uint16_t time   (��λ: ms)
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2022��11��16��
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
 ��������  : �趨��ԭ���ٶ�
 �������  : uint8_t idx
             int32_t vel   ����ģʽ�ٶ�(��λ: �û���λ/s),ͣ����Ч
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2022��11��16��
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
 ��������  : �趨ԭ��ع���ٶ�
 �������  : uint8_t idx
             uint32_t acc   ����ģʽ���ٶ�(��λ: �û���λ/s^2)
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2022��11��16��
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
 ��������  : �趨CANopen��ֹ���Ӳ�����
 �������  : uint8_t idx
             uint32_t optionCode  0:No action
                                  1:Set Fault Signal
                                  2:Device control command:Disable voltage
                                  3:Device control command:Quick stop
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2022��12��2��
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
 ��������  : ��ȡ������
 �������  : uint8_t idx
             uint16_t *errorCode  ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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
        //index 0x1002 compley�������������ַ
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
    if(DRIVER_TYPE_COMPLEY == driveType)//compley������������32λ��ÿλ����һ������
    {
        if(0 != *(uint16_t *)&msgRead.data[6])
        {
            rt_kprintf("compley id-%d high error: 0x%04x!\r\n", idx, *(uint16_t *)&msgRead.data[6]);
        }
    }

    return 0;
}

CAN_msg g_stMsgReadPDOFeedback = {0 , {0}, 0, 1, STANDARD_FORMAT, REMOTE_FRAME}; //��ȡPDO���ݵ�Զ��֡��id��ָ��
/*****************************************************************************
 ��������  : ��������״̬,ͨ��PDOģʽ��ȡ����������������TPDO1�ĵ�ַӳ��
 �������  : uint8_t idx
             uint16_t *status  ���״̬
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
    {
        g_stMsgReadPDOFeedback.id = 0x180 + idx;
        l_size = MotorCanDeviceRead(motorNum, &g_stMsgReadPDOFeedback, &msgRead);
    }
    else//sdoģʽ
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
 ��������  : ����ÿ�����������״̬
 �������  : uint8_t idx
             uint16_t *status  ���״̬
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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
 ��������  : �����ʹ��״̬
 �������  : uint8_t idx
             uint16_t *enableStatus  ʹ��״̬
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��27��
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
 ��������  : ʹ�ܵ��
 �������  : uint8_t idx
             uint8_t enableStatus  ʹ��״̬
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��27��
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
 ��������  : �趨������״̬
 �������  : uint8_t idx
             uint8_t controlStatus  ������״̬
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��4��10��
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
 ��������  : ��˲ʱ������ƽ������,ͨ��PDOģʽ��ȡ����������������TPDO4�ĵ�ַӳ��
 �������  : uint8_t idx
             int32_t *current         ˲ʱ����
             int32_t *currentAverage  ƽ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��13��
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
 ��������  : ��ƽ������,ͨ��SDOģʽ��ȡ, index-0x30d1, subindex-0x01
 �������  : uint8_t idx
             int32_t *currentAverage  ƽ������
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��14��
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
        *currentAverage = *(int16_t *)&msgRead.data[4] * 10;//��λ0.01A������10ת����mA
    }
    else if((DRIVER_TYPE_STAND == driveType) || (DRIVER_TYPE_NIMOTION == driveType))
    {
        *currentAverage = *(int16_t *)&msgRead.data[4];
        *currentAverage = *currentAverage * gStMotorData[motorNum].normalCurrent / 1000;
    }
    else if(DRIVER_TYPE_FDK == driveType)
    {
        *currentAverage = *(int16_t *)&msgRead.data[4] * 100;//��λ0.1A������100ת����mA
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        *currentAverage = *(int16_t *)&msgRead.data[4];
        *currentAverage = (*currentAverage * gStMotorData[motorNum].limitCurrent) >> 11;    //ת����mA
    }
    else 
    {
        *currentAverage = *(int32_t *)&msgRead.data[4];//��λmA
    }

    if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
    {
        *currentAverage = -(*currentAverage);
    }

    return 0;
}
/*****************************************************************************
 ��������  : ��ȡ��ǰ�ٶ�
 �������  : uint8_t idx
             int32_t *vel  �ٶ�
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��1��19��
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

    //compley��������λ0.1counts/s
    if(DRIVER_TYPE_COMPLEY == driveType)
    {
        *vel = (*vel) * 6 / counts; //0.1counts/sתrpm
    }
    else if((DRIVER_TYPE_STAND == driveType) || (DRIVER_TYPE_PUSI == driveType))
    {
        *vel = (*vel) * 60 / counts; //counts/sתrpm
    }
    else if(DRIVER_TYPE_KINCO_CAN == driveType)
    {
        *vel = ((*vel) * 1875 / counts) >> 9; //תrpm
    }

    if(motorNum < M_TOTAL_NUM)
    {
        if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
        {
            *vel = -(*vel);
        }
    }

    return 0;
}
/*****************************************************************************
 ��������  : ���õ�ǰʵ��λ�ã�compley��������֧��
 �������  : uint8_t idx
             int32_t pos  
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��1��20��
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
 ��������  : ��ȡ�����ǰλ��
 �������  : uint8_t idx
             int32_t *pos  ��ǰλ��
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2018��11��15��
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
 ��������  : ��IO��״̬
 �������  : uint8_t idx
             uint16_t *ioStatus  IO״̬
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��4��10��
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
 ��������  : ���Ͷ�ȡ״̬���ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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

    if(DRIVER_TYPE_EPOS == driveType)//pdoģʽ
    {
        g_stMsgReadPDOFeedback.id = 0x180 + idx;
        MotorCanDeviceWrite(motorNum, &g_stMsgReadPDOFeedback);
    }
    else//sdoģʽ
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
 ��������  : ���Ͷ�ȡ������״̬���ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ���Ͷ�ȡƽ���������ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ���Ͷ�ȡλ�����ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ���Ͷ�ȡ�ٶ����ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ���Ͷ�ȡ�������¶����ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ���Ͷ�ȡ����¶����ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ���Ͷ�ȡ��ѹ���ֻ���ͣ��첽���գ�
 �������  : uint8_t idx
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��4��
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
 ��������  : ͨ��SDO����canopen�ֵ�
 �������  : uint8_t idx
             uint8_t *data  
             uint8_t len    
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��1��16��
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
 ��������  : ͨ��SDO��ȡcanopen�ֵ������
 �������  : uint8_t idx
             uint8_t *data  
             uint8_t len    
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��1��16��
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
 ��������  : ��ֹ�������ķ���pdo
 �������  : uint8_t idx
 �������  : uint32_t    0��ʾ�ɹ�����0��ʾ����
 ��    ��  : ����
 ��    ��  : 2019��1��20��
*****************************************************************************/
uint32_t MotorCanDisableTransimitPdo(uint8_t idx)
{
    CAN_msg msg = {SDO_CLIENT_ID_BASE , {SDO_COMMAND_SPECIFIER_DOWNLOAD4, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00, 0x80}, 8, 1, STANDARD_FORMAT, DATA_FRAME};
    uint32_t motorNum;
            
    motorNum = GetMotorNumFromId(idx);
    msg.id += idx;

    msg.data[4] = 0x80 + idx;

    //index 0x1800 �ر�tpdo0
    msg.data[1] = 0x00;
    msg.data[2] = 0x18;
    msg.data[5] = 0x01;
    MotorCanDeviceWrite(motorNum, &msg);
    //index 0x1801 �ر�tpdo1
    msg.data[1] = 0x01;
    msg.data[2] = 0x18;
    msg.data[5] = 0x02;
    MotorCanDeviceWrite(motorNum, &msg);
    //index 0x1802 �ر�tpdo2
    msg.data[1] = 0x02;
    msg.data[2] = 0x18;
    msg.data[5] = 0x03;
    MotorCanDeviceWrite(motorNum, &msg);
    //index 0x1803 �ر�tpdo3
    msg.data[1] = 0x03;
    msg.data[2] = 0x18;
    msg.data[5] = 0x04;
    MotorCanDeviceWrite(motorNum, &msg);

    return 0;
}
/*****************************************************************************
 ��������  : ������Ʋ���
 �������  : rt_uint8_t* cmdData
             rt_uint8_t size      
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��12��
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
    if(0x01 == cmd)//�л�������
    {
        if(dataBuf[0] < M_TOTAL_NUM)
        {            
            gTestMotorNum = dataBuf[0];
            rt_kprintf("Set Motor num: %d.\r\n", gTestMotorNum);
        }
    }
    else if(0x02 == cmd)//�л�ģʽ
    {
        rt_kprintf("Set Motor mode: %d.\r\n", dataBuf[0]);
        ChangeMotorControlMode(gTestMotorNum, dataBuf[0]); 
    }
    else if(0x03 == cmd)//�趨�ٶ�
    {
        l_temp = *(int32_t*)dataBuf;
        rt_kprintf("Set target vel: %d.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(gTestMotorNum, l_temp);
        //MotorSetTargetVelocity(gStMotorData[gTestMotorNum].idx, l_temp, RT_FALSE);
    }
    else if(0x04 == cmd)//��/�ر���������Դ
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
    else if(0x05 == cmd)//��/�رձ�բ��Դ
    {
        if(0 == dataBuf[0])
        {
            SetMotorLock(M_TOTAL_NUM, LOCK_OFF, RT_TRUE);   //�رձ�բ
        }
        else
        {
            SetMotorLock(M_TOTAL_NUM, LOCK_ON, RT_TRUE);    //����բ
        }
    }
    else if(0x06 == cmd)//�л������������
    {
        rt_kprintf("Set control cmd: %d.\r\n", *((uint16_t*)dataBuf));
        MotorDeviceControlCmd(gStMotorData[gTestMotorNum].idx, *((uint16_t*)dataBuf));
    }
    else if(0x07 == cmd)//�趨λ��
    {
        l_temp = *(int16_t*)dataBuf;
        rt_kprintf("Set target pos: %d.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(gTestMotorNum, ConvertAngleToCounts(gTestMotorNum, l_temp, (PosType)dataBuf[2]));
    }
    else if(0x08 == cmd)//��ȡ�ٶ�
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
    else if(0x09 == cmd)//��ȡλ��
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
    else if(0x0a == cmd)//�趨������λ��
    {
        l_temp = *(int32_t*)dataBuf;
        rt_kprintf("Set encode pos: %d.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(gTestMotorNum, l_temp);
    }
    /*else if(0x0b == cmd)//��ȡ��λ1״̬
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
    else if(0x0c == cmd)//��ȡ��λ1״̬
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
    else if(0x0d == cmd)//��������״̬
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
    else if(0x0e == cmd)//�趨����
    {
        l_temp = *(int32_t*)dataBuf;
        rt_kprintf("Set target current: %d mA.\r\n", l_temp);
        CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(gTestMotorNum, l_temp);
    }
    else if(0x0f == cmd)//��/�ر���������Դ
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
    else if(0x10 == cmd)//��ȡ����
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





