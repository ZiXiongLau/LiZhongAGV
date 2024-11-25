/******************************************************************************/
/** ģ�����ƣ�ModbusͨѶ                                                     **/
/** �ļ����ƣ�mb.c                                                           **/
/** ��    ����V1.0.1                                                         **/
/** ��    �飺����ʵ��Modbus����Э��վADU�ķ�װ��������Լ�����              **/
/**           1��ʵ�ַ����������������                                      **/
/**           2��ʵ�ֶ��������Ӧ��Ϣ������                                  **/
/**           3����ʼ����ز�������洢���                                  **/
/**           2���������յ�����Ϣ���������ݶ��������Ӧ������������Ӧ��Ϣ    **/
/**                                                                          **/
/* MBAP���İ��������ݣ�                                                       */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |     ��      |  ����   | �ͻ��� | ������ |              ����            | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |����Ԫ��ʶ�� | 2���ֽ� |  ����  |  ����  |����/��Ӧ�������ʶ����     | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |Э���ʶ��   | 2���ֽ� |  ����  |  ����  |0=MODBUS Э��                 | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |    ����     | 2���ֽ� |  ����  |  ����  |�����ֽڵ�����                | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |��Ԫ��ʶ��   | 1���ֽ� |  ����  |  ����  |���ӵ�Զ�̴�վ��ʶ����        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   ������    | 1���ֽ� |  ����  |  ����  |Ҫʵ�ֵĹ���                  | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   ����      | N���ֽ� |  ����  |  ����  |������Ӧ���ܷ��ص�����        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* RTU���İ��������ݣ�                                                        */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |��Ԫ��ʶ��   | 1���ֽ� |  ����  |  ����  |���ӵ�Զ�̴�վ��ʶ����        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   ������    | 1���ֽ� |  ����  |  ����  |Ҫʵ�ֵĹ���                  | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   ����      | N���ֽ� |  ����  |  ����  |������Ӧ���ܷ��ص�����        | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |    CRC      | 2���ֽ� |  ����  |  ����  |�������������������CRC��     | */
/* +-------------+---------+--------+--------+------------------------------+ */
/**--------------------------------------------------------------------------**/
/** �޸ļ�¼��                                                               **/
/**     �汾      ����              ����           ˵��                      **/
/**     V1.0.0  2016-04-17          ľ��           �����ļ�                  **/
/**     V1.0.1  2023-06-15          Toyal          �Ż��ļ������rtuͨѶ֧�� **/
/**                                                                          **/
/******************************************************************************/ 

#include "mb.h"
#include "mbcommon.h"
#include "mbpdu.h"
#include "mbcrc.h"
#include "predef.h"

/*����Ȧ״̬����*/
static uint16_t ReadCoilStatusCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*������״̬����*/
static uint16_t ReadInputStatusCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*�����ּĴ�������*/
static uint16_t ReadHoldingRegisterCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*������Ĵ�������*/
static uint16_t ReadInputRegisterCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*д������Ȧ����*/
static uint16_t WriteSingleCoilCommand(eMBMode mode,uint16_t coilAddress,uint16_t coilValue,uint8_t *receivedMessage,uint8_t *respondBytes);
/*д�����Ĵ�������*/
static uint16_t WriteSingleRegisterCommand(eMBMode mode,uint16_t registerAddress,uint16_t registerValue,uint8_t *receivedMessage,uint8_t *respondBytes);
/*д�����Ȧ״̬*/
static uint16_t WriteMultipleCoilCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*д����Ĵ���״̬*/
static uint16_t WriteMultipleRegisterCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);

uint16_t (*HandleCommand[])(eMBMode,uint16_t,uint16_t,uint8_t *,uint8_t *) =
{   
    ReadCoilStatusCommand,
    ReadInputStatusCommand,
    ReadHoldingRegisterCommand,
    ReadInputRegisterCommand,
    WriteSingleCoilCommand,
    WriteSingleRegisterCommand,
    WriteMultipleCoilCommand,
    WriteMultipleRegisterCommand
};


/*�������յ�����Ϣ��������Ӧ����ĳ���*/
uint16_t ParsingAccessCommand(eMBMode mode, uint8_t *receivedMessage, uint32_t recvLen, uint8_t *respondBytes)
{
    uint16_t length = 0;
    FunctionCode fc;
    uint16_t startAddress = 0;
    uint16_t quantity = 0;

    switch (mode)
    {
        case MB_RTU:
            if (MB_UFO_SLAVE_ADDRESS == receivedMessage[0])
            {
                if ((recvLen >= MB_SER_PDU_SIZE_MIN)
                 && (MBCRC16(receivedMessage , recvLen) == 0)
                   )
                {
                    fc=(FunctionCode)(*(receivedMessage+1));
                    if(CheckFunctionCode(fc)!=Modbus_OK)
                    {
                        if(DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_87)
                        {
                            rt_kprintf("Invalid FunctionCode!\r\n");
                        }    
                        return 0;
                    }

                    startAddress=(uint16_t)(*(receivedMessage+2));
                    startAddress=(startAddress<<8)+(uint16_t)(*(receivedMessage+3));
                    quantity=(uint16_t)(*(receivedMessage+4));
                    quantity=(quantity<<8)+(uint16_t)(*(receivedMessage+5));
                }
                else
                {
                    if(DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_87)
                    {
                        rt_kprintf("data length error or crc error!\r\n");
                    }
                    return 0;
                }
            }
            else 
            {
                if(DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_87)
                {
                    rt_kprintf("Cmd id:%d, Cur id:%d, slave address match error!\r\n", receivedMessage[0], MB_UFO_SLAVE_ADDRESS);
                }
                return 0;
            }
            break;
            
        case MB_TCP:
            fc=(FunctionCode)(*(receivedMessage+7));
            if(CheckFunctionCode(fc)!=Modbus_OK)
            {
            return 0;
            }

            startAddress=(uint16_t)(*(receivedMessage+8));
            startAddress=(startAddress<<8)+(uint16_t)(*(receivedMessage+9));
            quantity=(uint16_t)(*(receivedMessage+10));
            quantity=(quantity<<8)+(uint16_t)(*(receivedMessage+11));
            break;
            
        default:
            rt_kprintf("Incorrect Mode!\r\n");
            return 0;
    }

    uint8_t index=(fc>0x08)?(fc-0x09):(fc-0x01);

    length=HandleCommand[index](mode,startAddress,quantity,receivedMessage,respondBytes);

    return length;
}

/*�ϳɶԷ��ʵ���Ӧ,����ֵΪ�����*/
static uint16_t SyntheticAccessRespond(eMBMode mode, uint8_t *receivedMessage,bool *statusList,uint16_t *registerList,uint8_t *respondBytes)
{
  uint16_t index=0;
  uint8_t respond[260];
  uint16_t bytesCount;
  uint16_t i;
  uint16_t CRC16;

  if (MB_RTU == mode)
  {
    bytesCount=GenerateMasterAccessRespond(receivedMessage,statusList,registerList,respond);
    
    for(i=0;i<bytesCount;i++)
    {
      respondBytes[index++]=respond[i];
    }
    
    /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
    CRC16 = MBCRC16(respondBytes, index );
    respondBytes[index++] = (uint8_t)(CRC16 & 0xFF);
    respondBytes[index++] = (uint8_t)(CRC16 >> 8);
  }
  else if (MB_TCP == mode)
  {
    respondBytes[index++]=*receivedMessage;
    respondBytes[index++]=*(receivedMessage+1);
    respondBytes[index++]=*(receivedMessage+2);
    respondBytes[index++]=*(receivedMessage+3);
    
    bytesCount=GenerateMasterAccessRespond(receivedMessage+6,statusList,registerList,respond);
    
    respondBytes[index++]=(bytesCount>>8);
    respondBytes[index++]=bytesCount;
    
    for(i=0;i<bytesCount;i++)
    {
      respondBytes[index++]=respond[i];
    }
  }
  else
  {
    ;
  }
  
  return index;
}

/*���ɶ�д���������*/
/*
static uint16_t SyntheticReadWriteCommand(eMBMode mode, ObjAccessInfo objInfo,bool *statusList,uint16_t *registerList,uint8_t *commandBytes)
{
  uint8_t command[256];
  uint16_t index;
  uint16_t bytesCount;
  uint16_t i;
  uint16_t CRC16;

  if (MB_RTU == mode)
  {
    index=0;
    bytesCount=GenerateReadWriteCommand(objInfo,statusList,registerList,command);
    for(i=0;i<bytesCount;i++)
    {
      commandBytes[index++]=command[i];
    }
    CRC16 = MBCRC16(commandBytes, index );
    commandBytes[index++] = (uint8_t)(CRC16 & 0xFF);
    commandBytes[index++] = (uint8_t)(CRC16 >> 8);
  }
  else if (MB_TCP == mode)
  {
    index=4;
    bytesCount=GenerateReadWriteCommand(objInfo,statusList,registerList,command);
    commandBytes[index++]=bytesCount>>8;
    commandBytes[index++]=bytesCount;

    for(i=0;i<bytesCount;i++)
    {
      commandBytes[index++]=command[i];
    }
  }
  else 
  {
    ;
  }
  
  return index;
}
*/


/*�������Ȧ״̬����*/
static uint16_t ReadCoilStatusCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  bool statusList[250];
  
  GetCoilStatus(startAddress,quantity,statusList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,statusList,NULL,respondBytes);
  
  return length;
}

/*���������״̬����*/
static uint16_t ReadInputStatusCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  bool statusList[250];
  
  GetInputStatus(startAddress,quantity,statusList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,statusList,NULL,respondBytes);
  
  return length;
}

/*��������ּĴ�������*/
static uint16_t ReadHoldingRegisterCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  uint16_t registerList[125];

  length = sizeof(registerList) / sizeof(uint16_t);
  if(quantity >= length)
  {
      quantity = length;
  }
  
  GetHoldingRegister(startAddress,quantity,registerList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,registerList,respondBytes);
  
  return length;
}

/*���������Ĵ�������*/
static uint16_t ReadInputRegisterCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  uint16_t registerList[125];
  
  GetInputRegister(startAddress,quantity,registerList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,registerList,respondBytes);
  
  return length;
}

/*����д������Ȧ����*/
static uint16_t WriteSingleCoilCommand(eMBMode mode, uint16_t coilAddress,uint16_t coilValue,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  bool value;
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,NULL,respondBytes);
  
  GetCoilStatus(coilAddress,1,&value);
  
  value=CovertSingleCommandCoilToBoolStatus(coilValue,value);
  SetSingleCoil(coilAddress,value);
  
  return length;
}

/*����д�����Ĵ�������*/
static uint16_t WriteSingleRegisterCommand(eMBMode mode, uint16_t registerAddress,uint16_t registerValue,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,NULL,respondBytes);
  
  SetSingleRegister(registerAddress,registerValue);
  
  return length;
}

/*����д�����Ȧ״̬*/
static uint16_t WriteMultipleCoilCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  bool statusValue[250];
  
  length=SyntheticAccessRespond(mode,receivedMessage,statusValue,NULL,respondBytes);
  
  SetMultipleCoil(startAddress,quantity,statusValue);
  
  return length;
}

/*����д����Ĵ���״̬*/
static uint16_t WriteMultipleRegisterCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0, lenTemp;
  uint16_t registerValue[125];
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,registerValue,respondBytes);

  lenTemp = sizeof(registerValue) / sizeof(uint16_t);
  if(quantity >= lenTemp)
  {
      quantity = lenTemp;
  }
  
  SetMultipleRegister(startAddress,quantity,registerValue);
  return length;
}


/*********** (C) COPYRIGHT 1999-2016 Moonan Technology *********END OF FILE****/
