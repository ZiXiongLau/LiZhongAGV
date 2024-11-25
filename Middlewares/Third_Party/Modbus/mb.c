/******************************************************************************/
/** 模块名称：Modbus通讯                                                     **/
/** 文件名称：mb.c                                                           **/
/** 版    本：V1.0.1                                                         **/
/** 简    介：用于实现Modbus总线协议站ADU的封装，相关属性及方法              **/
/**           1、实现访问数据命令的生成                                      **/
/**           2、实现对命令的响应信息的生成                                  **/
/**           3、初始化相关参数，如存储域等                                  **/
/**           2、解析接收到的消息，并对数据对象进行相应操作，返回响应消息    **/
/**                                                                          **/
/* MBAP报文包括的内容：                                                       */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |     域      |  长度   | 客户机 | 服务器 |              描述            | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |事务元标识符 | 2个字节 |  启动  |  复制  |请求/响应事务处理的识别码     | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |协议标识符   | 2个字节 |  启动  |  复制  |0=MODBUS 协议                 | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |    长度     | 2个字节 |  启动  |  启动  |以下字节的数量                | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |单元标识符   | 1个字节 |  启动  |  复制  |连接的远程从站的识别码        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   功能码    | 1个字节 |  启动  |  复制  |要实现的功能                  | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   数据      | N个字节 |  启动  |  启动  |根据相应功能返回的数据        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* RTU报文包括的内容：                                                        */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |单元标识符   | 1个字节 |  启动  |  复制  |连接的远程从站的识别码        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   功能码    | 1个字节 |  启动  |  复制  |要实现的功能                  | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |   数据      | N个字节 |  启动  |  启动  |根据相应功能返回的数据        | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |    CRC      | 2个字节 |  启动  |  启动  |根据以上数据运算出的CRC码     | */
/* +-------------+---------+--------+--------+------------------------------+ */
/**--------------------------------------------------------------------------**/
/** 修改记录：                                                               **/
/**     版本      日期              作者           说明                      **/
/**     V1.0.0  2016-04-17          木南           创建文件                  **/
/**     V1.0.1  2023-06-15          Toyal          优化文件，添加rtu通讯支持 **/
/**                                                                          **/
/******************************************************************************/ 

#include "mb.h"
#include "mbcommon.h"
#include "mbpdu.h"
#include "mbcrc.h"
#include "predef.h"

/*读线圈状态命令*/
static uint16_t ReadCoilStatusCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*读输入状态命令*/
static uint16_t ReadInputStatusCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*读保持寄存器命令*/
static uint16_t ReadHoldingRegisterCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*读输入寄存器命令*/
static uint16_t ReadInputRegisterCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*写单个线圈命令*/
static uint16_t WriteSingleCoilCommand(eMBMode mode,uint16_t coilAddress,uint16_t coilValue,uint8_t *receivedMessage,uint8_t *respondBytes);
/*写单个寄存器命令*/
static uint16_t WriteSingleRegisterCommand(eMBMode mode,uint16_t registerAddress,uint16_t registerValue,uint8_t *receivedMessage,uint8_t *respondBytes);
/*写多个线圈状态*/
static uint16_t WriteMultipleCoilCommand(eMBMode mode,uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes);
/*写多个寄存器状态*/
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


/*解析接收到的信息，返回响应命令的长度*/
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

/*合成对访问的响应,返回值为命令长度*/
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

/*生成读写对象的命令*/
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


/*处理读线圈状态命令*/
static uint16_t ReadCoilStatusCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  bool statusList[250];
  
  GetCoilStatus(startAddress,quantity,statusList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,statusList,NULL,respondBytes);
  
  return length;
}

/*处理读输入状态命令*/
static uint16_t ReadInputStatusCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  bool statusList[250];
  
  GetInputStatus(startAddress,quantity,statusList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,statusList,NULL,respondBytes);
  
  return length;
}

/*处理读保持寄存器命令*/
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

/*处理读输入寄存器命令*/
static uint16_t ReadInputRegisterCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  uint16_t registerList[125];
  
  GetInputRegister(startAddress,quantity,registerList);
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,registerList,respondBytes);
  
  return length;
}

/*处理写单个线圈命令*/
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

/*处理写单个寄存器命令*/
static uint16_t WriteSingleRegisterCommand(eMBMode mode, uint16_t registerAddress,uint16_t registerValue,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  
  length=SyntheticAccessRespond(mode,receivedMessage,NULL,NULL,respondBytes);
  
  SetSingleRegister(registerAddress,registerValue);
  
  return length;
}

/*处理写多个线圈状态*/
static uint16_t WriteMultipleCoilCommand(eMBMode mode, uint16_t startAddress,uint16_t quantity,uint8_t *receivedMessage,uint8_t *respondBytes)
{
  uint16_t length=0;
  bool statusValue[250];
  
  length=SyntheticAccessRespond(mode,receivedMessage,statusValue,NULL,respondBytes);
  
  SetMultipleCoil(startAddress,quantity,statusValue);
  
  return length;
}

/*处理写多个寄存器状态*/
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
