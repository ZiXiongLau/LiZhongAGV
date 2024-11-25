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

#ifndef __mb_h
#define __mb_h

#include "mbcommon.h"


/* ----------------------- Defines ------------------------------------------*/
#define MB_ADDRESS_BROADCAST   (0)   /*! Modbus broadcast address. */
#define MB_ADDRESS_MIN         (1)   /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX         (247) /*! Biggest possible slave address. */

#define MB_SER_PDU_SIZE_MIN    (4)       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX    (256)     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC    (2)       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF    (0)       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF     (1)       /*!< Offset of Modbus-PDU in Ser-PDU. */

#define MB_UFO_SLAVE_ADDRESS   (0x5)     // 平板modbus从机地址


/*解析接收到的信息，返回响应命令的长度*/
uint16_t ParsingAccessCommand(eMBMode mode, uint8_t *receivedMessage, uint32_t recvLen, uint8_t *respondBytes);

#endif
/*********** (C) COPYRIGHT 1999-2016 Moonan Technology *********END OF FILE****/
