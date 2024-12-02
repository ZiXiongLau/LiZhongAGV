#ifndef _MOTOR_DRIVEN_H
#define _MOTOR_DRIVEN_H

#include <preDef.h>

/*设定CAN读取等待超时时间us*/
#define CAN_WRITE_TIMEOUT_MS 	  5  //5ms
#define CAN_READ_TIMEOUT_MS 	  8  //8ms

//SDO传输命令字节定义
#define SDO_CLIENT_ID_BASE                      0x600   //SDO传输客户端基准ID
#define SDO_COMMAND_SPECIFIER_UPLOAD            0x40    //SDO读取命令
#define SDO_COMMAND_SPECIFIER_MULTI_UPLOAD      0x41    //SDO读取命令(大于4个数据字节)

#define SDO_COMMAND_SPECIFIER_DOWNLOAD4         0x23    //SDO写入命令，后面4个数据字节的长度
#define SDO_COMMAND_SPECIFIER_DOWNLOAD3         0x27    //SDO写入命令，后面3个数据字节的长度
#define SDO_COMMAND_SPECIFIER_DOWNLOAD2         0x2b    //SDO写入命令，后面2个数据字节的长度
#define SDO_COMMAND_SPECIFIER_DOWNLOAD1         0x2f    //SDO写入命令，后面1个数据字节的长度

//电机读取线程，异步接收数据
void MotorReadProcess(void);

//设置NMT服务命令字
#define SET_NMT_STATE_INITIALIZATION            0x81   //复位节点，进入初始化状态，上电时的状态
#define SET_NMT_STATE_OPERATIONAL               0x01   //进入操作状态，允许SDO和PDO传输
uint32_t SetEposNMTState(uint8_t cs, uint8_t idx);

//电机运行模式定义
#define MOTOR_OPERATION_MODE_UNKOWN             0x00    //未知模式
#define MOTOR_OPERATION_MODE_PPM                0x01    //文件配置位置模式
#define MOTOR_OPERATION_MODE_CSV                0x09    //周期性同步速度模式
#define MOTOR_OPERATION_MODE_PVM                0x03    //文件配置速度模式
#define MOTOR_OPERATION_MODE_HMM                0x06    //回零模式
#define MOTOR_OPERATION_MODE_PTM                0x04    //文件配置力矩模式
#define MOTOR_OPERATION_MODE_CST                0x0A    //周期性同步力矩模式
uint32_t MotorSetOperationMode(uint8_t idx, int8_t i8OperationMode);
uint32_t MotorGetOperationMode(uint8_t idx, int8_t* i8OperationMode);

//电机控制命令定义
#define DEVICE_CTROL_SWITCH_ON                  0x07    //电机上电，不一定会保持
#define DEVICE_CTROL_ENABLE_OPERATION           0x0F    //使能电机运动
#define DEVICE_CTROL_QUICK_STOP                 0x02    //快速停止并保持
#define DEVICE_CTROL_HALT                       0x10F   //立即停止
#define DEVICE_CTROL_SHUT_DOWN                  0x06    //关断电机电源
#define DEVICE_CTROL_DISABLE_VOLTAGE            0x00    //禁止电压输出
#define DEVICE_CTROL_FAULT_RESET                0x80    //复位故障
#define DEVICE_CTROL_START_HOMING_OPERATION     0x1F    //启动寻零模式
#define DEVICE_CTROL_ENABLE_OPERATION1          0x3F    //使能电机运动
uint32_t MotorDeviceControlCmd(uint8_t idx, uint16_t usControlMode);
uint32_t MotorDeviceControlCmdWithJudge(uint8_t idx, uint16_t usControlMode);
uint32_t MotorReadEnableStatus(uint8_t idx, uint16_t *enableStatus);
uint32_t MotorSetMotorEnableStatus(uint8_t idx, uint16_t enableStatus);

//电机控制器状态定义
#define DIVICE_CTROL_STATUS_EXT_STOP1           0x01    //外部停止1
#define DIVICE_CTROL_STATUS_EXT_STOP2           0x02    //外部停止2
#define DIVICE_CTROL_STATUS_STALL               0x04    //堵转
uint32_t MotorSetMotorControlStatus(uint8_t idx, uint8_t controlStatus);

uint32_t MotorSetAbortConnectionOptionCode(uint8_t idx, int16_t optionCode);


uint32_t MotorSetTargetVelocity(uint8_t idx, int32_t vel, rt_bool_t checkAck);
uint32_t MotorSetTargetPosition(uint8_t idx, int32_t pos, rt_bool_t checkAck);
uint32_t MotorSetTargetCurrent(uint8_t idx, int32_t current, rt_bool_t checkAck);
uint32_t MotorGetTargetVelocity(uint8_t idx, int32_t* vel, DRIVER_TYPE driveType, int32_t counts);
uint32_t MotorGetTargetPosition(uint8_t idx, int32_t* pos);
uint32_t MotorSetCurActualPosition(uint8_t idx, int32_t pos);
uint32_t MotorSetProfileVelocity(uint8_t idx, uint32_t vel, rt_bool_t checkAck);
uint32_t MotorSetProfileAcc(uint8_t idx, uint32_t acc, rt_bool_t checkAck);
uint32_t MotorSetProfileDec(uint8_t idx, uint32_t dec, rt_bool_t checkAck);

//归零模式方案定义
#define HOMMING_METHOD_37_ACTUAL_POSITION       0x25    //实际位置
uint32_t MotorSetHomingMethod(uint8_t idx, int8_t method);
uint32_t MotorGetHomingMethod(uint8_t idx, int8_t* method);
uint32_t MotorSetHomingPosition(uint8_t idx, int32_t pos);
uint32_t MotorSetHomingStallTorq(uint8_t idx, uint16_t torq); //堵转找寻原点时的检测转矩
uint32_t MotorSetHomingStallTime(uint8_t idx, uint16_t time); //堵转找寻原点时的检测时间
uint32_t MotorSetHomingVelocity(uint8_t idx, int32_t vel);    //找原点速度
uint32_t MotorSetHomingAcc(uint8_t idx, uint32_t acc);        //原点回归加速度

//电机状态定义
#define DEVICE_STATUS_MASK                      0x6f    //有效位0,1,2,3,5,6
#define DEVICE_STATUS_NOT_READY                 0x00    //电机驱动未使能
#define DEVICE_STATUS_SWITCH_ON_DISABLE         0x40    //电机驱动无电压输出
#define DEVICE_STATUS_READY_SWITCH_ON           0x21    //准备开启
#define DEVICE_STATUS_SWITCH_ON                 0x23    //开启
#define DEVICE_STATUS_OPERATION_ENABLE          0x27    //电机驱动使能
#define DEVICE_STATUS_QUICK_STOP                0x07    //快速停止
uint32_t MotorReadStatus(uint8_t idx, uint16_t *status);
uint32_t MotorReadFDKWorkStatus(uint8_t idx, uint16_t *status);

//驱动器读取相关接口
uint32_t MotorReadErrorCode(uint8_t idx, uint16_t *errorCode);
uint32_t MotorReadCurrentCmd(uint8_t idx, int32_t *current, int32_t *currentAverage);
uint32_t MotorReadAvarageCurrentCmd(uint8_t idx, int32_t *currentAverage);
uint32_t MotorReadVelocity(uint8_t idx, int32_t *vel);
uint32_t MotorReadPosition(uint8_t idx, int32_t *pos);
uint32_t MotorReadIOStatus(uint8_t idx, uint16_t *ioStatus);
uint32_t MotorSendReadStatus(uint8_t idx);
uint32_t MotorSendReadControlStatus(uint8_t idx);
uint32_t MotorSendReadAvarageCurrentCmd(uint8_t idx);
uint32_t MotorSendReadPosCmd(uint8_t idx);
uint32_t MotorSendReadVelocity(uint8_t idx);
uint32_t MotorSendReadDriverTmpCmd(uint8_t idx);
uint32_t MotorSendReadTmpCmd(uint8_t idx);
uint32_t MotorSendReadVolCmd(uint8_t idx);

//canopen对象字典访问
uint32_t MotorCanSdoSet(uint8_t idx, uint8_t *data, uint8_t len);
uint32_t MotorCanSdoGet(uint8_t idx, uint8_t *data, uint8_t len);
uint32_t MotorCanDisableTransimitPdo(uint8_t idx);

//电机控制测试
void TestMotorControl(uint8_t* cmdData, uint8_t size);

#endif /* _MOTOR_DRIVEN_H */

