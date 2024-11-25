#ifndef _MOTOR485_DRIVEN_H
#define _MOTOR485_DRIVEN_H

#include <preDef.h>

/*设定读取等待超时时间us*/
#define MOTOR485_WRITE_TIMEOUT_MS 	  5 //5ms
#define MOTOR485_READ_TIMEOUT_MS 	  6 //5ms

//控制模式定义
typedef enum
{
    MOTOR_MODE_SPWM_DIR = 2,  //单pwm+方向控制模式
    MOTOR_MODE_IO = 3,  //io控制模式
}HXC_MOTOR_MODE;


HAL_StatusTypeDef Motor485SetTargetVelocity(uint32_t motorNum, int32_t vel);
HAL_StatusTypeDef Motor485ReadErrorCode(uint32_t motorNum, uint16_t* errorCode);
HAL_StatusTypeDef Motor485SetTargetCurrent(uint32_t motorNum, int32_t current);
HAL_StatusTypeDef Motor485ReadVelocity(uint32_t motorNum, int32_t *vel);
HAL_StatusTypeDef Motor485ReadCurrent(uint32_t motorNum, int16_t *current);
HAL_StatusTypeDef Motor485SetMotorContrlmode(uint32_t motorNum, HXC_MOTOR_MODE mode);
HAL_StatusTypeDef Motor485SetMotorMove(uint32_t motorNum, ENUM_DIR moveDir, int32_t vel);
HAL_StatusTypeDef Motor485SetTargetAbsPosAcordCurPos(uint32_t motorNum, int32_t pos);
HAL_StatusTypeDef Motor485SetTargetAbsPos(uint32_t motorNum, int32_t pos);
HAL_StatusTypeDef Motor485SetTargetAbsPosRealtime(uint32_t motorNum, int32_t pos);
HAL_StatusTypeDef Motor485SetActualPos(uint32_t motorNum, int32_t pos);
HAL_StatusTypeDef Motor485ReadPos(uint32_t motorNum, int32_t *pos);
HAL_StatusTypeDef Motor485ReadError(uint32_t motorNum, uint16_t *error);
void Motor485Test(uint8_t motorNum, uint8_t* testData);

//电机状态定义
#define DEVICE_STATUS_MASK                      0x6f    //有效位0,1,2,3,5,6
#define DEVICE_STATUS_NOT_READY                 0x00    //电机驱动未使能
#define DEVICE_STATUS_SWITCH_ON_DISABLE         0x40    //电机驱动无电压输出
#define DEVICE_STATUS_READY_SWITCH_ON           0x21    //准备开启
#define DEVICE_STATUS_SWITCH_ON                 0x23    //开启
#define DEVICE_STATUS_OPERATION_ENABLE          0x27    //电机驱动使能
#define DEVICE_STATUS_QUICK_STOP                0x07    //快速停止
//uint32_t MotorReadStatus(uint8_t idx, uint16_t *status);


#endif /* _MOTOR485_DRIVEN_H */

