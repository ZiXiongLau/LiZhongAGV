#ifndef _MOTOR485_DRIVEN_H
#define _MOTOR485_DRIVEN_H

#include <preDef.h>

/*�趨��ȡ�ȴ���ʱʱ��us*/
#define MOTOR485_WRITE_TIMEOUT_MS 	  5 //5ms
#define MOTOR485_READ_TIMEOUT_MS 	  6 //5ms

//����ģʽ����
typedef enum
{
    MOTOR_MODE_SPWM_DIR = 2,  //��pwm+�������ģʽ
    MOTOR_MODE_IO = 3,  //io����ģʽ
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

//���״̬����
#define DEVICE_STATUS_MASK                      0x6f    //��Чλ0,1,2,3,5,6
#define DEVICE_STATUS_NOT_READY                 0x00    //�������δʹ��
#define DEVICE_STATUS_SWITCH_ON_DISABLE         0x40    //��������޵�ѹ���
#define DEVICE_STATUS_READY_SWITCH_ON           0x21    //׼������
#define DEVICE_STATUS_SWITCH_ON                 0x23    //����
#define DEVICE_STATUS_OPERATION_ENABLE          0x27    //�������ʹ��
#define DEVICE_STATUS_QUICK_STOP                0x07    //����ֹͣ
//uint32_t MotorReadStatus(uint8_t idx, uint16_t *status);


#endif /* _MOTOR485_DRIVEN_H */

