#ifndef _MOTOR_DRIVEN_H
#define _MOTOR_DRIVEN_H

#include <preDef.h>

/*�趨CAN��ȡ�ȴ���ʱʱ��us*/
#define CAN_WRITE_TIMEOUT_MS 	  5  //5ms
#define CAN_READ_TIMEOUT_MS 	  8  //8ms

//SDO���������ֽڶ���
#define SDO_CLIENT_ID_BASE                      0x600   //SDO����ͻ��˻�׼ID
#define SDO_COMMAND_SPECIFIER_UPLOAD            0x40    //SDO��ȡ����
#define SDO_COMMAND_SPECIFIER_MULTI_UPLOAD      0x41    //SDO��ȡ����(����4�������ֽ�)

#define SDO_COMMAND_SPECIFIER_DOWNLOAD4         0x23    //SDOд���������4�������ֽڵĳ���
#define SDO_COMMAND_SPECIFIER_DOWNLOAD3         0x27    //SDOд���������3�������ֽڵĳ���
#define SDO_COMMAND_SPECIFIER_DOWNLOAD2         0x2b    //SDOд���������2�������ֽڵĳ���
#define SDO_COMMAND_SPECIFIER_DOWNLOAD1         0x2f    //SDOд���������1�������ֽڵĳ���

//�����ȡ�̣߳��첽��������
void MotorReadProcess(void);

//����NMT����������
#define SET_NMT_STATE_INITIALIZATION            0x81   //��λ�ڵ㣬�����ʼ��״̬���ϵ�ʱ��״̬
#define SET_NMT_STATE_OPERATIONAL               0x01   //�������״̬������SDO��PDO����
uint32_t SetEposNMTState(uint8_t cs, uint8_t idx);

//�������ģʽ����
#define MOTOR_OPERATION_MODE_UNKOWN             0x00    //δ֪ģʽ
#define MOTOR_OPERATION_MODE_PPM                0x01    //�ļ�����λ��ģʽ
#define MOTOR_OPERATION_MODE_CSV                0x09    //������ͬ���ٶ�ģʽ
#define MOTOR_OPERATION_MODE_PVM                0x03    //�ļ������ٶ�ģʽ
#define MOTOR_OPERATION_MODE_HMM                0x06    //����ģʽ
#define MOTOR_OPERATION_MODE_PTM                0x04    //�ļ���������ģʽ
#define MOTOR_OPERATION_MODE_CST                0x0A    //������ͬ������ģʽ
uint32_t MotorSetOperationMode(uint8_t idx, int8_t i8OperationMode);
uint32_t MotorGetOperationMode(uint8_t idx, int8_t* i8OperationMode);

//������������
#define DEVICE_CTROL_SWITCH_ON                  0x07    //����ϵ磬��һ���ᱣ��
#define DEVICE_CTROL_ENABLE_OPERATION           0x0F    //ʹ�ܵ���˶�
#define DEVICE_CTROL_QUICK_STOP                 0x02    //����ֹͣ������
#define DEVICE_CTROL_HALT                       0x10F   //����ֹͣ
#define DEVICE_CTROL_SHUT_DOWN                  0x06    //�ضϵ����Դ
#define DEVICE_CTROL_DISABLE_VOLTAGE            0x00    //��ֹ��ѹ���
#define DEVICE_CTROL_FAULT_RESET                0x80    //��λ����
#define DEVICE_CTROL_START_HOMING_OPERATION     0x1F    //����Ѱ��ģʽ
#define DEVICE_CTROL_ENABLE_OPERATION1          0x3F    //ʹ�ܵ���˶�
uint32_t MotorDeviceControlCmd(uint8_t idx, uint16_t usControlMode);
uint32_t MotorDeviceControlCmdWithJudge(uint8_t idx, uint16_t usControlMode);
uint32_t MotorReadEnableStatus(uint8_t idx, uint16_t *enableStatus);
uint32_t MotorSetMotorEnableStatus(uint8_t idx, uint16_t enableStatus);

//���������״̬����
#define DIVICE_CTROL_STATUS_EXT_STOP1           0x01    //�ⲿֹͣ1
#define DIVICE_CTROL_STATUS_EXT_STOP2           0x02    //�ⲿֹͣ2
#define DIVICE_CTROL_STATUS_STALL               0x04    //��ת
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

//����ģʽ��������
#define HOMMING_METHOD_37_ACTUAL_POSITION       0x25    //ʵ��λ��
uint32_t MotorSetHomingMethod(uint8_t idx, int8_t method);
uint32_t MotorGetHomingMethod(uint8_t idx, int8_t* method);
uint32_t MotorSetHomingPosition(uint8_t idx, int32_t pos);
uint32_t MotorSetHomingStallTorq(uint8_t idx, uint16_t torq); //��ת��Ѱԭ��ʱ�ļ��ת��
uint32_t MotorSetHomingStallTime(uint8_t idx, uint16_t time); //��ת��Ѱԭ��ʱ�ļ��ʱ��
uint32_t MotorSetHomingVelocity(uint8_t idx, int32_t vel);    //��ԭ���ٶ�
uint32_t MotorSetHomingAcc(uint8_t idx, uint32_t acc);        //ԭ��ع���ٶ�

//���״̬����
#define DEVICE_STATUS_MASK                      0x6f    //��Чλ0,1,2,3,5,6
#define DEVICE_STATUS_NOT_READY                 0x00    //�������δʹ��
#define DEVICE_STATUS_SWITCH_ON_DISABLE         0x40    //��������޵�ѹ���
#define DEVICE_STATUS_READY_SWITCH_ON           0x21    //׼������
#define DEVICE_STATUS_SWITCH_ON                 0x23    //����
#define DEVICE_STATUS_OPERATION_ENABLE          0x27    //�������ʹ��
#define DEVICE_STATUS_QUICK_STOP                0x07    //����ֹͣ
uint32_t MotorReadStatus(uint8_t idx, uint16_t *status);
uint32_t MotorReadFDKWorkStatus(uint8_t idx, uint16_t *status);

//��������ȡ��ؽӿ�
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

//canopen�����ֵ����
uint32_t MotorCanSdoSet(uint8_t idx, uint8_t *data, uint8_t len);
uint32_t MotorCanSdoGet(uint8_t idx, uint8_t *data, uint8_t len);
uint32_t MotorCanDisableTransimitPdo(uint8_t idx);

//������Ʋ���
void TestMotorControl(uint8_t* cmdData, uint8_t size);

#endif /* _MOTOR_DRIVEN_H */

