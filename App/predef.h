#ifndef __PREDEF_H
#define __PREDEF_H

#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "FilterDef.h"

#define CODE_VERSION        "V1.002B077"

//Ӳ���汾��־λ
#define UFO_PRE_CHARGE_FLAG             0x01        //Ԥ���־
#define UFO_FOUR_DRIVER_FLAG            0x02        //������ͬ����־
#define UFO_CAN_RX_PI9                  0x04        //CAN���Ÿ���
#define UFO_FEEDBACK_MAXSPEED_FLAG      0x08        //�ٶȷ���ֵȡ�������ٶȱ�־(δ������ȡƽ���ٶ�)
#define UFO_PC_CAN_FLAG                 0x10        //���ػ�canͨ�ű�־
#define UFO_BATTERY_CAN2_FLAG           0x20        //���ʹ��can2ͨ�ű�־
#define UFO_ERROR_WAIT_FLAG             0x40        //���ϵȴ���־�������й����Իָ�
#define UFO_ENABLE_REMOTE_HIGH_VEL_FLAG 0x80        //����ң�ظ��ٱ�־
#define UFO_NEW_PROTOCOL                0x100       //��ͨ��Э��
#define UFO_ENABLE_LOCK                 0x200       //ʹ�ܱ�բ
#define UFO_ENABLE_POS_JUDEGE           0x400       //ʹ�ܵ�λλ���ж�
#define UFO_LOCK_TWO_CTRL               0x800       //��բ�����˿ڿ���
#define UFO_ENABLE_SEND_MOTOR_DATA      0x1000      //ʹ�ܵ�������Ϸ�
#define UFO_LEFT_RIGHT_ALONE_FLAG       0x2000      //���Ҳ����������Ʊ�־
#define UFO_MOTOR_AS_FEEDBACK_FLAG      0x4000      //����ٶ���Ϊpid������־
#define UFO_ENABLE_REMOTE_QUCIK_STOP    0x8000      //ʹ��ң�ؽ���ֹͣ
#define UFO_BMS_JBD                     0x10000     //JBDƷ��485ͨ��bms
#define UFO_BMS_JIAYUAN                 0x20000     //��ԴƷ��485ͨ��bms
#define UFO_BMS_YBT                     0x40000     //�ű���canͨ��bms
#define UFO_PRECHARGE_INVERSE           0x80000     //Ԥ�����ŷ���
#define UFO_BMS_JIKONG                  0x100000    //����canͨ��bms
#define UFO_LOCK_INVERSE                0x200000    //��բ���ŷ���
#define UFO_BMS_BAIWEI                  0x400000    //��άcanͨ��BMS
#define UFO_BMS_YBT_v2                  0x800000    //�ű���v2.0 canͨ��bms
#define UFO_DISABLE_COLLISION_DETECT    0x1000000   //��ֹ��ײ���
#define UFO_KINCO_NEW_PCB               0x2000000   //����������ʹ����pcb����
#define UFO_NOT_USE_GET_SET_ACC         0x4000000   //��ʹ�û�ȡ�����趨���ٶ�
#define UFO_BMS_YBT_v3                  0x8000000   //�ű���v3.0 canͨ��bms
#define UFO_BMS_YBT_THREE_LARGE         0x10000000  //�ű������ִ�ƽ����
#define UFO_BMS_JIUPU_FOUR_LARGE        0x20000000  //���ִ�ƽ����յ��
#define UFO_ENABLE_FLAG_ONE             0x80000000  //ʹ�ܱ�־λ1����ع��ܣ���λ�ɳ����Զ���1��ǿ�Ʋ������

//��־λ1��ض���
#define IS_UFOONE_FLAG_SET(flag)        (gStUfoData.flagOne & (flag))
#define UFOONE_SET_ACC_OFFSET_VALID     0x01        //�趨Ŀ����ٶȲ�����Ч�����������ٶ����
#define UFOONE_MOTION_SWITCH_DEAL       0x02        //�趨����״̬�л��������������ٶȹ���
#define UFOONE_PC_USART_FLAG            0x04        //���ػ�����ͨ�ű�־(��ǰʹ��Ϊ����5��������ʹ���������ڣ�����Ҫ����Ӧ����)
#define UFOONE_SOC_FROM_VOL             0x08        //����ֵ��Դ�ڵ�ѹ����
#define UFOONE_DIFF_DRIVER_CURRENT      0x10        //��ͬ����������־��������������ʲ�һ��
#define UFOONE_BMS_500K                 0x20        //BMS������ָ��500k
#define UFOONE_TWO_WHEL_DEFF_DRIV       0x40        //���ֲ�������
#define UFOONE_FOUR_ALONE_FLAG          0x80        //���ֶ�����־
#define UFOONE_BRAKE_DEC_TEST_FLAG      0x100       //ɲ�����ٶȲ���
#define UFOONE_FOUR_TURN_MOTOR          0x200       //�ĸ�ת������־
#define UFOONE_USE_MOTOR_RATIO          0x400       //�ٶȻ���ʹ�õ���ٱ�
#define UFOONE_BRAKE_ADVANCE_START      0x800       //ɲ����ǰ����
#define UFOONE_BRAKE_STOP_ACCORD_OIL    0x1000      //ɲ��������ѹֹͣ
#define UFOONE_MOTOR_STATE_DUAL_CTRL    0x2000      //����ϵ��ʹ�ֿܷ�����
#define UFOONE_NO_NAV_FLAG              0x4000      //�޹ߵ���־
#define UFOONE_DISABLE_MIN_VOL_KPH_FLAG 0x8000      //��ֹ��С��ѹ��Ӧʱ�����Ʊ�־
#define UFOONE_BMS_SMTK_FOUR_MID        0x1000000   //������ƽ��˼��̩�˵��

#pragma pack (1)  //1�ֽڶ���
/**
 * UFO Data
 */
typedef struct
{
    uint16_t                minVol;                 // �����͵�ѹ->2
    uint8_t                 fullVol;                // ��������ѹ->3
    uint8_t                 preChargePort;          // Ԥ�����ƶ˿�->4
    uint8_t                 preChargePin;           // Ԥ����������->5
    int16_t                 ratioVel;               // �ٶ�ת���ת�ٰٷֱ�ֵ[v(m/s)=n(rpm)/i/60*2��*R(mm)/1000 --> ratioVel=1/(1/i/60*2��*R(mm)/1000)]->7
    uint32_t                flag;                   // ��־λ->11
    uint32_t                oilPressStand;          // ��ѹ��׼ֵ->15
    uint8_t                 lockPort;               // ��բ���ƶ˿�->16
    uint8_t                 lockPin;                // ��բ��������->17
    uint32_t                flagOne;                // ��־λ1->21
    uint8_t                 ipLastAdr;              // ip���һ���ֽڵ�ַ->22
    uint8_t                 minVol20kph;            // 20kph����ʱ��Ҫ�����͵�ѹ->23
    uint8_t                 minVol40kph;            // 40kph����ʱ��Ҫ�����͵�ѹ->24
    uint8_t                 minVol60kph;            // 60kph����ʱ��Ҫ�����͵�ѹ->25
    uint8_t                 minVol80kph;            // 80kph����ʱ��Ҫ�����͵�ѹ->26
    uint8_t                 minVol100kph;           // 100kph����ʱ��Ҫ�����͵�ѹ->27
    uint8_t                 minVol110kph;           // 110kph����ʱ��Ҫ�����͵�ѹ->28
    uint8_t                 reserved[4];            // Ԥ��->32
}ST_UFO_DATA;
#pragma pack ()  //1�ֽڶ��� 

//�����Ŷ���
enum
{
    M_LEFT = 0,                         //���������
    M_RIGHT,                            //���������
    M_TURN,                             //ת����
    M_BRAKE,                            //ɲ�����
    M_LEFT_ONE,                         //���������1
    M_RIGHT_ONE,                        //���������1
    M_TOTAL_NUM,
    M_NUM_PI7
};

//ǰ������Ŷ���
typedef enum
{
    W_LEFT_FRONT = 0,                   //��ǰ��
    W_RIGHT_FRONT,                      //��ǰ��
    W_LEFT_REAR,                        //�����
    W_RIGHT_REAR,                       //�Һ���
    W_TOTAL_NUM,
}EN_WHEEL_ORDER;

//����������
#define ENABLE_CANOPEN_DRIVER           //ʹ��canopen������
enum
{
    DRIVER_TYPE_NONE,       //�����������ͣ���ʾ���������
    DRIVER_TYPE_STAND,      //��׼���������ͣ���ELMO������
    DRIVER_TYPE_EPOS,       //EPOS������
    DRIVER_TYPE_COMPLEY,    //COMPLEY������
    DRIVER_TYPE_NIMOTION,   //����ʤ������
    DRIVER_TYPE_PUSI,       //��˼������
    DRIVER_TYPE_QILING,     //����������
    DRIVER_TYPE_NOT_CANOPEN,//��canopen������
    DRIVER_TYPE_KINCO,      //����������
    DRIVER_TYPE_HCX,        //�괴��������
    DRIVER_TYPE_LEADSHINE,  //����������
    DRIVER_TYPE_DUOJI_GDW,  //GDW���8.4V 1520us/333Hz
    DRIVER_TYPE_NOT_CANOPEN_END = 30,//��canopen������������
    DRIVER_TYPE_FDK = 31,   //��ÿ�������
    DRIVER_TYPE_KINCO_CAN = 32,//����can�ӿ�������
};
typedef uint32_t DRIVER_TYPE;
#define IS_NOT_CANOPEN_DRIVER(driver)   ((driver >= DRIVER_TYPE_NOT_CANOPEN) && (driver <= DRIVER_TYPE_NOT_CANOPEN_END))

//�����־λ
#define ENABLE_AUTO_HOMMING             0x01        //ʹ���Զ�Ѱ��
//#define ENABLE_STO                      0x02        //ʹ��STO
#define MOTOR_USE_PI7_POWER             0x04        //ʹ��PI7��Դ���ƶ˿�
#define MOTOR_NOT_JUDGE_BOOTUP          0x08        //�ϵ粻��������¼�����Ϊ�յ������źź󲻴�������������
#define MOTOR_NEED_SET_RUN_PARAS        0x10        //��Ҫ�趨���в���
//#define MOTOR_FIX_DIRECT_POSOTIVE       0x20        //����̶�������
//#define MOTOR_HOMING_RECORD_POS         0x40        //���Ѱ���¼λ�ã��Է�ֹ�з��ֵ�����µ�����ܼ�ʱͣ����
//#define MOTOR_REACH_POS_NEED_RETURN     0x80        //�����λ����Ҫ��ת����Ҫ������ͱõ������ֹ��ѹ
#define MOTOR_CURRENT_ADJUST_SPEED      0x100       //��������ת��
#define MOTOR_RELATED_SYNCHRO           0x200       //���������ȫͬ����־
#define MOTOR_ENABLE_FLAG               0x400       //���ʹ�ܱ�־
#define MOTOR_DIR_FLAG                  0x800       //��������־
#define MOTOR_INVALID_FLAG              0x1000      //�����Ч��־
#define MOTOR_DIR_INVERSE_FLAG          0x2000      //��������־
//#define MOTOR_LAXIAN_FLAG               0x4000      //���߱궨��־
#define MOTOR_PRESS_SENSOR_FLAG         0x8000      //ѹ����������־
#define MOTOR_POWER_OFF_RECORDPOS       0x10000     //�ϵ��¼λ��
#define MOTOR_READ_POS_FLAG             0x20000     //��ȡλ�ñ�־
#define MOTOR_PWM_CONTRL_MODE           0x40000     //���pwm����ģʽ
#define MOTOR_POWER_ON_INVERSE          0x80000     //��Դ���ŷ���
#define MOTOR_DUAL_POWER                0x100000    //˫��Դ����
#define MOTOR_TURN_PID_TO_ACC           0x200000    //ת��pid�������ٶȲ�����
#define MOTOR_ENABLE_STALL_HOMMING      0x400000    //ʹ�ܶ�תѰ��

/* boolean type definitions */
typedef int                             rt_bool_t;      /**< boolean type */
#define RT_TRUE                         1               /**< boolean true  */
#define RT_FALSE                        0               /**< boolean fails */

#define FLT_EPSILON                     1e-5f

//LED��Ŷ���
enum
{
    LED0, LED1, LED2, LED3
};

//LED״̬����
typedef enum
{
    LED_ON,     //��led
    LED_OFF,    //�ر�led
}ENUM_LED_STATE;

//����ת������
typedef enum
{
    DIR_CW,     //��ת
    DIR_CCW,    //��ת
    DIR_STOP,   //ֹͣ
}ENUM_DIR;

//�������Ͷ���
typedef enum
{
    ERROR_NONE,          //�޹���
    ERROR_L_NORMAL,      //�������ϣ��ɳ���ͨ��������ָ��������λָ�ʧ�ܽ�ת��ɸߵȼ�����
    ERROR_L_HIHG,        //�ߵȼ����ϣ�ֻ��ͨ���ϵ糢�Իָ��������λָ�ʧ�ܽ�ת�����ߵȼ�����
    ERROR_L_VERY_HIGH,   //��ߵȼ����ϣ��޷��ָ�
}ERROR_LEVEL;

//�����ת��ģʽ
typedef enum
{
    BACK_DEFF_NONE = 0,         //�����޲���
	BACK_DEFF_ACCORD_TURN = 1,  //��������ת��ֵ��С���в���
	BACK_DEFF_MAX = 2,          //����������
} BackDeffMode;

//��������붨��
#define WARNING_MOTOR_NUM_OFFSET        20          //����ģ�����ƫ��ֵ
#define IS_WARNING_CODE(motorNum)       (motorNum >= WARNING_MOTOR_NUM_OFFSET)  //�ж��Ƿ��Ǿ�����
#define IS_NO_WARNING_OR_ERROR_CODE     (0 == gErrorResult) //�޾���͹���
#define IS_ENABLE_SET_WARNING_CODE      (IS_NO_WARNING_OR_ERROR_CODE || (IS_WARNING_CODE(gErrorMotorNum) && (sys_para->CAR_RTinf.Link & LINK_REV_STOP_CMD)))
#define SET_WARNING_CODE(motorNum, lResult) SetErrorCode((motorNum + WARNING_MOTOR_NUM_OFFSET), lResult, ERROR_NONE)
#define WARNING_CODE_BASE               0x1000
#define WARNING_CODE_LOW_VOLTAGE                    (WARNING_CODE_BASE + 0x01)  //����״̬ʱ�͵�ѹ����
#define WARNING_CODE_RUNNING_LOW_VOLTAGE            (WARNING_CODE_BASE + 0x02)  //���й����е͵�ѹ����(��������ٵͼ��ٶȳ�������)
#define WARNING_CODE_NAV_ABNORMAL                   (WARNING_CODE_BASE + 0x03)  //���й����йߵ��ٶ�ֵ�쳣
#define WARNING_CODE_ABNORMAL_DECELERATION          (WARNING_CODE_BASE + 0x04)  //���й������쳣����(��ײ��Ħ�ء���ѹ)
#define WARNING_CODE_OVER_LOADER_SINGLE             (WARNING_CODE_BASE + 0x05)  //������ι���
#define WARNING_CODE_OVER_LOADER_ONE_MINUTE         (WARNING_CODE_BASE + 0x06)  //���һ�ְ����ۼƹ���
#define WARNING_CODE_OVER_LOADER_NEED_REST          (WARNING_CODE_BASE + 0x07)  //����ѹ�����ֹͣ��ȴ�������ٽ�����һ�ι��ز���
#define WARNING_CODE_LOW_VOLTAGE_TO_KPH             (WARNING_CODE_BASE + 0x08)  //��ǰ�����ϵͲ�֧�ֵ�ǰ��������(��������ٳ�������)

//������ϴ��붨��
#define ERROR_CODE_BASE                 0x9000
#define ERROR_CODE_CAN_LINE_OFF                     (ERROR_CODE_BASE + 0x01)    //canͨ������
#define ERROR_CODE_OPERATRION_MODE_SET_FAILED       (ERROR_CODE_BASE + 0x02)    //����ģʽ����ʧ��
#define ERROR_CODE_HOMMING_TIMEOUT                  (ERROR_CODE_BASE + 0x03)    //Ѱ�㳬ʱ
#define ERROR_CODE_FLAP_SIN_TIMEOUT                 (ERROR_CODE_BASE + 0x04)    //�����Ķ���ʱ
#define ERROR_CODE_MASS_POS_TIMEOUT                 (ERROR_CODE_BASE + 0x05)    //���ĵ��������ָ��λ�ó�ʱ
#define ERROR_CODE_MAIN_POWER_NOT_OEPN              (ERROR_CODE_BASE + 0x06)    //�ܵ�Դδ��
#define ERROR_CODE_MAIN_POWER_CLOSE_ABNORMAL        (ERROR_CODE_BASE + 0x07)    //�ܵ�Դ��;�쳣�ر�
#define ERROR_CODE_ECoder11M_BASE_START             (ERROR_CODE_BASE + 0x08)    //����ֵ��������������ʼ��ַ
#define ERROR_CODE_ECoder11M_BASE_END               (ERROR_CODE_BASE + 0x0C)    //����ֵ�����������������ַ
#define ERROR_CODE_MEASURE_SENSOR                   (ERROR_CODE_BASE + 0x0D)    //�����������������
#define ERROR_CODE_MOTOR_START_ERROR                (ERROR_CODE_BASE + 0x0E)    //�����������
#define ERROR_CODE_MOTOR_STALL                      (ERROR_CODE_BASE + 0x0F)    //�����ת
#define ERROR_CODE_LIMIT_ABNORMAL                   (ERROR_CODE_BASE + 0x10)    //����󴥷���λ
#define ERROR_CODE_MOTOR_ERROR_CHECK                (ERROR_CODE_BASE + 0x11)    //���IO�ڼ�⵽����
#define ERROR_CODE_MOTOR_BRAKE_ERROR                (ERROR_CODE_BASE + 0x12)    //ɲ������
#define ERROR_CODE_MOTOR_BRAKE1_ERROR               (ERROR_CODE_BASE + 0x13)    //ɲ������1
//#define ERROR_CODE_MOTOR_STALL1                     (ERROR_CODE_BASE + 0x14)    //��⵽�豸���٣���ײ��Ħ�ػ�ߵ������쳣
#define ERROR_CODE_MOTOR_STALL2                     (ERROR_CODE_BASE + 0x15)    //�����ת2ִ�д���
#define ERROR_CODE_MOTOR_SKID                       (ERROR_CODE_BASE + 0x16)    //����򻬻�ߵ����ݹ���
#define ERROR_CODE_MOTOR_TURN_EXE_ERROR             (ERROR_CODE_BASE + 0x17)    //ת����
#define ERROR_CODE_485_LINE_OFF                     (ERROR_CODE_BASE + 0x18)    //485ͨ������
#define ERROR_CODE_REMOTE_LINE_OFF                  (ERROR_CODE_BASE + 0x19)    //ң��ͨ�ų�ʱ
#define ERROR_CODE_PC_LINE_OFF                      (ERROR_CODE_BASE + 0x20)    //���ؿ�ͨ�ų�ʱ
#define ERROR_CODE_ZX_EXE_ERROR                     (ERROR_CODE_BASE + 0x21)    //ת���޷�ִ��
#define ERROR_CODE_WRONG_PARAS                      (ERROR_CODE_BASE + 0x22)    //����Ĳ�������
#define ERROR_CODE_ROTATION_SYNC_ERROR              (ERROR_CODE_BASE + 0x23)    //���ת��ͬ������
#define ERROR_CODE_CONTINUOUS_SKID                  (ERROR_CODE_BASE + 0x24)    //�����ʱ���
#define ERROR_CODE_MOTOR_ABNORMAL_OPERATION         (ERROR_CODE_BASE + 0x25)    //����쳣ȥʹ��

//#define ERROR_CODE_NAV_DATA_INVALID                 (ERROR_CODE_BASE + 0x30)    //����������Ч
//#define ERROR_CODE_NAV_LANE_DEPARTURE1              (ERROR_CODE_BASE + 0x31)    //����ƫ�뱨��1�����ϴξ���ƫ�Ƴ���1m
//#define ERROR_CODE_NAV_LANE_DEPARTURE2              (ERROR_CODE_BASE + 0x32)    //����ƫ�뱨��2�����ϴξ���ƫ�Ƴ���3s
//#define ERROR_CODE_NAV_LANE_DEPARTURE3              (ERROR_CODE_BASE + 0x33)    //����ƫ�뱨��3���ȶ��󣬷�ɢƫ������Ʒ�Χ

//ң�ػ�pc����״̬����
#define LINK_NONE           0
#define LINK_REMOTE_DATA    0x01        //���յ�ң������
#define LINK_PC_DATA        0x02        //���յ�PC����
#define LINK_REMOTE         0x04        //ң�ؿ��� 
#define LINK_HAND_OVER      0x08        //ң�����ƽ�����
#define LINK_PC             0x10        //PC����
#define LINK_AUTO_NAV       0x20        //�Զ���������
#define LINK_REMOT_OFF      0x40        //ң�عرջ���ʧ��״̬
#define LINK_PC_LOST        0x80        //PC���ݶ�ʧ
#define LINK_QUICK_STOP     0x100       //����ֹͣ
#define LINK_POWER_ON       0x200       //�ϵ�״̬
#define LINK_OUT_OF_CTRL    0x400       //ʧ��״̬
#define LINK_ERROR_STOP     0x1000      //����ͣ��
#define LINK_REMOTE_STOP    0x2000      //ң��ͣ��
#define LINK_REV_STOP_CMD   0x4000      //���յ��ٶ�Ϊ0��ֹͣ������Ϻ������˱�־λ

#define IS_EMERGENCY_STOP   (sys_para->CAR_RTinf.Link & (LINK_QUICK_STOP | LINK_ERROR_STOP | LINK_REMOTE_STOP | LINK_OUT_OF_CTRL))
#define IS_EMERGENCY_STOP_NOT_ERR   (sys_para->CAR_RTinf.Link & (LINK_QUICK_STOP | LINK_REMOTE_STOP | LINK_OUT_OF_CTRL))

#define CMD_VALUE_OFFSET    1000        //����ƫ��ֵ
#define CMD_VALUE_MAX       700         //�������ֵ
#define CMD_VEL_VALUE_MAX   3400        //�ٶ��������ֵ0.01m/s
#define CMD_ACC_VALUE_MAX   900         //���ٶ��������ֵ0.01m/s^2
#define FOUR_TURN_FRONT_ONLY_VEL 300    //��ǰ��ת���л��ٶ���ֵ0.01m/s

//Modbus�Ĵ�����ַ��Χ�궨�� 
#define	REG_HOLD_START	1
#define REG_HOLD_NREGS	500

//Modbus�Ĵ�����ַ���� 
extern unsigned short   usRegHoldBuf[REG_HOLD_NREGS];

#define SBUS_RX_pack_LEN    sizeof(SBUS_RX_pack)
#pragma pack (1)  //1�ֽڶ���
typedef struct
{	
	uint16_t pack_Mark;//���ݰ���־
	uint16_t Link; //���ӱ�־
    uint16_t CH[50];//����ͨ��
} SBUS_RX_pack;

#define PC_Remote_pack_LEN    sizeof(PC_Remote_pack)
typedef struct
{	
    uint16_t Link; //���ӱ�־
    uint16_t YM;   //����/ɲ�� -д
    uint16_t ZX;   //ת��  -д
    uint16_t max_vel;  //����ٶ�
    uint16_t VEL;  //�ߵ��ٶ�ֵ
    uint16_t SetAcc; //�趨���ٶ�
    uint16_t decRemainTime; //������ٵ�ʣ��ʱ�䣬��λms���������65000ms������65sʱ��ֵ����65s
    int16_t setDec; //�������ٶ��趨ƽ�����ٶȣ���λ0.01m/s2
    uint16_t reserved[8]; //Ԥ���ռ�
} PC_Remote_pack;

#define CAR_RTinf_pack_LEN    sizeof(CAR_RTinf_pack)
typedef struct
{	
	uint16_t Link; //���ӱ�־ ң��������PC���Ʊ�־
    int16_t YM;   //����/ɲ��
	int16_t ZX;   //ת��
	int16_t vel;  //�ߵ��ٶ�
	int16_t SetAcc;  //�趨���ٶ�
	int16_t max_vel; //����ٶ�
} CAR_RTinf_pack;


#define  SYS_PARA_LEN    sizeof(SYS_PARA)  //ע��ýṹ�峤�Ȳ��ܴ���usRegHoldBuf����ĳ��ȣ�������ܻ�ѰַԽ��
typedef struct
{
    uint16_t            reserved[104];  //Ԥ��
	SBUS_RX_pack        SBUS_rx;        //SBUS�յ�����	��ַƫ���� 104(Modbus�Ĵ�����ַ����λuint16) �ýṹ�峤�� 52*2 �ֽ�
	PC_Remote_pack      PC_Remote;      //PCң������	��ַƫ���� 156(Modbus�Ĵ�����ַ����λuint16) �ýṹ�峤�� 16*2 �ֽ�
	CAR_RTinf_pack      CAR_RTinf;      //����������Ϣ	��ַƫ���� 172(Modbus�Ĵ�����ַ����λuint16) �ýṹ�峤�� 6*2 �ֽ�
	
} SYS_PARA;

//(2)״̬��Ϣ�ϴ��������Ϸ�
#pragma pack (1)  //1�ֽڶ���
typedef struct
{
    //id-0x991
    unsigned short  batteryVoltage; //��ص�ѹ����λ0.1V
    short           batteryCurrent; //��ص�������λ0.1A
    unsigned char   batterySoc;     //��ص����ٷֱ�ֵ��0��100
    unsigned char   faultModule; 	//����ģ�飬0-�������ϣ�1-��1�����2-��1�����3-ת��4-ɲ����5-��2�����6-��2�����10-���
    unsigned short  errorCode;      //������	
    //id-0x992
    unsigned char   batteryTmp[8];  //���1��8�¶ȣ���λƫ��-40�棬ʵ��2��
    //id-0x993
    unsigned char   motorTmp[8];    //���1��8�¶ȣ���λƫ��-40�棬ʵ��6��
    //id-0x994
    unsigned char   driverTmp[8];   //������1��8�¶ȣ���λƫ��-40�棬ʵ��6��
    //id-0x995
    unsigned char   moduleTmp[6];   //�ض�λ���¶�̽ͷ1��8�¶ȣ���λƫ��-40�棬ʵ��Ԥ��4��,0-���ؿ�,1-��������2-��Դ��
    //id-0x996
    unsigned short  linkState;      //ң��״̬��Ϣ��pc���� or ң�ؿ���
    unsigned char   ctrlState;      //����״̬��Ϣ�����ʹ��״̬��ң�ظ��е��ٵ�λ
    short           turnTargetPos;      //ת��Ŀ��λ��
    short           turnCurPos;         //ת��ǰλ��
}RESPOND_STATE;
#pragma pack ()  //1�ֽڶ���

/*���췢�ͽṹ��*/
#pragma pack (1)  //1�ֽڶ���
typedef struct
{
    unsigned char header1;  //ͷ��1
    unsigned char header2;  //ͷ��2
    unsigned short functional_parameter_len;    //�����볤��
    unsigned short functional_code;             //������
    RESPOND_STATE RESPOND_STATE_DATA;           //���͵�����
    unsigned short crc16;                       //crcУ��
}SEND_RESPOND_STATE;
#pragma pack ()

//�ϻ����Խṹ��
//�ϻ�����
#define OLD_TEST_PAUSE            0x01   //�ϻ���ͣ
#define OLD_TEST_END              0x02   //�ϻ�����
#define OLD_TEST_START            0x04   //�ϻ����Կ�ʼ
#define OLD_TEST_NEED_UPDATE_CMD  0x08   //��Ҫ��������
#define OLD_TEST_POWER_NEW_CMD    0x10   //�յ��µĶ����ϻ�����
#define OLD_TEST_TURN_NEW_CMD     0x20   //�յ��µ�ת���ϻ�����
#define OLD_TEST_BRAKE_NEW_CMD    0x40   //�յ��µ�ɲ���ϻ�����

//�ϻ�ģ���־
#define NONE_AGING                0x00   //��ģ�����ϻ�
#define POWER_AGING               0x01   //�����ϻ�
#define TURN_AGING                0x02   //ת���ϻ�
#define BRAKE_AGING               0x04   //ɲ���ϻ�

typedef struct
{
    uint16_t oldTestStep;       //�ϻ��������в���
    uint16_t moduleAgingFlg;    //ģ���ϻ���־
    uint16_t forwardTime;       //ǰ��ʱ��(��λ:s)
    uint16_t backTime;          //����ʱ��(��λ:s)
    int16_t  forwardVel;        //ǰ���ٶ�(��λ:cm/s)
    int16_t  backVel;           //�����ٶ�(��λ:cm/s)
    uint16_t powerGapTime;      //ǰ�������ת�����ʱ��(��λ:s)
    int16_t  turnVel;           //ת���ٶ�(��λ:rpm)
    uint16_t turnRange;         //ת��Χ(0-700)
    uint16_t turnGapTime;       //��ת����תת�����ʱ��(��λ:s)
    int16_t  brakeVel;          //ɲ���ٶ�(��λ:rpm)
    uint16_t brakeInitPos;      //��ɲ��λ��(0-700)
    uint16_t brakeLimitPos;     //��ɲ��λ��(0-700)
    uint16_t brakeGapTime;      //��ɲ�����ɲ��ת�����ʱ��(��λ:s)  
}OLD_TEST;

//�ϻ��������в���
typedef enum
{
    EXIT=0,       //�˳��ϻ�
    START,        //��ʼ�ϻ�
    STEP1,        //����1
    STEP2,        //����2
    STEP3         //����3
}AGING_STEP;
typedef enum
{
    POWER=0,      //����
    TURN,         //ת��
    BRAKE,        //ɲ��
    MODULE_NUM   //�ϻ�ģ������
}AGING_MODULE;



#define GPIO_PORT_TOTOAL_NUM    9
#define GPIO_PIN_MAX_NUM        16

extern SYS_PARA       *sys_para; 
extern GPIO_TypeDef* gGpioPortArray[GPIO_PORT_TOTOAL_NUM];

//ͨ�ù�ʽ����
#define DEF_PI  3.14159265358979f
#define ABS_VALUE(x)        (((x) >= 0) ? (x) : (-(x)))     //ȡ����ֵ
#define Limit(x, low, up)   ((x) <= (low) ? (low) : ((x) >= (up) ? (up) : (x))) //ȡ��λ���ֵ
#define IS_SAME_SYMBLE(x, y) ((((x) >= 0) && ((y) >= 0)) || (((x) <= 0) && ((y) <= 0))) //�ж��������������Ƿ���ͬ
#define IS_ACC_GTE_0(x, y)   ((((x) > 0) && ((y) >= 0)) || (((x) < 0) && ((y) <= 0))) //�жϼ��ٻ�����
#define MAX(x, y)           (x > y ? x : y)                 //ȡ�ϴ�ֵ
#define CountsToAngle(motorNum, lCounts) fmod((float)(lCounts) * 360.0f / gStMotorData[motorNum].ratio / gStMotorData[motorNum].counts, 360.0f)
#define AngleToCounts(motorNum, lAngle) (lAngle * gStMotorData[motorNum].ratio / 360.0f * gStMotorData[motorNum].counts)
#define GPIO_PORT(x)    ((x < GPIO_PORT_TOTOAL_NUM) ? gGpioPortArray[x] : gGpioPortArray[0])
#define GPIO_PIN(x)     ((uint16_t)0x01 << x)
#define CountsToRemteValue(motorNum, cur) (((gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) ? (-1) : 1) * ((cur >= gStMotorRunState[motorNum].PosInit) \
    ? ((cur - gStMotorRunState[motorNum].PosInit) * 700 / (gStMotorRunState[motorNum].limitPos2 - gStMotorRunState[motorNum].PosInit)) \
    : ((cur - gStMotorRunState[motorNum].PosInit) * 700 / (gStMotorRunState[motorNum].PosInit - gStMotorRunState[motorNum].limitPos1))) * 10 / 7) 
#define NAV_RATIO_SCALE         1000        //�ߵ��ٱȻ������
#define VelRpmToCm_s(velRpm)    ((velRpm) * NAV_RATIO_SCALE / gStUfoData.ratioVel + (((ABS_VALUE(velRpm) * NAV_RATIO_SCALE) % gStUfoData.ratioVel >= (gStUfoData.ratioVel >> 1)) ? ((velRpm > 0) ? 1 : (-1)) : 0))   //�ٶȵ�λ���㣬rpmתcm/s
#define VelCm_sToRpm(velCm_s)   ((int32_t)(velCm_s) * (int32_t)gStUfoData.ratioVel / NAV_RATIO_SCALE + (((ABS_VALUE((int32_t)(velCm_s)) * (int32_t)gStUfoData.ratioVel) % NAV_RATIO_SCALE >= (NAV_RATIO_SCALE >> 1)) ? ((velCm_s > 0) ? 1 : (-1)) : 0)) //�ٶȵ�λ���㣬cm/sתrpm
#define CalDifDriverCurrent(tarM, curM) ((gStMotorData[tarM].limitCurrent == gStMotorData[curM].limitCurrent) ? gStMotorRunState[curM].setCurrent : (((gStMotorRunState[curM].setCurrent << 11)  / ((gStMotorData[curM].limitCurrent == 0) ? 1 : gStMotorData[curM].limitCurrent) * gStMotorData[tarM].limitCurrent) >> 11)) //�õ�ǰ�����������Ŀ�����趨������������������ֵ�ɱ����Ŵ���С��Ϊ��ֹ�����˻�����int32λ���ֵ����������11λ�Ŵ�Ȼ���ȳ���ˣ������������11λ�л�����
#define WheelRpmToCm_s(Rpm)     ((Rpm) * NAV_RATIO_SCALE / gStUfoData.ratioVel)    //�ٶȵ�λ���㣬rpmתcm/s
#define IsBrakeLock ((gStUfoData.flag & UFO_ENABLE_LOCK) && (gStUfoData.lockPin < GPIO_PIN_MAX_NUM) && (LOCK_OFF == gLockFlag)) //��բ�رձ�־
#define CalRatioVelRpm(motorNum)  (IS_UFOONE_FLAG_SET(UFOONE_USE_MOTOR_RATIO) ? (gStMotorRevData[motorNum].speed * gStMotorData[motorNum].ratio) : gStMotorRevData[motorNum].speed) //������ٱȺ�ĵ��ת��
#define IsDiffOverPercent(val, standVal, percent)  ((ABS_VALUE((val - standVal) * 100) > ABS_VALUE(standVal * percent)) && (standVal != 0)) //��ֵ�Ա��Ƿ񳬹���׼ֵ�İٷֱ�

#define EVENT_READ_LOG          (0x01 << 0) //�����¼������λ 0
#define EVENT_WRITE_LOG         (0x01 << 1) //�����¼������λ 1
#define EVENT_PRINTF            (0x01 << 2) //�����¼������λ 2

void rt_kprintf(const char *fmt, ...);
void usart_kprintf(const char *fmt, ...);
void test_kprintf(const char *fmt, ...);
void rt_ksendData(uint8_t* dataBuf, uint16_t dataLen);
void rt_kprintfArray(void* array, uint16_t size, uint8_t format, uint8_t width);
void PrintfSendMsg(uint8_t* sendBuf, int sendLen, rt_bool_t useUdp);
void LockThread(void);
void UnLockThread(void);
uint16_t MODBUS_RTUCrc16(uint8_t *dataBuf, uint16_t len);
HAL_StatusTypeDef Motor485CheckRevData(uint8_t *revBuf, uint32_t revSize);
uint16_t Dichotomyfiltering(uint16_t* data, uint16_t dataLen);
void SetErrorInfo(uint16_t motorNum, uint16_t lResult);
void PrintErrorInfo(void);
void SetErrorCode(uint16_t motorNum, uint16_t lResult, ERROR_LEVEL errorLevel);
void TcpUpdateSendData(void);
void UdpRespondReved(uint8_t *data, uint16_t size);
void PrintfUfoVerAndSn(void);
uint8_t GetUfoControlStatus(void);

#include "flash_access.h"


//�������ݼ�¼���Ͷ���
#define DEBUG_DATA_TYPE_1       ((gFlashData.enableDebugDataSend & 0x81) == 0x01)   //һ������
#define DEBUG_DATA_TYPE_2       ((gFlashData.enableDebugDataSend & 0x82) == 0x02)   //��������
#define DEBUG_DATA_TYPE_3       ((gFlashData.enableDebugDataSend & 0x84) == 0x04)   //�������
#define DEBUG_DATA_TYPE_4       ((gFlashData.enableDebugDataSend & 0x88) == 0x08)   //PID����
#define DEBUG_DATA_TYPE_5       ((gFlashData.enableDebugDataSend & 0x90) == 0x10)   //��ʾ��ϸ��Ϣ
#define DEBUG_DATA_TYPE_6       ((gFlashData.enableDebugDataSend & 0xa0) == 0x20)   
#define DEBUG_DATA_TYPE_7       ((gFlashData.enableDebugDataSend & 0xc0) == 0x40)   //������ϸ����
#define DEBUG_DATA_TYPE_81      (gFlashData.enableDebugDataSend == 0x81)            //���ؿ�can����129
#define DEBUG_DATA_TYPE_82      (gFlashData.enableDebugDataSend == 0x82)            //ң������130
#define DEBUG_DATA_TYPE_83      (gFlashData.enableDebugDataSend == 0x83)            //������485ͨ������131
#define DEBUG_DATA_TYPE_84      (gFlashData.enableDebugDataSend == 0x84)            //�������132
#define DEBUG_DATA_TYPE_85      (gFlashData.enableDebugDataSend == 0x85)            //can�շ�����133
#define DEBUG_DATA_TYPE_86      (gFlashData.enableDebugDataSend == 0x86)            //������������134
#define DEBUG_DATA_TYPE_87      (gFlashData.enableDebugDataSend == 0x87)            //���ؿ�can���ת������135
#define DEBUG_DATA_TYPE_88      (gFlashData.enableDebugDataSend == 0x88)            //ң��ԭʼ����136
#define DEBUG_DATA_TYPE_89      (gFlashData.enableDebugDataSend == 0x89)            //ң�ؽ�������137
#define DEBUG_DATA_TYPE_8A      (gFlashData.enableDebugDataSend == 0x8a)            //��������쳣��ӡ138
#define DEBUG_DATA_TYPE_8B      (gFlashData.enableDebugDataSend == 0x8b)            //���������ϸ��ӡ139
#define DEBUG_DATA_TYPE_8C      (gFlashData.enableDebugDataSend == 0x8C)            //can1���ݼ��ٶ�����140
#define DEBUG_DATA_TYPE_8D      (gFlashData.enableDebugDataSend == 0x8D)            //���緢�����ݴ�ӡ141
#define DEBUG_DATA_TYPE_8E      (gFlashData.enableDebugDataSend == 0x8E)            //�ϵ�ȷ���������142
#define DEBUG_DATA_TYPE_8F      (gFlashData.enableDebugDataSend == 0x8F)            //ת���ٶ�����143
#define DEBUG_DATA_TYPE_90      (gFlashData.enableDebugDataSend == 0x90)            //�����ߵ��ٶȲ�ֵ����144
#define DEBUG_DATA_TYPE_91      (gFlashData.enableDebugDataSend == 0x91)            //��ʱ���жϵ���145
#define DEBUG_DATA_TYPE_92      (gFlashData.enableDebugDataSend == 0x92)            //pid����ʱ�����146
#define DEBUG_DATA_TYPE_93      (gFlashData.enableDebugDataSend == 0x93)            //fishComDebug����147
#define DEBUG_DATA_TYPE_94      (gFlashData.enableDebugDataSend == 0x94)            //can3����148
#define DEBUG_DATA_TYPE_95      (gFlashData.enableDebugDataSend == 0x95)            //udp�������ݴ��ڷ���149
#define DEBUG_DATA_TYPE_96      (gFlashData.enableDebugDataSend == 0x96)            //��������ж����ݴ�ӡ150
#define DEBUG_DATA_TYPE_97      (gFlashData.enableDebugDataSend == 0x97)            //flash��д��ӡ151
#define DEBUG_DATA_TYPE_98      (gFlashData.enableDebugDataSend == 0x98)            //���ƽ���ٶȼ����㵽����λ��ֵ��ӡ152

extern ST_UFO_DATA  gStUfoData;
extern uint32_t gUartNumWifiPrint;
extern int udp_print_connected_flag;
extern RESPOND_STATE gRespondState;                    //��Ӧ״̬��Ϣ
extern int32_t gRemoteStopDcc;
extern OLD_TEST gOldTest;                               //�ϻ�������ر���
extern uint8_t gU8DownloadType;                         //������������
extern uint8_t wdgDisableFlag;                          //��ֹ���Ź�ι����־
extern uint8_t gPrintfTestDataShowFlag;
extern rt_bool_t gMotorTestDataUploadFlag;              //���������ϴ���־
extern uint32_t gMotorTestDataUploadStartTime;          //��ʼ�ϴ�ʱ��
extern int32_t gAutoRatioVel;                           //��̬�ٱ�
extern int32_t gSteeringAngleVelDiff;                   //����ת��ʱת�Ƕ�Ӧ���ٶȲ�ֵcm/s
extern rt_bool_t gNoLoadFlag;                           //�޸��ر�־
extern BackDeffMode gBackWheelDeffDriverMode;           //���ֲ�������ģʽ��־
extern int32_t gMotorAvaVelFilter;                      //����˲����ٶ�
extern int32_t gMotorAvaPos;                            //���ƽ��λ��

#endif

