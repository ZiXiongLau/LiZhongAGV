#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include <preDef.h>
#include "motor_pidctrl.h"
#include "pid_fuzzy.h"
#include "math.h"
#include "kinematics.h"


#define MVEL_RECORD_NUM                     50     //�ߵ��͵���ٶȲ�ֵ��¼���������ٴ���4
#define MVEL_STOP_FILTER_NUM                10     //���ֹͣ�ж��˲�����������С�ڵ���MVEL_RECORD_NUM
#define MVEL_FILTER_NUM                     7      //�ٶ��˲����������ڼ��㵱ǰ���ʵ���ٶȣ�����С�ڵ���MVEL_RECORD_NUM
#define MVEL_CAR_DIR_JUDGE_TOTAL_NUM        10     //��ʾ�ߵ��ٶȷ���ĵ������Ч����������С�ڵ���MVEL_RECORD_NUM
#define MVEL_CAR_DIR_JUDGE_NUM              3      //�������ߵ��ٶȷ���ĵ������������С�ڵ���MVEL_RECORD_NUM
#define MVEL_EXCLUDE_NUM                    2      //������ʵ���ٶ��˲��ų������Сֵ����
#define MACC_EXCLUDE_NUM                    3      //���������ٶ��˲��ų������Сֵ����
#define MACC_CAL_PERIOD_NUM                 10     //������ٶȼ�������ڸ���������С�ڵ���MVEL_RECORD_NUM
#define MACC_CURRENT_LEARN_NUM              3      //����ѧϰ��¼����

#define SET_VEL_RECORD_NUM                  6      //�趨�ٶȼ�¼����
#define M_SET_CURRENT_EXE_DELAY_NUM         5      //����趨����ִ���ͺ�����ڸ���

#define NAV_CAR_VEL_RECORD_NUM              MVEL_RECORD_NUM //�ߵ����ټ�¼����
#define CALRATIOL_FILTER_NUM                15     //�����ٱ��˲�����������С�ڵ���MVEL_RECORD_NUM
#define CALRATIOL_EXCLUDE_NUM               3      //�����ٱ��˲��ų������Сֵ����
#define NAV_ACC_RECORD_NUM                  5      //�ߵ����ٶ����ݼ�¼����
#define NAV_ACC_CAL_PERIOD_NUM              6      //�ߵ����ٶȼ�������ڸ���������С�ڵ���CALRATIOL_FILTER_NUM

#define CAR_MOTOR_FILTER_EXCLUDE_NUM        5      //����ߵ����ٺ͵���ٶȲ��˲��ų�����ֵ����
#define CAR_MOTOR_MAX_OR_MIN_FILTER_NUM     10     //�ߵ������ٶȲ��ж�ʱ��ȡ��С�����ֵ�˲��ĸ�������С��MVEL_RECORD_NUM��һ��
#define CAR_MOTOR_VELERR_OUT_RANGE(percent)  ((ABS_VALUE((gCarVelFilter - gMotorVelFilter) * 100) > ABS_VALUE(gMotorVelFilter * percent)) && (ABS_VALUE(gMotorVelFilter) >= 100)) //�ߵ������ٶȳ��ްٷֱ�
#define IS_CAR_DECELERATION_AND_SKID(motorNum, tarVel)  ((((tarVel >= 0) && (gStMotorRunState[motorNum].setAcc < 0)) || ((tarVel <= 0) && (gStMotorRunState[motorNum].setAcc > 0))) /*Ŀ���ٶȣ�=0�����趨Ŀ����ٶȣ�0����ǰ�����ٶ�,Ŀ���ٶȣ�=0�����趨Ŀ����ٶȣ�0�������˼��ٶ�*/\
            && (ABS_VALUE((gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] - gMotorAvaVelFilter) * 100) > ABS_VALUE(gCarVel[NAV_CAR_VEL_RECORD_NUM - 1]) * 150))  //���Ӵ򻬣��ߵ������ٶȲ�ֵ�ٷֱ���ֵ�ж�
#define IsBrakeValid    ((DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType) && (sys_para->CAR_RTinf.Link & LINK_PC))    //(PC�����²���Ч)

//��������߳����ʱ�䶨��
#define	MOTORSTATUS_ANALYSIS_TIME		    (20)   //���״̬�����ѯʱ��ms
#define	MOTORSTATUS_TIMEOUT_TIME		    (500)  //���״̬��ⳬʱʱ��ms
#define	MOTORSTATUS_TIMEOUT_LONGTIME		(3000) //���״̬��ⳤ��ʱʱ��ms

#define MOTOR_ALL_POWERING_TIMEOUT          (10000) //���е���ϵ��ܳ�ʱʱ��10s��һ���������ʱ�䣬ֱ���ж�Ϊ���ϵ�
#define MOTOR_LOCK_OFF_DELAY_TIME           (25000) //�������ϵ�󣬱�բδ�ص�����£���ʱ�ر�բ��ʱ��
#define MOTOR_ALL_POWEROFF_TIME             (5000)  //���е���µ�ʱ��
#define MOTOR_ALL_PRE_CHARGE_TIME           (1800)  //Ԥ���ʱ��

#define	MOTOR_CAN_TIMEOUT_TIMES             (5)     //CANͨ�ų�ʱ��������
#define	MOTOR_PECT_HOMMING_TURN_TIMEOUT	    (30000) //Ѱ��תȦ��ʱʱ��ms
#define	MOTOR_AUTO_HOMMING_TIME	            (7200000)//�Զ�Ѱ����ʱ��ms

#define MOTOR_NO_LOAD_JUDGE_MIN_HALF_PERIOD 50      //�޸����ж���С�񵴰�����
#define MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD 750     //�޸����ж�����񵴰�����

//�����ȼ��߳����ʱ�䶨��
#define MOTOR_CONTROL_THREAD_BASE_TIME		(2)     //��������̻߳�׼��ѯʱ��ms
#define LOW_VOLTAGE_CHECK_PERIOD		    (100)   //��ѹ�������ʱ��ms

#define	OPERATION_MODE_SET_TIMEOUT		    (10000) //ģʽ�л���ʱʱ��ms

#define MOTOR_ERROR_RECOVERY_TIMEOUT	    (180000)//��������Իָ��ĳ�ʱʱ��ms
#define MOTOR_ERROR_RECOVERY_TIMES	        (4)     //��������Իָ������ڶ��ٴι��ϲŽ����ϱ���������������Իָ�

#define MOTOR_RUN_MODE_STEP_IDLE    100     //������п���
#define MOTOR_RUN_MODE_STEP_START   101     //�����ʼ����
#define MOTOR_RUN_COMMON_STEP1      102     //ͨ�ò���1
#define MOTOR_RUN_COMMON_STEP2      103     //ͨ�ò���2
#define MOTOR_RUN_COMMON_STEP3      104     //ͨ�ò���3
#define MOTOR_RUN_COMMON_STEP4      105     //ͨ�ò���4
#define MOTOR_RUN_COMMON_STEP5      106     //ͨ�ò���5
#define MOTOR_RUN_COMMON_STEP6      107     //ͨ�ò���6
#define MOTOR_RUN_MODE_STEP_END     110     //�����������

#define M_BUFFER_IIt_LEN            10      //����ʱ�仺�泤��
#define IIt_JUDGE_VALID             0x01u   //�����ж���Ч��־
#define IIt_ENTER_WAIT_RECOVERY_FLAG 0x02u  //���ؽ���ȴ��ָ���־����ʱ�������ٴν��й��ز���
#define IIt_POSITIVE_CURRENT_FLAG   0x04u   //������������־�������������л�������������ʱ��Ҫ�����¸�������
#define IIt_SET_FLAG(motorNum, x)   gStMotorRunState[motorNum].IItFlag |= x
#define IIt_CLEAR_FLAG(motorNum, x) gStMotorRunState[motorNum].IItFlag &= (~x)
#define IIt_IS_SET_FLAG(motorNum, x) (gStMotorRunState[motorNum].IItFlag & x)

#define M_PID_EXE_FLAG              0x01    //pid���ִ���ִ�б�־
#define M_R_ACCORD_L_FLAG           0x02    //�ҵ������������־
#define M_FOUR_ALONE_FLAG           0x04    //�ĵ��������־

#define M_PID_IDLE_FLAG             0       //PID���б�־
#define M_PID_RUN_FLAG              1       //PID���б�־
#define M_PID_END_FLAG              2       //PID������־

//�趨�ٶ�״̬�л�����
enum{
    SET_VEL_STATE_IDLE,             //�趨�ٶȿ���״̬
    SET_VEL_STATE_START,            //�趨�ٶȿ�ʼ����״̬
    SET_VEL_STATE_ACC,              //�趨�ٶȼ��ٽ׶�
    SET_VEL_STATE_ACC_END,          //�趨�ٶȼ��ٽ���״̬
    SET_VEL_STATE_UNI,              //�趨�ٶ�����״̬
    SET_VEL_STATE_DEC_START,        //�趨�ٶȼ��ٿ�ʼ״̬
    SET_VEL_STATE_DEC,              //�趨�ٶȼ��ٽ׶�
};

//�������ģʽ��־����
enum{
    MOTOR_RUN_MODE_POWER_OFF,       //�ϵ�ģʽ
    MOTOR_RUN_MODE_STOP,            //ֹͣģʽ
    MOTOR_RUN_MODE_FLAP_SIN,        //�����Ķ�ģʽ
    MOTOR_RUN_MODE_FLAP_FIX,        //�����Ķ�ģʽ
    MOTOR_RUN_MODE_POS,             //λ��ģʽ
    MOTOR_RUN_MODE_POS_INFRARED,    //����λ��ģʽ
    MOTOR_RUN_MODE_HOMMING,         //Ѱ��ģʽ
    MOTOR_RUN_MODE_SPEED,           //�ٶ�ģʽ
    MOTOR_RUN_MODE_FLAP_FIX_CURRENT,//���ٵ����Ķ�ģʽ
    MOTOR_RUN_MODE_CURRENT_SPEED,   //�����ٶ�ģʽ
};

enum{
    MOTOR_RUN_MODE_NO_CHANGE,       //ģʽ�޸ı�
    MOTOR_RUN_MODE_CHANGED,         //ģʽ�����ı�
    MOTOR_RUN_PARA_CHANGED,         //���������ı�
};

//�������״̬��־����
enum{
    MOTOR_STATE_RUN,    //����״̬
    MOTOR_STATE_STOP,   //ֹͣ/��բ״̬
    MOTOR_STATE_POWER_OFF,   //�ϵ�״̬
    MOTOR_STATE_ERROR   //����״̬
};

//����ϵ��־����
enum{
    MOTOR_POWER_FLAG_OFF,   //�ϵ�״̬
    MOTOR_POWEROFF_FLAG_TIMECNT,   //�µ��ʱ״̬
    MOTOR_POWER_FLAG_TIMECNT,   //�ϵ��ʱ״̬
    MOTOR_POWER_FLAG_ON,   //�ϵ��ʱ���
}; 

//�����Ķ���־����
enum{ 
    PECT_STATE_NORMAL,                      //�����Ķ�
    PECT_STATE_WAIT_RUN_TO_ZERO_PHASE,      //ֹͣ��ȴ��˶�������λ
    PECT_STATE_START,                       //��ʼ�Ķ�
};
//Ѱ���־����
enum{
    HOME_FLAG_UNDEF,  //δ֪λ��
    HOME_FLAG_FINISH,   //��λ������ɱ�־
    HOME_FLAG_SET_MOVE,    //�趨����
    HOME_FLAG_RECORD_POS,  //��¼λ��
    HOME_FLAG_CLEAR_STOP,   //���ֹͣ��־
    HOME_FLAG_WAIT_STOP,    //�ȴ�ֹͣ��־
    HOME_FLAG_MOVE_CW,      //��������
    HOME_FLAG_MOVE_CCW,     //��������
    HOME_FLAG_WAIT_CW_STOP, //�ȴ�����ֹͣ
    HOME_FLAG_WAIT_CCW_STOP,//�ȴ�����ֹͣ
    HOME_FLAG_WAIT_CCW_STALL_STOP,//�ȴ������תֹͣ
    HOME_FLAG_SET_POS,      //�趨λ��
    HOME_FLAG_SET_MODE_IO,  //�趨io����ģʽ
    HOME_FLAG_SET_MODE_SPWM_DIR,  //�趨pwm+�������ģʽ
    HOME_FLAG_STOP,         //ֹͣ���
    HOME_FLAG_RECORD_INIT_POS,  //��¼��ʼλ��
    HOME_FLAG_RECORD_SETMAXOIL_POS,   //��¼���õ������ѹ��Ӧ��λ��
};
//λ�ñ�־����
enum{
    POS_FLAG_TARGET,   //Ŀ��λ��
    POS_FLAG_UNKOWN,   //δ֪λ��
    POS_FLAG_LIMIT_UP,   //����λ��
    POS_FLAG_LIMIT_DOWN,   //����λ��
    POS_FLAG_LIMIT_UP_MAX,   //���޼���λ�ã���������λ�������Ч
    POS_FLAG_LIMIT_DOWN_MAX,   //���޼���λ�ã���������λ�������Ч
};
//��ȡ���ݲ��趨��
enum{
    MOTOR_READ_FINISH   = 0,        //��ȡ���
    MOTOR_READING       = 0x01,     //���ڶ�ȡ��־
    MOTOR_READ_CURRENT  = 0x02,     //��ȡ����
    MOTOR_READ_VEL      = 0x04,     //��ȡ�ٶ�
    MOTOR_READ_POS      = 0x08,     //��ȡλ��
    MOTOR_READ_TMP      = 0x10,     //��ȡ�¶�
    MOTOR_READ_VOL      = 0x20,     //��ȡ��ѹ
};
//�������ģʽ�л����趨��
enum{
    OPERATION_MODE_UNKOWN,//δ֪
    OPERATION_MODE_SET,   //����ģʽ
    OPERATION_MODE_SET_HOME_METHOD,   //���û��㷽��
    OPERATION_MODE_SHUT_DOWN,   //�л���׼��״̬
    OPERATION_MODE_AUTO_CHANGE,   //���ݷ��ص�״̬�Զ��л�
    OPERATION_MODE_SET_NEW_POSITION,   //λ��ģʽ�������µ�λ��
    OPERATION_MODE_SET_FINISH,   //�л�ģʽ���
};
//������в������ñ�־
enum{
    RUN_PARAS_NOT_SET = 0,  //���в���δ����
    RUN_PARAS_PROFILE_VEL_SET = 0x01,  //�����ٶ����趨
    RUN_PARAS_PROFILE_ACC_SET = 0x02,  //�������ٶ����趨
    RUN_PARAS_PROFILE_DEC_SET = 0x04,  //�������ٶ����趨
};
//���ֹͣ���趨��
enum{
    OPERATION_STOP_FINISH,   //ֹͣ���
    OPERATION_DISABLE_VOLTAGE,    //�ض���������ѹ
    OPERATION_QUCIK_STOP,   //����ֹͣ
    OPERATION_QUCIK_STOP_WAIT,   //�ȴ�����ֹͣ
    OPERATION_HALT,   //ɲ��ֹͣ
    OPERATION_HALT_WAIT,   //�ȴ�ɲ��ֹͣ
    OPERATION_SLOW_STOP,   //����ֹͣ
    OPERATION_SLOW_STOP_WAIT,   //�ȴ�����ֹͣ
    OPERATION_CURRENT_SLOW_STOP_WAIT,   //����ģʽ�ȴ�����ֹͣ
    OPERATION_STOP_ADJUST,  //�жϵ���Ƿ���ʵֹͣ
};
//���Ŀ��λ�á��ٶ��л��궨��
enum{
    MOTOR_SET_NOTHING,      //������
    MOTOR_SET_TARGET_POS,   //ֻ����Ŀ��λ��
    MOTOR_SET_TARGET_VEL,   //ֻ����Ŀ���ٶ�
    MOTOR_SET_TARGET_CURRENT,   //ֻ����Ŀ�����
    MOTOR_SET_TARGET_POS_NEED_JUDGE,   //ֻ����Ŀ��λ�ã�������Ҫ�ж��������Ƿ�ִ��
    MOTOR_SET_TARGET_VEL_NEED_JUDGE,   //ֻ����Ŀ���ٶȣ�������Ҫ�ж��������Ƿ�ִ��
    MOTOR_SET_TARGET_CURRENT_NEED_JUDGE,   //ֻ����Ŀ�������������Ҫ�ж��������Ƿ�ִ��
    MOTOR_SET_ZEOR_VEL_LOCKOFF_NEED_JUDGE, //ͣ������ر�բ
    MOTOR_SET_TARGET_VEL_JUDGE,     //��ʼ�жϵ���Ƿ����Ŀ���ٶ�
    MOTOR_SET_TARGET_POS_JUDGE,     //��ʼ�жϵ���Ƿ����Ŀ��λ��
    MOTOR_SET_TARGET_POS_CLEAR_START_FLAG,  //����λ�����к�������б�־
};		
//��Դ״̬
typedef enum
{
	POWER_OFF = 0,
	POWER_ON =1
} PowerState; 
//ɲ��״̬
typedef enum
{
	BRAKE_OFF = 0,      //ɲ���ϵ�״̬
	BRAKE_ON  = 1,      //ɲ���ϵ�״̬
	BRAKE_OFF_CMD = 2,  //ɲ���ϵ�����״̬
	BRAKE_POS_CMD = 3,  //ɲ��λ������״̬
	BRAKE_OFF_FORCE = 4,//ɲ��ǿ�ƶϵ�״̬
	BRAKE_OFF_FORCE_CMD = 5,    //ɲ��ǿ�ƶϵ�����״̬
} BrakeState;
//ɲ���ȴ�״̬
typedef enum
{
	BRAKE_WAIT_NONE = 0,//ɲ������ȴ�
	BRAKE_WAIT_PRE  = 1,//ɲ��Ԥ�ȴ�
	BRAKE_WAIT = 2,     //ɲ���ȴ�
} BrakeWaitState;

//��բ״̬
typedef enum
{
	LOCK_OFF = 0,       //��բ�ϵ�״̬
	LOCK_ON  = 1,       //��բ�ϵ�״̬
} LockState;
//λ������
typedef enum
{
    POS_TYPE_NULL = 0,
	POS_TYPE_SINGLE_ABSOLUTE = 1,   //��Ȧ����λ��
	POS_TYPE_SINGLE_RELATE = 2,     //��Ȧ���λ��
	POS_TYPE_MULTI_ABSOLUTE = 3,    //��Ȧ����λ��
	POS_TYPE_TWO_ELECT = 4          //���˹��λ��ֵ
} PosType; 
//Ѱ������
typedef enum
{
    HOMMING_TYPE_NULL = 0,
	HOMMING_TYPE_STAND = 1,         //��׼Ѱ�㣬��类����������λ
	HOMMING_TYPE_OUT_IN_OFFSET = 2, //������ƫ��Ѱ�㣬��Ϊ��Ƭ���к�ȵģ������ǰ���ڹ�紫����λ�ã�Ϊ��֤Ѱ��׼�ȣ������뿪�ٽ��룬�ٴν���ʱ��������һ��ƫ��λ��
	HOMMING_TYPE_TWO_ELECT = 3      //���˹��Ѱ��
} HommingType; 

/**
 * Motor Runmode Data
 */
typedef struct
{
    uint8_t              run_mode;                       //�������ģʽ
    HommingType          homming_type;                   //Ѱ����Ҫ���뿪��紫������־����Ϊ��Ƭ���к�ȵģ������ǰ���ڹ�紫����λ�ã�Ϊ��֤Ѱ��׼�ȣ������뿪�ٽ���
    int16_t              target_value;                   //Ŀ��λ�û�Ŀ���ٶ�
    PosType              posType;                        //λ������
    //int8_t               flap_amp;                       //�Ķ�����
    //float                flap_fre;                       //�Ķ�Ƶ��
}ST_MOTOR_RUN_MODE_DATA;

/**
 * Motor Runstate Data
 */
typedef struct
{
    //�������״̬���
    uint8_t              change_flag;                    //�仯��־
    uint8_t              run_step;                       //��ǰ���в���
    uint32_t             runTotalTime;                   //���м�ʱ
    uint32_t             runDelayTime;                   //������ʱ
    uint32_t             pidDelayTime;                   //pid������ʱ
    uint32_t             pidNoExeTime;                   //pidδִ�м�ʱ
    uint32_t             pidPeriodTime;                  //pid��������
    uint8_t              pidRunFlag;                     //pid���б�־
    uint8_t              homeFinishFlag;                 //Ѱ����ɱ�־
    uint8_t              startRemoteFlag;                //����Զ��֡��־������PDO��SDO����
    uint8_t              skidFlag;                       //����򻬱�־
    uint8_t              skidOffCnt;                     //����򻬹رռ���
    uint32_t             skidTotalTime;                  //�򻬼�ʱ
    //��Դ����բ����λ���
    uint8_t              powerFlag;                      //����ϵ���ɱ�־�������ϵ�ʱ��ȴ���
    uint32_t             powerTime;                      //����ϵ�ʱ��
    uint8_t              enableFlag;                     //ʹ�ܱ�־
    ENUM_DIR             dirFlag;                        //�����־
    uint32_t             limit1Time;                     //��λ1�˲�ʱ���ۼ�
    uint32_t             limit2Time;                     //��λ2�˲�ʱ���ۼ�
    //�ٶ�λ�õ������
    int32_t              lastPoweroffPos;                //�ϴζϵ�ʱ��λ��
    int32_t              targetPos;                      //Ŀ��λ��
    int32_t              curPos;                         //��ǰλ��
    int32_t              limitPos1;                      //��λ1ֵ
    int32_t              limitPos2;                      //��λ2ֵ
    int32_t              PosInit;                        //��ʼֵ
    int32_t              setTargetVel;                   //�趨��Ŀ���ٶ�
    int32_t              lastSetTargetVel;               //�ϴ��趨��Ŀ���ٶ�
    int32_t              targetVel;                      //Ŀ���ٶ�
    int32_t              targetVelOffset;                //Ŀ���ٶ�ƫ��ֵ
    int32_t              motorVelRecord;                 //����ٶȼ�¼
    int32_t              setCurrent;                     //�趨�ĵ���
    int32_t              lastSetCurrent;                 //�ϴ��趨�ĵ���ֵ
    int32_t              accCurrent;                     //���ٶȲ����ĵ���ֵ
    int32_t              limitCurrent;                   //�޶�����ֵ
    uint8_t              posFlag;                        //���λ�ñ�־
    uint8_t              readStep;                       //��ȡ����
    uint8_t              targetPosJudgeFlag;             //λ�õ�λ�ж�
    uint8_t              targetPosFlag;                  //λ�û�Ѱ��ģʽʱ������Ŀ��λ�ñ�־
    uint8_t              targetPosVelSetFlag;            //�趨Ŀ��λ�ú�Ŀ���ٶȱ�־
    uint8_t              pidCurrentStartFlag;            //����pid���ڱ�־
    uint8_t              clearStartMoveFlag;             //�����ʼ���б�־
    uint32_t             readTimeCnt;                    //��ȡ��ʱ
    uint32_t             posJudgeTimeCnt;                //λ���жϼ�ʱ
    uint32_t             posJudgeCnt;                    //λ���жϼƴ�
    int32_t              lastPosRecord;                  //�ϴ�λ�ü�¼
    //�������
    uint32_t             infraFaultTimeTotal;            //�������ʱ���ۼ�
    uint32_t             infraTimeDelay;                 //�����ٶȵ�����ʱ
    float                infraPosRecord;                 //����λ�ü�¼
    float                infraPosTotal;                  //����λ��ֵ�ۼ�
    uint32_t             infraPosCnt;                    //����λ��ֵ����
	//�������ģʽ�л����
    int8_t               operationMode;                  //�������ģʽ
    int8_t               operationStopJudgeFlag;         //���ֹͣ�жϱ�־���жϵ��ʵ��ת���Ƿ���ֹͣ��ֹͣ���л�ģʽ��ʹ�ܵ��
    uint16_t             operModeSetStep;                //�������ģʽ���ò���
    uint16_t             operSetNewStep;                 //��λ�����ò���
    uint16_t             operStopStep;                   //�������ʱֹͣ����
    uint32_t             operTimedelay;                  //ʱ���ӳ�
    uint32_t             operModeSetTimeOut;             //�������ģʽ���ó�ʱ����
    uint32_t             retryTime;                      //�������ģʽ����ʧ�����ԵĴ���
    uint32_t             retrySetTargetTime;             //���Ŀ��ֵ����ʧ�����ԵĴ���
    uint32_t             runParasSetFlag;                //������в������ñ�־
    //�����ж����
    uint8_t              resetFlag;                      //��λ��־
    uint8_t              errorCnt;                       //���ϼ���
    uint8_t              stallFlag;                      //��ת��־
    uint8_t              reverseFlag;                    //�����־
    uint8_t              offLineCnt;                     //���߼���
    uint8_t              abnormalOperationCnt;           //�쳣ȥʹ�ܼ���
    ERROR_LEVEL          newErrorLevel;                  //�¹��ϵȼ�
    ERROR_LEVEL          errorLevel;                     //���ϵȼ�
    uint32_t             offLineTime;                    //����ϵ�󣬼����״̬����ʱ��
    //pid����
    PIDParas_incres      pidIncresParas;                 //����ʽpid����
    PIDParas_incres      pidIncresParasOne;              //����ʽpid����1
    PID                  pidFuzzyParas;                  //ģ��pid����
    //int32_t              curMVelFilter;                  //��ǰ����˲�����ٶ�ֵ
    int32_t              setAcc;                         //�趨���ٶ�
    int32_t              curAcc;                         //���μ��ٶ�
    ST_KALMAN_DATA       setAccKalmanData;               //�趨���ٶȿ������˲��ṹ��
    int32_t              lastSetAcc;                     //��һ���趨���ٶ�
    uint32_t             pcLostRecoveryTime;             //ʧ���ָ�ʱ��
    int32_t              setVelRecord[MVEL_RECORD_NUM];  ////�趨�ٶȼ�¼
    int32_t              setCarVel[SET_VEL_RECORD_NUM];  //�滮�ٶ�ֵ��¼����
    uint16_t             setCarVelValidNum;              //�滮�ٶ�ֵ��Ч����
    uint16_t             setVelState;                    //�趨�ٶ�״̬
    uint32_t             setCarVelTime[SET_VEL_RECORD_NUM];//�滮���ټ�¼ʱ��
    int32_t              setCarAcc;                     //�滮���ٶȼ���ֵ
    uint32_t             IItBuffer[M_BUFFER_IIt_LEN];   //����ֵ����
    uint32_t             IItStartTime[M_BUFFER_IIt_LEN];//ÿ�ι���ʱ�Ŀ�ʼʱ��
    uint32_t             IItIndex;                      //���ػ���ָ��
    uint32_t             IItOverLastTime;               //�ϴι���ʱ��
    uint32_t             IItOverTimeRecord;             //����ʱ���¼
    uint32_t             IItLimit;                      //��������ֵ
    uint32_t             IItFlag;                       //���ر�־λ
}ST_MOTOR_RUN_STATE_DATA; 
#pragma pack (1)  //1�ֽڶ���
/**
 * Motor Data
 */
typedef struct
{
    uint8_t                 idx;                            //canͨ��id->1
    uint8_t                 relatedMotor;                   //�����ĵ�����->2
    uint8_t                 powerPort;                      //��Դ���ƶ˿�->3
    uint8_t                 powerPin;                       //��Դ��������->4
    DRIVER_TYPE             driverType;                     //����������->8
    uint32_t                flag;                           //��־λ->12
    uint32_t                powerDelayTime;                 //����ϵ���ʱʱ��->16
    int32_t                 counts;                         //���һת��Ӧ������->20
    float                   ratio;                          //������ٱ�->24
    int32_t                 limitSpeed;                     //����ת��rpm->28
    int32_t                 speedGrad;                      //�Ӽ����ݶ�rpm->32
    uint32_t                speedAdjustPeriod;              //�ٶȵ�������ms->36
    int32_t                 homingSpeed;                    //Ѱ���ٶ�->40
    int32_t                 normalCurrent;                  //�����mA->44
    uint32_t                enableDelayTime;                //ʹ����ʱʱ��->48
    float                   kp;                             //kp->52
    float                   ki;                             //ki->56
    uint32_t                currentAdjustPeriod;            //������������ms->60
    int32_t                 limitCurrent;                   //���Ƶ���mA->64
    int32_t                 startCurrent;                   //�𶯵���ֵmA->68
    uint32_t                start_fault_time;               //�����й���ʱ��->72
    int32_t                 pos_limit1;                     //λ������1->76
    int32_t                 pos_limit2;                     //λ������2->80
    uint32_t                profileVelocity ;               //�����ٶ�rpm->84
    uint32_t                profileAcc;                     //�������ٶ�rpm/s2 ->88
    int32_t                 initPos;                        //�궨λ��->92
    uint32_t                overCurrentTime;                //����ʱ��->96
    uint8_t                 reserved[32];                   //Ԥ��->128
}ST_MOTOR_DATA;
#pragma pack ()  //1�ֽڶ���

//���Ͻṹ����Ϣ�����ڷ�������ԭ��
#pragma pack (1)  //1�ֽڶ���
typedef struct
{
    //����ͨ����Ϣ
    uint16_t                errorResult;                    //������
    uint16_t                errorMotorNum;                  //�������
    uint32_t                errRecordTime;                  //���ϼ�¼ʱ��
    //1003���������Ϣ
    int32_t                 setCurrentPositiveChangeIndex;  //�趨�������仯����ֵ
    int32_t                 setCurrentNegativeChangeIndex;  //�趨�������仯����ֵ
    int32_t                 carFilter;                      //�ߵ��˲�ֵ
    int32_t                 motorFilter;                    //����˲�ֵ
    int16_t                 ratioVelAuto;                   // �ٶ�ת���ת�ٰٷֱ�ֵ(ʵʱ����)
    //1004���������Ϣ
    int32_t                 setAcc;                         //�趨���ٶ�
    int32_t                 curAcc;                         //���μ��ٶ�
    //pid������Ϣ��¼
    uint8_t                 idx;                            //canͨ��id
    int32_t                 setVelRecord[MVEL_RECORD_NUM];  ////�趨�ٶȼ�¼
    int32_t                 carVelRecord[MVEL_RECORD_NUM];  //�ߵ��ٶȼ�¼
    int32_t                 mVel[MVEL_RECORD_NUM];          //����ٶ�ֵ��¼����
    int32_t                 setCurrentRecord[MVEL_RECORD_NUM];//�趨������¼
    //1002���������Ϣ
    uint16_t                driverVoltage[M_TOTAL_NUM];     //��������ѹֵ��¼
    //9025���������Ϣ
    uint16_t                preState;
    uint16_t                afterIndex;
    uint16_t                afterState[6];
    uint32_t                afterTime[6];
    uint16_t                afterWorkState[5];
}ST_ERROR_INFO;
#pragma pack ()  //1�ֽڶ���


extern ST_MOTOR_RUN_STATE_DATA gStMotorRunState[M_TOTAL_NUM];
extern ST_MOTOR_DATA gStMotorData[M_TOTAL_NUM];

void MotorControlEntry(TickType_t curTime);
void InitMotorPara(void);
void SetMotorRunModeData(uint32_t motorNum, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode);
int32_t ConvertAngleToCounts(uint32_t motorNum, int32_t angle, PosType posType);

void SetMotorPower(uint32_t motorNum, PowerState powerState);
void SetMotorLock(uint32_t motorNum, LockState brakeState, rt_bool_t notRelateBrake);
void ChangeMotorControlMode(uint32_t motorNum, int8_t i8OperationMode);
HAL_StatusTypeDef ChangeMotorTargetValue(uint32_t motorNum, int32_t value, uint8_t changeFlag);
void ClearErrorCode(uint16_t motorNum);

void TestUfoControl(uint8_t* cmdData, uint8_t size);
void OldTestRevCmdAnalysis(uint8_t* pData, uint8_t size);

void PrintfMotorStaus(void);
void UfoLeftRightControl(int16_t targetValue, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode);
void RecordCarSetVelAndAcc(uint32_t motorNum, int32_t curSetCarVel, uint32_t curTime);
void ErrorAutoStopCmd(ST_MOTOR_RUN_MODE_DATA* lMotorRunMode);

void CalcPid_incres_driver(uint32_t motorNum, int32_t tar, int32_t cur, int32_t velErrAcc, uint32_t processTimeMs);
int PID_realize(ST_MOTOR_RUN_STATE_DATA *structpid, int32_t s, int32_t in, uint32_t processTimeMs);
void PIDBrakeAdjust(uint32_t motorNum, int32_t tar, int32_t liSetTarAcc);


#define CHANGE_MOTOR_TARGET_POS(num, pos)     ChangeMotorTargetValue(num, pos, MOTOR_SET_TARGET_POS)
#define CHANGE_MOTOR_TARGET_VEL(num, vel)     ChangeMotorTargetValue(num, vel, MOTOR_SET_TARGET_VEL)
#define CHANGE_MOTOR_TARGET_CURRENT(num, current)     ChangeMotorTargetValue(num, current, MOTOR_SET_TARGET_CURRENT)
#define CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(num, pos)     ChangeMotorTargetValue(num, pos, MOTOR_SET_TARGET_POS_NEED_JUDGE)
#define CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(num, vel)     ChangeMotorTargetValue(num, vel, MOTOR_SET_TARGET_VEL_NEED_JUDGE)
#define CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(num, current)    ChangeMotorTargetValue(num, current, MOTOR_SET_TARGET_CURRENT_NEED_JUDGE)


/**
 * Motor Rev Data
 */
typedef struct
{
    uint32_t     timeCnt;                   //��ȡ��ʱ
    uint32_t     rev_flag;                  //���յ����ݱ�־
    uint16_t     status;                    //���״̬��Ϣ
    int32_t      current;                   //�������
    int32_t      speed;                     //���ת��
    int32_t      pos;                       //λ������
    int16_t      tmp;                       //�������¶�����
    int16_t      motorTmp;                  //����¶�����
    uint16_t     vol;                       //��ѹ����
    uint32_t     eMergencyErrorCode;        //����������
    uint8_t      ctrlStatus;                //���������״̬��Ϣ
    uint32_t     lastSendTime;              //�ϴη���ʱ��
}ST_MOTOR_REV_DATA;

extern ST_MOTOR_REV_DATA gStMotorRevData[M_TOTAL_NUM];
extern uint16_t gErrorResult;
extern uint16_t gErrorMotorNum;
extern uint16_t gErrorRunStateFlag;
extern uint32_t gErrorTime;
extern ST_ERROR_INFO gStErrorInfo;
extern BrakeState gBrakeFlag;                           //ɲ����־
extern BrakeWaitState gBrakeWaitOnFlag;
extern uint16_t gBrakeValue; 
extern LockState gLockFlag;                             //��բ��־

//����ɲ������
#define SET_BRAKE(value)    if((BRAKE_OFF_FORCE != gBrakeFlag) && (BRAKE_OFF_FORCE_CMD != gBrakeFlag))\
                            {\
                                gBrakeFlag = BRAKE_POS_CMD; \
                                gBrakeValue = value;\
                            }
#define CLOSE_BRAKE         if((BRAKE_OFF != gBrakeFlag) && (BRAKE_OFF_FORCE != gBrakeFlag) && (BRAKE_OFF_FORCE_CMD != gBrakeFlag)) gBrakeFlag = BRAKE_OFF_CMD
//ǿ�ƹ�ɲ��
#define CLOSE_BRAKE_FORCE   if((BRAKE_OFF_FORCE != gBrakeFlag) && (gStUfoData.flag & UFO_ENABLE_LOCK)\
                            && (gStMotorData[M_BRAKE].powerPort == gStUfoData.lockPort)\
                            && (gStMotorData[M_BRAKE].powerPin == gStUfoData.lockPin)\
                            && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))\
                            {\
                                gBrakeFlag = BRAKE_OFF_FORCE_CMD;\
                            }

//���ر�բ������ɲ����բǿ�йرգ���ɲ���ϵ�
#define OPEN_LOCK   if((LOCK_ON != gLockFlag) && (gStUfoData.flag & UFO_ENABLE_LOCK)) \
                    {\
                        gLockFlag = LOCK_ON;\
                        SetMotorLock(M_TOTAL_NUM, LOCK_ON, RT_FALSE);\
                        if((BRAKE_OFF_FORCE == gBrakeFlag) || (BRAKE_OFF_FORCE_CMD == gBrakeFlag))\
                        {\
                            gBrakeFlag = BRAKE_POS_CMD;\
                        }\
                    }
#define CLOSE_LOCK  if((LOCK_OFF != gLockFlag) && (gStUfoData.flag & UFO_ENABLE_LOCK)) \
                    {\
                        gLockFlag = LOCK_OFF;\
                        SetMotorLock(M_TOTAL_NUM, LOCK_OFF, RT_FALSE);\
                    }

//����������ݱ�־����
#define REV_MOTOR_BOOT_UP_FLAG      0x01u   //���յ�������Ϣ
#define REV_MOTOR_STATUS_FLAG       0x02u   //���յ�״̬��Ϣ
#define REV_MOTOR_CURRENT_FLAG      0x04u   //���յ���������
#define REV_MOTOR_SPEED_FLAG        0x08u   //���յ��ٶ���Ϣ
#define REV_MOTOR_EMERGENCY_FLAG    0x10u   //���յ�����������Ϣ
#define REV_MOTOR_CTRL_STATUS_FLAG  0x20u   //���յ�������״̬��Ϣ
#define REV_MOTOR_POS_FLAG          0x40u   //���յ�λ����Ϣ
#define REV_MOTOR_TMP_FLAG          0x80u   //���յ��¶���Ϣ
#define REV_MOTOR_VOL_FLAG          0x100u  //���յ���ѹ��Ϣ
#define REV_MOTOR_DRIVER_TMP_FLAG   0x200u  //���յ��������¶���Ϣ

#define SET_MOTOR_REV_DATA_FLAG(motorNum, x)        gStMotorRevData[motorNum].rev_flag |= x
#define CLEAR_MOTOR_REV_DATA_FLAG(motorNum, x)      gStMotorRevData[motorNum].rev_flag &= (~x)
#define IS_REV_MOTOR_DATA(motorNum, x)  (gStMotorRevData[motorNum].rev_flag & x)


//AGVС�������������
#define WHEEL_RADIUS          0.09f   	//���Ӱ뾶m
#define GEAR_RATIO            16.0f     //���ֱ�
#define WHEEL_DIST_X          0.00f     //����X������
#define WHEEL_DIST_Y          0.00f   	//����Y������
#define WHEEL_COUNT 			2

//�����ͱȽ�
#define eps 1e-8f 
#define FLOAT_MORE(a, b) (((a) - (b)) > (eps))
#define FLOAT_LESS(a, b) (((a) - (b)) < (-eps))
#define FLOAT_EQU(a, b) ((fabs((a) - (b))) < (eps))
#define PI 3.1415926f



void car_create(void);
void MyMotorSet(void);
void MyMotorVelSet_test(void);
int agv_velocity_set(struct velocity target_velocity);



#endif  /* __MOTOR_CONTROL_H */

