#include "motor_control.h"
#include <motor485_driven.h>
#include "math.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "pid_fuzzy.h"
//#include <s_curve.h>
#include "delay.h"
#include "battery.h"

#ifdef ENABLE_CANOPEN_DRIVER
#include <motor_driven.h>
#endif

uint16_t gErrorResult = 0;
uint16_t gErrorMotorNum = 0;
uint16_t gErrorRunStateFlag = 0;  //���ϳ���������״̬��־
uint32_t gErrorTime = 0;
uint32_t gErrorEmergencyResult = 0;
uint16_t gErrorEmergencyMotorNum = 0;
ST_ERROR_INFO gStErrorInfo = {0};

uint8_t  gErrorTimes = 0;                 //һ�������ڵĴ������
uint32_t gErrorStartTick;                 //����ʼʱ��tick
uint32_t gErrorRecoveryTime;              //����ָ���ʱ

static ST_MOTOR_RUN_MODE_DATA gTargetRunMode[M_TOTAL_NUM];
static ST_MOTOR_RUN_MODE_DATA gCurtRunMode[M_TOTAL_NUM];
ST_MOTOR_RUN_STATE_DATA gStMotorRunState[M_TOTAL_NUM];

ST_MOTOR_REV_DATA gStMotorRevData[M_TOTAL_NUM] = {0};
BackDeffMode gBackWheelDeffDriverMode = BACK_DEFF_NONE;//���ֲ�������ģʽ��־

int32_t gCarVel[NAV_CAR_VEL_RECORD_NUM];//�ߵ��ٶ�ֵ��¼����
uint16_t gCarVelValidNum;               //�洢��Ч����
uint32_t gCarVelTime[NAV_CAR_VEL_RECORD_NUM];   //���ټ�¼ʱ��
int32_t gCarOverVel;                    //�����������ֵ
uint32_t gCarOverVelTime;               //�����������ֵʱ��
uint32_t gCarEnterUniTime;              //�������ٽ׶ε�ʱ��
int32_t gCarNavCurAcc;                  //�ߵ���ǰ���ٶ�
int32_t gWheelSpeed[W_TOTAL_NUM];       //����
int32_t gMotorAvaVel[MVEL_RECORD_NUM];  //������ٶ�
uint16_t gMotorAvaVelValidNum;          //�洢��Ч����
uint32_t gMotorAvaVelTime[MACC_CAL_PERIOD_NUM]; //����ٶȼ�¼ʱ��
int32_t  gMotorAvaSetCurrentRecord[MVEL_RECORD_NUM];//�趨������¼
int32_t  gMotorAvaSetCurrentPositiveChangeIndex;    //�趨�������仯����ֵ
int32_t  gMotorAvaSetCurrentNegativeChangeIndex;    //�趨�������仯����ֵ
int32_t gMotorAvaVelFilter = 0;         //����˲����ٶ�
int32_t gMotorAvaPos = 0;               //���ƽ��λ��
int32_t gMotorCurAcc;                   //���ݵ���ٶȼ���ĵ�ǰ���ٶ�
int32_t gMotorAccPeriodAvaAcc;          //���ٽ׶�ƽ�����ٶ�
int32_t gMotorAccPeriodAvaCurrent;      //���ٽ׶�ƽ������
int32_t gMotorUniPeriodAvaAcc;          //���ٽ׶�ƽ�����ٶ�
int32_t gMotorUniPeriodAvaCurrent;      //���ٽ׶�ƽ������
int32_t gMotorAccCurrent = 0;           //1m/s2���ٶȶ�Ӧ����ѧϰֵ
int32_t gMotorAccCurrentRecord[MACC_CURRENT_LEARN_NUM];//1m/s2���ٶȶ�Ӧ����ѧϰֵ��¼
int32_t gMotorAccCurrentValidNum = 0;   //1m/s2���ٶȶ�Ӧ����ѧϰֵ��¼��Ч����

rt_bool_t gNoLoadFlag = RT_FALSE;       //�޸��ر�־
int32_t gMotorVelErrPeak;               //����ٶ�����ֵ��¼
int32_t gMotorVelErrValley;             //����ٶ�����ֵ��¼
uint32_t gMotorVelErrPeakTime;          //����ٶ�����ֵʱ�̼�¼
uint32_t gMotorVelErrValleyTime;        //����ٶ�����ֵʱ�̼�¼
int32_t gMotorLastLastVelErr;           //������ϴ��ٶ����
int32_t gMotorLastVelErr;               //����ϴ��ٶ����
rt_bool_t gMotorVelOscillationFlag;     //����ٶȿ����񵴱�־
int32_t gMotorAccMax;                   //��������ٶ�
int32_t gMotorAccMin;                   //�����С���ٶ�
int32_t gMotorAccTemp;                  //acc����
uint32_t gMotorAccMaxTime;              //��������ٶȶ�Ӧʱ��
uint32_t gMotorAccMinTime;              //�����С���ٶȶ�Ӧʱ��
int32_t gMotorAccMaxCurrent;            //��������ٶȶ�Ӧ�ĵ���ֵ
int32_t gMotorAccMinCurrent;            //�����С���ٶȶ�Ӧ�ĵ���ֵ
rt_bool_t gMotorAccOscillationFlag;     //������ٶȿ����񵴱�־

int32_t gMotorVelFilter;                //����͹ߵ��˲������ĵ���ٶ�
int32_t gCarVelFilter;                  //����͹ߵ��˲������Ĺߵ��ٶ�
int32_t gAutoRatioVel;                  //��̬�ٱ�
int32_t gSteeringAngleVelDiff;          //����ת��ʱת�Ƕ�Ӧ���ٶȲ�ֵcm/s

ST_KALMAN_DATA  gStCarNavAccKalmanData; //�ߵ����ٶȿ������˲��ṹ��
ST_KALMAN_DATA  gStMotorAccKalmanData;  //������ٶȿ������˲��ṹ��

uint8_t gCurrentBuffer[USART7_REV_BUFFER_LEN];
uint16_t gCurrentIndex = 0;
uint8_t gPreChargeState = MOTOR_POWER_FLAG_OFF;
uint8_t gPreChargeStateOne = MOTOR_POWER_FLAG_OFF;
BrakeState gBrakeFlag = BRAKE_OFF;              //ɲ����־
BrakeWaitState gBrakeWaitOnFlag = BRAKE_WAIT_NONE;     //�ȴ�ɲ���ϵ��־

uint16_t gBrakeValue = 0;                       //ɲ��ǿ�ȣ�0��700,    1000��ʾ�ϵ�
LockState gLockFlag = LOCK_OFF;                 //��բ��־
uint32_t gPreChargeTimeCnt;
uint32_t gPreChargeTimeCntOne;
//SCurveParas gStMotorScurveParas;

rt_bool_t gMotorTestDataUploadFlag = RT_FALSE;  //���������ϴ���־
uint32_t gMotorTestDataUploadStartTime = 0;     //��ʼ�ϴ�ʱ��

GPIO_TypeDef* gGpioPortArray[GPIO_PORT_TOTOAL_NUM] =
{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

static void MotorsPowerCheck(uint32_t processTimeMs);
static void GetMotorStatus(void);
#ifdef ENABLE_CANOPEN_DRIVER
static void ChangeDriveMotorControlMode(uint32_t processTimeMs);
#endif
static void ErrorRecoveryStepMotorCmd(int errorRecoveryStep);
static void ProcessMotorRunModePowerOff(uint32_t motorNum);
static void OldTestProcess(uint32_t processTimeMs);
static void TestMotorDataUploadProcess(uint32_t motorNum, rt_bool_t pidStartFlag, int32_t tarVel, int32_t curVel);
/*****************************************************************************
 ��������  : ������ݳ�ʼ��
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��27��
*****************************************************************************/
void InitMotorPara(void)
{
    uint32_t i;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        gTargetRunMode[i].run_mode = MOTOR_RUN_MODE_POWER_OFF;
        gCurtRunMode[i].run_mode = MOTOR_RUN_MODE_STEP_IDLE;
        gStMotorRunState[i].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
        gStMotorRunState[i].run_step = MOTOR_RUN_MODE_STEP_IDLE;
        gStMotorRunState[i].pidNoExeTime = 0;
        gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_OFF;
        gStMotorRunState[i].powerTime = 0;
        gStMotorRunState[i].startRemoteFlag = 0;
        gStMotorRunState[i].homeFinishFlag = HOME_FLAG_UNDEF;
        gStMotorRunState[i].infraPosTotal = 0;
        gStMotorRunState[i].infraPosCnt = 0;
        gStMotorRunState[i].infraFaultTimeTotal = 0;        
        gStMotorRunState[i].infraTimeDelay = 0;
        gStMotorRunState[i].infraPosRecord = 0;
        gStMotorRunState[i].posFlag = POS_FLAG_TARGET;
        gStMotorRunState[i].readStep = MOTOR_READ_FINISH;
        gStMotorRunState[i].targetPosJudgeFlag = RT_FALSE;
        gStMotorRunState[i].pidCurrentStartFlag = M_PID_IDLE_FLAG;
        gStMotorRunState[i].clearStartMoveFlag = RT_FALSE;
        gStMotorRunState[i].limitCurrent = gStMotorData[i].limitCurrent;
        if(DRIVER_TYPE_QILING == gStMotorData[i].driverType)
        {
            gStMotorRunState[i].clearStartMoveFlag = RT_TRUE;
        }
        gStMotorRunState[i].pidDelayTime = 0;
        gStMotorRunState[i].resetFlag = 0;
        gStMotorRunState[i].stallFlag = 0;
        gStMotorRunState[i].newErrorLevel = ERROR_NONE;
        gStMotorRunState[i].errorLevel = ERROR_NONE;
        gStMotorRunState[i].curPos = 0;
        gStMotorRunState[i].limitPos1 = gStMotorData[i].pos_limit1;
        if(gStMotorData[i].flag & MOTOR_PRESS_SENSOR_FLAG)
        {
            gStMotorRunState[i].limitPos2 = gStMotorData[i].pos_limit1 + 
                (gStMotorData[i].pos_limit2 - gStMotorData[i].pos_limit1) * 7 / 10;
        }
        else
        {
            gStMotorRunState[i].limitPos2 = gStMotorData[i].pos_limit2;
        }
        gStMotorRunState[i].PosInit = gStMotorData[i].initPos;
        gStMotorRunState[i].targetPos = 0;
        gStMotorRunState[i].lastPoweroffPos = 0;
        gStMotorRunState[i].targetPosFlag = 0;
        gStMotorRunState[i].limit1Time = 0;
        gStMotorRunState[i].limit2Time = 0;
        gStMotorRunState[i].offLineTime = xTaskGetTickCount();
        gStMotorRunState[i].offLineCnt = 0;
        gStMotorRunState[i].abnormalOperationCnt = 0;
        gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;
        gStMotorRunState[i].operationStopJudgeFlag = 0;
        gStMotorRunState[i].operTimedelay = 0;
        gStMotorRunState[i].operModeSetTimeOut = 0;
        gStMotorRunState[i].operModeSetStep = OPERATION_MODE_UNKOWN;
        gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
        gStMotorRunState[i].setCarVelValidNum = 0;
        gStMotorRunState[i].setVelState = SET_VEL_STATE_IDLE;
        gStMotorRunState[i].IItIndex = 0;
        gStMotorRunState[i].IItOverLastTime = 0;
        gStMotorRunState[i].IItOverTimeRecord = 0;
        gStMotorRunState[i].IItLimit = 0;
        gStMotorRunState[i].IItFlag = 0;
        memset((uint8_t*)(gStMotorRunState[i].IItBuffer), 0, sizeof(uint32_t) * M_BUFFER_IIt_LEN);
        memset((uint8_t*)(gStMotorRunState[i].IItStartTime), 0, sizeof(uint32_t) * M_BUFFER_IIt_LEN);
        InitPidParas_incres(&gStMotorRunState[i].pidIncresParas, 
            gStMotorData[i].kp, gStMotorData[i].ki, 0, 0);
        //PID_Set(&gStMotorRunState[i].pidFuzzyParas, gStMotorData[i].kp, gStMotorData[i].ki, 0, 20);
 #ifdef ENABLE_CANOPEN_DRIVER
        ChangeMotorControlMode(i, MOTOR_OPERATION_MODE_UNKOWN);
 #endif
        ChangeMotorTargetValue(i, 0, MOTOR_SET_NOTHING);
    }
    if(!(gStMotorData[M_TURN].flag & ENABLE_AUTO_HOMMING))
    {
        gStMotorRunState[M_TURN].limitPos1 = gStMotorData[M_TURN].initPos - gStMotorData[M_TURN].pos_limit1;
        gStMotorRunState[M_TURN].limitPos2 = gStMotorData[M_TURN].initPos + gStMotorData[M_TURN].pos_limit1;
    }
    //��ʼ�������������Դ����բ��Դ����˿ڣ���紫��������˿�
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        if(gStMotorData[i].powerPin < GPIO_PIN_MAX_NUM)
        {
            GPIO_InitStruct.Pin = GPIO_PIN(gStMotorData[i].powerPin);
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIO_PORT(gStMotorData[i].powerPort), &GPIO_InitStruct);
            SetMotorPower(i, POWER_OFF);
        }
        if(gStMotorData[i].flag & MOTOR_USE_PI7_POWER)    //ʹ��PI7��Դ���ƶ˿ڣ�ԭ��Դ���ŵ���Ԥ������
        {
            GPIO_InitStruct.Pin = GPIO_PIN_7;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
            SetMotorPower(M_NUM_PI7, POWER_OFF);
        }
    }

    if(gStUfoData.preChargePin < GPIO_PIN_MAX_NUM)
    {
        GPIO_InitStruct.Pin = GPIO_PIN(gStUfoData.preChargePin);
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIO_PORT(gStUfoData.preChargePort), &GPIO_InitStruct);
        SetMotorPower(M_TOTAL_NUM, POWER_OFF);
    }

    if(gStUfoData.flag & UFO_ENABLE_LOCK)  //��բʹ��
    {
        if(gStUfoData.lockPin < GPIO_PIN_MAX_NUM)
        {
            GPIO_InitStruct.Pin = GPIO_PIN(gStUfoData.lockPin);
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIO_PORT(gStUfoData.lockPort), &GPIO_InitStruct);
            SetMotorLock(M_TOTAL_NUM, LOCK_OFF, RT_FALSE);
        }
    }

    gMotorAccCurrent = gStMotorData[M_LEFT].startCurrent;   //1m/s2���ٶȶ�Ӧ����ֵ
    //�����ٶ��ݶ�3����Ӧת���µļ��ٶȼ�3000/(22/3)s����Ӧ�Ӽ��ٶ�ȡʱ��1.2s�������ٶ�/1.2s
    //InitSCurveParas(&gStMotorScurveParas, gStMotorData[M_LEFT].limitSpeed, 410.0f, 340.0f, 4.0f);
}
/*****************************************************************************
 ��������  : �л���ǰ�������ģʽ
 �������  : uint32_t motorNum  ������
             uint8_t runMode    ָ������ģʽ�����Ϊ���У����Զ�ָ����һ��
             uint8_t runStep    ָ�����в���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��15��
*****************************************************************************/
void ChangeMotorCurRunMode(uint32_t motorNum, uint8_t runMode, uint8_t runStep)
{
    uint8_t lCurRunMode = gCurtRunMode[motorNum].run_mode;
    
    //ָ����һģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE != runMode)
    {
        memcpy(&gCurtRunMode[motorNum], &gTargetRunMode[motorNum], sizeof(ST_MOTOR_RUN_MODE_DATA));
        gCurtRunMode[motorNum].run_mode = runMode;
        gStMotorRunState[motorNum].run_step = runStep;
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
    }
    //ģʽִ��������Զ��л�����һģʽ��������ȴ�ģʽִ����
    else if(MOTOR_RUN_MODE_STEP_IDLE == gStMotorRunState[motorNum].run_step)
    {
        memcpy(&gCurtRunMode[motorNum], &gTargetRunMode[motorNum], sizeof(ST_MOTOR_RUN_MODE_DATA));
	    gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
        //����ִ����ת�����
        if(lCurRunMode == gTargetRunMode[motorNum].run_mode)
        {
            gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;
        }
        //�л�����Ӧģʽ
        else
        {
            gCurtRunMode[motorNum].run_mode = gTargetRunMode[motorNum].run_mode;
            gStMotorRunState[motorNum].run_step = MOTOR_RUN_MODE_STEP_START;
        }
    }
    else
    {
        return;
    }

    /*//�������Ķ�ģʽ���������Ķ�ģʽʱ����Ҫ��Ѱ��
    if((lCurRunMode != MOTOR_RUN_MODE_FLAP_FIX) && (lCurRunMode != MOTOR_RUN_MODE_HOMMING) 
        && (gCurtRunMode[motorNum].run_mode == MOTOR_RUN_MODE_FLAP_FIX) && (gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM))
    {
        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;    //��Ҫ����Ѱ��
    }*/

    if(DEBUG_DATA_TYPE_6)
    {
        rt_kprintf("id-%d Change-mode:%d, %d.\r\n", gStMotorData[motorNum].idx,
            gCurtRunMode[motorNum].run_mode, gStMotorRunState[motorNum].run_step);
    }
    
    gStMotorRunState[motorNum].runTotalTime = 0;
    gStMotorRunState[motorNum].errorCnt = 0;
    gStMotorRunState[motorNum].runDelayTime = 0;
}
/*****************************************************************************
 ��������  : ���ֲ���ģʽ����
 �������  : ST_MOTOR_RUN_MODE_DATA* lMotorRunMode
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��12��12��
*****************************************************************************/
static void BackDeffModeDeal(ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    //��ת��ʱ�л���������ģʽ
    if((ABS_VALUE(lMotorRunMode->target_value) >= 600) && (ABS_VALUE(gRespondState.turnCurPos) >= 600)
        && (ABS_VALUE(gMotorAvaVelFilter) <= FOUR_TURN_FRONT_ONLY_VEL + 50))
    {
        gBackWheelDeffDriverMode = BACK_DEFF_MAX;
    }
    //����ʱ�л�����ת��ֵ�����Ĳ�������ģʽ
    else if(BACK_DEFF_ACCORD_TURN != gBackWheelDeffDriverMode)
    {
        if((ABS_VALUE(gMotorAvaVelFilter) <= FOUR_TURN_FRONT_ONLY_VEL - 100)
            && (ABS_VALUE(VelRpmToCm_s(gStMotorRunState[M_LEFT].setTargetVel)) <= FOUR_TURN_FRONT_ONLY_VEL - 100))
        {
            gBackWheelDeffDriverMode = BACK_DEFF_ACCORD_TURN;
        }
    }
    //�ٶȴ�����ֵʱ�л����޲���ģʽ
    if(BACK_DEFF_NONE != gBackWheelDeffDriverMode)
    {
        if((ABS_VALUE(gMotorAvaVelFilter) >= FOUR_TURN_FRONT_ONLY_VEL)
            && (ABS_VALUE(VelRpmToCm_s(gStMotorRunState[M_LEFT].setTargetVel)) >= FOUR_TURN_FRONT_ONLY_VEL))
        {
            gBackWheelDeffDriverMode = BACK_DEFF_NONE;
        }
    }
}
/*****************************************************************************
 ��������  : ǿ�����õ������ģʽ����
 �������  : uint32_t motorNum
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��4��9��
*****************************************************************************/
static void SetMotorRunModeDataForce(uint32_t motorNum, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }

    if(motorNum == M_TURN)
    {
        if(IS_UFOONE_FLAG_SET(UFOONE_TWO_WHEL_DEFF_DRIV))   //���ֲ�������
        {
            BackDeffModeDeal(lMotorRunMode);
        }
    }

    if((M_BRAKE == motorNum) && (gStMotorData[M_BRAKE].flag & MOTOR_DIR_INVERSE_FLAG)
        && (!(gStMotorData[M_BRAKE].flag & MOTOR_CURRENT_ADJUST_SPEED)))  //�������
    {
        lMotorRunMode->target_value = Limit(lMotorRunMode->target_value, 0, 700);
        lMotorRunMode->target_value = 700 - lMotorRunMode->target_value;
    }

    if((M_LEFT == motorNum) || (M_RIGHT == motorNum) || (M_LEFT_ONE == motorNum) || (M_RIGHT_ONE == motorNum))
    {
        if(sys_para->CAR_RTinf.Link & LINK_PC)   //PC����
        {
            RecordCarSetVelAndAcc(motorNum, lMotorRunMode->target_value, HAL_GetTick());
        }
    }

    //�Ƚ������Ƿ����ı䣬δ�ı���ֱ�ӷ���
    if(0 == memcmp(&gTargetRunMode[motorNum], lMotorRunMode, sizeof(ST_MOTOR_RUN_MODE_DATA)))
    {
        return;
    }

    if((gTargetRunMode[motorNum].run_mode == lMotorRunMode->run_mode)
        && (gTargetRunMode[motorNum].run_mode == gCurtRunMode[motorNum].run_mode))
    {
        if((M_TURN == motorNum) && (DRIVER_TYPE_QILING == gStMotorData[M_TURN].driverType)
            && (ABS_VALUE(lMotorRunMode->target_value - gTargetRunMode[motorNum].target_value) < 3))
        {
            return;
        }
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_PARA_CHANGED;   //���������ı�
    }
    else
    {
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_CHANGED;   //ģʽ�����ı�
    }
    
    memcpy(&gTargetRunMode[motorNum], lMotorRunMode, sizeof(ST_MOTOR_RUN_MODE_DATA));

    if(DEBUG_DATA_TYPE_6 || DEBUG_DATA_TYPE_87)
    {
        rt_kprintf("motor%d mode: %d, value: %d, posType: %d.\r\n", motorNum, lMotorRunMode->run_mode, lMotorRunMode->target_value, lMotorRunMode->posType);
    }

    ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE);    //�Զ��л�����ģʽ

    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������������ͬ����
    {
        if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
        {
            if(M_LEFT == motorNum)
            {
                if((gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                    (DRIVER_TYPE_NONE != gStMotorData[M_LEFT_ONE].driverType))   //ͬ���������������ͬ����
                {
                    SetMotorRunModeData(M_LEFT_ONE, lMotorRunMode);
                }
            }
        }
        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
        {
            SetMotorRunModeData(gStMotorData[motorNum].relatedMotor, lMotorRunMode);
            if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
            {
                if(M_RIGHT == gStMotorData[motorNum].relatedMotor)
                {
                    if((gStMotorData[M_RIGHT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                        (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType))   //ͬ���������������ͬ����
                    {
                        SetMotorRunModeData(M_RIGHT_ONE, lMotorRunMode);
                    }
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : ���õ������ģʽ����
 �������  : uint32_t motorNum
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��14��
*****************************************************************************/
void SetMotorRunModeData(uint32_t motorNum, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }
    else if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //�����ϲ�ֹͣ��ִ���ⲿ����(���������ң�أ�ǰ�����л���ֹͣ)
    {
        if(gErrorResult && (!(IS_WARNING_CODE(gErrorMotorNum) && (!(sys_para->CAR_RTinf.Link & LINK_PC)) && (sys_para->CAR_RTinf.Link & LINK_REV_STOP_CMD))))
        {
            return;
        }
    }

    SetMotorRunModeDataForce(motorNum, lMotorRunMode);
}
/*****************************************************************************
 ��������  : ��Դ��բ�ϵ���
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��18��
*****************************************************************************/
static void MotorsPowerCheck(uint32_t processTimeMs)
{
    uint8_t i, k; 

    //���µ�ȴ���ʱ����ñ�־λ
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        //�ϵ��ʱ
        if(MOTOR_POWER_FLAG_TIMECNT == gStMotorRunState[i].powerFlag)
        {
            gStMotorRunState[i].powerTime += processTimeMs;
            if(IS_NOT_CANOPEN_DRIVER(gStMotorData[i].driverType))    //��canopen������
            {
                if(gStMotorRunState[i].powerTime >= gStMotorData[i].powerDelayTime)
                {
                    gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_ON;//�ϵ����
                    gStMotorRunState[i].powerTime = 0;
                }
            }
            else
            {
                if(gStMotorRunState[i].powerTime >= MOTOR_ALL_POWERING_TIMEOUT)
                {
                    gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_ON;//�ϵ����
                    gStMotorRunState[i].powerTime = 0;
                }
                //����������¼�
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_BOOT_UP_FLAG) && (!(gStMotorData[i].flag & MOTOR_NOT_JUDGE_BOOTUP)))
                {
                    CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_BOOT_UP_FLAG);
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("Boot-up, id-%d.\r\n", gStMotorData[i].idx);
                    }
                    gStMotorRunState[i].powerTime = MOTOR_ALL_POWERING_TIMEOUT - 1800;
                    gStMotorRunState[i].offLineTime = xTaskGetTickCount();
                    gStMotorRunState[i].offLineCnt = 0;
                }
            }
        }
        //�µ��ʱ
        else if(MOTOR_POWEROFF_FLAG_TIMECNT == gStMotorRunState[i].powerFlag)
        {
            gStMotorRunState[i].powerTime += processTimeMs;
            if((gStMotorRunState[i].powerTime >= MOTOR_ALL_POWEROFF_TIME) 
                && (gStMotorRunState[i].powerTime >= gStMotorData[i].powerDelayTime))   //�����µ��ʱ���볤���ϵ��ʱ���жϣ��������������µ�Ƚ���
            {
                gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_OFF;//�µ����
                gStMotorRunState[i].powerTime = 0;
            }
        }
    }

    //�ܵ�Դ�����ϵ�
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        //�ϵ��ж�
        if(MOTOR_POWER_FLAG_OFF == gStMotorRunState[i].powerFlag)
        {
            //�����������ֱ�ӷ���
            if((DRIVER_TYPE_NONE == gStMotorData[i].driverType) || (gStMotorData[i].flag & MOTOR_INVALID_FLAG))
            {
                continue;
            }
            if(MOTOR_RUN_MODE_POWER_OFF != gTargetRunMode[i].run_mode)    
        	{
                SetMotorPower(i, POWER_ON);        //��������Ԥ�书�ܣ��˴��򿪵���Ԥ��̵���
                gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_TIMECNT;
                gStMotorRunState[i].powerTime = 0;
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_BOOT_UP_FLAG);

                //����������һ����Դ����Ҳ�ϵ�
                if(gStMotorData[i].powerPin < GPIO_PIN_MAX_NUM)
                {
                    for(k = 0; k < M_TOTAL_NUM; k++)
                    {
                        if((k != i) && (gStMotorData[i].powerPort == gStMotorData[k].powerPort)
                            && (gStMotorData[i].powerPin == gStMotorData[k].powerPin)
                            && (DRIVER_TYPE_NONE != gStMotorData[k].driverType))
                        {
                            if(MOTOR_POWER_FLAG_ON != gStMotorRunState[k].powerFlag)
                            {
                                gStMotorRunState[k].powerTime = 0;
                                gStMotorRunState[k].powerFlag = MOTOR_POWER_FLAG_TIMECNT;
                            }
                        }
                    }
                }
        	}
        } 
    }

    //Ԥ��紦��
    if(MOTOR_POWER_FLAG_TIMECNT == gPreChargeState)
    {
        gPreChargeTimeCnt += processTimeMs;
        if(gPreChargeTimeCnt >= MOTOR_ALL_PRE_CHARGE_TIME)
        {
            gPreChargeState = MOTOR_POWER_FLAG_ON;
            SetMotorPower(M_TOTAL_NUM, POWER_ON);  //��������Ԥ�书�ܣ��˴��򿪵���ֱ���Ӵ���
        }
    }
    if(MOTOR_POWER_FLAG_TIMECNT == gPreChargeStateOne)
    {
        gPreChargeTimeCntOne += processTimeMs;
        if(gPreChargeTimeCntOne >= MOTOR_ALL_PRE_CHARGE_TIME)
        {
            gPreChargeStateOne = MOTOR_POWER_FLAG_ON;
            SetMotorPower(M_NUM_PI7, POWER_ON);  //��������Ԥ�书�ܣ��˴��򿪵��Ƿ�Ԥ��Ӵ���
        }
    }
}
/*****************************************************************************
 ��������  : �����״̬��ȡ
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��23��
*****************************************************************************/
static void GetMotorStatus(void)
{
    static uint32_t motorNum = M_LEFT;
#ifdef ENABLE_CANOPEN_DRIVER
    uint16_t lResult = 0;
    uint16_t l_status;
    uint32_t l_tick_temp;
#endif

    if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //��canopen������
    {
        if(MOTOR_POWER_FLAG_ON == gStMotorRunState[motorNum].powerFlag)
        {
            if(ERROR_NONE == gStMotorRunState[motorNum].newErrorLevel)
            {
                if(0 == gStMotorRunState[motorNum].startRemoteFlag)
                {
                    gStMotorRunState[motorNum].startRemoteFlag = 1;
                }
                else if(DRIVER_TYPE_KINCO == gStMotorData[motorNum].driverType) //����������
                {
                    if(HAL_OK == Motor485ReadErrorCode(motorNum, &lResult))
                    {
                        if(0 != lResult)
                        {
                            SetErrorCode(motorNum, lResult, ERROR_L_HIHG);
                        }
                    }
                }
                else if(DRIVER_TYPE_LEADSHINE == gStMotorData[motorNum].driverType) //����������
                {
                    /*if(ERROR == GPIOGetMotorState(motorNum))
                    {
                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_ERROR_CHECK, ERROR_L_HIHG);
                    }*/
                }
            }
        }
    }
#ifdef ENABLE_CANOPEN_DRIVER
    //����ϵ�֮��ʼ��ȡ���״̬
    else if(MOTOR_POWER_FLAG_ON == gStMotorRunState[motorNum].powerFlag)
    {
        if(ERROR_NONE == gStMotorRunState[motorNum].newErrorLevel)
        {
            if(0 == gStMotorRunState[motorNum].startRemoteFlag)
            {
                gStMotorRunState[motorNum].startRemoteFlag = 1;
                SetEposNMTState(SET_NMT_STATE_OPERATIONAL, gStMotorData[motorNum].idx);//����PDO����
                if((DRIVER_TYPE_EPOS != gStMotorData[motorNum].driverType) && (DRIVER_TYPE_PUSI != gStMotorData[motorNum].driverType))//sdoģʽ
                {
                    MotorCanDisableTransimitPdo(gStMotorData[motorNum].idx);//�ر�����������pdo����ֹ�䷢�Ͷ�������
                }

                if (DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                {
                    MotorSetAbortConnectionOptionCode(gStMotorData[motorNum].idx, 0);        //CANopen����ͨѶ���ֹ��ϣ���������������
                }
                
                if(gStMotorRunState[motorNum].resetFlag)   //�ָ��ϵ�ǰ��λ��
                {
                    /*if((M_LEFT == motorNum) || (M_RIGHT == motorNum))
                    {
                        gStMotorRunState[motorNum].resetFlag = 0;
                        /#if(DEBUG_DATA_TYPE_5)
                        {
                            rt_kprintf("id-%d-poweron-pos:%d.\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].lastPoweroffPos); 
                        }#/
                        if(MOTOR_RUN_MODE_HOMMING != gCurtRunMode[motorNum].run_mode)
                        {
                            ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_HOMMING, HOME_FLAG_SET);    //������λ
                        }
                    }*/
                }
                gStMotorRunState[motorNum].offLineTime = xTaskGetTickCount();
                gStMotorRunState[motorNum].offLineCnt = 0;
            }
            else
            {
                if(0 == MotorReadStatus(gStMotorData[motorNum].idx, &l_status))
                {
                    gStMotorRunState[motorNum].offLineTime = xTaskGetTickCount();
                    gStMotorRunState[motorNum].offLineCnt = 0;
                    //�����������
                    if(DRIVER_TYPE_PUSI == gStMotorData[motorNum].driverType)
                    {
                        lResult = l_status & 0x3f;
                        if(lResult)
                        {
                            SetErrorCode(motorNum, lResult, ERROR_L_HIHG);
                        }
                        /*else if((MOTOR_STATE_ERROR == gStMotorRunState[motorNum].runState)
                            && (ERROR_L_NORMAL == gStMotorRunState[motorNum].errorLevel))
                        {
                            ClearErrorCode(motorNum);
                        }*/
                    }
                    else if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                    {
                        if(lResult)
                        {
                            SetErrorCode(motorNum, lResult, ERROR_L_HIHG);
                        }
                    }
                    else if(l_status & 0x08)
                    {        
                        if(0 == MotorReadErrorCode(gStMotorData[motorNum].idx, &lResult))
                        {
                            SetErrorCode(motorNum, lResult, ERROR_L_HIHG);
                        }
                    }
                    //�޹��ϣ����ǵ���쳣�˳�ʹ�ܣ��򱨹���
                    else if((DEVICE_STATUS_OPERATION_ENABLE != (l_status & DEVICE_STATUS_MASK))
                        && (MOTOR_RUN_MODE_HOMMING != gCurtRunMode[motorNum].run_mode)
                        && (MOTOR_RUN_MODE_STOP != gCurtRunMode[motorNum].run_mode))
                    {
                        if((ERROR_NONE == gStMotorRunState[motorNum].errorLevel)
                            && (OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep))
                        {
                            gStMotorRunState[motorNum].abnormalOperationCnt++;
                            if(gStMotorRunState[motorNum].abnormalOperationCnt > 3)
                            {
                                if(0 == MotorReadErrorCode(gStMotorData[motorNum].idx, &lResult))
                                {
                                    if(0 != lResult)
                                    {
                                        SetErrorCode(motorNum, lResult, ERROR_L_HIHG);
                                    }
                                    else
                                    {
                                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_ABNORMAL_OPERATION, ERROR_L_NORMAL);
                                        SetErrorInfo(motorNum, ERROR_CODE_MOTOR_ABNORMAL_OPERATION);
                                        memset((uint8_t*)(gStErrorInfo.afterState), 0, sizeof(gStErrorInfo.afterState));
                                        memset((uint8_t*)(gStErrorInfo.afterTime), 0, sizeof(gStErrorInfo.afterTime));
                                        memset((uint8_t*)(gStErrorInfo.afterWorkState), 0, sizeof(gStErrorInfo.afterWorkState));
                                        gStErrorInfo.afterIndex = 0;
                                        gStErrorInfo.afterState[5] = l_status;
                                        gStErrorInfo.afterTime[5] = HAL_GetTick();
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        gStMotorRunState[motorNum].abnormalOperationCnt = 0;
                        gStErrorInfo.preState = l_status;
                    }
                }
                else
                {
                    //SetEposNMTState(SET_NMT_STATE_OPERATIONAL, gStMotorData[motorNum].idx);//����PDO����,���������һ�η���
                    l_tick_temp = xTaskGetTickCount() - gStMotorRunState[motorNum].offLineTime;
                    gStMotorRunState[motorNum].offLineCnt++;
                    if((l_tick_temp >= MOTORSTATUS_TIMEOUT_LONGTIME)
                        || ((l_tick_temp >= MOTORSTATUS_TIMEOUT_TIME) && (gStMotorRunState[motorNum].offLineCnt >= 3)))
                    {
                        gStMotorRunState[motorNum].offLineCnt = 0;
                        gStMotorRunState[motorNum].offLineTime = xTaskGetTickCount();
                        SetErrorCode(motorNum, ERROR_CODE_CAN_LINE_OFF, ERROR_L_HIHG);
                    }
                }
            }
        }		
    }
    else if(MOTOR_POWER_FLAG_TIMECNT == gStMotorRunState[motorNum].powerFlag)//����ﵽԤ�ϵ�ʱ��ʱ�����Ž���ͨ�ţ��Է�ֹδ�յ�������Ϣʱ��̫��ʱ��
    {
        if(gStMotorRunState[motorNum].powerTime >= gStMotorData[motorNum].powerDelayTime)
        {
            l_tick_temp = xTaskGetTickCount() - gStMotorRunState[motorNum].offLineTime;
            if(l_tick_temp >= 1000)//ÿ��1s����һ��
            {
                if(DEBUG_DATA_TYPE_6)
                {
                    rt_kprintf("Try communication, id = %d, time: %d.\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].powerTime);
                }
                gStMotorRunState[motorNum].offLineTime = xTaskGetTickCount();
                SetEposNMTState(SET_NMT_STATE_OPERATIONAL, gStMotorData[motorNum].idx);//����PDO����
                if(0 == MotorReadStatus(gStMotorData[motorNum].idx, &l_status))
                {
                    gStMotorRunState[motorNum].powerTime = MOTOR_ALL_POWERING_TIMEOUT;//�����ϵ��ʱ
                }
                /*else if(0 == gStMotorRunState[motorNum].firstPoweroffRetry)
                {
                    gStMotorRunState[motorNum].firstPoweroffRetry = 1;
                    SetMotorPower(motorNum, POWER_OFF);
                    gStMotorRunState[motorNum].startRemoteFlag = 0;
                    gStMotorRunState[motorNum].powerFlag = MOTOR_POWEROFF_FLAG_TIMECNT;
                    gStMotorRunState[motorNum].powerTime = 0;
                    rt_kprintf("First poweroff retry, id = %d.\r\n", gStMotorData[motorNum].idx);
                }*/
            }
        }
    }

    //9025�쳣ȥʹ��״̬���ݼ�¼
    if((ERROR_CODE_MOTOR_ABNORMAL_OPERATION == gErrorResult) && (motorNum == gStErrorInfo.errorMotorNum))
    {
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))
        {
            if(MOTOR_POWER_FLAG_OFF != gStMotorRunState[motorNum].powerFlag)
            {
                if(0 == MotorReadStatus(gStMotorData[motorNum].idx, &l_status))
                {
                    if(gStErrorInfo.afterIndex < 5)
                    {
                        gStErrorInfo.afterState[gStErrorInfo.afterIndex] = l_status;
                        gStErrorInfo.afterTime[gStErrorInfo.afterIndex] = HAL_GetTick();
                        if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)    //��ÿ���������ȡ����״̬
                        {
                            if(0 == MotorReadFDKWorkStatus(gStMotorData[motorNum].idx, &l_status))
                            {
                                gStErrorInfo.afterWorkState[gStErrorInfo.afterIndex] = l_status;
                            }
                        }
                    }
                    gStErrorInfo.afterIndex++;
                }
            }
        }
    }
    

    //�������ϸ�ֵ
    if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_EMERGENCY_FLAG))
    {
        CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_EMERGENCY_FLAG);
        gErrorEmergencyMotorNum = motorNum;
        gErrorEmergencyResult = gStMotorRevData[motorNum].eMergencyErrorCode;
    }
#endif
    
    motorNum++;
    if(M_TOTAL_NUM == motorNum)
    {
        motorNum = M_LEFT;
    }

    //�����ж�ʱ���Ƿ�׼ȷ
    /*if(DEBUG_DATA_TYPE_7)
    {
        rt_kprintf("%d ms\r\n", TickType_t());
    }*/
}
/*****************************************************************************
 ��������  : �л��������ģʽ
 �������  : uint32_t motorNum       ������
             int8_t i8OperationMode  ����ģʽ
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��1��
*****************************************************************************/
#ifdef ENABLE_CANOPEN_DRIVER
void ChangeMotorControlMode(uint32_t motorNum, int8_t i8OperationMode)
{
    if(motorNum < M_TOTAL_NUM)
    {
        //�����ͬģʽ�������������л�
        if((i8OperationMode != gStMotorRunState[motorNum].operationMode)
            || (OPERATION_MODE_SET_FINISH != gStMotorRunState[motorNum].operModeSetStep)) 
        {
            if(DEBUG_DATA_TYPE_6)
            {
                rt_kprintf("id-%d-change-drivermode: %d.\r\n", gStMotorData[motorNum].idx, i8OperationMode);
            }
            if(MOTOR_OPERATION_MODE_UNKOWN != i8OperationMode)
            {
                if(DRIVER_TYPE_STAND ==  gStMotorData[motorNum].driverType)
                {
                    gStMotorRunState[motorNum].operationStopJudgeFlag = 1;  //�л�ģʽʱ��Ҫ���жϵ���Ƿ���ֹͣ
                }
                if(MOTOR_OPERATION_MODE_PPM != i8OperationMode)
                {
                    gStMotorRunState[motorNum].runParasSetFlag = RUN_PARAS_NOT_SET;     //�л�ģʽ����Ҫ�����趨���в���
                }
            }
            gStMotorRunState[motorNum].operationMode = i8OperationMode;
            gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_SET;
            gStMotorRunState[motorNum].targetVel = 0;
            gStMotorRunState[motorNum].setCurrent = 0;
        }
        else if(i8OperationMode == MOTOR_OPERATION_MODE_PPM)    //λ��ģʽ���µ�λ��
        {
            if(DRIVER_TYPE_PUSI == gStMotorData[motorNum].driverType)
            {
                gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_AUTO_CHANGE;
            }
            else
            {
                
            }
        }
        else
        {
            gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_AUTO_CHANGE;
        }
        gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
        gStMotorRunState[motorNum].operModeSetTimeOut = 0;
        gStMotorRunState[motorNum].retryTime = 0;
        CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG); 
    }
} 
#endif
/*****************************************************************************
 ��������  : �л����Ŀ��λ�á��ٶȻ����
 �������  : uint32_t motorNum   ������
             int32_t value       Ŀ��λ�á��ٶȻ����
             uint8_t changeFlag  �л���־
 �������  : HAL_StatusTypeDef
 ��    ��  : ����
 ��    ��  : 2018��12��1��
*****************************************************************************/
HAL_StatusTypeDef ChangeMotorTargetValue(uint32_t motorNum, int32_t value, uint8_t changeFlag)
{
    HAL_StatusTypeDef lResult = HAL_OK, lResult1 = HAL_OK;
    
    if(motorNum < M_TOTAL_NUM)
    {
        if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //��canopen������
        {
            if((MOTOR_SET_TARGET_VEL == changeFlag) ||
                (MOTOR_SET_TARGET_VEL_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].targetVel = value;
                if(DRIVER_TYPE_KINCO == gStMotorData[motorNum].driverType)
                {
                    //PWMSetMotorValue(motorNum, ABS_VALUE(value));
                    Motor485SetTargetVelocity(motorNum, value);
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                    {
                        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                        {
                            Motor485SetTargetVelocity(gStMotorData[motorNum].relatedMotor, value);
                            //PWMSetMotorValue(gStMotorData[motorNum].relatedMotor, ABS_VALUE(value));
                        }
                    }
                }
            }
            else if((MOTOR_SET_TARGET_POS == changeFlag) ||
                (MOTOR_SET_TARGET_POS_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].targetPos = value;
                if((DRIVER_TYPE_DUOJI_GDW == gStMotorData[motorNum].driverType)
                    || (DRIVER_TYPE_LEADSHINE == gStMotorData[motorNum].driverType))
                {
                    PWMSetMotorValue(motorNum, value);
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                    {
                        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                        {
                            PWMSetMotorValue(gStMotorData[motorNum].relatedMotor, value);
                        }
                    }
                }
                else if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
                {
                    if(gStMotorData[motorNum].flag & MOTOR_PWM_CONTRL_MODE)  //pwm���Ʒ�ʽ
                    {
                        PWMSetMotorValue(motorNum, value);
                    }
                    else
                    {
                        return Motor485SetTargetAbsPosRealtime(motorNum, value);
                    }
                }
            }
            else if((MOTOR_SET_TARGET_CURRENT == changeFlag) ||
                (MOTOR_SET_TARGET_CURRENT_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].setCurrent = value;
                if(DRIVER_TYPE_KINCO == gStMotorData[motorNum].driverType)
                {
                    //PWMSetMotorValue(motorNum, ABS_VALUE(value));
                    lResult = Motor485SetTargetCurrent(motorNum, value);
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                    {
                        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                        {
                            lResult1 = Motor485SetTargetCurrent(gStMotorData[motorNum].relatedMotor, value);
                        }
                    }
                    if(MOTOR_SET_TARGET_CURRENT_NEED_JUDGE == changeFlag)
                    {
                        return ((lResult == HAL_OK) ? lResult1 : lResult);
                    }
                }
            }
        }
        else
        {
            if((MOTOR_SET_TARGET_POS == changeFlag) ||
                (MOTOR_SET_TARGET_POS_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].targetPos = value;
            }
            else if((MOTOR_SET_TARGET_VEL == changeFlag) ||
                (MOTOR_SET_TARGET_VEL_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].targetVel = value;
            }
            else if((MOTOR_SET_TARGET_CURRENT == changeFlag) ||
                (MOTOR_SET_TARGET_CURRENT_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].setCurrent = value;
            }
            else
            {
                gStMotorRunState[motorNum].targetPos = 0;
                gStMotorRunState[motorNum].targetVel = 0;
                gStMotorRunState[motorNum].setCurrent = 0;
            }
            gStMotorRunState[motorNum].targetPosVelSetFlag = changeFlag;
            gStMotorRunState[motorNum].retrySetTargetTime = 0;
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������������ͬĿ��ֵ
            {
                if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
                {
                    if(M_LEFT == motorNum)
                    {
                        if((gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                            (DRIVER_TYPE_NONE != gStMotorData[M_LEFT_ONE].driverType))
                        {
                            gStMotorRunState[M_LEFT_ONE].targetPos = gStMotorRunState[motorNum].targetPos;
                            gStMotorRunState[M_LEFT_ONE].targetVel = gStMotorRunState[motorNum].targetVel;
                            gStMotorRunState[M_LEFT_ONE].setCurrent = gStMotorRunState[motorNum].setCurrent;
                            gStMotorRunState[M_LEFT_ONE].targetPosVelSetFlag = changeFlag;
                            gStMotorRunState[M_LEFT_ONE].retrySetTargetTime = 0;
                            if(IS_UFOONE_FLAG_SET(UFOONE_DIFF_DRIVER_CURRENT))  //������������ֵ�ɱ����Ŵ���С
                            {
                                gStMotorRunState[M_LEFT_ONE].setCurrent = CalDifDriverCurrent(M_LEFT_ONE, motorNum);
                            }
                        }
                    }
                }
                if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                {
                    gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetPos = gStMotorRunState[motorNum].targetPos;
                    gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVel = gStMotorRunState[motorNum].targetVel;
                    gStMotorRunState[gStMotorData[motorNum].relatedMotor].setCurrent = gStMotorRunState[motorNum].setCurrent;
                    gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetPosVelSetFlag = changeFlag;
                    gStMotorRunState[gStMotorData[motorNum].relatedMotor].retrySetTargetTime = 0;
                    if(IS_UFOONE_FLAG_SET(UFOONE_DIFF_DRIVER_CURRENT))  //������������ֵ�ɱ����Ŵ���С
                    {
                        gStMotorRunState[gStMotorData[motorNum].relatedMotor].setCurrent = CalDifDriverCurrent(gStMotorData[motorNum].relatedMotor, motorNum);
                    }
                    if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
                    {
                        if(M_RIGHT == gStMotorData[motorNum].relatedMotor)
                        {
                            if((gStMotorData[M_RIGHT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                                (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType))
                            {
                                gStMotorRunState[M_RIGHT_ONE].targetPos = gStMotorRunState[motorNum].targetPos;
                                gStMotorRunState[M_RIGHT_ONE].targetVel = gStMotorRunState[motorNum].targetVel;
                                gStMotorRunState[M_RIGHT_ONE].setCurrent = gStMotorRunState[motorNum].setCurrent;
                                gStMotorRunState[M_RIGHT_ONE].targetPosVelSetFlag = changeFlag;
                                gStMotorRunState[M_RIGHT_ONE].retrySetTargetTime = 0;
                                if(IS_UFOONE_FLAG_SET(UFOONE_DIFF_DRIVER_CURRENT))  //������������ֵ�ɱ����Ŵ���С
                                {
                                    gStMotorRunState[M_RIGHT_ONE].setCurrent = CalDifDriverCurrent(M_RIGHT_ONE, motorNum);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return HAL_OK;
}
/*****************************************************************************
 ��������  : ����������Ŀ��λ�á��ٶȻ����
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ��ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��1��
*****************************************************************************/
#ifdef ENABLE_CANOPEN_DRIVER
static void SetDriveMotorTargetValue(uint32_t motorNum, uint32_t processTimeMs)
{
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }

    gStMotorRunState[motorNum].operTimedelay += processTimeMs;
    
    //������������־
    if(gStMotorRunState[motorNum].clearStartMoveFlag)
    {
        if(0 == MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_DISABLE_VOLTAGE))
        {
            gStMotorRunState[motorNum].clearStartMoveFlag = RT_FALSE;
        }
        else
        {
            gStMotorRunState[motorNum].retrySetTargetTime++;
            if(gStMotorRunState[motorNum].retrySetTargetTime >= 10)
            {
                gStMotorRunState[motorNum].retrySetTargetTime = 0;
                rt_kprintf("clear start failed, id = %d!\r\n", gStMotorData[motorNum].idx);
                 gStMotorRunState[motorNum].clearStartMoveFlag = RT_FALSE;
            }
        }
        if(RT_FALSE == gStMotorRunState[motorNum].clearStartMoveFlag)
        {
            if(MOTOR_SET_TARGET_POS_CLEAR_START_FLAG == gStMotorRunState[motorNum].targetPosVelSetFlag)
            {
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            }
        }
    }
    //�л��ٶȻ�λ�ã�����Ҫ����
    else if(MOTOR_SET_TARGET_POS == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        MotorSetTargetPosition(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].targetPos, RT_FALSE);
        MotorDeviceControlCmd(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION1);//��������ó�����״̬
        if((DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)
            || (DRIVER_TYPE_STAND== gStMotorData[motorNum].driverType)
            || (DRIVER_TYPE_NIMOTION== gStMotorData[motorNum].driverType))
        {
            gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_SET_NEW_POSITION;
        }
        gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
    }
    else if(MOTOR_SET_TARGET_VEL == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        MotorSetTargetVelocity(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].targetVel, RT_FALSE);
        gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
        if(MOTOR_OPERATION_MODE_PVM == gStMotorRunState[motorNum].operationMode)
        {
            if(DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)
            {
                MotorDeviceControlCmd(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION);//��ʼ�˶�
            }
        }
    }
    //�л��ٶȣ���Ҫ�ж�
    else if(MOTOR_SET_TARGET_VEL_NEED_JUDGE == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        if(0 == MotorSetTargetVelocity(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].targetVel, RT_TRUE))
        {
            if(MOTOR_OPERATION_MODE_CSV != gStMotorRunState[motorNum].operationMode)
            {
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            }
            else
            {
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_TARGET_VEL_JUDGE;
                
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                MotorSendReadStatus(gStMotorData[motorNum].idx);
            }

            if(MOTOR_OPERATION_MODE_PVM == gStMotorRunState[motorNum].operationMode)
            {
                if(DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)
                {
                    MotorDeviceControlCmd(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION);//��ʼ�˶�
                }
            }
        }
        else
        {
            gStMotorRunState[motorNum].retrySetTargetTime++;
            if(gStMotorRunState[motorNum].retrySetTargetTime >= 10)
            {
                gStMotorRunState[motorNum].retrySetTargetTime = 0;
                rt_kprintf("vel set failed, id = %d!\r\n", gStMotorData[motorNum].idx);
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            }
        }
    } 
    //�жϵ���Ƿ�ִ��Ŀ���ٶ�
    else if(MOTOR_SET_TARGET_VEL_JUDGE == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        if((gStMotorRunState[motorNum].operTimedelay >= 50)
            || IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_STATUS_FLAG))
        {
            gStMotorRunState[motorNum].operTimedelay = 0;
            if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_STATUS_FLAG))
            {
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                if((gStMotorRevData[motorNum].status & 0x1000) && (MOTOR_OPERATION_MODE_CSV == gStMotorRunState[motorNum].operationMode))
                {
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
                }
                else
                {
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_TARGET_VEL_NEED_JUDGE;//δִ������Ҫ��������һ���ٶ�
                }
            }
            else
            {
                MotorSendReadStatus(gStMotorData[motorNum].idx);
            }
            gStMotorRunState[motorNum].retrySetTargetTime++;
            if(gStMotorRunState[motorNum].retrySetTargetTime >= 10)
            {
                gStMotorRunState[motorNum].retrySetTargetTime = 0;
                rt_kprintf("vel exe failed, id = %d!\r\n", gStMotorData[motorNum].idx);
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            }
        }
    }
    //�л�λ�ã���Ҫ�ж�
    else if(MOTOR_SET_TARGET_POS_NEED_JUDGE == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        if(OPERATION_MODE_SET == gStMotorRunState[motorNum].operSetNewStep)
        {
            gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_UNKOWN;
            MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION);
        }
        if(0 == MotorSetTargetPosition(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].targetPos, RT_TRUE))
        {
            if(0 == MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION1))//��������ó�����״̬
            {
                gStMotorRunState[motorNum].operTimedelay = 8;
                gStMotorRunState[motorNum].retrySetTargetTime = 0;
                if(DRIVER_TYPE_QILING == gStMotorData[motorNum].driverType)
                {
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_TARGET_POS_CLEAR_START_FLAG;
                    gStMotorRunState[motorNum].clearStartMoveFlag = RT_TRUE;
                }
                else
                {
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_TARGET_POS_JUDGE;
                    CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
                    if((DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)
                        || (DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                        || (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType))
                    {
                        gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_SET;
                    }
                }
                gStMotorRunState[motorNum].posFlag = POS_FLAG_UNKOWN;
            }
            else
            {
                gStMotorRunState[motorNum].retrySetTargetTime++;
            }
        }
        else
        {
            gStMotorRunState[motorNum].retrySetTargetTime++;
        }

        if(gStMotorRunState[motorNum].retrySetTargetTime >= 10)
        {
            gStMotorRunState[motorNum].retrySetTargetTime = 0;
            gStMotorRunState[motorNum].retryTime = 0;
            rt_kprintf("pos set failed, id = %d!\r\n", gStMotorData[motorNum].idx);
            gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
        }
    } 
    //�жϵ���Ƿ�ִ��Ŀ��λ��
    else if(MOTOR_SET_TARGET_POS_JUDGE == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        if(gStMotorRunState[motorNum].operTimedelay >= 8)
        {
            gStMotorRunState[motorNum].operTimedelay = 0;            
            gStMotorRunState[motorNum].retrySetTargetTime++;
            if(gStMotorRunState[motorNum].retrySetTargetTime >= 5)
            {
                gStMotorRunState[motorNum].retrySetTargetTime = 0;
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_TARGET_POS_NEED_JUDGE;
                gStMotorRunState[motorNum].retryTime++;
                if(gStMotorRunState[motorNum].retryTime >= 3)
                {
                    gStMotorRunState[motorNum].retryTime = 0;
                    rt_kprintf("pos exe failed, id = %d!\r\n", gStMotorData[motorNum].idx);
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
                }
            }
            else
            {
                MotorSendReadStatus(gStMotorData[motorNum].idx);
            }
        }
        
        if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_STATUS_FLAG))
        {
            if((DRIVER_TYPE_EPOS == gStMotorData[motorNum].driverType)
                || (DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                || (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType))
            {
                if(gStMotorRevData[motorNum].status & 0x1000)
                {
                    gStMotorRunState[motorNum].retryTime = 0;
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
                    gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_SET_NEW_POSITION;
                }
            }
            else if((DRIVER_TYPE_PUSI == gStMotorData[motorNum].driverType) && (0 == (gStMotorRevData[motorNum].status & 0x1400)))
            {
                gStMotorRunState[motorNum].retryTime = 0;
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            }
            CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
        }
    }
    //�л����������ж�
    else if(MOTOR_SET_TARGET_CURRENT == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        MotorSetTargetCurrent(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].setCurrent, RT_FALSE);
        gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
        if(DEBUG_DATA_TYPE_92)
        {
            rt_kprintf("M%d set current:%d.\r\n", motorNum, HAL_GetTick());
        }
    }
    //�л���������Ҫ�ж�
    else if(MOTOR_SET_TARGET_CURRENT_NEED_JUDGE == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        if(0 == MotorSetTargetCurrent(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].setCurrent, RT_TRUE))
        {
            gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            if(DEBUG_DATA_TYPE_92)
            {
                rt_kprintf("M%d set current1:%d.\r\n", motorNum, HAL_GetTick());
            }
        }
        else
        {
            gStMotorRunState[motorNum].retrySetTargetTime++;
            if(gStMotorRunState[motorNum].retrySetTargetTime >= 5)
            {
                gStMotorRunState[motorNum].retrySetTargetTime = 0;
                rt_kprintf("current set failed, id = %d!\r\n", gStMotorData[motorNum].idx);
                gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
            }
        }
    }
}
#endif
/*****************************************************************************
 ��������  : �л��������������ģʽ
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��12��1��
*****************************************************************************/
static void ChangeDriveMotorControlMode(uint32_t processTimeMs)
{
    uint32_t i;
#ifdef ENABLE_CANOPEN_DRIVER
    int8_t lOperationMode;
    uint16_t l_status;
#endif

    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        //�ȴ�����Զ��֡����
        if(0 == gStMotorRunState[i].startRemoteFlag)
        {
            continue;
        }
        else if(gStMotorData[i].flag & MOTOR_ENABLE_FLAG) //תʹ��
        {
            if(DISABLE == gStMotorRunState[i].enableFlag)
            {
                if(OPERATION_MODE_UNKOWN != gStMotorRunState[i].operModeSetStep)
                {
                    gStMotorRunState[i].operModeSetStep = OPERATION_MODE_UNKOWN;
                }
            }
            else if(OPERATION_MODE_SET_FINISH != gStMotorRunState[i].operModeSetStep)
            {
                if(OPERATION_MODE_SET != gStMotorRunState[i].operModeSetStep)
                {
                    gStMotorRunState[i].operModeSetTimeOut = 0;
                    gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET;
                }
                else
                {
                    gStMotorRunState[i].operModeSetTimeOut += processTimeMs;
                    if(gStMotorRunState[i].operModeSetTimeOut >= gStMotorData[i].enableDelayTime)
                    {
                        gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;
                    }
                }
            }
            continue;
        }
        else if(IS_NOT_CANOPEN_DRIVER(gStMotorData[i].driverType))    //��canopen������
        {
            continue;
        }
        //�Ƿ���Ҫ�л�
        if(OPERATION_MODE_UNKOWN == gStMotorRunState[i].operModeSetStep)
        {
            continue;
        }
        else if(OPERATION_MODE_SET_FINISH == gStMotorRunState[i].operModeSetStep)
        {
            if((OPERATION_MODE_UNKOWN == gStMotorRunState[i].operSetNewStep)
                || (OPERATION_MODE_SET == gStMotorRunState[i].operSetNewStep))
            {
                continue;
            }
        }
#ifdef ENABLE_CANOPEN_DRIVER
        else if(MOTOR_OPERATION_MODE_UNKOWN == gStMotorRunState[i].operationMode)
        {
            if(OPERATION_STOP_FINISH != gStMotorRunState[i].operStopStep)
            {
                gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;
            }
            continue;
        }        
        //�������ʱ������
        if(ERROR_NONE != gStMotorRunState[i].newErrorLevel)
        {
            gStMotorRunState[i].operationMode = MOTOR_OPERATION_MODE_UNKOWN;
            continue;
        }
        //��ʱ����
        gStMotorRunState[i].operModeSetTimeOut += processTimeMs;
        gStMotorRunState[i].operTimedelay += processTimeMs;
        if(gStMotorRunState[i].operModeSetTimeOut >= OPERATION_MODE_SET_TIMEOUT)
        {
            rt_kprintf("Motor operate mode change failed, id = %d!\r\n", gStMotorData[i].idx);            
            gStMotorRunState[i].operModeSetTimeOut = 0;
            gStMotorRunState[i].retryTime++;
            if(OPERATION_STOP_FINISH == gStMotorRunState[i].operStopStep) 
            {
                gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET;
            }
            if(gStMotorRunState[i].retryTime >= 3)
            {
                gStMotorRunState[i].retryTime = 0;
                gStMotorRunState[i].operationMode = MOTOR_OPERATION_MODE_UNKOWN;
                SetErrorCode(i, ERROR_CODE_OPERATRION_MODE_SET_FAILED, ERROR_L_HIHG);
            }
        }
        //ֹͣ���
        if(OPERATION_STOP_FINISH != gStMotorRunState[i].operStopStep)
        {
            if((DRIVER_TYPE_PUSI ==  gStMotorData[i].driverType)
                || (DRIVER_TYPE_QILING ==  gStMotorData[i].driverType))
            {
                gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;//ֹͣ���
            }
            else if(gStMotorRunState[i].operTimedelay >= 60)
            {
                gStMotorRunState[i].operTimedelay = 0;
                if((OPERATION_DISABLE_VOLTAGE == gStMotorRunState[i].operStopStep)
                    || (OPERATION_QUCIK_STOP == gStMotorRunState[i].operStopStep))
                {
                    if(0 == MotorReadStatus(gStMotorData[i].idx, &l_status))
                    {
                        l_status &= DEVICE_STATUS_MASK;
                        if(DEVICE_STATUS_OPERATION_ENABLE != l_status)
                        {
                            gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;//ֹͣ���
                        }
                        if(OPERATION_QUCIK_STOP == gStMotorRunState[i].operStopStep)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_QUICK_STOP);
                        }
                        else
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);
                        }
                    } 
                }
                else
                {
                    gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;//ֹͣ���
                }
                if(OPERATION_STOP_FINISH == gStMotorRunState[i].operStopStep)
                {
                    gStMotorRunState[i].operModeSetTimeOut = 0;
                    gStMotorRunState[i].retryTime = 0;
                    if(DRIVER_TYPE_STAND ==  gStMotorData[i].driverType)
                    {
                        gStMotorRunState[i].operationStopJudgeFlag = 1;  //ֹͣ��ɺ���Ҫ�жϵ��ʵ��ת���Ƿ���ֹͣ
                    }
                }
            }
            continue;
        }
        //�жϵ��ʵ��ת���Ƿ���ֹͣ
        if((gStMotorRunState[i].operationStopJudgeFlag) && (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operModeSetStep))
        {
            if(1 == gStMotorRunState[i].operationStopJudgeFlag) //��ȡʵ��ת��
            {
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_SPEED_FLAG);
                MotorSendReadVelocity(gStMotorData[i].idx);
                gStMotorRunState[i].operationStopJudgeFlag = 2;
            }
            else if(2 == gStMotorRunState[i].operationStopJudgeFlag) //�ж��Ƿ���յ�ʵ��ת��
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_SPEED_FLAG))
                {
                    gStMotorRunState[i].operationStopJudgeFlag = 3;
                }
            }
            if(gStMotorRunState[i].operTimedelay >= 300)
            {
                gStMotorRunState[i].operTimedelay = 0;
                if(3 == gStMotorRunState[i].operationStopJudgeFlag)
                {
                    if(ABS_VALUE(gStMotorRevData[i].speed) <= 10)   //���ʵ�ʽӽ�ֹͣ
                    {
                        gStMotorRunState[i].operModeSetTimeOut = 0;
                        gStMotorRunState[i].retryTime = 0;
                        gStMotorRunState[i].operationStopJudgeFlag = 0;
                    }
                    else 
                    {
                        gStMotorRunState[i].operationStopJudgeFlag = 1;
                    }
                    if(DEBUG_DATA_TYPE_5)
                    {
                        rt_kprintf("id-%d-get-vel: %d.\r\n", gStMotorData[i].idx,
                            gStMotorRevData[i].speed);
                    }
                }
                else
                {
                    gStMotorRunState[i].operationStopJudgeFlag = 1;
                }
            }
            continue;
        }
        //���õ�����в���
        if(gStMotorData[i].flag & MOTOR_NEED_SET_RUN_PARAS)
        {
            if(gStMotorRunState[i].operationMode == MOTOR_OPERATION_MODE_PPM)   //����λ��ģʽ������в���
            {
                if(!(gStMotorRunState[i].runParasSetFlag & RUN_PARAS_PROFILE_VEL_SET)) //�趨�����ٶ�
                {
                    if(0 == MotorSetProfileVelocity(gStMotorData[i].idx, gStMotorData[i].profileVelocity, RT_TRUE))
                    {
                        gStMotorRunState[i].runParasSetFlag |= RUN_PARAS_PROFILE_VEL_SET;
                    }
                    continue;
                }
                if(!(gStMotorRunState[i].runParasSetFlag & RUN_PARAS_PROFILE_ACC_SET)) //�趨�������ٶ�
                {
                    if(0 == MotorSetProfileAcc(gStMotorData[i].idx, gStMotorData[i].profileAcc, RT_TRUE))
                    {
                        gStMotorRunState[i].runParasSetFlag |= RUN_PARAS_PROFILE_ACC_SET;
                    }
                    continue;
                }
                if(!(gStMotorRunState[i].runParasSetFlag & RUN_PARAS_PROFILE_DEC_SET)) //�趨�������ٶ�
                {
                    if(0 == MotorSetProfileDec(gStMotorData[i].idx, gStMotorData[i].profileAcc, RT_TRUE)) //��ʱ�ͼ��ٶ�ֵһ��
                    {
                        gStMotorRunState[i].runParasSetFlag |= RUN_PARAS_PROFILE_DEC_SET;
                    }
                    continue;
                }
            }
        }
        //λ��ģʽ��׼�������µ�λ��
        if(OPERATION_MODE_UNKOWN != gStMotorRunState[i].operSetNewStep)
        {
            if(OPERATION_MODE_SET_FINISH != gStMotorRunState[i].operModeSetStep)
            {
                gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
            }
        }
        if(OPERATION_MODE_SET_NEW_POSITION == gStMotorRunState[i].operSetNewStep)
        {
            if(0 == MotorDeviceControlCmdWithJudge(gStMotorData[i].idx, DEVICE_CTROL_ENABLE_OPERATION))//ʹ�ܵ���˶�
            {
                gStMotorRunState[i].operSetNewStep = OPERATION_MODE_AUTO_CHANGE;    //����ģʽ���
            }
        }
        //��ʼ�л�ģʽ
        if(OPERATION_MODE_SET == gStMotorRunState[i].operModeSetStep)
        {
            if(0 == MotorReadStatus(gStMotorData[i].idx, &l_status))
            {
                l_status &= DEVICE_STATUS_MASK;
                if(DEVICE_STATUS_OPERATION_ENABLE != l_status)
                {
                    if(0 == MotorGetOperationMode(gStMotorData[i].idx, &lOperationMode))
                    {
                        if(lOperationMode != gStMotorRunState[i].operationMode)
                        {
                            MotorSetOperationMode(gStMotorData[i].idx, gStMotorRunState[i].operationMode);//����ģʽ
                        }
                        else if(lOperationMode == MOTOR_OPERATION_MODE_HMM)
                        {
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_HOME_METHOD;//�л������û��㷽��
                        }
                        else
                        {
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SHUT_DOWN;//�л���׼��״̬
                        }
                    }
                }
                else//�Ͽ����������
                {
                    MotorDeviceControlCmdWithJudge(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);
                }
            }
        }
        //��ʼ�л�Ѱ�㷽��
        else if(OPERATION_MODE_SET_HOME_METHOD == gStMotorRunState[i].operModeSetStep)
        {
            if(0 == MotorGetHomingMethod(gStMotorData[i].idx, &lOperationMode))
            {
                if(lOperationMode != HOMMING_METHOD_37_ACTUAL_POSITION)
                {
                    MotorSetHomingMethod(gStMotorData[i].idx, HOMMING_METHOD_37_ACTUAL_POSITION);
                }
                else
                {
                    gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SHUT_DOWN;//�л���׼��״̬
                }
            }
        }
        //��ʼ�л�����״̬������Firmware-Specification�ĵ��е�2.2���豸״̬����
        else if((OPERATION_MODE_SHUT_DOWN == gStMotorRunState[i].operModeSetStep)
            || (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operModeSetStep)
            || (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operSetNewStep))
        {
            if((gStMotorRunState[i].operTimedelay >= 8) || (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operSetNewStep))
            {
                gStMotorRunState[i].operTimedelay = 0;
                if(DRIVER_TYPE_PUSI == gStMotorData[i].driverType)
                {
                    if(0 == MotorReadEnableStatus(gStMotorData[i].idx, &l_status))
                    {
                        if(l_status)
                        {
                            gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;//�л�ģʽ���
                        }
                        else
                        {
                            MotorSetMotorEnableStatus(gStMotorData[i].idx, 1);
                        }
                    }
                }
                else if(DRIVER_TYPE_QILING == gStMotorData[i].driverType)
                {
                    gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
                    gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;//�л�ģʽ���
                }
                else if(0 == MotorReadStatus(gStMotorData[i].idx, &l_status))
                {
                    l_status &= DEVICE_STATUS_MASK;
                    if(OPERATION_MODE_SHUT_DOWN == gStMotorRunState[i].operModeSetStep)
                    {
                        if(DEVICE_STATUS_QUICK_STOP == l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);//��ֹ��ѹ���
                        }
                        else if(DEVICE_STATUS_READY_SWITCH_ON != l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_SHUT_DOWN);//�ضϵ����Դ
                        }
                        else
                        {
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;//���ݷ��ص�״̬�Զ��л�
                        }
                    }
                    if(((OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operModeSetStep)
                        || (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operSetNewStep))
                        && (0 == gStMotorRunState[i].operationStopJudgeFlag))
                    {
                        if(DEVICE_STATUS_QUICK_STOP == l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);//��ֹ��ѹ���
                        }
                        else if(DEVICE_STATUS_SWITCH_ON_DISABLE & l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_SHUT_DOWN);//�ضϵ����Դ
                        }
                        else if((DEVICE_STATUS_READY_SWITCH_ON == l_status)
                            && (DRIVER_TYPE_FDK == gStMotorData[i].driverType))
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_SWITCH_ON);//�ȴ����ʹ��
                        }
                        else if((DEVICE_STATUS_READY_SWITCH_ON == l_status)
                            || (DEVICE_STATUS_SWITCH_ON == l_status))
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_ENABLE_OPERATION);//ʹ�ܵ���˶�
                        }
                        else if(DEVICE_STATUS_OPERATION_ENABLE == l_status)
                        {
                            //��ÿ�����������Ҫ�ж�2001��״̬
                            if(DRIVER_TYPE_FDK == gStMotorData[i].driverType)
                            {
                                l_status = 0;
                                if(0 == MotorReadFDKWorkStatus(gStMotorData[i].idx, &l_status))
                                {
                                    if(l_status & 0x03)
                                    {
                                        l_status = DEVICE_STATUS_OPERATION_ENABLE;
                                    }
                                }
                            }
                            //ʹ�����
                            if(DEVICE_STATUS_OPERATION_ENABLE == l_status)
                            {
                                gStMotorRunState[i].retryTime = 0;
                                gStMotorRunState[i].operModeSetTimeOut = 0;
                                gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
                                gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;//�л�ģʽ���
                                if(DEBUG_DATA_TYPE_6)
                                {
                                    rt_kprintf("id-%d-change-drivermode-finish.\r\n", gStMotorData[i].idx);
                                }
                            }
                        }
                    }
                }
            }
        } 
#endif
    }    
}
/*****************************************************************************
 ��������  : ���ù��Ͻṹ����Ϣ�����ڼ�¼���ϵĹ�������
 �������  : uint16_t motorNum
             uint16_t lResult   
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��8��31��
*****************************************************************************/
void SetErrorInfo(uint16_t motorNum, uint16_t lResult)
{
    int pidInfoRecordFlag = 0;

    gStErrorInfo.errorMotorNum = motorNum;
    gStErrorInfo.errorResult = lResult;
    gStErrorInfo.errRecordTime = HAL_GetTick();
    
    //1003����
    if(WARNING_CODE_NAV_ABNORMAL == lResult)
    {
        if(motorNum < M_TOTAL_NUM)
        {
            gStErrorInfo.setCurrentPositiveChangeIndex = gMotorAvaSetCurrentPositiveChangeIndex;
            gStErrorInfo.setCurrentNegativeChangeIndex = gMotorAvaSetCurrentNegativeChangeIndex;
        }
        pidInfoRecordFlag = 1;
    }
    //1004����
    if(WARNING_CODE_ABNORMAL_DECELERATION == lResult)
    {
        pidInfoRecordFlag = 1;
    }

    if(pidInfoRecordFlag && (motorNum < M_TOTAL_NUM))
    {
        gStErrorInfo.idx = gStMotorData[motorNum].idx;
        memcpy((uint8_t*)gStErrorInfo.setVelRecord, (uint8_t*)gStMotorRunState[motorNum].setVelRecord, sizeof(int32_t) * MVEL_RECORD_NUM);
        memcpy((uint8_t*)gStErrorInfo.carVelRecord, (uint8_t*)gCarVel, sizeof(int32_t) * MVEL_RECORD_NUM);
        memcpy((uint8_t*)gStErrorInfo.mVel, (uint8_t*)gMotorAvaVel, sizeof(int32_t) * MVEL_RECORD_NUM);
        memcpy((uint8_t*)gStErrorInfo.setCurrentRecord, (uint8_t*)gMotorAvaSetCurrentRecord, sizeof(int32_t) * MVEL_RECORD_NUM);
    }
}
/*****************************************************************************
 ��������  : ��ӡ������ϸ��Ϣ
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��8��31��
*****************************************************************************/
void PrintErrorInfo(void)
{
    int k;
    
    if(gStErrorInfo.errorResult)
    {
        if(WARNING_CODE_RUNNING_LOW_VOLTAGE == gStErrorInfo.errorResult)
        {
            rt_kprintf("DriverVol:%dv,%dv,%dv,%dv.\r\n", gStErrorInfo.driverVoltage[M_LEFT], gStErrorInfo.driverVoltage[M_RIGHT],
                gStErrorInfo.driverVoltage[M_LEFT_ONE], gStErrorInfo.driverVoltage[M_RIGHT_ONE]);
        }
        else
        {
            for(k = 0; k < MVEL_RECORD_NUM; k++)
            {
                rt_kprintf("id-%d-t:%d,c:%d,m:%d,s:%d,tv:%d,sa:%d,ca:%d,mf:%d.\r\n", gStErrorInfo.idx,
                    gStErrorInfo.setVelRecord[k], gStErrorInfo.carVelRecord[k], gStErrorInfo.mVel[k], gStErrorInfo.setCurrentRecord[k], 0, 
                    0, 0, 0);
            }
        }
        rt_kprintf("M%d errorCode: 0x%04x, t:%d!\r\n", gStErrorInfo.errorMotorNum, gStErrorInfo.errorResult, gStErrorInfo.errRecordTime);
        if(WARNING_CODE_NAV_ABNORMAL == gStErrorInfo.errorResult)
        {
            rt_kprintf("Car:%d,mvel:%d,%d%%,pChange:%d,nChange:%d,ratio:%d,autoRatio:%d.\r\n", gStErrorInfo.carFilter, gStErrorInfo.motorFilter,
                (0 == gStErrorInfo.motorFilter) ? 0 : (ABS_VALUE((gStErrorInfo.carFilter - gStErrorInfo.motorFilter) * 100) / ABS_VALUE(gStErrorInfo.motorFilter)),
                gStErrorInfo.setCurrentPositiveChangeIndex, gStErrorInfo.setCurrentNegativeChangeIndex, gStUfoData.ratioVel, gStErrorInfo.ratioVelAuto);
        }
        else if(WARNING_CODE_ABNORMAL_DECELERATION == gStErrorInfo.errorResult)
        {
            rt_kprintf("Tar:%d,car:%d,setAcc:%d,curAcc:%d.\r\n", gStErrorInfo.setVelRecord[MVEL_RECORD_NUM - 1], gStErrorInfo.carVelRecord[MVEL_RECORD_NUM - 1],
                gStErrorInfo.setAcc, gStErrorInfo.curAcc);
        }
        rt_kprintf("Err9025 ps:0x%04x, ps:0x%04x, t:%d.\r\n", gStErrorInfo.preState, gStErrorInfo.afterState[5], gStErrorInfo.afterTime[5]);
        for(k = 0; k < 5; k++)
        {
            rt_kprintf("as:0x%04x, aws:0x%04x, t:%d.\r\n", gStErrorInfo.afterState[k], gStErrorInfo.afterWorkState[k], gStErrorInfo.afterTime[k]);
        }
    }
    else
    {
        rt_kprintf("No error detail info!\r\n");
    }
}
/*****************************************************************************
 ��������  : �趨�����־���������ڴ������������Ĵ���
 �������  : uint16_t motorNum        ������
             uint16_t lResult         ������ϴ���
             ERROR_LEVEL errorLevel   ���ϵȼ�
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��12��
*****************************************************************************/
void SetErrorCode(uint16_t motorNum, uint16_t lResult, ERROR_LEVEL errorLevel)
{
    uint16_t motorNumBackup;

    motorNumBackup = motorNum;
    if(motorNum >= WARNING_MOTOR_NUM_OFFSET)
    {
        motorNum -= WARNING_MOTOR_NUM_OFFSET;
    }

    //�Ƿ��˶������г��ֵĹ���
    if(!((M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT].pidCurrentStartFlag)
        && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
        && (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
        && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)))
    {
        gErrorRunStateFlag = 1;
    }

    //������ʷ���ϼ�¼
    UpdateHistoryError((motorNum == M_TOTAL_NUM) ? (motorNumBackup - M_TOTAL_NUM) : (motorNumBackup + 1), lResult);
    
    sys_para->CAR_RTinf.Link |= LINK_ERROR_STOP; //������Ͻ���ֹͣ״̬
    sys_para->CAR_RTinf.Link &= ~LINK_REV_STOP_CMD; //����ʱ����յ�ֹͣ����ı�־
    gErrorTime = HAL_GetTick(); //��¼�����ϵ�ʱ��
        
    if(DEBUG_DATA_TYPE_1)
    {
        rt_kprintf("Enter error stop.\r\n");
    }
    if(motorNum < M_TOTAL_NUM)
    {
        gErrorMotorNum = motorNumBackup + 1;
        gErrorResult = lResult;
        if(errorLevel > gStMotorRunState[motorNum].errorLevel)
        {
            gStMotorRunState[motorNum].errorLevel = errorLevel;
        }
        if(IS_WARNING_CODE(gErrorMotorNum))
        {
            rt_kprintf("M%d warningCode: 0x%04x!\r\n", motorNum, lResult);
        }
        else
        {
            rt_kprintf("M%d errorCode: 0x%04x!\r\n", motorNum, lResult);
        }
        if(gStUfoData.flag & UFO_ERROR_WAIT_FLAG)   //���ϵȴ���־�������й����Իָ�
        {
            gStMotorRunState[motorNum].errorLevel = ERROR_L_VERY_HIGH;
        }
        gStMotorRunState[motorNum].newErrorLevel = gStMotorRunState[motorNum].errorLevel;
    }
    else if(motorNum == M_TOTAL_NUM)
    {
        //�ϴηǹ����������²Ÿ����µľ��������룬��ֹ�������滻��������
        if(!((!IS_WARNING_CODE(gErrorMotorNum)) && (0 != gErrorResult)))
        {
            gErrorMotorNum = motorNumBackup - M_TOTAL_NUM;
            gErrorResult = lResult;
        }
        if(IS_WARNING_CODE(motorNumBackup - M_TOTAL_NUM))
        {
            rt_kprintf("WarningCode: 0x%04x!\r\n", lResult);
        }
        else
        {
            rt_kprintf("ErrorCode: 0x%04x!\r\n", lResult);
        }
    }
}
/*****************************************************************************
 ��������  : ����������
 �������  : uint16_t motorNum  ������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��4��8��
*****************************************************************************/
void ClearErrorCode(uint16_t motorNum)
{
    int i;
    
    if(motorNum < M_TOTAL_NUM)
    {
        if(DEBUG_DATA_TYPE_6 && (ERROR_NONE != gStMotorRunState[motorNum].errorLevel))
        {
            rt_kprintf("M%d error clear!\r\n", motorNum);
        }
        //����ɻָ����ϣ��ָ���ֹͣ״̬
        if(ERROR_L_NORMAL == gStMotorRunState[motorNum].errorLevel)
        {
            //gStMotorRunState[motorNum].runState = MOTOR_STATE_STOP;
            gStMotorRunState[motorNum].errorLevel = ERROR_NONE;
        }
        //�ϵ���ָܻ��Ĺ��ϣ��л����ϵ�״̬
        else if(ERROR_L_HIHG == gStMotorRunState[motorNum].errorLevel)
        {
            //gStMotorRunState[motorNum].runState = MOTOR_STATE_POWER_OFF;
            gStMotorRunState[motorNum].errorLevel = ERROR_NONE;
        }
    }
    else if(gErrorResult)
    {
        for(i = M_LEFT; i < M_TOTAL_NUM; i++)
        {
            if(ERROR_NONE != gStMotorRunState[i].errorLevel)   //����״̬
            {
                break;
            }
        }
        if((M_TOTAL_NUM == i) && (!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)))   //����޹��ϲ��ҹ���ֹͣ״̬ʱ���������
        {
            if(DEBUG_DATA_TYPE_6)
            {
                rt_kprintf("Error flag clear!\r\n");
            }
            gErrorResult = 0;   //������ϱ�־
            gErrorRunStateFlag = 0;
        }
    }
}
/*****************************************************************************
 ��������  : �����Զ�ֹͣ����
 �������  : ST_MOTOR_RUN_MODE_DATA* lMotorRunMode
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��7��19��
*****************************************************************************/
void ErrorAutoStopCmd(ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    uint32_t motorNum;
    
    for(motorNum = M_LEFT; motorNum < M_TOTAL_NUM; motorNum++)
    {
        if(M_PID_IDLE_FLAG != gStMotorRunState[motorNum].pidCurrentStartFlag)
        {
            if(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)  //��Ϊ�ǲ��Թ��߷����������ʱ��Ҫ�����ٶ�0������
            {
                if(((MOTOR_RUN_MODE_FLAP_FIX_CURRENT == gTargetRunMode[motorNum].run_mode)
                    || (MOTOR_RUN_MODE_SPEED == gTargetRunMode[motorNum].run_mode))
                    && (0 != gTargetRunMode[motorNum].target_value))
                {
                    lMotorRunMode->run_mode = gCurtRunMode[motorNum].run_mode;
                    lMotorRunMode->target_value = 0;
        
                    LockThread();
                    SetMotorRunModeData(motorNum, lMotorRunMode);
                    UnLockThread();
                }
            }
        }
        else if(((MOTOR_RUN_MODE_FLAP_FIX_CURRENT == gTargetRunMode[motorNum].run_mode)
            || (MOTOR_RUN_MODE_SPEED == gTargetRunMode[motorNum].run_mode))
            && (0 == gTargetRunMode[motorNum].target_value)
            && (!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)))
        {
            sys_para->CAR_RTinf.Link |= LINK_REV_STOP_CMD;  //���յ�ֹͣ����
        }
    }
}
/*****************************************************************************
 ��������  : ���ϻָ���ͬ����ʱ�ĵ������
 �������  : int errorRecoveryStep  ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��4��9��
*****************************************************************************/
static void ErrorRecoveryStepMotorCmd(int errorRecoveryStep)
{
    int i;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode;
    
    if((0 == errorRecoveryStep) || (1 == errorRecoveryStep))
    {
        //����ϵ�
        for(i = M_LEFT; i < M_TOTAL_NUM; i++)
        {
            if(0 == errorRecoveryStep)  //����ϵ�ָ��Ĺ���
            {
                ChangeMotorCurRunMode(i, gTargetRunMode[i].run_mode, MOTOR_RUN_MODE_STEP_START);
            }
            else    //��ϵ�ָ��Ĺ���
            {
                ChangeMotorCurRunMode(i, MOTOR_RUN_MODE_POWER_OFF, MOTOR_RUN_MODE_STEP_START);
            }
        }
    }
    else if(2 == errorRecoveryStep)
    {
        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
        //ת�����
        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
        lMotorRunMode.target_value = 0;
        SetMotorRunModeDataForce(M_TURN, &lMotorRunMode);
        //�����ٶ�Ϊ0
        if(gStMotorData[M_LEFT].flag & MOTOR_CURRENT_ADJUST_SPEED) //��������ת��
        {
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_FLAP_FIX_CURRENT;
        }
        else
	    {
	        lMotorRunMode.run_mode = MOTOR_RUN_MODE_SPEED;
	    }
        lMotorRunMode.target_value = 0;
        SetMotorRunModeDataForce(M_LEFT, &lMotorRunMode);
        if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)  //���Ҳ����������Ʊ�־
        {
            SetMotorRunModeDataForce(M_RIGHT, &lMotorRunMode);
        }
        if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
        {
            SetMotorRunModeDataForce(M_LEFT_ONE, &lMotorRunMode);
            SetMotorRunModeDataForce(M_RIGHT_ONE, &lMotorRunMode);
        }
        //ɲ������ʼλ��
        lMotorRunMode.target_value = 0;
        lMotorRunMode.homming_type = HOMMING_TYPE_NULL;
        if(gStMotorData[M_BRAKE].flag & MOTOR_CURRENT_ADJUST_SPEED) //��������ת��
        {
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_CURRENT_SPEED;
            lMotorRunMode.posType = POS_TYPE_NULL;
        }
        else
        {
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
            lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
        }

        SetMotorRunModeDataForce(M_BRAKE, &lMotorRunMode);
    }
}
/*****************************************************************************
 ��������  : ���ϴ�����ָ����ָ����Ժ�ȴ���λ�����������������
 �������  : uint32_t processTimeMs  ��ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��4��8��
*****************************************************************************/
static void ErrorDealAndRecovery(uint32_t processTimeMs)
{
    static int errorRecoveryStep = 0;
    int errorFlag = 0;
    uint32_t lCurTick;
    int i;

    //������Ѱ
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if(DRIVER_TYPE_NONE == gStMotorData[i].driverType)
        {
            continue;
        }
        //����ɻָ�����
        if(ERROR_L_NORMAL == gStMotorRunState[i].newErrorLevel)
        {
            gStMotorRunState[i].newErrorLevel = ERROR_NONE;
            errorRecoveryStep = 0;  //���¹��ϣ��ӵ�һ����ʼ
            if(0 == errorFlag)
            {
                errorFlag = 1;
            }
        }
        //�ϵ�ɻָ�����
        else if(ERROR_L_HIHG == gStMotorRunState[i].newErrorLevel)
        {
            gStMotorRunState[i].newErrorLevel = ERROR_NONE;
            errorRecoveryStep = 0;  //���¹��ϣ��ӵ�һ����ʼ
            if(3 != errorFlag)
            {
                errorFlag = 2;
            }
        }
        //���ɻָ�����
        else if(ERROR_L_VERY_HIGH == gStMotorRunState[i].newErrorLevel)
        {
            errorFlag = 3;
            errorRecoveryStep = 0;  //���¹��ϣ��ӵ�һ����ʼ
            break;
        }
    }

    //���ϴ�����ָ�
    if((1 == errorFlag) || (2 == errorFlag))
    {
        if(0 == errorRecoveryStep)  //��һ���������
        {
            if(1 == errorFlag) 
            {
                ErrorRecoveryStepMotorCmd(0);
            }
            else
            {
                ErrorRecoveryStepMotorCmd(1);
            }
            lCurTick = xTaskGetTickCount();
            if(0 == gErrorTimes)
            {
                gErrorStartTick = lCurTick;
            }
            gErrorTimes++;
            lCurTick -= gErrorStartTick;
            if((gErrorTimes > MOTOR_ERROR_RECOVERY_TIMES)
                && (lCurTick < (MOTOR_ERROR_RECOVERY_TIMEOUT << 1)))
            {
                rt_kprintf("Error Recovery timeout!\r\n");
                gErrorRunStateFlag = 1;//���ϻָ�ʧ����Ҳ��Ϊ���й����й��ϣ���Ҫ�ϱ�
                gErrorTimes = 0;
                //��Ϊ���ع���
                for(i = M_LEFT; i < M_TOTAL_NUM; i++)
                {
                    if(DRIVER_TYPE_NONE != gStMotorData[i].driverType)
                    {
                        if(1 == errorFlag) 
                        {
                            gStMotorRunState[i].errorLevel = ERROR_L_HIHG;
                            gStMotorRunState[i].newErrorLevel = ERROR_L_HIHG;
                        }
                        else
                        {
                            gStMotorRunState[i].errorLevel = ERROR_L_VERY_HIGH;
                            gStMotorRunState[i].newErrorLevel = ERROR_L_VERY_HIGH;
                        }
                    }
                }
            }
            else
            {
                errorRecoveryStep = 1;  //ת����һ���������Ϊ��ʼ״̬
            }
            if(lCurTick >= MOTOR_ERROR_RECOVERY_TIMEOUT)
            {
                gErrorTimes = 0;//ת����һ����
            }
        }
    }
    if(1 == errorRecoveryStep)  //�ڶ����������Ϊ��ʼ״̬
    {
        ErrorRecoveryStepMotorCmd(2);
        errorRecoveryStep = 2;
    }
    else if(2 == errorRecoveryStep)  //���������жϵ���Ƿ�ȫ�������ϵ�
    {
        for(i = M_LEFT; i < M_TOTAL_NUM; i++)
        {
            if(DRIVER_TYPE_NONE != gStMotorData[i].driverType)
            {
                if(MOTOR_POWER_FLAG_ON != gStMotorRunState[i].powerFlag)
                {
                    break;
                }
            }
        }
        //ȫ���ϵ�
        if(M_TOTAL_NUM == i)
        {
            errorRecoveryStep = 3;
            gErrorRecoveryTime = 0;
        }
    }
    else if(3 == errorRecoveryStep)  //���Ĳ����жϹ����Ƿ����
    {
        //�ϵ�һ��ʱ�����޹������������
        gErrorRecoveryTime += processTimeMs;
        if(gErrorRecoveryTime > (MOTORSTATUS_TIMEOUT_LONGTIME << 1))
        {
            errorRecoveryStep = 0;
            //�������
            for(i = M_LEFT; i < M_TOTAL_NUM; i++)
            {
                if(ERROR_NONE != gStMotorRunState[i].errorLevel)
                {
                    ClearErrorCode(i);
                }
            }
            //����Ǿ�ֹ״̬�µĹ�����������·�Ŀ���ٶ�Ϊ0���Զ���������룬�ɼ�������
            if(0 == gErrorRunStateFlag)
            {
                //pc�����������������ֵ�����л�����̬���ϣ���Ҫ�ϱ�
                if((sys_para->CAR_RTinf.Link & LINK_PC) && (0 != sys_para->CAR_RTinf.YM) && !IS_EMERGENCY_STOP)
                {
                    gErrorRunStateFlag = 1;
                }
                else
                {
                    ClearErrorCode(M_TOTAL_NUM);
                }
            }
        }
    } 
}
/*****************************************************************************
 ��������  : ���õ����Դ����
 �������  : uint32_t motorNum
             PowerState powerState  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��27��
*****************************************************************************/
void SetMotorPower(uint32_t motorNum, PowerState powerState)
{
    GPIO_PinState lPinState;
    
    if(motorNum < M_TOTAL_NUM)
    {
        if(gStMotorData[motorNum].powerPin < GPIO_PIN_MAX_NUM)
        {
            if(POWER_ON == powerState)
            {
                if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                {
                    rt_kprintf("Power-on, id-%d.\r\n", gStMotorData[motorNum].idx);
                }
                lPinState = GPIO_PIN_SET;
                if(gStMotorData[motorNum].flag & MOTOR_POWER_ON_INVERSE) //��Դ���ŷ���
                {
                    lPinState = GPIO_PIN_RESET;
                }
                HAL_GPIO_WritePin(GPIO_PORT(gStMotorData[motorNum].powerPort), GPIO_PIN(gStMotorData[motorNum].powerPin), lPinState);
            }
            else
            {
                if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                {
                    rt_kprintf("Power-off, id-%d.\r\n", gStMotorData[motorNum].idx);
                }
                lPinState = GPIO_PIN_RESET;
                if(gStMotorData[motorNum].flag & MOTOR_POWER_ON_INVERSE) //��Դ���ŷ���
                {
                    lPinState = GPIO_PIN_SET;
                }
                HAL_GPIO_WritePin(GPIO_PORT(gStMotorData[motorNum].powerPort), GPIO_PIN(gStMotorData[motorNum].powerPin), lPinState);
                gStMotorRunState[motorNum].runParasSetFlag = RUN_PARAS_NOT_SET;     //�ϵ���Ҫ�����趨���в���
            }
        }

        //����բ
        /*if(POWER_ON == powerState)  
        {
            if((M_LEFT == motorNum) || (M_RIGHT == motorNum) || (M_LEFT_ONE == motorNum) || (M_RIGHT_ONE == motorNum))
            {
                OPEN_LOCK;
            }
        }*/

        if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //Ԥ���ʹ��
        {
            if(((gStMotorData[motorNum].powerPin == gStMotorData[M_LEFT].powerPin)
                && (gStMotorData[motorNum].powerPort == gStMotorData[M_LEFT].powerPort))
                || ((gStMotorData[motorNum].powerPin == gStMotorData[M_RIGHT].powerPin)
                && (gStMotorData[motorNum].powerPort == gStMotorData[M_RIGHT].powerPort))
                || ((gStMotorData[motorNum].powerPin == gStMotorData[M_LEFT_ONE].powerPin)
                && (gStMotorData[motorNum].powerPort == gStMotorData[M_LEFT_ONE].powerPort))
                || ((gStMotorData[motorNum].powerPin == gStMotorData[M_RIGHT_ONE].powerPin)
                && (gStMotorData[motorNum].powerPort == gStMotorData[M_RIGHT_ONE].powerPort)))
            {
                if(POWER_ON == powerState)
                {
                    if(MOTOR_POWER_FLAG_OFF == gPreChargeState)
                    {
                        gPreChargeState = MOTOR_POWER_FLAG_TIMECNT;
                        gPreChargeTimeCnt = 0;
                    }
                }
                else if(MOTOR_POWER_FLAG_OFF != gPreChargeState)
                {
                    gPreChargeState = MOTOR_POWER_FLAG_OFF;
                    SetMotorPower(M_TOTAL_NUM, POWER_OFF);
                }
            }
            if(gStMotorData[motorNum].flag & MOTOR_USE_PI7_POWER)    //ʹ��PI7��Դ���ƶ˿ڣ�ԭ��Դ���ŵ���Ԥ������
            {
                if(POWER_ON == powerState)
                {
                    if(MOTOR_POWER_FLAG_OFF == gPreChargeStateOne)
                    {
                        gPreChargeStateOne = MOTOR_POWER_FLAG_TIMECNT;
                        gPreChargeTimeCntOne = 0;
                    }
                }
                else if(MOTOR_POWER_FLAG_OFF != gPreChargeStateOne)
                {
                    gPreChargeStateOne = MOTOR_POWER_FLAG_OFF;
                    SetMotorPower(M_NUM_PI7, POWER_OFF);
                }
            }
        }
    }
    else if(M_TOTAL_NUM == motorNum)
    {
        if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //Ԥ���ʹ��
        {
            if(gStUfoData.preChargePin < GPIO_PIN_MAX_NUM)
            {
                if(POWER_ON == powerState)
                {
                    if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                    {
                        rt_kprintf("PreCharge-on.\r\n");
                    }
                    lPinState = GPIO_PIN_SET;
                    if(gStUfoData.flag & UFO_PRECHARGE_INVERSE)  //Ԥ�䷴��
                    {
                        lPinState = GPIO_PIN_RESET;
                    }
                    HAL_GPIO_WritePin(GPIO_PORT(gStUfoData.preChargePort), GPIO_PIN(gStUfoData.preChargePin), lPinState);
                }
                else
                {
                    if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                    {
                        rt_kprintf("PreCharge-off.\r\n");
                    }
                    lPinState = GPIO_PIN_RESET;
                    if(gStUfoData.flag & UFO_PRECHARGE_INVERSE)  //Ԥ�䷴��
                    {
                        lPinState = GPIO_PIN_SET;
                    }
                    HAL_GPIO_WritePin(GPIO_PORT(gStUfoData.preChargePort), GPIO_PIN(gStUfoData.preChargePin), lPinState);
                }
            }
        }
    }
    else if(M_NUM_PI7 == motorNum)  //����ת��ɲ��Ԥ�����ţ��̶��˿�PI5
    {
        if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //Ԥ���ʹ��
        {
            if(POWER_ON == powerState)
            {
                if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                {
                    rt_kprintf("PreChargeOne-on.\r\n");
                }
                lPinState = GPIO_PIN_RESET;
                HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, lPinState);
            }
            else
            {
                if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                {
                    rt_kprintf("PreChargeOne-off.\r\n");
                }
                lPinState = GPIO_PIN_SET;
                HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, lPinState);
            }
        }
    }
}
/*****************************************************************************
 ��������  : ���õ����բ����
 �������  : uint32_t motorNum
             LockState lockState  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��27��
*****************************************************************************/
void SetMotorLock(uint32_t motorNum, LockState lockState, rt_bool_t notRelateBrake)
{
    GPIO_PinState lPinState;
    
    if(motorNum < M_TOTAL_NUM)
    {
        /*if(0 != gStMotorData[motorNum].lockPin)
        {
            if(LOCK_ON == brakeState)
            {
                /#if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("Lock-on, id-%d.\r\n", gStMotorData[motorNum].idx);
                }#/
                /#SetPower24V(POWER_ON);   //�򿪱�բǰ�ȴ�24V�ܵ�Դ
                GPIO_SetBits(gStMotorData[motorNum].lockPort, gStMotorData[motorNum].lockPin);#/
                gStMotorRunState[motorNum].lockFlag = LOCK_ON;
            }
            else
            {
                /#if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("Lock-off, id-%d.\r\n", gStMotorData[motorNum].idx);
                }#/
                /#GPIO_ResetBits(gStMotorData[motorNum].lockPort, gStMotorData[motorNum].lockPin);#/
                gStMotorRunState[motorNum].lockFlag = LOCK_OFF;
            }
        }*/
    }
    else if(M_TOTAL_NUM == motorNum)
    {
        if(gStUfoData.flag & UFO_ENABLE_LOCK)  //��բʹ��
        {
            if(gStUfoData.lockPin < GPIO_PIN_MAX_NUM)
            {
                if(LOCK_ON == lockState)
                {
                    if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                    {
                        rt_kprintf("Lock-on.\r\n");
                    }
                    lPinState = GPIO_PIN_SET;
                    if(gStUfoData.flag & UFO_LOCK_INVERSE)  //Ԥ�䷴��
                    {
                        lPinState = GPIO_PIN_RESET;
                    }
                    HAL_GPIO_WritePin(GPIO_PORT(gStUfoData.lockPort), GPIO_PIN(gStUfoData.lockPin), lPinState);
                    
                    //�����բʹ��ɲ����Դ�����ɲ����Դ
                    if(gStUfoData.flag & UFO_LOCK_TWO_CTRL)  //��բ�����˿ڿ���
                    {
                        if((DRIVER_TYPE_NONE == gStMotorData[M_BRAKE].driverType) || notRelateBrake)
                        {
                            SetMotorPower(M_BRAKE, POWER_ON);
                        }
                        else
                        {
                            SET_BRAKE(0);
                        }
                    }
                }
                else
                {
                    if(DEBUG_DATA_TYPE_1 || DEBUG_DATA_TYPE_8E)
                    {
                        rt_kprintf("Lock-off.\r\n");
                    }
                    lPinState = GPIO_PIN_RESET;
                    if(gStUfoData.flag & UFO_LOCK_INVERSE)  //��բ����
                    {
                        lPinState = GPIO_PIN_SET;
                    }
                    HAL_GPIO_WritePin(GPIO_PORT(gStUfoData.lockPort), GPIO_PIN(gStUfoData.lockPin), lPinState);
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : ���Ƕȣ���λΪ�ȣ�ת��Ϊ��������Ӧ��λ��ֵ
 �������  : uint32_t motorNum
             int32_t angle  �Ƕ�
             PosType posType   λ������
 �������  : int32_t ������λ��
 ��    ��  : ����
 ��    ��  : 2018��12��1��
*****************************************************************************/
int32_t ConvertAngleToCounts(uint32_t motorNum, int32_t angle, PosType posType)
{
    int32_t  pulsecnt = 0, angle_to_pulse;
     
    angle_to_pulse = gStMotorData[motorNum].ratio * gStMotorData[motorNum].counts;

    if(POS_TYPE_SINGLE_ABSOLUTE == posType)
    {
        pulsecnt = fmod(angle, 360) * angle_to_pulse / 360;
    }
    else if(POS_TYPE_MULTI_ABSOLUTE == posType)
    {
        pulsecnt = angle;
    }

    return pulsecnt;
}
/*****************************************************************************
 ��������  : ��ȡб�¼Ӽ���(ע����ô˺���֮ǰsetTargetVel��Ԥ�趨)
 �������  : uint32_t motorNum  ������
 �������  : int32_t    �����µ��ٶ�ֵ
 ��    ��  : ����
 ��    ��  : 2019��11��17��
*****************************************************************************/
int32_t GetSlopeSpeed(uint32_t motorNum, int32_t setTarVel, int32_t curVel, int32_t carMotorVelOutRangeFlag, int32_t* liTempData, uint32_t processTimeMs)
{
    int32_t lTargetSpeed = gStMotorRunState[motorNum].targetVel;
    int32_t lSpeedGrad = 0, curAccLimit = 400, navAccLimit = 400, remoteAccLimit = 200;  
    uint32_t lCurTime;

    //ң�ؼ��ٶ�����
    if((gStMotorData[M_LEFT].profileAcc >= 100) && (gStMotorData[M_LEFT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        remoteAccLimit = gStMotorData[M_LEFT].profileAcc;
    }
    
    //�ߵ��Զ�ģʽ���ٶ�����
    if((gStMotorData[M_RIGHT].profileAcc >= 50) && (gStMotorData[M_RIGHT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        navAccLimit = gStMotorData[M_RIGHT].profileAcc;
    }
    lCurTime = HAL_GetTick();
    
    //�޶�����ֹͣ���ٶ�Ϊ10
    if(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP)
    {
        //lTempVel = OVER_VEL_MAX << 3;
        lSpeedGrad = 1000;
        curAccLimit = lSpeedGrad + 100;
        //�����趨���ٶ��л�ʱ�ӣ���ֹ�����е��¶���
        //��Խ���ٽ�2ʱ���趨���ٶ�С�ڵ���0ʱ�����и��趨���ٶȣ��趨���ٶȴ���0ʱ�������л���0�����л������趨���ٶ�
        if((curVel > setTarVel + 100) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = -lSpeedGrad;
        }
        //��Խ���ٽ�1ʱ���趨���ٶȴ��ڵ���0ʱ�����趨���ٶ�Ϊ0
        else if((curVel > setTarVel) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        } 
        //��Խ���ٽ�2ʱ���趨���ٶȴ��ڵ���0ʱ���������趨���ٶȣ��趨���ٶ�С��0ʱ�������л���0�����л������趨���ٶ�
        else if((curVel < setTarVel - 100) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = lSpeedGrad;
        }
        //��Խ���ٽ�1ʱ���趨���ٶ�С�ڵ���0ʱ�����趨���ٶ�Ϊ0
        else if((curVel < setTarVel) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        }
    }
    //pc����ʱ�Ӽ��ٶȺ͹滮��һ��
    else if(sys_para->CAR_RTinf.Link & LINK_PC)   //PC����
    {
        if(gStUfoData.flag & UFO_NOT_USE_GET_SET_ACC) //��ʹ�û�ȡ�����趨���ٶ�
        {
            //�˲���ù滮Ŀ����ٶ�
            /*for(k = 0; k < SET_ACC_RECORD_NUM; k++)
            {
                liTempData[k] = gStMotorRunState[motorNum].setCarAcc[k];
            }
            FilterMidAverage(liTempData, SET_ACC_RECORD_NUM, 1, gStMotorRunState[motorNum].setAcc, k, lTempVel);*/
            //�趨���ٶȿ������˲�
            if(gStMotorRunState[motorNum].setCarVelValidNum > 0)
            {
                KalmanFilter(gStMotorRunState[motorNum].setAccKalmanData, Limit(gStMotorRunState[motorNum].setCarAcc, -navAccLimit, navAccLimit));
                gStMotorRunState[motorNum].setAcc = (int32_t)(gStMotorRunState[motorNum].setAccKalmanData.out);
            }
        }
        else
        {
            gStMotorRunState[motorNum].setAcc = sys_para->CAR_RTinf.SetAcc;
            if(lCurTime - gStMotorRunState[motorNum].setCarVelTime[SET_VEL_RECORD_NUM - 1] >= 200)
            {
                gStMotorRunState[motorNum].setAcc = 0;
                rt_kprintf("M%d SetAcc0:%d,%d.\r\n", motorNum, lCurTime, gStMotorRunState[motorNum].setCarVelTime[SET_VEL_RECORD_NUM - 1]);
            }
        }
        gStMotorRunState[motorNum].setAcc = Limit(gStMotorRunState[motorNum].setAcc, -navAccLimit, navAccLimit);
    
        lTargetSpeed = setTarVel;
        gStMotorRunState[motorNum].targetVel = lTargetSpeed;
        lSpeedGrad = 0;
        curAccLimit = navAccLimit + 100;
    }
    //�޶�ң�ؼ��ٶȼ��ٶ�Ϊ2��ֹͣʱ���ٶ�Ϊ6
    else
    {
        //lTempVel = OVER_VEL_MAX << 2;
        if((0 != setTarVel) && IS_SAME_SYMBLE(setTarVel, curVel))   //Ŀ���ٶȲ�Ϊ0������Ŀ���ٶȺ͵�ǰ�ٶȷ���һ�£����趨���ٶȹ滮������ֹͣ���ٶȹ滮
        {
            if((remoteAccLimit < gRemoteStopDcc) && (gRemoteStopDcc == ABS_VALUE(gStMotorRunState[motorNum].setAcc)))   //�ϴ����˽ϴ�ļ��ٶ�
            {
                lTargetSpeed = curVel;  //Ŀ��滮�ٶȸ�������ǰ�ٶ�
                gStMotorRunState[motorNum].targetVel = lTargetSpeed;
            }
            lSpeedGrad = remoteAccLimit;
            curAccLimit = lSpeedGrad + 300;
        }
        else
        {
            lSpeedGrad = gRemoteStopDcc;
            curAccLimit = lSpeedGrad + 300;
        }
        gStMotorRunState[motorNum].lastSetTargetVel = setTarVel;
        //�����趨���ٶ��л�ʱ�ӣ���ֹ�����е��¶���
        //��Խ���ٽ�2ʱ���趨���ٶ�С�ڵ���0ʱ�����и��趨���ٶȣ��趨���ٶȴ���0ʱ�������л���0�����л������趨���ٶ�
        if((curVel > setTarVel + 50) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = -lSpeedGrad;
        }
        //��Խ���ٽ�1ʱ���趨���ٶȴ��ڵ���0ʱ�����趨���ٶ�Ϊ0
        else if((curVel > setTarVel) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        } 
        //��Խ���ٽ�2ʱ���趨���ٶȴ��ڵ���0ʱ���������趨���ٶȣ��趨���ٶ�С��0ʱ�������л���0�����л������趨���ٶ�
        else if((curVel < setTarVel - 50) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = lSpeedGrad;
        }
        //��Խ���ٽ�1ʱ���趨���ٶ�С�ڵ���0ʱ�����趨���ٶ�Ϊ0
        else if((curVel < setTarVel) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        }
    }
    //���ݼ��ٶȼ����ٶ��ݶ�ֵ
    lSpeedGrad = lSpeedGrad * (int32_t)processTimeMs / 1000;
    if(0 == lSpeedGrad)
    {
        lSpeedGrad = 1; //�޶���С�ٶ��ݶ�ֵ
    }

    if(lTargetSpeed != setTarVel)
    {
        //�ж��Ƿ񵽴��趨Ŀ���ٶ�
        if(ABS_VALUE(lTargetSpeed - setTarVel) <= lSpeedGrad)
        {
            lTargetSpeed = setTarVel;
        }
        //����Ŀ���ٶȼӼ���
        else if(lTargetSpeed > setTarVel)
        {
            lTargetSpeed -= lSpeedGrad;
        }
        else
        {
            lTargetSpeed += lSpeedGrad;
        }
        gStMotorRunState[motorNum].targetVel = lTargetSpeed;
        if(DEBUG_DATA_TYPE_2)
        {
            rt_kprintf("SetV:%d,TarV:%d,grad:%d.\r\n", setTarVel, lTargetSpeed, lSpeedGrad);
        }
    }

    //�޶���С�ٶ�ΪѰ���ٶ�
    /*if(0 != setTarVel)
    {
        lTempVel = ABS_VALUE(gStMotorData[motorNum].homingSpeed);
        if(ABS_VALUE(lTargetSpeed) < lTempVel)
        {
            lTargetSpeed = (setTarVel > 0) ? lTempVel : (-lTempVel);
        }
    }*/

    //�˲���ùߵ����ٶ�
    /*for(k = 0; k < NAV_ACC_RECORD_NUM; k++)
    {
        liTempData[k] = gCarNavAcc[k];
    }
    FilterMidAverage(liTempData, NAV_ACC_RECORD_NUM, 1, gCarNavCurAcc, k, lTempVel);*/

    //�˲���õ�ǰ�����õļ��ٶ�
    /*for(k = 0; k < MACC_RECORD_NUM; k++)
    {
        liTempData[k] = gStMotorRunState[motorNum].mAcc[k];
    }
    FilterMidAverage(liTempData, MACC_RECORD_NUM, MACC_EXCLUDE_NUM, gStMotorRunState[motorNum].motorCurAcc, k, lTempVel);*/

    //ʵ�ʼ��ٶȼ��㣬����ֹͣʱ���ùߵ����ٶ�
    if((!IS_EMERGENCY_STOP) && (sys_para->CAR_RTinf.Link & LINK_PC) && (!carMotorVelOutRangeFlag))   //PC���ƣ��ҹߵ��������ٶȲ50cm/s
    {
        gStMotorRunState[motorNum].curAcc = gCarNavCurAcc;  //�ߵ����ٶ�
    }
    else
    {
        gStMotorRunState[motorNum].curAcc = gMotorCurAcc;   //������ٶ�
    }
    gStMotorRunState[motorNum].curAcc = Limit(gStMotorRunState[motorNum].curAcc, -curAccLimit, curAccLimit);

    return lTargetSpeed;
}
/*****************************************************************************
 ��������  : ����ϵ紦��
 �������  : uint32_t motorNum   ������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��15��
*****************************************************************************/
static void ProcessMotorRunModePowerOff(uint32_t motorNum)
{
    uint8_t k;
    
    if(MOTOR_POWER_FLAG_OFF != gStMotorRunState[motorNum].powerFlag)    //�ص�Դ����������
    {
        if((M_LEFT == motorNum) || (M_RIGHT == motorNum) || (M_LEFT_ONE == motorNum) || (M_RIGHT_ONE == motorNum))
        {
            CLOSE_LOCK; //�رձ�բ
            CLOSE_BRAKE;//ɲ���ϵ�
        }
        #ifdef ENABLE_CANOPEN_DRIVER
        if(gStMotorData[motorNum].flag & MOTOR_POWER_OFF_RECORDPOS) //�ϵ��¼λ��
        {
            if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
            {
                if(HAL_OK == Motor485ReadPos(motorNum, &(gStMotorRunState[motorNum].lastPoweroffPos)))
                {
                    gStMotorRunState[motorNum].resetFlag = 1;
                }
                else if(HAL_OK == Motor485ReadPos(motorNum, &(gStMotorRunState[motorNum].lastPoweroffPos)))
                {
                    gStMotorRunState[motorNum].resetFlag = 1;
                }
                else
                {
                    gStMotorRunState[motorNum].lastPoweroffPos = 0;
                    //gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;    //��Ҫ����Ѱ��
                }
                if(DEBUG_DATA_TYPE_5 && (1 == gStMotorRunState[motorNum].resetFlag))
                {
                    rt_kprintf("id-%d-poweroff-pos:%d.\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].lastPoweroffPos); 
                }
            }
        }
        else
        #endif
        {
            if((DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)            //ÿ�ε���ϵ綼������Ѱ��
                || (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType))  //ÿ�ε���ϵ綼������Ѱ��
            {
                gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;    //��Ҫ����Ѱ��
            }
        }
        SetMotorPower(motorNum, POWER_OFF);
        gStMotorRunState[motorNum].startRemoteFlag = 0;
        gStMotorRunState[motorNum].operationStopJudgeFlag = 0;  //�ϵ����Ҫ�жϵ��ʵ��ת���Ƿ���ֹͣ
        gStMotorRunState[motorNum].powerFlag = MOTOR_POWEROFF_FLAG_TIMECNT;
        gStMotorRunState[motorNum].powerTime = 0;
        gStMotorRunState[motorNum].targetPosJudgeFlag = RT_FALSE;
        ChangeMotorTargetValue(motorNum, 0, MOTOR_SET_NOTHING);
        #ifdef ENABLE_CANOPEN_DRIVER
        ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_UNKOWN); 
        #endif	

        if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG) //תʹ��
        {
            if(ENABLE == gStMotorRunState[motorNum].enableFlag)
            {
                CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, 0);
                GPIOEnableMotor(motorNum, DISABLE);
            }
        }

        //����������һ����Դ����Ҳ�ϵ�
        if(gStMotorData[motorNum].powerPin < GPIO_PIN_MAX_NUM)
        {
            for(k = 0; k < M_TOTAL_NUM; k++)
            {
                if((k != motorNum) && (((gStMotorData[motorNum].powerPort == gStMotorData[k].powerPort)
                    && (gStMotorData[motorNum].powerPin == gStMotorData[k].powerPin))
                    || ((gStMotorData[motorNum].powerPort == gStUfoData.lockPort)
                    && (gStMotorData[motorNum].powerPin == gStUfoData.lockPin)
                    && (gStMotorData[k].flag & MOTOR_DUAL_POWER))
                    || ((gStMotorData[k].powerPort == gStUfoData.lockPort)
                    && (gStMotorData[k].powerPin == gStUfoData.lockPin)
                    && (gStMotorData[motorNum].flag & MOTOR_DUAL_POWER))
                    || ((gStMotorData[motorNum].powerPort == gStUfoData.preChargePort)
                    && (gStMotorData[motorNum].powerPin == gStUfoData.preChargePin)
                    && (gStUfoData.flag & UFO_PRE_CHARGE_FLAG) && ((k == M_LEFT) || (k == M_LEFT_ONE) || (k == M_RIGHT) || (k == M_RIGHT_ONE))))
                  )
                {
                    if((MOTOR_POWEROFF_FLAG_TIMECNT != gStMotorRunState[k].powerFlag)
                        && (MOTOR_POWER_FLAG_OFF != gStMotorRunState[k].powerFlag))
                    {
                        ChangeMotorCurRunMode(k, MOTOR_RUN_MODE_POWER_OFF, MOTOR_RUN_MODE_STEP_START);    //�л����ϵ�ģʽ
                        ProcessMotorRunModePowerOff(k);
                    }
                }
            }
        }
    } 

    gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_POWER_OFF;
    gStMotorRunState[motorNum].run_step = MOTOR_RUN_MODE_STEP_IDLE;

    ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
}
/*****************************************************************************
 ��������  : ���Ѱ�㴦��
 �������  : rt_uint32_t motorNum      ������
             rt_uint32_t processTimeMs ������ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��15��
*****************************************************************************/
void ProcessMotorRunModeHomming(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    uint16_t zeroState;
    int32_t liTemp;
    float lfTemp;

    // 1.��ʱ����
    if((gStMotorRunState[motorNum].runTotalTime >= MOTOR_PECT_HOMMING_TURN_TIMEOUT)
        || (gStMotorRunState[motorNum].errorCnt > MOTOR_CAN_TIMEOUT_TIMES))
    {
        gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;     //�л�������ģʽ
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        SetErrorCode(motorNum, ERROR_CODE_HOMMING_TIMEOUT, ERROR_L_HIHG);
        return;
    }

    // 2.������ı䴦�����账��
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
    }

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if((MOTOR_RUN_MODE_STEP_START == *pRunStep) || (HOME_FLAG_SET_MOVE == *pRunStep) || (HOME_FLAG_CLEAR_STOP == *pRunStep))
    {
        if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 20)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                if(HAL_OK == Motor485SetMotorContrlmode(motorNum, MOTOR_MODE_IO))
                {
                    if(GPIO_PIN_RESET == GetMotorLimit1State(motorNum)) //���ڵ�
                    {
                        *pRunStep = HOME_FLAG_MOVE_CCW;
                    }
                    else
                    {
                        *pRunStep = HOME_FLAG_MOVE_CW;
                    }
                    gStMotorRunState[motorNum].runDelayTime = 0;
                }
            }
        }
        else if(DRIVER_TYPE_PUSI == gStMotorData[motorNum].driverType)
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 50)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                MotorSendReadControlStatus(gStMotorData[motorNum].idx);
                gStMotorRunState[motorNum].errorCnt++;
            }
            if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_CTRL_STATUS_FLAG))
            {
                zeroState = gStMotorRevData[motorNum].ctrlStatus & (DIVICE_CTROL_STATUS_EXT_STOP1 | DIVICE_CTROL_STATUS_EXT_STOP2 | DIVICE_CTROL_STATUS_STALL);
                if(zeroState)
                {
                    MotorSetMotorControlStatus(gStMotorData[motorNum].idx, zeroState);  //���״̬λ
                }
                else
                {
                    gStMotorRunState[motorNum].errorCnt = 0;
                    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)
                    {
                        ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PVM);
                        CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, -(gStMotorData[motorNum].profileVelocity >> 1));
                        *pRunStep = HOME_FLAG_WAIT_STOP;
                    }
                    else if(HOME_FLAG_SET_MOVE == *pRunStep)
                    {
                        ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PVM);
                        CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, gStMotorRunState[motorNum].reverseFlag ? (-gStMotorRunState[motorNum].targetVel) : gStMotorRunState[motorNum].targetVel);
                        *pRunStep = HOME_FLAG_WAIT_STOP;
                    }
                    else
                    {
                        if(ABS_VALUE(gStMotorRunState[motorNum].limitPos1 - gStMotorRunState[motorNum].limitPos2) > gStMotorData[motorNum].initPos)
                        {
                            if(gStMotorRunState[motorNum].limitPos1 > gStMotorRunState[motorNum].limitPos2)
                            {
                                liTemp = gStMotorRunState[motorNum].limitPos1;
                                gStMotorRunState[motorNum].limitPos1 = gStMotorRunState[motorNum].limitPos2;
                                gStMotorRunState[motorNum].limitPos2 = liTemp;
                            }
                            gStMotorRunState[motorNum].limitPos1 += gStMotorData[motorNum].pos_limit1;
                            gStMotorRunState[motorNum].limitPos2 -= gStMotorData[motorNum].pos_limit2;
                            gStMotorRunState[motorNum].PosInit = gStMotorRunState[motorNum].limitPos1
                                + ((gStMotorRunState[motorNum].limitPos2 - gStMotorRunState[motorNum].limitPos1) >> 1);
                            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                            gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //Ѱ�����
                            if(DEBUG_DATA_TYPE_2)
                            {
                                rt_kprintf("Pos limit1: %d, limit2: %d\r\n", gStMotorRunState[motorNum].limitPos1, gStMotorRunState[motorNum].limitPos2);
                            }
                        }
                        else
                        {
                            *pRunStep = MOTOR_RUN_MODE_STEP_START;   //���¿�ʼ
                        }
                    }
                }
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
            }
        }
        else
        {
            if(gStMotorData[motorNum].flag & MOTOR_ENABLE_STALL_HOMMING)    //��תѰ��
            {
                ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PTM);
                *pRunStep = HOME_FLAG_MOVE_CW;                //��������
            }
            else
            {
                ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PVM);
                CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, gStMotorData[motorNum].homingSpeed);
                *pRunStep = HOME_FLAG_WAIT_STOP;
            }
        }
    }
    else if(HOME_FLAG_WAIT_STOP == *pRunStep)
    {
        if(DRIVER_TYPE_PUSI == gStMotorData[motorNum].driverType)
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 100)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                MotorSendReadControlStatus(gStMotorData[motorNum].idx);
            }
            if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_CTRL_STATUS_FLAG))
            {
                if(gStMotorRevData[motorNum].ctrlStatus & (DIVICE_CTROL_STATUS_EXT_STOP1 | DIVICE_CTROL_STATUS_EXT_STOP2))    //�ⲿֹͣ
                {
                    *pRunStep = HOME_FLAG_RECORD_POS;  //��¼λ��
                }
                else if(gStMotorRevData[motorNum].ctrlStatus & DIVICE_CTROL_STATUS_STALL)    //��ת
                {
                    if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
                    {
                        *pRunStep = HOME_FLAG_SET_MOVE;  //����
                        gStMotorRunState[motorNum].reverseFlag = 1;
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;   //Ѱ��δ֪
                        gStMotorRunState[motorNum].stallFlag = 1;
                        gStMotorRunState[motorNum].curPos = liTemp;
                    }
                }
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
            }
        }
        else
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 100)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                if(0 == MotorReadIOStatus(gStMotorData[motorNum].idx, &zeroState))
                {
                    if(DEBUG_DATA_TYPE_5)
                    {
                        rt_kprintf("IO status: %d.\r\n", zeroState & 0x01);
                    }
                    if(zeroState & 0x01)
                    {
                        *pRunStep = HOME_FLAG_RECORD_POS;  //��¼λ��
                    }
                }
            }
        }
    }
    else if(HOME_FLAG_STOP == *pRunStep)
    {
        if(gStMotorRunState[motorNum].runDelayTime >= 1500)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            if(0 == MotorReadStatus(gStMotorData[motorNum].idx, &zeroState))
            {
                zeroState &= DEVICE_STATUS_MASK;
                if(DEVICE_STATUS_OPERATION_ENABLE != zeroState)
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                    gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_UNKOWN; //���Ϳ���ֹͣʱģʽ����ָ���δ֪����ֹ��9025����
                    gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //Ѱ�����
                }
                else//�Ͽ����������
                {
                    MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_QUICK_STOP);
                }
            }
        }
    }
    else if(HOME_FLAG_RECORD_INIT_POS == *pRunStep)                         //����ʼλ��
    {
        if(gStMotorRunState[motorNum].runDelayTime >= 50)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;                
            if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
            {           
                gStMotorRunState[motorNum].errorCnt = 0;
                gStMotorRunState[motorNum].PosInit = liTemp;

                if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("P-init:%d, ticks:%d\r\n", gStMotorRunState[motorNum].PosInit, HAL_GetTick());
                }
                
                if (gStMotorRunState[motorNum].PosInit > gStMotorRunState[motorNum].limitPos2)       //��ʼλ�ô��ڼ���λ�ã���Ҫ����
                {
                    gStMotorRunState[motorNum].PosInit = gStMotorRunState[motorNum].limitPos2;
                    gStMotorRunState[motorNum].limitPos2 = liTemp;
                }

                if(DEBUG_DATA_TYPE_2)
                {
                    //�����������ɲ��λ�����ɲ��λ��֮�����1500(��λ:cnt)����Ѱ��ɹ�������Ѱ��ʧ��
                    rt_kprintf("M%d homming %s, P-init:%d, P-limit2:%d, ticks:%d\r\n", motorNum, ((ABS_VALUE(gStMotorRunState[motorNum].limitPos2 - gStMotorRunState[motorNum].PosInit) > 1500) ? "succeed" : "fail"),
                                                gStMotorRunState[motorNum].PosInit, gStMotorRunState[motorNum].limitPos2, HAL_GetTick());
                }

                if (0 == MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_QUICK_STOP))
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                    gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_UNKOWN; //���Ϳ���ֹͣʱģʽ����ָ���δ֪����ֹ��9025����
                    lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                    if (lfTemp < 8)    //��ɲ��λ�õ���ѹ��0.8Mpa
                    {
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //Ѱ�����
                    }
                    else
                    {
                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_BRAKE_ERROR, ERROR_L_NORMAL); //��ɲ��λ������ɲ��������ɲ������
                    }
                }
            }
            else
            {
                gStMotorRunState[motorNum].errorCnt++;
            }
        }
    }
    else if(HOME_FLAG_RECORD_POS == *pRunStep)
    {
        if(gStMotorRunState[motorNum].runDelayTime >= 50)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
            {
                gStMotorRunState[motorNum].errorCnt = 0;
                if(DRIVER_TYPE_PUSI == gStMotorData[motorNum].driverType)
                {
                    if(gStMotorRunState[motorNum].stallFlag)    //��ת��Ĵ���
                    {
                        gStMotorRunState[motorNum].stallFlag = 0;
                        *pRunStep = HOME_FLAG_SET_MOVE;  //����
                        if(ABS_VALUE(liTemp - gStMotorRunState[motorNum].curPos) < gStMotorData[motorNum].initPos)
                        {                        
                            gStMotorRunState[motorNum].reverseFlag = 0;
                        }
                        else
                        {
                            gStMotorRunState[motorNum].limitPos1 = liTemp;
                            gStMotorRunState[motorNum].reverseFlag = 1;
                            gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_RECORD_POS;
                            if(DEBUG_DATA_TYPE_2)
                            {
                                rt_kprintf("Stall record pos1: %d\r\n", liTemp);
                            }
                        }
                    }
                    else if(HOME_FLAG_RECORD_POS != gStMotorRunState[motorNum].homeFinishFlag)
                    {
                        *pRunStep = HOME_FLAG_SET_MOVE;  //����
                        gStMotorRunState[motorNum].limitPos1 = liTemp;
                        gStMotorRunState[motorNum].reverseFlag = 1;
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_RECORD_POS;
                        if(DEBUG_DATA_TYPE_2)
                        {
                            rt_kprintf("Record pos1: %d\r\n", liTemp);
                        }
                    }
                    else
                    {
                        gStMotorRunState[motorNum].limitPos2 = liTemp;
                        *pRunStep = HOME_FLAG_CLEAR_STOP;  //���ֹͣ��־
                        if(DEBUG_DATA_TYPE_2)
                        {
                            rt_kprintf("Record pos2: %d\r\n", liTemp);
                        }
                    }
                    CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
                }
                else
                {
                    /*if(0 != gStMotorData[motorNum].counts)
                    {
                        liTempOne = gFlashData.turnInitPos % gStMotorData[motorNum].counts;
                        gStMotorRunState[motorNum].PosInit = liTemp - gFlashData.turnInitPos + liTempOne;
                        liTempOne = gStMotorRunState[motorNum].PosInit - 
                            gStMotorRunState[motorNum].PosInit % gStMotorData[motorNum].counts + liTempOne;
                        if(gStMotorRunState[motorNum].PosInit > liTempOne + (gStMotorData[motorNum].counts >> 1))
                        {
                            gStMotorRunState[motorNum].PosInit = liTempOne + gStMotorData[motorNum].counts;
                        }
                        else if(gStMotorRunState[motorNum].PosInit + (gStMotorData[motorNum].counts >> 1) < liTempOne)
                        {
                            gStMotorRunState[motorNum].PosInit = liTempOne - gStMotorData[motorNum].counts;
                        }
                        else
                        {
                            gStMotorRunState[motorNum].PosInit = liTempOne;
                        }
                    }*/
                    gStMotorRunState[motorNum].PosInit = liTemp - gStMotorData[motorNum].initPos;
                    gStMotorRunState[motorNum].limitPos1 = gStMotorRunState[motorNum].PosInit - gStMotorData[motorNum].pos_limit1;
                    gStMotorRunState[motorNum].limitPos2 = gStMotorRunState[motorNum].PosInit + gStMotorData[motorNum].pos_limit1;
                    *pRunStep = HOME_FLAG_STOP;     //ֹͣ���
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("P-record:%d, P-init:%d, P-limit1: %d, P-limit2: %d\r\n", liTemp, gStMotorRunState[motorNum].PosInit, gStMotorRunState[motorNum].limitPos1, gStMotorRunState[motorNum].limitPos2);
                    }
                    MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_QUICK_STOP);
                }
            }
            else
            {
                gStMotorRunState[motorNum].errorCnt++;
            }
        }
    }
    else if(HOME_FLAG_MOVE_CW == *pRunStep)
    {
        if (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
        {
            if((OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep))
            {
                if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, gStMotorData[motorNum].homingSpeed))//����Ŀ��ת��(0.1%)
                {
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("limitPos2 setCurent:%d, ticks:%d\r\n", gStMotorData[motorNum].homingSpeed, HAL_GetTick());
                    }
                    *pRunStep = HOME_FLAG_WAIT_CW_STOP;          //�ȴ���������ֹͣ
                    gStMotorRunState[motorNum].runDelayTime = 0;
                }
            }
        }
        else
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 500)
            {
                gStMotorRunState[motorNum].runDelayTime = 480;
                if(HAL_OK == Motor485SetMotorMove(motorNum, DIR_CW, 100))
                {
                    *pRunStep = HOME_FLAG_WAIT_CW_STOP;
                    gStMotorRunState[motorNum].limit1Time = 0;
                }
            }
        }
    }
    else if(HOME_FLAG_MOVE_CCW == *pRunStep)
    {
        if (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 50)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                liTemp = -gStMotorData[motorNum].startCurrent;
                if(0 == liTemp)
                {
                    liTemp = -(gStMotorData[motorNum].homingSpeed >> 1);
                }
                if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, liTemp))//����Ŀ��ת��(0.1%)����������
                {
                    lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("posInit setCurent:%d,oil:%.1fMpa,setMaxOil:%.1fMpa,ticks:%d\r\n", liTemp, lfTemp / 10, ADC1_BRAKE_MAX_THRETHOLDS /10.0f, HAL_GetTick());
                    }
                    /*if(lfTemp < 2)  //��ѹС��0.2Mpa�����ն�ת�߼��˻أ���������ѹ��С�߼��˻�
                    {
                        *pRunStep = HOME_FLAG_WAIT_CCW_STALL_STOP;    //�ȴ������תֹͣ
                    }
                    else
                    {
                        *pRunStep = HOME_FLAG_WAIT_CCW_STOP;          //�ȴ���������ֹͣ
                    }*/
                    if (ADC1_BRAKE_MAX_THRETHOLDS > 0) 
                    {
                        *pRunStep = HOME_FLAG_RECORD_SETMAXOIL_POS;    //��¼���õ������ѹ��Ӧ��λ��
                        gStMotorRunState[motorNum].limit1Time = 0;
                    }
                    else
                    {
                        *pRunStep = HOME_FLAG_WAIT_CCW_STALL_STOP;    //�ȴ������תֹͣ
                        gStMotorRunState[motorNum].limit1Time = 0;
                    }
                }
            }
        }
        else 
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 500)
            {
                gStMotorRunState[motorNum].runDelayTime = 480;
                if(HAL_OK == Motor485SetMotorMove(motorNum, DIR_CCW, 20))
                {
                    *pRunStep = HOME_FLAG_WAIT_CCW_STOP;
                    gStMotorRunState[motorNum].limit1Time = 0;
                }
            }
        }
    }
    else if((HOME_FLAG_WAIT_CW_STOP == *pRunStep) || (HOME_FLAG_WAIT_CCW_STALL_STOP == *pRunStep) || (HOME_FLAG_RECORD_SETMAXOIL_POS == *pRunStep))
    {
        if (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
        {
            if (HOME_FLAG_RECORD_SETMAXOIL_POS == *pRunStep)
            {
                lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                if (lfTemp <= (IS_UFOONE_FLAG_SET(UFOONE_BRAKE_STOP_ACCORD_OIL) ? (ADC1_BRAKE_MAX_THRETHOLDS + 5) : ADC1_BRAKE_MAX_THRETHOLDS))    //��ǰ��ѹֵ���趨�������ѹֵ(0.1Mpa)
                {
                    if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
                    {
                        gStMotorRunState[motorNum].errorCnt = 0;
                        gStMotorRunState[motorNum].limitPos2 = liTemp; //ɲ�����λ��
                        if(DEBUG_DATA_TYPE_2)
                        {
                            rt_kprintf("P-limit2:%d, oil:%.1fMpa, ticks:%d\r\n", gStMotorRunState[motorNum].limitPos2, lfTemp / 10, HAL_GetTick());
                        }

                        *pRunStep = HOME_FLAG_WAIT_CCW_STALL_STOP;    //�ȴ������תֹͣ
                        gStMotorRunState[motorNum].limit1Time = 0;
                    }
                    else
                    {
                        gStMotorRunState[motorNum].errorCnt++;
                    }
                }
                
                if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("id-%d-t:%d,c:%d,m:%d,s:%d,tv:%d,sa:%d,ca:%d,mf:%d,r:%d,b:%d,sk:%d,oil:%d,t:%d.\r\n", 1,
                        0, 0, gStMotorRevData[M_BRAKE].speed, gStMotorRevData[M_BRAKE].current, 0, 
                        gStMotorData[M_BRAKE].profileAcc, 0, 0, 0, ((gStMotorData[M_BRAKE].flag & MOTOR_DIR_INVERSE_FLAG) ? (700-gTargetRunMode[M_BRAKE].target_value):(gTargetRunMode[M_BRAKE].target_value)), 0, (uint32_t)(lfTemp*10), HAL_GetTick());
                }
            }
            
            if (gStMotorRunState[motorNum].runDelayTime >= 500)
            {
                gStMotorRunState[motorNum].runDelayTime = 450;                  //�״ν�������Ϊ500ms��������������Ϊ50ms
                if(0 == MotorReadVelocity(gStMotorData[motorNum].idx, &liTemp))
                {
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("stall velocity:%d, ticks:%d\r\n", liTemp, HAL_GetTick());
                    }
                    gStMotorRunState[motorNum].errorCnt = 0;
                    if(ABS_VALUE(liTemp) < 10)
                    {
                        gStMotorRunState[motorNum].limit1Time += 1;
                        if(gStMotorRunState[motorNum].limit1Time >= 3)  //����3������ת��С��10rpm
                        {
                            if(HOME_FLAG_WAIT_CW_STOP == *pRunStep)
                            {
                                *pRunStep = HOME_FLAG_SET_POS;  //������λ��
                                gStMotorRunState[motorNum].runDelayTime = 0;
                            }
                            else if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0)) //����ת��Ϊ0
                            {
                                *pRunStep = HOME_FLAG_RECORD_INIT_POS;   //��ȡ��ʼλ��
                                gStMotorRunState[motorNum].runDelayTime = 0;
                            }
                        }
                    }
                    else
                    {
                        gStMotorRunState[motorNum].limit1Time = 0;
                    }
                }
                else
                {
                    gStMotorRunState[motorNum].errorCnt++;
                }
            }
        }
        else
        {
            if(GPIO_PIN_RESET == GetMotorLimit1State(motorNum)) //���ڵ�
            {
                gStMotorRunState[motorNum].limit1Time += processTimeMs;
                if(gStMotorRunState[motorNum].limit1Time >= 30)
                {
                    if(HAL_OK == Motor485SetMotorMove(motorNum, DIR_STOP, 50))
                    {
                        *pRunStep = HOME_FLAG_MOVE_CCW;
                        gStMotorRunState[motorNum].runDelayTime = 0;
                    }
                }
            }
            else
            {
                gStMotorRunState[motorNum].limit1Time = 0;
            }
        }
    }
    else if(HOME_FLAG_WAIT_CCW_STOP == *pRunStep)
    {
        if (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
        {
            if (gStMotorRunState[motorNum].runDelayTime >= 10)
            {
                gStMotorRunState[motorNum].runDelayTime = 0u;
                lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                if(FLT_EPSILON >= lfTemp)     //��ѹ �� 0MPa����ȡ��ѹֵΪfloat�ͣ�
                {
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("ADC1_%d: %.2f, ticks:%d\r\n", ADC1_FRONT_BRAKE_CHANNEL, lfTemp, HAL_GetTick());
                    }

                    if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0)) //����ת��Ϊ0
                    {
                        *pRunStep = HOME_FLAG_RECORD_INIT_POS;   //��ȡ��ʼλ��
                    }
                }
            }
        }
        else
        {
            if(GPIO_PIN_SET == GetMotorLimit1State(motorNum)) //δ���ڵ�
            {
                gStMotorRunState[motorNum].limit1Time += processTimeMs;
                if(gStMotorRunState[motorNum].limit1Time >= 30)
                {
                    if(HAL_OK == Motor485SetMotorMove(motorNum, DIR_STOP, 50))
                    {
                        *pRunStep = HOME_FLAG_SET_POS;
                        gStMotorRunState[motorNum].runDelayTime = 0;
                    }
                }
            }
            else
            {
                gStMotorRunState[motorNum].limit1Time = 0;
            }
        }
    }
    else if(HOME_FLAG_SET_POS == *pRunStep)
    {
        if (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType)
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 50)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
                {
                    gStMotorRunState[motorNum].errorCnt = 0;
                    gStMotorRunState[motorNum].limitPos2 = liTemp; //ɲ�����λ��
                    *pRunStep = HOME_FLAG_MOVE_CCW;                //��������

                    lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("P-limit2: %d, oil:%.1fMpa, ticks:%d\r\n", gStMotorRunState[motorNum].limitPos2, lfTemp / 10, HAL_GetTick());
                    }
                }
                else
                {
                    gStMotorRunState[motorNum].errorCnt++;
                }

                /*if (AdcGetSensorValue(ADC1_IN4, ADC1_IN4_OFFSET, ((gStUfoData.oilPressStand >> 8) & 0xFF)) > ((gStUfoData.oilPressStand & 0xFF)))
                {
                    SetErrorCode(motorNum, ERROR_CODE_MOTOR_BRAKE1_ERROR, ERROR_L_NORMAL);   //ɲ��λ���ѵ����ޣ�����ѹ�����������ù�����
                }*/
            }
        }
        else
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 20)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                if(HAL_OK == Motor485SetActualPos(motorNum, gStMotorData[motorNum].initPos))
                {
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("Set pos: %d\r\n", gStMotorData[motorNum].initPos);
                    }
                    if(gStMotorData[motorNum].flag & MOTOR_PWM_CONTRL_MODE)  //pwm���Ʒ�ʽ
                    {
                        *pRunStep = HOME_FLAG_SET_MODE_SPWM_DIR;    //�л���pwm����ģʽ
                    }
                    else
                    {
                        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //Ѱ�����
                    }
                }
            }
        }
    }
    else if(HOME_FLAG_SET_MODE_SPWM_DIR == *pRunStep)
    {
        if(gStMotorRunState[motorNum].runDelayTime >= 20)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            if(HAL_OK == Motor485SetMotorContrlmode(motorNum, MOTOR_MODE_SPWM_DIR))
            {
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //Ѱ�����
            }
        }
    }

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
}
/*****************************************************************************
 ��������  : ��ȡ����Ƕ�
 �������  : uint32_t motorNum  ������
             float *angle          ����Ƕ�ֵ
 �������  : HAL_StatusTypeDef   ��ȡ���
 ��    ��  : ����
 ��    ��  : 2020��6��2��
*****************************************************************************/
#ifdef ENABLE_CANOPEN_DRIVER
HAL_StatusTypeDef ReadMotorAngle(uint32_t motorNum, float *angle)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int32_t liTemp;

    /*if(gStMotorData[motorNum].flag & MOTOR_USE_RS485_ECODER) //ʹ���ⲿ485����ֵ������
    {
        lResult = ReadMotorCurrentPosFromExterSensor(motorNum, angle);
    }
    else */if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
    {
        lResult = HAL_OK;
        
        *angle = CountsToAngle(motorNum, liTemp);
    }

    return lResult;
}
#endif
/*****************************************************************************
 ��������  : �����ɼ�
 �������  : void
 �������  : int32_t    ����ֵ  mA
 ��    ��  : ����
 ��    ��  : 2021��3��22��
*****************************************************************************/
int32_t CollectCurrent(void)
{
    int32_t lCurrent = 0;
    uint32_t i, endIndex = 0, pointIndex = 0, findFlag = 0;
    uint8_t lBuffer[USART7_REV_BUFFER_LEN + 1];
    
    gCurrentIndex += UsartDeviceRead(USART7_DEVICE, &gCurrentBuffer[gCurrentIndex], USART7_REV_BUFFER_LEN - gCurrentIndex);

    if(gCurrentIndex > USART7_REV_BUFFER_LEN)
    {
        gCurrentIndex = 0;
    }
    else if(gCurrentIndex > 5)
    {
        for(i = gCurrentIndex - 1; i > 3; i--)
        {
            if(0 == endIndex)
            {
                if(('.' == gCurrentBuffer[i]) && ('(' == gCurrentBuffer[i - 3]))
                {
                    endIndex = i - 3;
                    i = i - 4;
                }
            }
            else if(0 == pointIndex)
            {
                if('.' == gCurrentBuffer[i])
                {
                    pointIndex = i;
                }
            }
            else if(' ' == gCurrentBuffer[i])
            {
                findFlag = 1;
                for(i = i + 1; i < pointIndex; i++)
                {
                    if((gCurrentBuffer[i] >= '0') && (gCurrentBuffer[i] <= '9'))
                    {
                        lCurrent = lCurrent * 10 + gCurrentBuffer[i] - '0';
                    }
                    else
                    {
                        findFlag = 2;
                        break;
                    }
                }
                for(i = pointIndex + 1; i <= pointIndex + 3; i++)
                {
                    if(i < endIndex)
                    {
                        if((gCurrentBuffer[i] >= '0') && (gCurrentBuffer[i] <= '9'))
                        {
                            lCurrent = lCurrent * 10 + gCurrentBuffer[i] - '0';
                        }
                        else
                        {
                            findFlag = 2;
                            break;
                        }
                    }
                    else
                    {
                        lCurrent *= 10;
                    }
                }
                memcpy(lBuffer, gCurrentBuffer, endIndex + 4);
                lBuffer[endIndex + 4] = 0;
                rt_kprintf("*%s*", lBuffer);
                if(endIndex + 4 < gCurrentIndex)
                {
                    memcpy(gCurrentBuffer, &(gCurrentBuffer[endIndex + 4]), gCurrentIndex - endIndex - 4);
                    gCurrentIndex = gCurrentIndex - endIndex - 4;
                }
                else
                {
                    gCurrentIndex = 0;
                }
                break;
            }
        }
    }

    if((!findFlag) && (USART7_REV_BUFFER_LEN == gCurrentIndex))
    {
        gCurrentIndex = 0;
        memcpy(lBuffer, gCurrentBuffer, USART7_REV_BUFFER_LEN);
        lBuffer[USART7_REV_BUFFER_LEN] = 0;
        rt_kprintf("#%s#", lBuffer);
    }
    else if(2 == findFlag)
    {
        lCurrent = 0;
    }
		
    return lCurrent;
}
/*****************************************************************************
 ��������  : ������Сֵ���
 �������  : int32_t* pValue  ����
             int32_t len      ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��7��9��
*****************************************************************************/
int SearchMinValue(int32_t* pValue, int32_t len)
{
    int32_t i;
    int32_t min = 0;
    //Ѱ����Сֵ
    for(i = MVEL_RECORD_NUM - 1; i > 0; i--)
    {
        if(ABS_VALUE(pValue[i]) <= ABS_VALUE(pValue[min]))
        {
            min = i;
        }
    }

    return min;
}
/*****************************************************************************
 ��������  : ����м�ֵ
 �������  : int32_t* pValue
             int32_t len      
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��11��25��
*****************************************************************************/
int GetMidValue(int32_t* pValue, int32_t len)
{
    uint16_t i;
    int32_t lMin = pValue[0];
    int32_t lMax = lMin;
    int32_t lMid, lHighNum, lMinTemp, lMaxTemp;
    uint16_t lEndFlag = 0x02;
    
    //����С���ֵ
    for(i = 0; i < len; i++)
    {
        if(pValue[i] < lMin)
        {
            lMin = pValue[i];
        }
        else if(pValue[i] > lMax)
        {
            lMax = pValue[i];
        }
    }
    
    //Ѱ����ֵ�����ֲ���
    while((lEndFlag <= len) && (lMin + 2 < lMax))
    {
        lEndFlag <<= 1;
        lMid = (lMin + lMax) >> 1;
        lHighNum = 0;
        lMinTemp = lMax;
        lMaxTemp = lMin;
        for(i = 0; i < len; i++)
        {   
            if(pValue[i] > lMid)
            {
                lHighNum++;
                if(pValue[i] < lMinTemp)
                {
                    lMinTemp = pValue[i];
                }
            }
            else if(pValue[i] > lMaxTemp)
            {
                lMaxTemp = pValue[i];
            }
        }
        //С����ֵ�Ľ϶࣬���´���С����ֵ����֮����ֲ���
        if(lHighNum + 1 < (len >> 1))
        {
            lMax = lMaxTemp;
        }
        //������ֵ�Ľ϶࣬���´��ڴ�����ֵ����֮����ֲ���
        else if(lHighNum > (len >> 1) + 1)
        {
            lMin = lMinTemp;
        }
        else
        {
            break;
        }
    }
    
    lMid = (lMin + lMax) >> 1;    //��ϵͳ�У�����Ϊ��������

    return lMid;
}

/*****************************************************************************
 ��������  : ��ȡ��ص���ٶ�ֵ
 �������  : uint32_t motorNum
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��9��16��
*****************************************************************************/
static HAL_StatusTypeDef GetMotorSpeed(uint32_t motorNum, int32_t *vel)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    uint32_t tickstart = HAL_GetTick();
    int32_t rotateSpeed[4] = {0, 0, 0, 0};
    int32_t min, max, sum;
    uint8_t i, speedValidCnt = 0, liFlag = 0;
    
    CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_SPEED_FLAG);
    MotorSendReadVelocity(gStMotorData[motorNum].idx);
    if(((gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO)//ͬ���������������ͬĿ��ֵ
        && (gStUfoData.flag & UFO_FOUR_DRIVER_FLAG) //��������
        && (gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO))
        || (IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG)))    //���ֶ�����־
    {
        if(M_LEFT == motorNum)
        {
            if(DRIVER_TYPE_NONE != gStMotorData[M_LEFT_ONE].driverType)
            {
                CLEAR_MOTOR_REV_DATA_FLAG(M_LEFT_ONE, REV_MOTOR_SPEED_FLAG);
                MotorSendReadVelocity(gStMotorData[M_LEFT_ONE].idx);
            }
            if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
            {
                if(DRIVER_TYPE_NONE != gStMotorData[M_RIGHT].driverType)
                {
                    CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT, REV_MOTOR_SPEED_FLAG);
                    MotorSendReadVelocity(gStMotorData[M_RIGHT].idx);
                }
                if(DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType)
                {
                    CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT_ONE, REV_MOTOR_SPEED_FLAG);
                    MotorSendReadVelocity(gStMotorData[M_RIGHT_ONE].idx);
                }
            }
        }
    }
    if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
    {
        CLEAR_MOTOR_REV_DATA_FLAG(gStMotorData[motorNum].relatedMotor, REV_MOTOR_SPEED_FLAG);
        MotorSendReadVelocity(gStMotorData[gStMotorData[motorNum].relatedMotor].idx);
        if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
        {
            if(M_RIGHT == gStMotorData[motorNum].relatedMotor)
            {
                if((gStMotorData[M_RIGHT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                    (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType))
                {
                    CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT_ONE, REV_MOTOR_SPEED_FLAG);
                    MotorSendReadVelocity(gStMotorData[M_RIGHT_ONE].idx);
                }
            }
        }
    }

    while(1)
	{
        MotorReadProcess();
        
        if((HAL_GetTick() - tickstart) > CAN_WRITE_TIMEOUT_MS)   //��ʱ
        {
             break;
        }

        if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_SPEED_FLAG))
        {
            lResult = HAL_OK;

            if (!(liFlag & 0x1u))
            {
                liFlag |= 0x1u;
                rotateSpeed[speedValidCnt] = CalRatioVelRpm(motorNum);
                speedValidCnt++;
            }
             
            if(((gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO)//ͬ���������������ͬĿ��ֵ
                && (gStUfoData.flag & UFO_FOUR_DRIVER_FLAG) //��������
                && (gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO))
                || (IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG)))    //���ֶ�����־
            {
                if(IS_REV_MOTOR_DATA(M_LEFT_ONE, REV_MOTOR_SPEED_FLAG))
                {
                    if (!(liFlag & 0x2u))
                    {
                        liFlag |= 0x2u;
                        rotateSpeed[speedValidCnt] = CalRatioVelRpm(M_LEFT_ONE);
                        speedValidCnt++;
                    }
                    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
                    {
                        if(IS_REV_MOTOR_DATA(M_RIGHT, REV_MOTOR_SPEED_FLAG))
                        {
                            if (!(liFlag & 0x4u))
                            {
                                liFlag |= 0x4u;
                                rotateSpeed[speedValidCnt] = CalRatioVelRpm(M_RIGHT);
                                speedValidCnt++;
                            }
                            if(IS_REV_MOTOR_DATA(M_RIGHT_ONE, REV_MOTOR_SPEED_FLAG))
                            {
                                if (!(liFlag & 0x8u))
                                {
                                    liFlag |= 0x8u;
                                    rotateSpeed[speedValidCnt] = CalRatioVelRpm(M_RIGHT_ONE);
                                    speedValidCnt++;
                                }
                            }
                            else
                            {
                                lResult = HAL_ERROR;
                            }
                        }
                        else
                        {
                            lResult = HAL_ERROR;
                        }
                    }
                }
                else
                {
                    lResult = HAL_ERROR;
                }
            }
            if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
            {
                if(IS_REV_MOTOR_DATA(gStMotorData[motorNum].relatedMotor, REV_MOTOR_SPEED_FLAG))
                {
                    if (!(liFlag & 0x4u))
                    {
                        liFlag |= 0x4u;
                        rotateSpeed[speedValidCnt] = CalRatioVelRpm(gStMotorData[motorNum].relatedMotor);
                        speedValidCnt++;
                    }
                }
                else
                {
                    lResult = HAL_ERROR;
                }
                if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
                {
                    if(M_RIGHT == gStMotorData[motorNum].relatedMotor)
                    {
                        if((gStMotorData[M_RIGHT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                            (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType))
                        {
                            if(IS_REV_MOTOR_DATA(M_RIGHT_ONE, REV_MOTOR_SPEED_FLAG))
                            {
                                if (!(liFlag & 0x8u))
                                {
                                    liFlag |= 0x8u;
                                    rotateSpeed[speedValidCnt] = CalRatioVelRpm(M_RIGHT_ONE);
                                    speedValidCnt++;
                                }
                            }
                            else
                            {
                                lResult = HAL_ERROR;
                            }
                        }
                    }
                }
            }
        }

        if(HAL_OK == lResult)
        {
            min = rotateSpeed[0];
            max = rotateSpeed[0];
            sum = rotateSpeed[0];
            
            for(i=1u; i<speedValidCnt; i++)
            {
                if (rotateSpeed[i] <= min)
                {
                    min = rotateSpeed[i];
                }
                else if (rotateSpeed[i] >= max)
                {
                    max = rotateSpeed[i];
                }
                else 
                {
                    ;
                }
                    
                sum += rotateSpeed[i];
            }

            if(gStUfoData.flag & UFO_FEEDBACK_MAXSPEED_FLAG) //�����ٶ�ֵȡ����ٶ�
            {
                if(gStMotorRunState[motorNum].targetVel < 0)
                {
                    *vel = min;
                }
                else if(gStMotorRunState[motorNum].targetVel > 0)
                {
                    *vel = max;
                }
                else
                {
                    if(ABS_VALUE(min) <= ABS_VALUE(max))
                    {
                        *vel = max;
                    }
                    else
                    {
                        *vel = min;
                    }
                }
            }
            else 
            {
                if (speedValidCnt == 1u)
                {
                    *vel = sum;
                }
                else if (speedValidCnt == 2u)
                {
                    *vel = sum >> 1u;                               /* ȡƽ��ֵ */
                }
                else if (speedValidCnt > 2u)
                {
                    *vel = (sum - max - min) / (speedValidCnt - 2); /* �޳������Сֵ��ȡƽ��ֵ */
                }
                else
                {
                    ;
                } 
            }
            min = VelRpmToCm_s(min);
            max = VelRpmToCm_s(max); 
            if (max - min > 50)    //���ת�ٲ�ֵ��50cm/s
            {
                //SetErrorCode(M_TOTAL_NUM, ERROR_CODE_ROTATION_SYNC_ERROR, ERROR_L_NORMAL);
            } 
             
            break;
        }
        delay_us(50);
	} 

    return lResult;
}
/*****************************************************************************
 ��������  : ��ȡ��ص������
 �������  : uint32_t motorNum
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��9��16��
*****************************************************************************/
static HAL_StatusTypeDef IsMotorCurrentBelowStartCurrent(uint32_t motorNum, uint8_t lExeFlag)
{
    HAL_StatusTypeDef lResult = HAL_OK;
    uint32_t tickstart = HAL_GetTick();
    int32_t actualCurrent[2] = {0, 0};
    int32_t sum, ava = 0;
    uint8_t i, currentValidCnt = 0, liFlag = 0;

    if(DRIVER_TYPE_FDK != gStMotorData[motorNum].driverType)
    {
        return lResult;
    }

    //���е��Ŀ�������0
    gStMotorRunState[motorNum].setCurrent = 0;
    if(lExeFlag & M_R_ACCORD_L_FLAG)   //�ҵ������������־
    {
        if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
        {
            gStMotorRunState[M_RIGHT].setCurrent = 0;
        }
    }
    else if(lExeFlag & M_FOUR_ALONE_FLAG)   //�ĵ��������־
    {
        if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
        {
            gStMotorRunState[M_RIGHT].setCurrent = 0;
        }
        if(M_PID_RUN_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
        {
            gStMotorRunState[M_LEFT_ONE].setCurrent = 0;
        }
        if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)
        {
            gStMotorRunState[M_RIGHT_ONE].setCurrent = 0;
        }
    }
    
    CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
    MotorSendReadAvarageCurrentCmd(gStMotorData[motorNum].idx);
    if((motorNum == M_LEFT) && (gStMotorRunState[M_RIGHT].pidCurrentStartFlag == M_PID_RUN_FLAG))
    {
        CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT, REV_MOTOR_CURRENT_FLAG);
        MotorSendReadAvarageCurrentCmd(gStMotorData[M_RIGHT].idx);
    }

    while(1)
	{
        MotorReadProcess();
        
        if((HAL_GetTick() - tickstart) > CAN_WRITE_TIMEOUT_MS)   //��ʱ
        {
             break;
        }

        if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_CURRENT_FLAG))
        {
            lResult = HAL_OK;

            if (!(liFlag & 0x1u))
            {
                liFlag |= 0x1u;
                actualCurrent[currentValidCnt] = gStMotorRevData[motorNum].current;
                currentValidCnt++;
            }
             
            if((motorNum == M_LEFT) && (gStMotorRunState[M_RIGHT].pidCurrentStartFlag == M_PID_RUN_FLAG))
            {
                if(IS_REV_MOTOR_DATA(M_RIGHT, REV_MOTOR_CURRENT_FLAG))
                {
                    if (!(liFlag & 0x2u))
                    {
                        liFlag |= 0x2u;
                        actualCurrent[currentValidCnt] = gStMotorRevData[M_RIGHT].current;
                        currentValidCnt++;
                    }
                }
                else
                {
                    lResult = HAL_ERROR;
                }
            }

            if(HAL_OK == lResult)
            {
                sum = actualCurrent[0];
                
                for(i=1u; i<currentValidCnt; i++)
                {
                    sum += actualCurrent[i];
                }

                if (currentValidCnt == 1u)
                {
                    ava = sum;
                }
                else if (currentValidCnt == 2u)
                {
                    ava = sum >> 1u;                               /* ȡƽ��ֵ */
                }

                if((ava > (gStMotorData[motorNum].startCurrent >> 1)) && (ava > 5000))
                {
                    lResult = HAL_ERROR;
                }
                 
                break;
            }
        }
        
        delay_us(50);
	} 

    return lResult;
}
/*****************************************************************************
 ��������  : ��ȡ�������ֵ
 �������  : ST_TIM_DATA* tim            ��ʱ������
             uint8_t counts              ����һת��Ӧ������
             uint32_t processTimeMs      ������������(ms)
 �������  : lwheelSpeed ����(rpm)
 ��    ��  : ����
 ��    ��  : 2023��10��17��
*****************************************************************************/
static uint32_t GetWheelSpeed(ST_TIM_DATA* tim, uint8_t ch, uint8_t counts, uint32_t processTimeMs)
{
    int32_t lwheelSpeed;
    uint8_t lfactor;     //��������
    
    uint32_t tim_f0;     //��ʱ������Ƶ��
    uint32_t tim_m0;     //��ʱ��������
    uint32_t pluse_m1;   //������������

    if (counts == 0)      //����һת��Ӧ����������Ϊ��
    {
        return 0;
    }

    tim_f0 = tim->cntFre;
    tim_m0 = tim->CCRRecord[ch] - tim->lastCCRRecord[ch] + (tim->overFlowCounts * tim->device->Init.Period);  //�ȼ���ӣ��Է�ֹ�������
    pluse_m1 = tim->captureCounts[ch];

    //���¼������ʱ��
    tim->lastCCRRecord[ch] = tim->CCRRecord[ch];
    tim->captureCounts[ch] = 0;
    
    
    if (tim_m0 == 0)     //��ʱ������������Ϊ��
    {
        return 0;
    }
    
    //��������(rpm)��M/T���������۹�ʽn=(60*f0*M1)/(Z*M0),Ϊ��ֹ������������¹�ʽ����n=60*f0/M0*M1/Z
    lfactor = tim_f0/108000000 + 1;  //ͨ����������ֵ���ڱ�֤���㾫�ȵ�������ٴη�ֹ���,����Ƶ�ʣ�108MHz����Ҫ��Ƶ����
    lwheelSpeed = (int32_t)((60 / lfactor * tim_f0) / tim_m0 * pluse_m1 * lfactor / counts);
  
    return lwheelSpeed;
}
/*****************************************************************************
 ��������  : ResetTimer(ST_TIM_DATA* tim)
 �������  : ST_TIM_DATA* tim            ��ʱ������
 �������  : lwheelSpeed ����(rpm)
 ��    ��  : ����
 ��    ��  : 2023��10��17��
*****************************************************************************/
static void ResetTimer(ST_TIM_DATA* tim)
{
    tim->overFlowCounts = 0;    
}
/*****************************************************************************
 ��������  : �����ȥһ��ʱ���ڵ�ƽ�����ٶȺ�ƽ������
 �������  : int32_t* accOut
             int32_t* currentOut  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2024��4��30��
*****************************************************************************/
void CalAvaAccAndCurrent(int32_t* accOut, int32_t* currentOut)
{
    int32_t liTemp, k;
    uint32_t time = 0;
    
    //��¼��MVEL_RECORD_NUM������
    if((gCarVelValidNum >= MVEL_RECORD_NUM) && (gMotorAvaVelValidNum >= MVEL_RECORD_NUM))
    {
        liTemp = gCarVel[MVEL_RECORD_NUM - 1] - gCarVel[0];
        time = gCarVelTime[MVEL_RECORD_NUM - 1] - gCarVelTime[0];
        if(time > 0)
        {
            //����ƽ���ߵ����ٶ�
            liTemp = liTemp * 1000 / (int32_t)time;
            *accOut = Limit(liTemp, -CMD_ACC_VALUE_MAX, CMD_ACC_VALUE_MAX);
            //����ƽ������
            liTemp = 0;
            for(k = 0; k < MVEL_RECORD_NUM; k++)
            {
                liTemp += gMotorAvaSetCurrentRecord[k];
            }
            *currentOut = liTemp / MVEL_RECORD_NUM;
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("CarAvaAcc:%d,current:%d.\r\n", *accOut, *currentOut);
            }
        }
    }
}
/*****************************************************************************
 ��������  : �Զ�ѧϰ1m/s2���ٶȶ�Ӧ����
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2024��4��30��
*****************************************************************************/
void AutoLearnAccCurrent(void)
{
    int32_t k, liTemp;
    
    //���ٽ׶������ٶȣ����Ҽ��ٶ�������0.9m/s2ʱ����Ϊ��Ч
    if((gMotorAccPeriodAvaAcc > 0) && (gMotorAccPeriodAvaAcc > gMotorUniPeriodAvaAcc + 90)
        && (0 != gMotorAccPeriodAvaCurrent) && (0 != gMotorUniPeriodAvaCurrent))
    {
        liTemp = (gMotorAccPeriodAvaCurrent - gMotorUniPeriodAvaCurrent) * 100 / (gMotorAccPeriodAvaAcc - gMotorUniPeriodAvaAcc);
        if(!IsDiffOverPercent(liTemp, gStMotorData[M_LEFT].startCurrent, 30))
        {
            //��¼ѧϰֵ
            for(k = 1; k < MACC_CURRENT_LEARN_NUM; k++)
            {
                gMotorAccCurrentRecord[k - 1] = gMotorAccCurrentRecord[k];
            }
            gMotorAccCurrentRecord[k - 1] = liTemp;
            if(gMotorAccCurrentValidNum < MACC_CURRENT_LEARN_NUM)
            {
                gMotorAccCurrentValidNum++;
            }
            liTemp = 0;
            for(k = MACC_CURRENT_LEARN_NUM - gMotorAccCurrentValidNum; k < MACC_CURRENT_LEARN_NUM; k++)
            {
                liTemp += gMotorAccCurrentRecord[k];
            }
            gMotorAccCurrent = liTemp / gMotorAccCurrentValidNum;
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("CarCurAccCurrent:%d,ava:%d(%d,%d,%d,%d).\r\n", gMotorAccCurrentRecord[k - 1], gMotorAccCurrent,
                    gMotorAccPeriodAvaAcc, gMotorUniPeriodAvaAcc, gMotorAccPeriodAvaCurrent, gMotorUniPeriodAvaCurrent);
            }
        }
    }
}
/*****************************************************************************
 ��������  : �趨�ٶ�״̬�л�����
 �������  : uint32_t motorNum  ������
             int32_t* curSetCarVel��ǰ�趨�ٶ�
             uint32_t curTime ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��8��29��
*****************************************************************************/
void SetVelStateSwitchDeal(uint32_t motorNum, int32_t* curSetCarVel, uint32_t curTime)
{
    int16_t scenario_max_vel; //����ٶ�
    int32_t carVel;           //�ߵ��ٶ�
    int32_t liTemp;
    uint32_t time = 0;
    
    scenario_max_vel = ABS_VALUE(VelRpmToCm_s(VelCm_sToRpm(sys_para->CAR_RTinf.max_vel)));  //��ֹ��λ�л�����ֵ��������-1�����
    carVel = ABS_VALUE(sys_para->CAR_RTinf.vel);

    if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //�޹ߵ���־ʱ�ߵ��ٶ�ֱ�Ӹ�ֵ����ٶ�
    {
        carVel = ABS_VALUE(gMotorAvaVelFilter);
    }

    //////״̬�л��ж�
    //����ʱ��
    if(SET_VEL_STATE_START == gStMotorRunState[motorNum].setVelState)
    {
        gStMotorRunState[motorNum].setCarVelValidNum = 1;//ֻ�������һ���趨���ٶ�ֵ�������
        gStMotorRunState[motorNum].setAccKalmanData.LastP = QUICKLY_FOLLOW_LASTP;//�趨���ٶȿ��ٸ���
        gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_ACC;//ת���ȼ���״̬
        if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
        {
            rt_kprintf("M%d-state-start,mxv:%d,carv:%d.\r\n", motorNum, scenario_max_vel, carVel);
        }
    }
    //���ٽ׶�
    if(SET_VEL_STATE_ACC == gStMotorRunState[motorNum].setVelState)
    {
        //��ǰ���ٸ���Ŀ�공��
        if(carVel > ABS_VALUE(*curSetCarVel))
        {
            //��ǰ���ٴ��ڳ�������ٶȲ��ҳ�������ٶ���Ч������Ŀ���ٶȲ������仯��������ٽ����׶�
            if(((carVel > scenario_max_vel) && (ABS_VALUE(*curSetCarVel) + 5 >= scenario_max_vel)
                && (ABS_VALUE(*curSetCarVel) <= scenario_max_vel))
                || (*curSetCarVel == gStMotorRunState[motorNum].setCarVel[0]))
            {
                gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_ACC_END;//ת�����ٽ���״̬
                if(M_LEFT == motorNum)  //�������ĩ�˼��ٶȶ�Ӧ�ĵ���ֵ
                {
                    CalAvaAccAndCurrent(&gMotorAccPeriodAvaAcc, &gMotorAccPeriodAvaCurrent);
                    gCarOverVel = carVel;
                    gCarOverVelTime = curTime;
                }
                gStMotorRunState[motorNum].lastSetCurrent = gStMotorRunState[motorNum].setCurrent;  //��¼��ʱ����ֵ���������н�����ٽ���ʱ�ָ�֮ǰ����ֵ
                if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
                {
                    rt_kprintf("M%d-state-acc-to-acc-end,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
                }
            }
            //Ŀ���ٶȱ�С�����뵽����
            else if(ABS_VALUE(*curSetCarVel) < ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
            {
                gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC_START;   //ת�����ٿ�ʼʱ��
                if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
                {
                    rt_kprintf("M%d-state-acc-to-dec,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
                }
            }
        } 
    }
    //���ٽ���״̬�����г�������
    if(SET_VEL_STATE_ACC_END == gStMotorRunState[motorNum].setVelState)
    {
        //Ŀ���ٶȱ��
        if(ABS_VALUE(*curSetCarVel) > ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
        {
            //Ŀ���ٶȴ��ڳ����ٶȣ�����ܴ��ں��˹����У������ٶ�ֵ��Ч���ص��ȼ���״̬
            if(ABS_VALUE(*curSetCarVel) > scenario_max_vel)
            {
                gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_ACC; //ת���ȼ���״̬
                if(M_LEFT == motorNum)
                {
                    gMotorAccPeriodAvaAcc = 0;
                    gMotorAccPeriodAvaCurrent = 0;
                }
                gStMotorRunState[motorNum].setCurrent = gStMotorRunState[motorNum].lastSetCurrent;  //�ָ�֮ǰ���趨����ֵ
                if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
                {
                    rt_kprintf("M%d-state-acc-end-to-acc,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
                }
            }
        }
        //Ŀ���ٶȱ�С
        else if(ABS_VALUE(*curSetCarVel) < ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
        {
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC_START;   //ת�����ٿ�ʼʱ��
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-acc-end-to-dec,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
            }
        }
        //Ŀ���ٶȲ��䣬���ٵ���Ŀ�공�٣��������ٶ�
        else if(carVel + 2 < ABS_VALUE(*curSetCarVel))
        {
            gStMotorRunState[motorNum].setCarVelValidNum = 1;//ֻ�������һ���趨���ٶ�ֵ�������
            gStMotorRunState[motorNum].setAccKalmanData.LastP = IMMEDIATELY_FOLLOW_LASTP;//�趨���ٶ���������
            //���㳬���׶���������ƽ�����ٶ�
            liTemp = gCarOverVel - carVel;
            time = curTime - gCarOverVelTime;
            if(time > 0)    //���ݴ��������ٶȼ�������ʱ��Ҫ�ĵ���ֵ
            {
                liTemp = liTemp * 1000 / (int32_t)time;
                liTemp = Limit(liTemp, -CMD_ACC_VALUE_MAX, CMD_ACC_VALUE_MAX);
                gStMotorRunState[motorNum].setCurrent = liTemp * gMotorAccCurrent / 100;
                if(*curSetCarVel < 0)
                {
                    gStMotorRunState[motorNum].setCurrent = -gStMotorRunState[motorNum].setCurrent;
                }
                gStMotorRunState[motorNum].lastSetAcc = 0;
            }
            if(M_LEFT == motorNum)
            {
                gStCarNavAccKalmanData.LastP = IMMEDIATELY_FOLLOW_LASTP;//��ǰ���ٶ���������
                if(*curSetCarVel > 0)   //ǰ��ʱ�ż���
                {
                    gCarEnterUniTime = curTime;
                }
            }
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_UNI; //�������ٽ׶�
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-acc-end-to-uni,mxv:%d,carv:%d,dec:%d,aCurrent:%d.\r\n", motorNum, scenario_max_vel, carVel, liTemp, gMotorAccCurrent);
            }
        }
        if(M_LEFT == motorNum)  //��¼����ٶ�ֵ��ʱ��
        {
            if(carVel > gCarOverVel)
            {
                gCarOverVel = carVel;
                gCarOverVelTime = curTime;
            }
        }
    }
    //���ٽ׶�
    if(SET_VEL_STATE_UNI == gStMotorRunState[motorNum].setVelState)
    {
        //Ŀ���ٶȱ�С
        if(ABS_VALUE(*curSetCarVel) < ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
        {
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC_START;   //ת�����ٿ�ʼʱ��
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-uni-to-dec,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
            }
        }
        if(M_LEFT == motorNum)//�������ٶ�һ��ʱ���������ٶε�ƽ�����ٶȺ�ƽ������ֵ���������1m/s2���ٶȶ�Ӧ����ѧϰֵ
        {
            if(curTime > gCarEnterUniTime)
            {
                time = curTime - gCarEnterUniTime;
                if(time >= 800)
                {
                    CalAvaAccAndCurrent(&gMotorUniPeriodAvaAcc, &gMotorUniPeriodAvaCurrent);
                    AutoLearnAccCurrent();
                    gCarEnterUniTime = 0xffffffff;
                }
            }
        }
    }
    //���ٿ�ʼʱ��
    if(SET_VEL_STATE_DEC_START == gStMotorRunState[motorNum].setVelState)
    {
        gStMotorRunState[motorNum].setCarVelValidNum = 1;//ֻ�������һ���趨���ٶ�ֵ�������
        gStMotorRunState[motorNum].setAccKalmanData.LastP = QUICKLY_FOLLOW_LASTP;//�趨���ٶȿ��ٸ���
        gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC;//ת�����ٽ׶�
    }
    //���ٽ׶�
    if(SET_VEL_STATE_DEC == gStMotorRunState[motorNum].setVelState)
    {
        if(ABS_VALUE(*curSetCarVel) == scenario_max_vel)    //�������ٽ׶�ʵ�����յ��쳣�ļ����趨�ٶ�ֵʱ�����»ָ�������״̬
        {
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_UNI;//ת������״̬
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-dec-to-uni,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
            }
        }
    }
}
/*****************************************************************************
 ��������  : ��¼������ٶȼ��ϼ��ٶ�
 �������  : int32_t curMotorAvaVel ��ǰ������ٶ�,��λrpm
             int32_t curMotorAvaCurrent ��ǰ����ϵ�������λmA
             uint32_t curTime     ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��3��24��
*****************************************************************************/
void RecordMotorVelAndAcc(int32_t curMotorAvaVel, int32_t curMotorAvaCurrent, uint32_t curTime)
{
    uint32_t k, time = 0;
    int32_t velDif;
    uint16_t velValidNum;
    int32_t navAccLimit = 400;
    
    //�ߵ��Զ�ģʽ���ٶ�����
    if((gStMotorData[M_RIGHT].profileAcc >= 50) && (gStMotorData[M_RIGHT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        navAccLimit = gStMotorData[M_RIGHT].profileAcc;
    }
    
    curMotorAvaVel = VelRpmToCm_s(curMotorAvaVel);//ת����λ��0.01m/s
    if(gMotorAvaVelValidNum > 0)
    {
        velValidNum = gMotorAvaVelValidNum;
        if(velValidNum > MACC_CAL_PERIOD_NUM)
        {
            velValidNum = MACC_CAL_PERIOD_NUM;
        }
        velDif = curMotorAvaVel - gMotorAvaVel[MVEL_RECORD_NUM - velValidNum];//������ٶ��õ��ٶȲ�
        time = curTime - gMotorAvaVelTime[MACC_CAL_PERIOD_NUM - velValidNum];//������ٶ��õ�ʱ���
    }

    //��¼�ٶ�
    for(k = 1; k < MVEL_RECORD_NUM; k++)
    {
        gMotorAvaVel[k - 1] = gMotorAvaVel[k];
        gMotorAvaSetCurrentRecord[k - 1] = gMotorAvaSetCurrentRecord[k];
    }
    gMotorAvaVel[k - 1] = curMotorAvaVel;
    gMotorAvaSetCurrentRecord[MVEL_RECORD_NUM - 1] = curMotorAvaCurrent;
    
    //��¼�������л�ʱ������
    if((gMotorAvaSetCurrentPositiveChangeIndex > -M_SET_CURRENT_EXE_DELAY_NUM)
        && (gMotorAvaSetCurrentPositiveChangeIndex < MVEL_RECORD_NUM))
    {
        gMotorAvaSetCurrentPositiveChangeIndex--; //�����µ����ĸ��£�����ֵ��ǰ��һ
    }
    else if(gMotorAvaVelValidNum > M_SET_CURRENT_EXE_DELAY_NUM)
    {
        for(k = 1; k < MVEL_RECORD_NUM; k++)    //������һ���������л���
        {
            if((gMotorAvaSetCurrentRecord[k - 1] < 0) && (gMotorAvaSetCurrentRecord[k] >= 0))
             {
                 gMotorAvaSetCurrentPositiveChangeIndex = k;
                 break;
             }
        }
    }
    //��¼�������л�ʱ������
    if((gMotorAvaSetCurrentNegativeChangeIndex > -M_SET_CURRENT_EXE_DELAY_NUM)
        && (gMotorAvaSetCurrentNegativeChangeIndex < MVEL_RECORD_NUM))
    {
        gMotorAvaSetCurrentNegativeChangeIndex--; //�����µ����ĸ��£�����ֵ��ǰ��һ
    }
    else if(gMotorAvaVelValidNum > M_SET_CURRENT_EXE_DELAY_NUM)
    {
        for(k = 1; k < MVEL_RECORD_NUM; k++)    //������һ���������л���
        {
            if((gMotorAvaSetCurrentRecord[k - 1] >= 0) && (gMotorAvaSetCurrentRecord[k] < 0))
             {
                 gMotorAvaSetCurrentNegativeChangeIndex = k;
                 break;
             }
        }
    }

    //��¼ʱ��
    for(k = 1; k < MACC_CAL_PERIOD_NUM; k++)
    {
        gMotorAvaVelTime[k - 1] = gMotorAvaVelTime[k];
    }
    gMotorAvaVelTime[k - 1] = curTime;
    
    //������ٶȲ���¼
    if(gMotorAvaVelValidNum > 0)
    {
        if(time > 0)
        {
            //������ٶ�
            velDif = velDif * 1000 / (int32_t)time;
            velDif = Limit(velDif, -navAccLimit, navAccLimit);

            //������ٶȿ������˲�
            KalmanFilter(gStMotorAccKalmanData, velDif);
            gMotorCurAcc = (int32_t)gStMotorAccKalmanData.out;
        }
    }

    if(gMotorAvaVelValidNum < MVEL_RECORD_NUM)
    {
        gMotorAvaVelValidNum++;
    }
}
/*****************************************************************************
 ��������  : ��¼�趨�ٶȼ����ٶ�
 �������  : int32_t curSetCarVel  ��ǰ�趨�ٶ�,��λrpm
             uint32_t curTime      ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��3��24��
*****************************************************************************/
void RecordCarSetVelAndAcc(uint32_t motorNum, int32_t curSetCarVel, uint32_t curTime)
{
    uint32_t k, time = 0;
    int32_t velDif = 0;

    curSetCarVel = VelRpmToCm_s(curSetCarVel);  //ת����λ��0.01m/s

    //����״̬�£�Ŀ���ٶȴ�0�л��ɷ�0ֵʱ�����ٶȼ���ֻ�������һ��ֵ�������
    if(SET_VEL_STATE_IDLE == gStMotorRunState[motorNum].setVelState)
    {
        if((0 != curSetCarVel) && (ABS_VALUE(curSetCarVel) <= 6) && (0 == gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1])
            && (0 == gStMotorRunState[motorNum].setCarVel[0]))
        {
            gStMotorRunState[motorNum].setCarVelValidNum = 1;//ֻ�������һ���趨���ٶ�ֵ�������
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-idle-to-start,setv:%d.\r\n", motorNum, curSetCarVel);
            }
        }
    }

    //�趨�ٶ�״̬�л����ҹߵ���Чʱ
    if(IS_UFOONE_FLAG_SET(UFOONE_MOTION_SWITCH_DEAL) && (!IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG)))
    {
        SetVelStateSwitchDeal(motorNum, &curSetCarVel, curTime);
    }
    
    if((gStMotorRunState[motorNum].setCarVelValidNum > 0) && (gStMotorRunState[motorNum].setCarVelValidNum <= SET_VEL_RECORD_NUM))
    {
        velDif = curSetCarVel - gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - gStMotorRunState[motorNum].setCarVelValidNum];    //������ٶ��õ��ٶȲ�
        time = curTime - gStMotorRunState[motorNum].setCarVelTime[SET_VEL_RECORD_NUM - gStMotorRunState[motorNum].setCarVelValidNum];       //������ٶ��õ�ʱ���
    }

    //��¼�ٶ�
    for(k = 1; k < SET_VEL_RECORD_NUM; k++)
    {
        gStMotorRunState[motorNum].setCarVel[k - 1] = gStMotorRunState[motorNum].setCarVel[k];
    }
    gStMotorRunState[motorNum].setCarVel[k - 1] = curSetCarVel;

    //��¼ʱ��
    for(k = 1; k < SET_VEL_RECORD_NUM; k++)
    {
        gStMotorRunState[motorNum].setCarVelTime[k - 1] = gStMotorRunState[motorNum].setCarVelTime[k];
    }
    gStMotorRunState[motorNum].setCarVelTime[k - 1] = curTime;
    
    //������ٶȲ���¼
    if(gStMotorRunState[motorNum].setCarVelValidNum > 0)
    {
        if(time > 500) //��ʱ��δ����������֮ǰ�����ݣ����趨Ŀ����ٶ�Ϊ0
        {
            gStMotorRunState[motorNum].setCarVelValidNum = 1;
            gStMotorRunState[motorNum].setCarAcc = 0;
        }
        else if(time > 0)
        {
            gStMotorRunState[motorNum].setCarAcc = velDif * 1000 / (int32_t)time;
        }
    }

    if(gStMotorRunState[motorNum].setCarVelValidNum < SET_VEL_RECORD_NUM)
    {
        gStMotorRunState[motorNum].setCarVelValidNum++;
    }
}
/*****************************************************************************
 ��������  : ��¼�ߵ��ٶȼ����ٶ�
 �������  : int32_t curCarVel  ��ǰ�ߵ��ٶ�
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��3��24��
*****************************************************************************/
void RecordCarNavVelAndAcc(int32_t curCarVel, uint32_t curTime)
{
    uint32_t k, time = 0;
    int32_t velDif;
    uint16_t velValidNum;
    int32_t navAccLimit = 400;
    
    //�ߵ��Զ�ģʽ���ٶ�����
    if((gStMotorData[M_RIGHT].profileAcc >= 50) && (gStMotorData[M_RIGHT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        navAccLimit = gStMotorData[M_RIGHT].profileAcc;
    }
        
    if(gCarVelValidNum > 0)
    {
        velValidNum = gCarVelValidNum;
        if(velValidNum > NAV_ACC_CAL_PERIOD_NUM)
        {
            velValidNum = NAV_ACC_CAL_PERIOD_NUM;
        }
        velDif = curCarVel - gCarVel[NAV_CAR_VEL_RECORD_NUM - velValidNum]; //������ٶ��õ��ٶȲ�
        time = curTime - gCarVelTime[NAV_CAR_VEL_RECORD_NUM - velValidNum]; //������ٶ��õ�ʱ���
    }

    //��¼�ٶȺ�ʱ��
    for(k = 1; k < NAV_CAR_VEL_RECORD_NUM; k++)
    {
        gCarVel[k - 1] = gCarVel[k];
        gCarVelTime[k - 1] = gCarVelTime[k];
    }
    gCarVel[k - 1] = curCarVel;
    gCarVelTime[k - 1] = curTime; 
    
    //������ٶȲ���¼
    if(gCarVelValidNum > 0)
    {
        if(time > 0)
        {
            //����ߵ����ٶ�
            velDif = velDif * 1000 / (int32_t)time;
            velDif = Limit(velDif, -navAccLimit, navAccLimit);

            //�ߵ����ٶȿ������˲�
            KalmanFilter(gStCarNavAccKalmanData, velDif);
            gCarNavCurAcc = (int32_t)gStCarNavAccKalmanData.out;
        }
    }

    if(gCarVelValidNum < NAV_CAR_VEL_RECORD_NUM)
    {
        gCarVelValidNum++;
    }
}
/*****************************************************************************
 ��������  : ������ж�
 �������  : uint32_t motorNum       ������
             int32_t motorVel        ����ٶȣ���λcm/s
             int32_t targetVel       ʵ��Ŀ���ٶȣ���λcm/s
             int32_t setAcc          �趨���ٶȣ���λcm/s^2
             uint32_t processTimeMs  ��������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��5��29��
*****************************************************************************/
void MotorSkidJudge(uint32_t motorNum, int32_t motorVel, int32_t targetVel, int32_t setAcc, uint32_t processTimeMs)
{
    if (processTimeMs == 0)
    {
        processTimeMs = 10;   //���ۻ�ʱ���������Ϊ��
    }
    
    if (((motorVel - targetVel > 150) && (IS_ACC_GTE_0(targetVel, setAcc)))        //���ٶλ����ٶΣ���������ٶ�Զ����ʵ��Ŀ���ٶ�
	 || ((targetVel - motorVel > 150) && (!(IS_ACC_GTE_0(targetVel, setAcc))))     //����ٶΣ���������ٶ�ԶС��ʵ��Ŀ���ٶ�
	   )
    {
        gStMotorRunState[motorNum].skidFlag = 1;            //����򻬱�־
        gStMotorRunState[motorNum].skidOffCnt= 0;
        
    }
    else
    {
        if ((ABS_VALUE(motorVel - targetVel) < 50)                                   //��������ٶ�����ʵ��Ŀ���ٶ�
         || ((targetVel - motorVel >= 80) && (IS_ACC_GTE_0(targetVel, setAcc)))    //�� ���ٶλ����ٶΣ�������ܲ�����δ��������޷�����ʵ��Ŀ���ٶ�
           )
        {
            if (gStMotorRunState[motorNum].skidOffCnt >= 5) //�������������������5�ν���
            {
                gStMotorRunState[motorNum].skidFlag = 0;
            }
            else
            {
                gStMotorRunState[motorNum].skidOffCnt++;
            }
        }
    }

    if (gStMotorRunState[motorNum].skidFlag)
    {
        gStMotorRunState[motorNum].skidTotalTime += processTimeMs;
    }
    else
    {
        gStMotorRunState[motorNum].skidTotalTime = 0;
    }
    if(gStMotorRunState[motorNum].skidTotalTime > 5000)    //������5000ms
    {
        gStMotorRunState[motorNum].skidTotalTime = 0;   //��ֹ����д����
        //SetErrorCode(motorNum, ERROR_CODE_CONTINUOUS_SKID, ERROR_L_NORMAL);
        return;
    }
}
/*****************************************************************************
 ��������  : �������ֵ��¼���»���ָ�룬ָ���¸�����λ��
 �������  : uint32_t motorNum
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��7��14��
*****************************************************************************/
void MotorOverloaderUpdateIndex(uint32_t motorNum)
{
    uint32_t currentLimitAmp, currentLimitTime0_01s;

    //���1.2���������ڲ����й����ж�
    if((gStMotorData[motorNum].normalCurrent * 6 / 5)  > gStMotorData[motorNum].limitCurrent)
    {
        IIt_CLEAR_FLAG(motorNum, IIt_JUDGE_VALID);
        return;
    }
    
    //mAת������λA
    currentLimitAmp = gStMotorData[motorNum].limitCurrent / 1000;
    if((0 == currentLimitAmp) || (currentLimitAmp > 500))   //Ŀǰֻ֧��500A������Χ�ڵļ���
    {
        IIt_CLEAR_FLAG(motorNum, IIt_JUDGE_VALID);
        return;
    }

    //msת������λ0.01s
    currentLimitTime0_01s = gStMotorData[motorNum].overCurrentTime / 10;
    if((0 == currentLimitTime0_01s) || (currentLimitTime0_01s > 15000)) //Ŀǰֻ֧��150s����ʱ���ڵļ���
    {
        IIt_CLEAR_FLAG(motorNum, IIt_JUDGE_VALID);
        return;
    }

    IIt_SET_FLAG(motorNum, IIt_JUDGE_VALID);
    
    if(gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex] > 0)
    {
        gStMotorRunState[motorNum].IItIndex++;
        if(gStMotorRunState[motorNum].IItIndex >= M_BUFFER_IIt_LEN)
        {
            gStMotorRunState[motorNum].IItIndex = 0;
        }
        gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex] = 0;
        gStMotorRunState[motorNum].IItStartTime[gStMotorRunState[motorNum].IItIndex] = 0;
        gStMotorRunState[motorNum].IItOverLastTime = 0;
    }

    //��������ֵ���㣬������λA��ʱ�䵥λ0.01s
    gStMotorRunState[motorNum].IItLimit = currentLimitAmp * currentLimitAmp * currentLimitTime0_01s;

    if(DEBUG_DATA_TYPE_96)
    {
        rt_kprintf("M%d overcurrent%d limit:%d.\r\n", motorNum, gStMotorRunState[motorNum].IItIndex, gStMotorRunState[motorNum].IItLimit);
    }
}
/*****************************************************************************
 ��������  : ��������ж�
 �������  : uint32_t motorNum       ������
             rt_bool_t motorStartFlag���������־
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��7��13��
*****************************************************************************/
void MotorOverloaderJudge(uint32_t motorNum, rt_bool_t motorStartFlag)
{
    uint32_t curTime, temp;
    uint32_t setCurrentAbs, IItTemp;
    
    if(gStMotorRunState[motorNum].IItIndex >= M_BUFFER_IIt_LEN)
    {
        gStMotorRunState[motorNum].IItIndex = 0;
    }

    //����ʱ���»�������ָ��
    if(motorStartFlag)
    {
        MotorOverloaderUpdateIndex(motorNum);
        if(0 != gStMotorRunState[motorNum].IItOverTimeRecord)   //��¼������ʱ�̣��������������ʱ��Ϊ�ȴ��ָ�״̬���������ٴι���
        {
            IIt_SET_FLAG(motorNum, IIt_ENTER_WAIT_RECOVERY_FLAG);
        }
    }
    
    if(IIt_IS_SET_FLAG(motorNum, IIt_JUDGE_VALID))
    {
        curTime = HAL_GetTick();

        //��ȡ�趨��������ֵ
        setCurrentAbs = ABS_VALUE(gStMotorRunState[motorNum].setCurrent);
        
        //���ι���ֵ����
        if(setCurrentAbs > gStMotorData[motorNum].normalCurrent)
        {
            temp = curTime - gStMotorRunState[motorNum].IItOverLastTime;
            if(0 == gStMotorRunState[motorNum].IItOverLastTime)
            {
                gStMotorRunState[motorNum].IItOverLastTime = curTime;
            }
            else if(temp >= 100)    //0.1s����1��
            {
                //����Ƿ��л���������أ����л�������¸�����λ��
                if(gStMotorRunState[motorNum].setCurrent > 0)
                {
                    if(!IIt_IS_SET_FLAG(motorNum, IIt_POSITIVE_CURRENT_FLAG))
                    {
                        MotorOverloaderUpdateIndex(motorNum);
                        IIt_SET_FLAG(motorNum, IIt_POSITIVE_CURRENT_FLAG);
                    }
                }
                else
                {
                    if(IIt_IS_SET_FLAG(motorNum, IIt_POSITIVE_CURRENT_FLAG))
                    {
                        MotorOverloaderUpdateIndex(motorNum);
                        IIt_CLEAR_FLAG(motorNum, IIt_POSITIVE_CURRENT_FLAG);
                    }
                }

                //������ڵȴ��ָ����ֽ��뵽����״̬���򱨾���
                if(IIt_IS_SET_FLAG(motorNum, IIt_ENTER_WAIT_RECOVERY_FLAG))
                {
                    if(IS_ENABLE_SET_WARNING_CODE)
                    {
                        SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_OVER_LOADER_NEED_REST);
                    }
                }
                
                setCurrentAbs /= 1000;  //mAת������λA
                temp = temp / 10;   //����ʱ�䵥λ0.01s
                //IIt�ۼ�
                gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex] += setCurrentAbs * setCurrentAbs * temp;
                if(0 == gStMotorRunState[motorNum].IItStartTime[gStMotorRunState[motorNum].IItIndex])
                {
                    gStMotorRunState[motorNum].IItStartTime[gStMotorRunState[motorNum].IItIndex] = curTime;
                }
                gStMotorRunState[motorNum].IItOverLastTime = curTime;

                //���ι��ر�����
                if(gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex] >= gStMotorRunState[motorNum].IItLimit)
                {
                    if(IS_ENABLE_SET_WARNING_CODE)
                    {
                        SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_OVER_LOADER_SINGLE);
                    }
                    if(0 == gStMotorRunState[motorNum].IItOverTimeRecord)   //��¼����ʱ�̣�����Ϣ3����
                    {
                        gStMotorRunState[motorNum].IItOverTimeRecord = curTime;
                    }
                }
    
                if(DEBUG_DATA_TYPE_96)
                {
                    rt_kprintf("M%d IIt%d:%d,t:%d.\r\n", motorNum, gStMotorRunState[motorNum].IItIndex, gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex], curTime);
                }
            }
        }
        else
        {
            gStMotorRunState[motorNum].IItOverLastTime = 0;
        }
    
        //1�ְ������ۼƹ���ֵ����(�������ٺͼ���)
        IItTemp = 0;
        for(temp = gStMotorRunState[motorNum].IItIndex + 1; ; temp++)
        {
            if(temp >= M_BUFFER_IIt_LEN)
            {
                temp = 0;
            }
            if(curTime - gStMotorRunState[motorNum].IItStartTime[temp] < 90000) //һ�ְ����ڵ���Ч����
            {
                IItTemp += gStMotorRunState[motorNum].IItBuffer[temp];
            }
            if(temp == gStMotorRunState[motorNum].IItIndex)
            {
                break;
            }
        }
        //����2������ʱ��ʱ������ۼƹ��ؾ���
        if((IItTemp >> 1) >= gStMotorRunState[motorNum].IItLimit)
        {
            if(IS_ENABLE_SET_WARNING_CODE)
            {
                SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_OVER_LOADER_ONE_MINUTE);
            }
            if(0 == gStMotorRunState[motorNum].IItOverTimeRecord)   //��¼����ʱ�̣�����Ϣ3����
            {
                gStMotorRunState[motorNum].IItOverTimeRecord = curTime;
            }
        }
        
        if(motorStartFlag && DEBUG_DATA_TYPE_96)
        {
            rt_kprintf("M%d one minute IIt:%d,t:%d.\r\n", motorNum, IItTemp, curTime);
        }
    }
}
/*****************************************************************************
 ��������  : ������صȴ��ָ�(�ȵ����ȴ)
 �������  :
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��7��14��
*****************************************************************************/
void MotorOverLoaderWaitRecovery(void)
{
    uint32_t motorNum;
    uint32_t curTime;

    curTime = HAL_GetTick();
    
    for(motorNum = M_LEFT; motorNum < M_TOTAL_NUM; motorNum++)
    {
        if(IIt_IS_SET_FLAG(motorNum, IIt_JUDGE_VALID))
        {
            if(0 != gStMotorRunState[motorNum].IItOverTimeRecord)   //��¼������ʱ��
            {
                if(curTime - gStMotorRunState[motorNum].IItOverTimeRecord >= 180000)   //�����ϴι���ʱ�̳���3����ʱ���˳����ر���״̬
                {
                    gStMotorRunState[motorNum].IItOverTimeRecord = 0;
                    
                    IIt_CLEAR_FLAG(motorNum, IIt_ENTER_WAIT_RECOVERY_FLAG); //����ȴ��ָ���־
                    
                    if(DEBUG_DATA_TYPE_96)
                    {
                        rt_kprintf("M%d over loader recovery, E:0x%04x, t:%d.\r\n", motorNum, gErrorResult, curTime);
                    }
    
                    if(WARNING_CODE_OVER_LOADER_NEED_REST == gErrorResult)
                    {
                        gErrorResult = 0;
                    }
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : ����pidһЩ���в���������˫���ͬ�����������ڣ�pid������־���ٶ�ֵ���ߵ��ٱ�
 �������  : uint32_t motorNum
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��10��13��
*****************************************************************************/
static void CalPidRunParas(uint32_t motorNum, uint8_t lExeFlag, int32_t* liTempData, uint32_t processTimeMs)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int32_t liMotorVel, lTempVel, k, liTemp;
    static uint32_t luTime;

    if(0 == gStUfoData.ratioVel)
    {
        SetErrorCode(motorNum, ERROR_CODE_WRONG_PARAS, ERROR_L_NORMAL);
        return;
    }
    
    //���ڼ�ʱ
    gStMotorRunState[motorNum].pidDelayTime += processTimeMs;
    if((gStMotorRunState[motorNum].pidDelayTime >= gStMotorData[motorNum].currentAdjustPeriod)
        && (gStMotorRunState[motorNum].pidDelayTime > 0))
    {
        gStMotorRunState[motorNum].pidPeriodTime = gStMotorRunState[motorNum].pidDelayTime;
        gStMotorRunState[motorNum].pidDelayTime = 0;
        if(DEBUG_DATA_TYPE_92)
        {
            rt_kprintf("M%d get speed:%d.\r\n", motorNum, HAL_GetTick());
        }
        //��ȡ����ٶ�
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen������
        {
            lResult = GetMotorSpeed(motorNum, &liMotorVel);
        }
        else
        {
            lResult = Motor485ReadVelocity(motorNum, &liMotorVel);
        }
        if(DEBUG_DATA_TYPE_92)
        {
            rt_kprintf("M%d exit get speed:%d,%d.\r\n", motorNum, lResult, HAL_GetTick());
        }
        if(HAL_OK == lResult)
        {
            //����ٶȼ�¼
            if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
            {
                gStMotorRunState[motorNum].motorVelRecord = CalRatioVelRpm(motorNum);
                gStMotorRunState[M_RIGHT].motorVelRecord = CalRatioVelRpm(M_RIGHT);
                gStMotorRunState[M_LEFT_ONE].motorVelRecord = CalRatioVelRpm(M_LEFT_ONE);
                gStMotorRunState[M_RIGHT_ONE].motorVelRecord = CalRatioVelRpm(M_RIGHT_ONE);
                gStMotorRunState[motorNum].pidRunFlag = RT_TRUE;    //����pid����
                gStMotorRunState[M_RIGHT].pidRunFlag = RT_TRUE;    //����pid����
                gStMotorRunState[M_LEFT_ONE].pidRunFlag = RT_TRUE;    //����pid����
                gStMotorRunState[M_RIGHT_ONE].pidRunFlag = RT_TRUE;    //����pid����
                //��¼���ת�ٺ͵���
                liTemp = (gStMotorRunState[motorNum].setCurrent + gStMotorRunState[M_RIGHT].setCurrent
                    + gStMotorRunState[M_LEFT_ONE].setCurrent + gStMotorRunState[M_RIGHT_ONE].setCurrent) >> 2;
                RecordMotorVelAndAcc(liMotorVel, liTemp, HAL_GetTick());
            }
            else if(lExeFlag & M_R_ACCORD_L_FLAG)   //���Ҳ����������Ʊ�־
            {
                gStMotorRunState[motorNum].motorVelRecord = liMotorVel;
                if(DEBUG_DATA_TYPE_92)
                {
                    rt_kprintf("M%d get speed:%d.\r\n", M_RIGHT, HAL_GetTick());
                }
                //��ȡ����ٶ�
                if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[M_RIGHT].driverType))    //canopen������
                {
                    lResult = GetMotorSpeed(M_RIGHT, &liMotorVel);
                }
                else
                {
                    lResult = Motor485ReadVelocity(M_RIGHT, &liMotorVel);
                }
                if(DEBUG_DATA_TYPE_92)
                {
                    rt_kprintf("M%d exit get speed:%d,%d.\r\n", M_RIGHT, lResult, HAL_GetTick());
                }
                if(HAL_OK == lResult)
                {
                    gStMotorRunState[motorNum].pidRunFlag = RT_TRUE;    //����pid����
                    gStMotorRunState[M_RIGHT].pidPeriodTime = gStMotorRunState[motorNum].pidPeriodTime;
                    gStMotorRunState[M_RIGHT].motorVelRecord = liMotorVel;
                    gStMotorRunState[M_RIGHT].pidRunFlag = RT_TRUE;     //����pid����
                    liMotorVel = (gStMotorRunState[motorNum].motorVelRecord + liMotorVel) >> 1;  //���Ҳ���ٶ�
                    //��¼���ת�ٺ͵���
                    liTemp = (gStMotorRunState[motorNum].setCurrent + gStMotorRunState[M_RIGHT].setCurrent) >> 1;
                    RecordMotorVelAndAcc(liMotorVel, liTemp, HAL_GetTick());
                }
            }
            else
            {
                gStMotorRunState[motorNum].motorVelRecord = liMotorVel;
                gStMotorRunState[motorNum].pidRunFlag = RT_TRUE;    //����pid����
                //��¼���ת�ٺ͵���
                RecordMotorVelAndAcc(liMotorVel, gStMotorRunState[motorNum].setCurrent, HAL_GetTick());
            }
        }
    }

    //���ټ���
    luTime += processTimeMs;
    if ((luTime >= gStTim5Data.whlSpdCalcPeriod)
     && (luTime > 0)
       )
    {
        gWheelSpeed[W_LEFT_FRONT] = GetWheelSpeed(&gStTim5Data, CH2, gStTim5Data.whlCounts, luTime);
        gWheelSpeed[W_RIGHT_FRONT] = GetWheelSpeed(&gStTim5Data, CH3, gStTim5Data.whlCounts, luTime);
        ResetTimer(&gStTim5Data);
        luTime = 0;
        
        if(DEBUG_DATA_TYPE_92)
        {
            rt_kprintf("W%d speed:%d,W%d speed:%d.\r\n", W_LEFT_FRONT, gWheelSpeed[W_LEFT_FRONT], W_RIGHT_FRONT, gWheelSpeed[W_RIGHT_FRONT]);
        }
        
    }

    //����ߵ��ٱ�(�ǽ���ֹͣ��)
    if((gStUfoData.flag & UFO_MOTOR_AS_FEEDBACK_FLAG)    //����ٶ���Ϊpid������־
        && (sys_para->CAR_RTinf.Link & LINK_PC)              //PC����ʱ�Ŷ�̬�����ٱ�
        && (M_LEFT == motorNum)
        && (RT_TRUE == gStMotorRunState[motorNum].pidRunFlag)
        && (!IS_EMERGENCY_STOP))
    {
        lTempVel = VelRpmToCm_s(gStMotorRunState[motorNum].setTargetVel);
        if((ABS_VALUE(gCarVel[NAV_CAR_VEL_RECORD_NUM - 1]) > 20)   //�ߵ��ٶȲ�Ϊ0
            &&(ABS_VALUE(gCarVel[0]) > 20)
            && (ABS_VALUE(lTempVel) > 30)       //����ֵ����30cm/s
            && (gMotorAvaVelValidNum >= CALRATIOL_FILTER_NUM)
            && (gCarVelValidNum >= CALRATIOL_FILTER_NUM)    //����һ������
            )
        {
            //ȡ�ߵ��ٶ��˲�
            for(k = 1; k <= CALRATIOL_FILTER_NUM; k++)
            {
                liTempData[k] = gCarVel[NAV_CAR_VEL_RECORD_NUM - k];
            }
            FilterMidAverage(liTempData, CALRATIOL_FILTER_NUM, CALRATIOL_EXCLUDE_NUM, lTempVel, k, liTemp);
            lTempVel = ABS_VALUE(lTempVel);
            //ȡ����ٶ��˲�
            for(k = 1; (k <= CALRATIOL_FILTER_NUM) && (k <= MVEL_RECORD_NUM); k++)
            {
                liTempData[k] = gMotorAvaVel[MVEL_RECORD_NUM - k];
            }
            FilterMidAverage(liTempData, CALRATIOL_FILTER_NUM, CALRATIOL_EXCLUDE_NUM, liMotorVel, k, liTemp);
            liMotorVel = ABS_VALUE(liMotorVel);
            //�����ٱ�
            if((lTempVel > 0))
            {
                gAutoRatioVel = liMotorVel * (int32_t)gStUfoData.ratioVel / lTempVel;
                gAutoRatioVel = Limit(gAutoRatioVel, gStUfoData.ratioVel * 8 / 10, gStUfoData.ratioVel * 11 / 10);
                if(0 == gAutoRatioVel)
                {
                    gAutoRatioVel = NAV_RATIO_SCALE;
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : �ߵ��ٶȷ������ж�(˫�ر�׼�˲���һ����һЩ��һ���ϸ�һЩ�������Щ�ж�������ֵͻ��̫�����ϸ�һЩ���ж�)
 �������  : int32_t setTargetVel  �趨�ٶ�
             int32_t mVelFilter    ����˲��ٶ�
 �������  : rt_bool_t  Ϊ���ʾ����
 ��    ��  : ����
 ��    ��  : 2023��7��20��
*****************************************************************************/
rt_bool_t NavNegativeDirJudge(int32_t setTargetVel, int32_t mVelFilter)
{
    int32_t lTemp, lTempOne = 0, k, positiveCnt, negativeCnt, strictPositiveCnt, strictNegativeCnt;
    
    rt_bool_t lbNegativeDirFlag = RT_FALSE;
    uint32_t lDirJudgeValidCnt, lDirJudgeTotalNum;

    positiveCnt = 0;
    negativeCnt = 0;
    strictPositiveCnt = 0;
    strictNegativeCnt = 0;
    lDirJudgeValidCnt = 0;
    lDirJudgeTotalNum = MVEL_CAR_DIR_JUDGE_TOTAL_NUM;
    if(gMotorAvaVelValidNum < MVEL_RECORD_NUM)
    {
        lDirJudgeTotalNum = MVEL_CAR_DIR_JUDGE_NUM;
    }

    //���ζԵ���ٶȺ͹ߵ��ٶȾ���ֵ���бȽϣ�����ӽ�����Чֵ+1����MVEL_CAR_DIR_JUDGE_TOTAL_NUM��ֵ�ӽ����жϷ�����Ч
    if(gMotorAvaVelValidNum >= MVEL_CAR_DIR_JUDGE_TOTAL_NUM)
    {
        for(k = MVEL_RECORD_NUM - 1; k >= 0; k--)
        {
            //����ٶȾ���ֵ��ߵ��ٶȾ���ֵ��ֵ�ľ���ֵ
            lTemp = ABS_VALUE(gMotorAvaVel[k]);
            if(0 != lTemp)
            {
                lTemp = ABS_VALUE(lTemp - ABS_VALUE(gCarVel[k])) * 100 / lTemp; //��ֵռ����İٷֱ�
                if(k == MVEL_RECORD_NUM - 1)
                {
                    lTempOne = lTemp;
                }
                if(lTemp < 30)  //30%���ڶ���Ϊ�ӽ�
                {
                    lDirJudgeValidCnt++;
                    if(lDirJudgeValidCnt <= MVEL_CAR_DIR_JUDGE_NUM)    //���뷽���жϵĸ����ڽ������������ж�
                    {
                        if(gMotorAvaVel[k] > 0)
                        {
                            positiveCnt++;
                        }
                        else if(gMotorAvaVel[k] < 0)
                        {
                            negativeCnt++;
                        }
                    }
                    if(gMotorAvaVel[k] > 0)
                    {
                        strictPositiveCnt++;
                    }
                    else if(gMotorAvaVel[k] < 0)
                    {
                        strictNegativeCnt++;
                    }
                    if(lDirJudgeValidCnt >= lDirJudgeTotalNum)  //�ҵ�����Ҫ����������˳�
                    {
                        break;
                    }
                }
            }
        }
    }

    if(DEBUG_DATA_TYPE_90)
    {
        rt_kprintf("Car-dir-judge:%d%%,p:%d,n:%d,cnt:%d,totalCnt:%d.\r\n", lTempOne, positiveCnt, negativeCnt, lDirJudgeValidCnt, lDirJudgeTotalNum);
    }
 
    lTemp = MVEL_CAR_DIR_JUDGE_NUM >> 1;    
    //�����жϹߵ������������ж���Ч�������ݵ���������
    if((lDirJudgeValidCnt >= lDirJudgeTotalNum)
        && ((positiveCnt > lTemp) || (negativeCnt > lTemp)))
    {
        if(negativeCnt > lTemp) //�������
        {
            lbNegativeDirFlag = RT_TRUE;
        }

        //�����жϴ��ڴ������ֵͻ��������ϸ��ж�
        if(((gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] > 50) && (RT_TRUE == lbNegativeDirFlag))
            || ((gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] < -50) && (RT_FALSE == lbNegativeDirFlag)))
        {
            lbNegativeDirFlag = RT_FALSE;
            /////�ϸ��жϹߵ�����
            lTemp = lDirJudgeTotalNum - 1;
            //�������ж���Ч�������ݵ���������
            if((lDirJudgeValidCnt >= lDirJudgeTotalNum)
                && ((strictPositiveCnt > lTemp) || (strictNegativeCnt > lTemp)))
            {
                if(strictNegativeCnt > lTemp)   //�������
                {
                    lbNegativeDirFlag = RT_TRUE;
                }
                if(DEBUG_DATA_TYPE_90)
                {
                    rt_kprintf("Car-strict-dir-judge-valid:p:%d,n:%d,nDir:%d.\r\n", strictPositiveCnt, strictNegativeCnt, lbNegativeDirFlag);
                }
                return lbNegativeDirFlag;   //�ϸ��ж���ȷ��ֱ�ӷ��عߵ��ж��ķ���
            }
        }
        else    //�����ж���ȷ��ֱ�ӷ��عߵ��ж��ķ���
        {
            return lbNegativeDirFlag;
        }
    }

    /////���ɻ��ϸ��ж�δͨ��������������ж�
    lbNegativeDirFlag = RT_FALSE;
    //����˲��ٶ�Ϊ0���ϴιߵ��ٶ�Ϊ0����������Ч��������ʱ����Ŀ���ٶȶ���
    if((0 == mVelFilter) || (0 == gCarVel[NAV_CAR_VEL_RECORD_NUM - 1])
        || (gMotorAvaVelValidNum < lDirJudgeTotalNum))
    {
        if(setTargetVel < 0)
        {
            lbNegativeDirFlag = RT_TRUE;
        }
    }
    //�������ϴεķ���
    else if(gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] < 0)
    {
        lbNegativeDirFlag = RT_TRUE;
    }

    return lbNegativeDirFlag;
}
/*****************************************************************************
 ��������  : ��ȡ����͹ߵ��˲��ٶ�
 �������  : int32_t* liTempData        ��ʱ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��7��21��
*****************************************************************************/
void GetMotorAndNavFilterVel(int32_t* liTempData)
{
    int32_t liTemp, liTempVel = 0, positiveCurrentNum;
    int8_t i, k, li8Temp, liPositiveFilterNum, liNegativeFilterNum, li8Index[CAR_MOTOR_MAX_OR_MIN_FILTER_NUM];
    
    //ͳ������������
    positiveCurrentNum = 0;
    for(i = 0; i < MVEL_RECORD_NUM; i++)
    {
        if(gMotorAvaSetCurrentRecord[i] >= 0)
        {
            positiveCurrentNum++;
        }
        if(i < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM)
        {
            li8Index[i] = 0;    //����ֵ
        }
    }

    liPositiveFilterNum = (positiveCurrentNum + 2) * CAR_MOTOR_MAX_OR_MIN_FILTER_NUM / MVEL_RECORD_NUM;
    liNegativeFilterNum = CAR_MOTOR_MAX_OR_MIN_FILTER_NUM - liPositiveFilterNum;
    //������ʱȡ�ٶ���Сֵ�˲�ʱ
    if(liPositiveFilterNum > 0)
    {
        //��Ԥ����
        k = 0;
        for(i = 0; i < MVEL_RECORD_NUM; i++)
        {
            if(k >= liPositiveFilterNum)
            {
                break;
            }
            if(gMotorAvaSetCurrentRecord[i] >= 0)
            {
                if((i >= gMotorAvaSetCurrentPositiveChangeIndex)
                    && (i < gMotorAvaSetCurrentPositiveChangeIndex + M_SET_CURRENT_EXE_DELAY_NUM))
                {
                    liTempData[k] = gCarVel[i]; //�ӳ�������ȡ�ߵ��ٶ�
                }
                else
                {
                    liTempData[k] = gMotorAvaVel[i];
                }
                li8Index[k] = i;    //��¼��ţ����ں͹ߵ��ٶ�ֵ��Ӧλ�ö�Ӧ
                k++;
            }
        }

        //���Һ������У���Сֵ�滻��֮ǰ����������ֵ
        liTemp = 1; //��Ҫ��Ѱ���ֵ
        for(; i < MVEL_RECORD_NUM; i++)
        {
            //�ҳ�����ֵ�����
            if(liTemp)
            {
                liTemp = 0;
                li8Temp = 0;
                for(k = 1; k < liPositiveFilterNum; k++)
                {
                    if(liTempData[k] > liTempData[li8Temp])
                    {
                        li8Temp = k;
                    }
                }
            }
            //�����ǰ���ҵ�ֵС�ڶ����������ֵ������滻
            if(gMotorAvaSetCurrentRecord[i] >= 0)
            {
                if((i >= gMotorAvaSetCurrentPositiveChangeIndex)
                    && (i < gMotorAvaSetCurrentPositiveChangeIndex + M_SET_CURRENT_EXE_DELAY_NUM))
                {
                    liTempVel = gCarVel[i]; //�ӳ�������ȡ�ߵ��ٶ�
                }
                else
                {
                    liTempVel = gMotorAvaVel[i];
                }
                if(liTempVel < liTempData[li8Temp])
                {
                    liTemp = 1; //��Ҫ������Ѱ���ֵ
                    liTempData[li8Temp] = liTempVel;
                    li8Index[li8Temp] = i;  //��¼��ţ����ں͹ߵ��ٶ�ֵ��Ӧλ�ö�Ӧ
                }
            }
        }
    }
    //������ʱȡ�ٶ����ֵ�˲�ʱ
    if((liNegativeFilterNum > 0) && (liPositiveFilterNum < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM))
    {
        //��Ԥ����
        k = liPositiveFilterNum;
        for(i = 0; i < MVEL_RECORD_NUM; i++)
        {
            if(k >= CAR_MOTOR_MAX_OR_MIN_FILTER_NUM)
            {
                break;
            }
            if(gMotorAvaSetCurrentRecord[i] < 0)
            {
                if((i >= gMotorAvaSetCurrentNegativeChangeIndex)
                    && (i < gMotorAvaSetCurrentNegativeChangeIndex + M_SET_CURRENT_EXE_DELAY_NUM))
                {
                    liTempData[k] = gCarVel[i]; //�ӳ�������ȡ�ߵ��ٶ�
                }
                else
                {
                    liTempData[k] = gMotorAvaVel[i];
                }
                li8Index[k] = i;    //��¼��ţ����ں͹ߵ��ٶ�ֵ��Ӧλ�ö�Ӧ
                k++;
            }
        }

        //���Һ������У��ô�ֵ�滻��֮ǰ���������Сֵ
        liTemp = 1; //��Ҫ��Ѱ��Сֵ
        for(; i < MVEL_RECORD_NUM; i++)
        {
            //�ҳ���С��ֵ�����
            if(liTemp)
            {
                liTemp = 0;
                li8Temp = liPositiveFilterNum;
                for(k = liPositiveFilterNum + 1; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
                {
                    if(liTempData[k] < liTempData[li8Temp])
                    {
                        li8Temp = k;
                    }
                }
            }
            //�����ǰ���ҵ�ֵ���ڶ��������С��ֵ������滻
            if(gMotorAvaSetCurrentRecord[i] < 0)
            {
                if((i >= gMotorAvaSetCurrentNegativeChangeIndex)
                    && (i < gMotorAvaSetCurrentNegativeChangeIndex + M_SET_CURRENT_EXE_DELAY_NUM))
                {
                    liTempVel = gCarVel[i]; //�ӳ�������ȡ�ߵ��ٶ�
                }
                else
                {
                    liTempVel = gMotorAvaVel[i];
                }
                if(liTempVel > liTempData[li8Temp])
                {
                    liTemp = 1; //��Ҫ������Ѱ��Сֵ
                    liTempData[li8Temp] = liTempVel;
                    li8Index[li8Temp] = i;  //��¼��ţ����ں͹ߵ��ٶ�ֵ��Ӧλ�ö�Ӧ
                }
            }
        } 
    } 

    //����ٶȴ�ӡ
    if(DEBUG_DATA_TYPE_90)
    {
        rt_kprintf("Filter-motorvel");
        for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
        {
            rt_kprintf(",%d", liTempData[k]);
        }
        rt_kprintf(".\r\n");
    }
    
    //�ߵ��ٶȴ�ӡ
    if(DEBUG_DATA_TYPE_90)
    {
        rt_kprintf("Filter-navvel");
        for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
        {
            rt_kprintf(",%d", gCarVel[li8Index[k]]);
        }
        rt_kprintf(".\r\nFilter-index");
    }

    //������ֵ��n���������
    liTemp = 0;
    while(liTemp < CAR_MOTOR_FILTER_EXCLUDE_NUM)
    {
        li8Temp = -1;
        for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
        {
            if(-1 == li8Index[k])
            {
                continue;       //�Ѿ���Ч����������
            }
            else if(-1 == li8Temp)
            {
                li8Temp = k;    //li8Temp����ֵ
            }
            else if(ABS_VALUE(liTempData[li8Temp] - gCarVel[li8Index[li8Temp]]) < ABS_VALUE(liTempData[k] - gCarVel[li8Index[k]]))
            {
                li8Temp = k;    //li8Temp�滻������ֵ������
            }
        }
        if((li8Temp >= 0) && (li8Temp < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM))
        {
            li8Index[li8Temp] = -1; //���ҵ�������ֵ��ֵ����Ϊ-1
        }
        liTemp++;   //�����¸��δ�ֵ
    }

    //δ��ֵ����ֵ����ƽ��ֵ������������ӡ����
    gMotorVelFilter = 0;
    gCarVelFilter = 0;
    liTemp = 0;
    for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
    {
        if(li8Index[k] >= 0)
        {
            gMotorVelFilter += liTempData[k];
            gCarVelFilter += gCarVel[li8Index[k]];
            liTemp++;
        }
        if(DEBUG_DATA_TYPE_90)
        {
            rt_kprintf(",%d", li8Index[k]);
        }
    }
    //�������˲����

    if(liTemp > 0)
    {
        gMotorVelFilter /= liTemp;
        liTempVel = gMotorVelFilter;
        gMotorVelFilter = gMotorVelFilter * (int32_t)gStUfoData.ratioVel / gAutoRatioVel;
        gCarVelFilter /= liTemp;
    }
    
    if(DEBUG_DATA_TYPE_90)
    {
        rt_kprintf(".\r\nFilter-result:mVel:%d,mAutoVel:%d,nav:%d,%d%%,num:%d,p:%d,n:%d,pIndex:%d,nIndex:%d.\r\n", liTempVel, gMotorVelFilter, gCarVelFilter,
            (0 == gMotorVelFilter) ? 0 : (ABS_VALUE((gCarVelFilter - gMotorVelFilter) * 100) / ABS_VALUE(gMotorVelFilter)), liTemp,
            liPositiveFilterNum, liNegativeFilterNum, gMotorAvaSetCurrentPositiveChangeIndex,
            gMotorAvaSetCurrentNegativeChangeIndex);
    }
}
/*****************************************************************************
 ��������  : �����Զ�����
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��12��14��
*****************************************************************************/
static void CurrentBanlance(void)
{
    int32_t lBanlanceCurrent = 0;

    if(RT_FALSE == gNoLoadFlag) //�и���ʱ�Ž��е�������
    {
        if((gStMotorData[M_RIGHT].speedGrad > 0) && (gStMotorData[M_RIGHT].speedGrad < 3000))
        {
            lBanlanceCurrent = gStMotorData[M_RIGHT].speedGrad;
        }
    }
    if(!(sys_para->CAR_RTinf.Link & LINK_PC))   //��PC���ƾ������Сһ��
    {
        lBanlanceCurrent >>= 1;
    }

    //��������ʱ�����е�������
    if(BACK_DEFF_NONE != gBackWheelDeffDriverMode)
    {
        lBanlanceCurrent = 0;
    }
    
    //���ֶ�������
    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))
    {
        //�Ҳ������
        if(gStMotorRunState[M_RIGHT].setCurrent + gStMotorRunState[M_RIGHT_ONE].setCurrent >= 
            gStMotorRunState[M_LEFT].setCurrent + gStMotorRunState[M_LEFT_ONE].setCurrent)
        {
            //�ұ߼�����
            if(gStMotorRunState[M_RIGHT].setCurrent >= gStMotorRunState[M_RIGHT_ONE].setCurrent)
            {
                gStMotorRunState[M_RIGHT].setCurrent -= lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_RIGHT_ONE].setCurrent -= lBanlanceCurrent;
            }
            //��߼ӵ���
            if(gStMotorRunState[M_LEFT].setCurrent >= gStMotorRunState[M_LEFT_ONE].setCurrent)
            {
                gStMotorRunState[M_LEFT_ONE].setCurrent += lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_LEFT].setCurrent += lBanlanceCurrent;
            }
        }
        //��������
        else
        {
            //�ұ߼ӵ���
            if(gStMotorRunState[M_RIGHT].setCurrent >= gStMotorRunState[M_RIGHT_ONE].setCurrent)
            {
                gStMotorRunState[M_RIGHT_ONE].setCurrent += lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_RIGHT].setCurrent += lBanlanceCurrent;
            }
            //��߼�����
            if(gStMotorRunState[M_LEFT].setCurrent >= gStMotorRunState[M_LEFT_ONE].setCurrent)
            {
                gStMotorRunState[M_LEFT].setCurrent -= lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_LEFT_ONE].setCurrent -= lBanlanceCurrent;
            }
        }
    }
    //�����ֿ���
    else if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)
    {
        if(gStMotorRunState[M_RIGHT].setCurrent >= gStMotorRunState[M_LEFT].setCurrent)
        {
            gStMotorRunState[M_RIGHT].setCurrent -= lBanlanceCurrent;
            gStMotorRunState[M_LEFT].setCurrent += lBanlanceCurrent;
        }
        else
        {
            gStMotorRunState[M_RIGHT].setCurrent += lBanlanceCurrent;
            gStMotorRunState[M_LEFT].setCurrent -= lBanlanceCurrent;
        }
    }
}
/*****************************************************************************
 ��������  : ����ת��Ŀ�����ټ���
 �������  : uint32_t motorNum  ������
             int32_t tarSetVel  Ŀ����ٶ�
 �������  : int32_t    ���ز��ٺ��Ŀ���ٶ�
 ��    ��  : ����
 ��    ��  : 2023��12��26��
*****************************************************************************/
int32_t WheelDeffTarSetVelCal(uint32_t motorNum, int32_t tarSetVel)
{
    float lfTemp, lfTempOne;
    
    if(ABS_VALUE(gSteeringAngleVelDiff) >= 5)   //ת�Ǵ�ʱ�ż���
    {
        lfTemp = 0.67f / tan(ABS_VALUE(gSteeringAngleVelDiff) * DEF_PI / 180.0f);    //����ת��ԭ�㵽�����ߵľ��룬���1340mm��һ����0.67m
        lfTempOne = 0.67f * 0.67f;
        if((((M_LEFT == motorNum) || (M_LEFT_ONE == motorNum)) && (gSteeringAngleVelDiff < 0))
            || (((M_RIGHT == motorNum) || (M_RIGHT_ONE == motorNum)) && (gSteeringAngleVelDiff > 0)))
        {
            //���������������ߵ�ת��뾶��ֵ���־�1480mm��һ����0.74m
            lfTemp = sqrt(((lfTemp - 0.74f) * (lfTemp - 0.74f) + lfTempOne) / (lfTemp * lfTemp + lfTempOne));
            tarSetVel = lfTemp * tarSetVel; //��ȡĿ���ٶ�
        }
        else
        {
            //���������������ߵ�ת��뾶��ֵ���־�1480mm��һ����0.74m
            lfTemp = sqrt(((lfTemp + 0.74f) * (lfTemp + 0.74f) + lfTempOne) / (lfTemp * lfTemp + lfTempOne));
            tarSetVel = lfTemp * tarSetVel; //��ȡĿ���ٶ�
        }
    }

    return tarSetVel;
}
/*****************************************************************************
 ��������  : ����ٶȲ��η��������岨�ȣ�
 �������  : int32_t velErr ��ǰ�ٶȺ�Ŀ���ٶ��ٶ����ֵ
             uint32_t curTime      ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2024��1��18��
*****************************************************************************/
static void MotorVelWaveAnalysis(int32_t velErr, uint32_t curTime)
{
    static uint8_t lExitLoadCnt = 0;
    uint32_t lTime = 0;
    int32_t liTemp = 0, lCurrentIndex = MVEL_RECORD_NUM - MACC_CAL_PERIOD_NUM;
    
    //����ֵ�ж�
    if((gMotorLastVelErr > 0) && (gMotorLastLastVelErr < gMotorLastVelErr) && (velErr < gMotorLastVelErr))
    {
        if(gMotorVelErrPeakTime > gMotorVelErrValleyTime)   //�ϴθ��µĲ���
        {
            if(gMotorLastVelErr > gMotorVelErrPeak) //�µĲ���ֵ��������ϴεĲ���ֵ
            {
                gMotorVelErrPeak = gMotorLastVelErr;
                gMotorVelErrPeakTime = curTime;
                lTime = gMotorVelErrValleyTime;
            }
        }
        else    //�ϴθ��µĲ���ֵ����ֱ�Ӹ����µĲ���ֵ
        {
            gMotorVelErrPeak = gMotorLastVelErr;
            gMotorVelErrPeakTime = curTime;
            lTime = gMotorVelErrValleyTime;
        }
    }
    //����ֵ�ж�
    else if((gMotorLastVelErr < 0) && (gMotorLastLastVelErr > gMotorLastVelErr) && (velErr > gMotorLastVelErr))
    {
        if(gMotorVelErrPeakTime > gMotorVelErrValleyTime)   //�ϴθ��µĲ��壬��ֱ�Ӹ����µĲ���ֵ
        {
            gMotorVelErrValley = gMotorLastVelErr;
            gMotorVelErrValleyTime = curTime;
            lTime = gMotorVelErrPeakTime;
        }
        else    //�ϴθ��µĲ���ֵ
        {
            if(gMotorLastVelErr < gMotorVelErrValley)   //�µĲ���ֵС��������ϴεĲ���ֵ
            {
                gMotorVelErrValley = gMotorLastVelErr;
                gMotorVelErrValleyTime = curTime;
                lTime = gMotorVelErrPeakTime;
            }
        }
    }

    if(velErr != gMotorLastVelErr)    //����ȲŸ�ֵ����ֹ�жϲ��˲���Ͳ���
    {
        gMotorLastLastVelErr = gMotorLastVelErr;
        gMotorLastVelErr = velErr;
    }

    //�������ж�
    if(0 != lTime)
    {
        if(DEBUG_DATA_TYPE_90)
        {
            rt_kprintf("New p:%d,%lu,v:%d,%lu.\r\n", gMotorVelErrPeak, gMotorVelErrPeakTime,
                gMotorVelErrValley, gMotorVelErrValleyTime);
        }
        //���岨���񵴳���0.4m/s����ʱ���ڿ�����������
        if((gMotorVelErrPeak >= 40) && (gMotorVelErrValley <= -40))
        {
            lTime = curTime - lTime;
            if((lTime > MOTOR_NO_LOAD_JUDGE_MIN_HALF_PERIOD) && (lTime < MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD))  //��������Ϊ�޸���
            {
                gMotorVelOscillationFlag = RT_TRUE;
                if(DEBUG_DATA_TYPE_90)
                {
                    rt_kprintf("Oscillation vel period: %lu.\r\n", lTime);
                }
            }
            else
            {
                gMotorVelOscillationFlag = RT_FALSE;
            }
        }
        else
        {
            gMotorVelOscillationFlag = RT_FALSE;
        }
    }
    else if(((gMotorVelErrPeakTime > gMotorVelErrValleyTime) ? (curTime - gMotorVelErrPeakTime) : (curTime - gMotorVelErrValleyTime)) > MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD)
    {
        gMotorVelOscillationFlag = RT_FALSE;    //��������������񵴱�־
    }

    lTime = 0;
    //���ٶ����ж�
    if(gMotorAvaVelValidNum >= MACC_CAL_PERIOD_NUM)
    {
        //��ֵ���ٶ��ж�
        if(gMotorCurAcc > 0)
        {
            if(gMotorAvaSetCurrentRecord[lCurrentIndex] > 0)    //ֻ�жϵ�������0���������ֹ���ٶδ�
            {
                if(gMotorCurAcc < gMotorAccTemp)    //���ٶȱ�С���ҵ��µļ��ٶȷ�ֵ
                {
                    if(gMotorAccMaxTime > gMotorAccMinTime)   //�ϴθ��µĲ���
                    {
                        if(gMotorAccTemp > gMotorAccMax) //�µĲ���ֵ��������ϴεĲ���ֵ
                        {
                            gMotorAccMax = gMotorAccTemp;
                            gMotorAccMaxCurrent = gMotorAvaSetCurrentRecord[lCurrentIndex];
                            gMotorAccMaxTime = curTime;
                            lTime = gMotorAccMinTime;
                        }
                    }
                    else    //�ϴθ��µĲ���ֵ����ֱ�Ӹ����µĲ���ֵ
                    {
                        gMotorAccMax = gMotorAccTemp;
                        gMotorAccMaxCurrent = gMotorAvaSetCurrentRecord[lCurrentIndex];
                        gMotorAccMaxTime = curTime;
                        lTime = gMotorAccMinTime;
                    }
                }
                else
                {
                    gMotorAccTemp = gMotorCurAcc;
                }
            }
        }
        //��ֵ���ٶ��ж�
        else
        {
            if(gMotorAvaSetCurrentRecord[lCurrentIndex] < 0)    //ֻ�жϵ���С��0���������ֹ���ٶδ�
            {
                if(gMotorCurAcc > gMotorAccTemp)    //���ٶȱ���ҵ��µ��ٶȹ�ֵ
                {
                    if(gMotorAccMaxTime > gMotorAccMinTime)   //�ϴθ��µĲ��壬��ֱ�Ӹ����µĲ���ֵ
                    {
                        gMotorAccMin = gMotorAccTemp;
                        gMotorAccMinCurrent = gMotorAvaSetCurrentRecord[lCurrentIndex];
                        gMotorAccMinTime = curTime;
                        lTime = gMotorAccMaxTime;
                    }
                    else    //�ϴθ��µĲ���ֵ
                    {
                        if(gMotorAccTemp > gMotorAccMin) //�µĲ���ֵС��������ϴεĲ���ֵ
                        {
                            gMotorAccMin = gMotorAccTemp;
                            gMotorAccMinCurrent = gMotorAvaSetCurrentRecord[lCurrentIndex];
                            gMotorAccMinTime = curTime;
                            lTime = gMotorAccMaxTime;
                        }
                    }
                }
                else
                {
                    gMotorAccTemp = gMotorCurAcc;
                }
            }
        }
    }

    //���ٶ��ж�
    if(0 != lTime)
    {
        if(DEBUG_DATA_TYPE_90)
        {
            rt_kprintf("NewAcc p:%d,%d,%lu,v:%d,%d,%lu.\r\n", gMotorAccMax, gMotorAccMaxTime, gMotorAccMaxCurrent,
                gMotorAccMin, gMotorAccMinCurrent, gMotorAccMinTime);
        }
        liTemp = 15000;
        if(gStMotorData[M_LEFT].startCurrent > 0)
        {
            liTemp = gStMotorData[M_LEFT].startCurrent >> 1;
        }
        else if(gStMotorData[M_LEFT].limitCurrent > 0)
        {
            liTemp = gStMotorData[M_LEFT].limitCurrent >> 3;
        }
        //���岨���񵴳���1m/s2����ʱ���ڿ����������ڣ��ҵ�����С
        if((gMotorAccMax >= 100) && (gMotorAccMin <= -100)
            && gMotorAccMaxCurrent < (gMotorAccMax * liTemp / 100)
            && gMotorAccMinCurrent > (gMotorAccMin * liTemp / 100))
        {
            lTime = curTime - lTime;
            if((lTime > MOTOR_NO_LOAD_JUDGE_MIN_HALF_PERIOD) && (lTime < MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD))  //��������Ϊ�޸���
            {
                gMotorAccOscillationFlag = RT_TRUE;
                if(DEBUG_DATA_TYPE_90)
                {
                    rt_kprintf("Oscillation acc period: %lu.\r\n", lTime);
                }
            }
            else
            {
                gMotorAccOscillationFlag = RT_FALSE;
            }
        }
        else
        {
            gMotorAccOscillationFlag = RT_FALSE;
        }
    }
    else if(((gMotorAccMaxTime > gMotorAccMinTime) ? (curTime - gMotorAccMaxTime) : (curTime - gMotorAccMinTime)) > MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD)
    {
        gMotorAccOscillationFlag = RT_FALSE;    //��������������񵴱�־
    }
    
    //�޸����ж�
    if(sys_para->CAR_RTinf.Link & LINK_PC)  //PC����ʱֱ����Ϊ�Ƿǿ���
    {
        gNoLoadFlag = RT_FALSE;
    }
    else if(gMotorVelOscillationFlag && gMotorAccOscillationFlag)   //�ٶȺͼ��ٶ�ͬʱ�񵴣�����Ϊ�ǿ���
    {
        gNoLoadFlag = RT_TRUE;
    }

    //�и����ж�
    if(gNoLoadFlag && (gMotorAvaVelValidNum >= MACC_CAL_PERIOD_NUM))
    {
        liTemp = 15000;
        if(gStMotorData[M_LEFT].startCurrent > 0)
        {
            liTemp = gStMotorData[M_LEFT].startCurrent >> 1;
        }
        else if(gStMotorData[M_LEFT].limitCurrent > 0)
        {
            liTemp = gStMotorData[M_LEFT].limitCurrent >> 3;
        }
        liTemp = gMotorCurAcc * liTemp / 100;
        //���ٶȳ���0.5m/s2���ҵ����ϴ��������һ
        if(((gMotorCurAcc >= 50) && (gMotorAvaSetCurrentRecord[lCurrentIndex] > liTemp))
            || ((gMotorCurAcc <= -50) && (gMotorAvaSetCurrentRecord[lCurrentIndex] < liTemp)))
        {
            lExitLoadCnt++;
            if(lExitLoadCnt >= 10)  //����10�ε����ͼ��ٶ�ƥ������Ϊ�и���
            {
                gNoLoadFlag = RT_FALSE;
                if(DEBUG_DATA_TYPE_90)
                {
                    rt_kprintf("Exit load acc:%d,c:%d,cs:%d!\r\n", gMotorCurAcc, gMotorAvaSetCurrentRecord[lCurrentIndex], liTemp);
                }
            }
        }
        else if(lExitLoadCnt > 2)
        {
            lExitLoadCnt -= 2;
        }
    }
    else
    {
        lExitLoadCnt = 0;
    }
}
/*****************************************************************************
 ��������  : pid����ת�ٵ���
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ��ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��7��15��
*****************************************************************************/
static void PIDCurrentAdjust(uint32_t motorNum, uint32_t processTimeMs)
{
    int32_t liTemp, liRealVel = 0, liMotorVelCal, liMotorLRVel, liCarVel = 0, liSetTargetVelRef, liTargetVel, liTargetVelRef, k, liWheelVel[W_TOTAL_NUM];
    int32_t liTempData[MAX(MVEL_FILTER_NUM, MAX(NAV_CAR_VEL_RECORD_NUM, NAV_ACC_RECORD_NUM))];
    uint8_t lExeFlag = 0;
    uint32_t luOilPressure = 0;

    //�ж�ĳЩ�����Ƿ�ִ�У������Ƿ�ͬ����־
    //������Ҳ����������ƣ����Ҳ���������������Ѽ������
    if(!((M_LEFT != motorNum)
        && (gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)))  //���Ҳ����������Ʊ�־
    {
        lExeFlag |= M_PID_EXE_FLAG;
    }
    
    if((M_LEFT == motorNum) && IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
    {
        lExeFlag |= M_FOUR_ALONE_FLAG;
        //�ȴ�4�����pid��������
        if(((M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            || (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            || (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag))
            && (M_PID_RUN_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag))
        {
            return;
        }
    }
    else if((M_LEFT == motorNum)
        && (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT].driverType)
        && (gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG))  //���Ҳ����������Ʊ�־
    {
        lExeFlag |= M_R_ACCORD_L_FLAG;
        //�ȴ�����pid��������
        if((M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            && (M_PID_RUN_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag))
        {
            return;
        }
    }

    if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
    {
        CalPidRunParas(motorNum, lExeFlag, liTempData, processTimeMs);
    }
    
    //�ɹ�ȡ����ǰ�ٶ�ֵ
    gStMotorRunState[motorNum].pidNoExeTime += processTimeMs;
    if((gStMotorRunState[motorNum].pidRunFlag) && (M_PID_RUN_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag))
    {
        gStMotorRunState[motorNum].pidRunFlag = RT_FALSE;
        gStMotorRunState[motorNum].pidNoExeTime = 0;
        //�趨�Ĳο�Ŀ�공�٣���λ0.01m/s
        liSetTargetVelRef = VelRpmToCm_s(gStMotorRunState[motorNum].setTargetVel);
        //���ٺͳ����ٶȵ�λת��
        liMotorVelCal = VelRpmToCm_s(gStMotorRunState[motorNum].motorVelRecord);   //����ٶ�ת���ɳ��ٵ�λ0.01m/s
        liWheelVel[W_LEFT_FRONT] = WheelRpmToCm_s(gWheelSpeed[W_LEFT_FRONT]);      //����ת���ɳ��ٵ�λ0.01m/s
        liWheelVel[W_RIGHT_FRONT] = WheelRpmToCm_s(gWheelSpeed[W_RIGHT_FRONT]);    //����ת���ɳ��ٵ�λ0.01m/s
        
        //���ҵ�����ٶȣ�����һЩ���ϵ��ж��ٶ�
        liMotorLRVel = liMotorVelCal;
        if(lExeFlag & M_R_ACCORD_L_FLAG)   //�ҵ������������־
        {
            liMotorLRVel = (gStMotorRunState[motorNum].motorVelRecord + gStMotorRunState[M_RIGHT].motorVelRecord) >> 1;
            liMotorLRVel = VelRpmToCm_s(liMotorLRVel);
        }
        else if(lExeFlag & M_FOUR_ALONE_FLAG)   //�ĵ��������־
        {
            liMotorLRVel = (gStMotorRunState[motorNum].motorVelRecord + gStMotorRunState[M_RIGHT].motorVelRecord
                + gStMotorRunState[M_LEFT_ONE].motorVelRecord + gStMotorRunState[M_RIGHT_ONE].motorVelRecord) >> 2;
            liMotorLRVel = VelRpmToCm_s(liMotorLRVel);
        }
        
        //��ǰ�ߵ�ʵ�ʳ��ٸ�ֵ�������ж�
        if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            //�˲����㵱ǰ���ʵ�ʺ��ٶ�
            for(k = 1; (k <= MVEL_FILTER_NUM) && (k <= MVEL_RECORD_NUM); k++)
            {
                liTempData[k] = gMotorAvaVel[MVEL_RECORD_NUM - k];
            }
            FilterMidAverage(liTempData, MVEL_FILTER_NUM, MVEL_EXCLUDE_NUM, gMotorAvaVelFilter, k, liTemp); //�˲���ֹ�ߵ��ٶȷ�������

            liCarVel = sys_para->CAR_RTinf.vel;
            if(NavNegativeDirJudge(liSetTargetVelRef, gMotorAvaVelFilter))  //�����ж�
            {
                liCarVel = -liCarVel;   //����
            }
            if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //�޹ߵ���־ʱ�ߵ��ٶ�ֱ�Ӹ�ֵ����ٶ�
            {
                liCarVel = gMotorAvaVelFilter;
            }
        }
        
        //�������Ƶ���
        if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            if (!((0 == liSetTargetVelRef) && (ABS_VALUE(gMotorAvaVelFilter) < 150)))   //������ �趨�ٶ�Ϊ0���˲���ĵ���ٶȣ�150cm/s
            {
                gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].limitCurrent;
            }
        }
        
        //��¼�ߵ����ټ����ٶ�
        if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            RecordCarNavVelAndAcc(liCarVel, HAL_GetTick());
            //�������ٶȺ͹ߵ��ٶȵĲ�ֵ����ֵ�������жϹߵ�׼ȷ��
            GetMotorAndNavFilterVel(liTempData);
        }
        
        //liMotorVelFilter = liMotorVelFilter * (int32_t)gStUfoData.ratioVel / gStMotorRunState[motorNum].ratioVel;
        
        //��ǰʵ��Ŀ�공��(�趨���ٶȴ�������������ٶ��»����趨��Ŀ�공�ٲ�һ��)����λ0.01m/s
        liTargetVelRef = GetSlopeSpeed(motorNum, liSetTargetVelRef, gMotorAvaVelFilter, CAR_MOTOR_VELERR_OUT_RANGE(25), liTempData, gStMotorRunState[motorNum].pidPeriodTime);
        //�������ת��ʱ��Ŀ���ٶȵĲ���ֵ
        liTargetVel = liTargetVelRef;
        if(IS_UFOONE_FLAG_SET(UFOONE_TWO_WHEL_DEFF_DRIV))   //���ֲ�������
        {
            if(((gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG) && (BACK_DEFF_NONE != gBackWheelDeffDriverMode))  //���Ҳ����������Ʊ�־
                || IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
            {
                liTargetVel = WheelDeffTarSetVelCal(motorNum, liTargetVel);
            }
        }
        //���㶯̬�ٱ��µ�ʵ��Ŀ���ٶ�
        liTargetVel = liTargetVel * gAutoRatioVel / (int32_t)gStUfoData.ratioVel;
        
        //��¼�趨�ٶ�ֵ
        for(k = 1; k < MVEL_RECORD_NUM; k++)
        {
            gStMotorRunState[motorNum].setVelRecord[k - 1] = gStMotorRunState[motorNum].setVelRecord[k];
        }
        gStMotorRunState[motorNum].setVelRecord[MVEL_RECORD_NUM - 1] = liTargetVelRef;
        //�ж�ֹͣ
        if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            if(0 == liSetTargetVelRef)    //Ŀ���ٶ�Ϊ0�����ʱ�ж�ֹͣ���˳�pid����
            {
                liTemp = 0;
                for(k = MVEL_RECORD_NUM - MVEL_STOP_FILTER_NUM; k < MVEL_RECORD_NUM; k++)
                {
                    if(k >= 0)
                    {
                        liTemp += gMotorAvaVel[k];
                        //�����ٶ���ֵ���ƣ�Ҫ���˲������������ٶ�ֵ������ֵ��Χ�ڣ����ڷ�Χ�ڲ�����ͣ
                        if(ABS_VALUE(gMotorAvaVel[k]) >= 100)
                        {
                            liTemp = 500;
                            break;
                        }
                    }
                }
                if(ABS_VALUE(liTemp) < (IsBrakeLock ? 200 : 20))    //����ۻ�֮��С����ֵʱ�ж�Ϊֹͣ����բ��ʱ��ſ�����
                {
                    if(HAL_OK == IsMotorCurrentBelowStartCurrent(motorNum, lExeFlag))
                    {
                        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;    //�ر�pid����
                    }
                }
                else
                {
                    liTemp = 0;
                    //��������������ĸ��������и�Ҳ����ͬ������
                    for(k = 1; k < MVEL_RECORD_NUM; k++)
                    {
                        if(gMotorAvaSetCurrentRecord[k] >= 0)
                        {
                            if(gMotorAvaSetCurrentRecord[k- 1] < 0)
                            {
                                liTemp++;
                            }
                        }
                    }
                    //��������4�θ����������ٶ��˲���ֵ��С����Ϊ�ǿ��ض�����Ҳ�ж�Ϊֹͣ
                    if((liTemp >= 4) && (ABS_VALUE(gMotorAvaVelFilter) < 60))
                    {
                        if(DEBUG_DATA_TYPE_4)
                        {
                            rt_kprintf("Judge-stop n-to-p:%d, vel:%d.\r\n", liTemp, gMotorAvaVelFilter);
                        }
                        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;//�ر�pid����
                    }
                }
                if((ABS_VALUE(gMotorAvaVelFilter) < 40)   //�ٶȽӽ���ȫֹͣʱ���޶�����Ϊ������������ֹֹͣ����
                    && (gStMotorData[motorNum].startCurrent < gStMotorData[motorNum].normalCurrent)
                    && (gStMotorData[motorNum].startCurrent >= (gStMotorData[motorNum].normalCurrent >> 2)))    
                {
                    gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].startCurrent;
                }
                else if(ABS_VALUE(gMotorAvaVelFilter) < 100) //�ٶȽӽ�ֹͣʱ���޶�����Ϊ���������ֹֹͣ����
                {
                    gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].normalCurrent;
                }
            }
        }
        //��ֵ�����ǰ�ٶ�ֵ
        liRealVel = liMotorVelCal;
        //��ֵ���ڼ��ٶȲ������ٶ����ֵ���Զ�����ʱ�Թߵ�ֵΪ׼������ֹͣʱ�Ե��Ϊ׼����Ϊ�ߵ��ٶ����˲����������󣬵������ӹߵ��ٶ������ٶȲ���жϣ���ֹ�ߵ������쳣����ʧ��
        if((!IS_EMERGENCY_STOP) && (sys_para->CAR_RTinf.Link & LINK_PC) && (!((CAR_MOTOR_VELERR_OUT_RANGE(25)) || (IS_CAR_DECELERATION_AND_SKID(motorNum, liSetTargetVelRef)))))   //PC����
        {
            liTemp = liTargetVelRef - liCarVel;
            gStMotorRunState[motorNum].skidFlag = 0;
        }
        else
        {
            liTemp = liTargetVel - gMotorAvaVelFilter;
            gStMotorRunState[motorNum].skidFlag = 1;
        }
        //����ٶȲ��η�������Ҫ��ȡ���岨����Ϣ���������޸����ж�
        if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            MotorVelWaveAnalysis(gMotorAvaVelFilter - liTargetVel, HAL_GetTick());
        }
        //pid��������ת��
        CalcPid_incres_driver(motorNum, liTargetVel, liRealVel, liTemp, gStMotorRunState[motorNum].pidPeriodTime);
        //�����������
        if(!(lExeFlag & M_PID_EXE_FLAG))   //pid���ִ��벻ִ�б�־�������ҵ��
        {
            CurrentBanlance();
        }
        //��������
        /*if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            if(gStMotorRunState[motorNum].limitCurrent > gStMotorData[motorNum].normalCurrent)
            {
                if(!IS_SAME_SYMBLE(gStMotorRunState[motorNum].setCurrent, liSetTargetVelRef)
                    || (0 == liSetTargetVelRef))
                {
                    gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].normalCurrent << 1;
                }
            }
        }*/
        if(lExeFlag & M_R_ACCORD_L_FLAG)   //�ҵ������������־
        {
            gStMotorRunState[M_RIGHT].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
        }
        else if(lExeFlag & M_FOUR_ALONE_FLAG)   //�ĵ��������־
        {
            gStMotorRunState[M_RIGHT].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
            gStMotorRunState[M_LEFT_ONE].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
            gStMotorRunState[M_RIGHT_ONE].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
        }
        
        gStMotorRunState[motorNum].setCurrent = Limit(gStMotorRunState[motorNum].setCurrent,
            -gStMotorRunState[motorNum].limitCurrent, gStMotorRunState[motorNum].limitCurrent);
        gStMotorRunState[motorNum].setCurrent = Limit(gStMotorRunState[motorNum].setCurrent,
            -gStMotorData[motorNum].limitCurrent, gStMotorData[motorNum].limitCurrent);
        if(lExeFlag & M_PID_EXE_FLAG)   //pid���ִ���ִ�б�־
        {
            //���������ж�
            if(ABS_VALUE(liMotorLRVel) <= 5)
            {
                if(liSetTargetVelRef >= 20)
                {
                    gStMotorRunState[motorNum].infraFaultTimeTotal += gStMotorRunState[motorNum].pidPeriodTime;
                    if(gStMotorRunState[motorNum].infraFaultTimeTotal >= gStMotorData[motorNum].start_fault_time)
                    {
                        gStMotorRunState[motorNum].infraFaultTimeTotal = 0;
                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_START_ERROR, ERROR_L_NORMAL);
                        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;//�ر�pid����
                    }
                }
                else
                {
                    gStMotorRunState[motorNum].infraFaultTimeTotal = 0;
                }
            }
            else
            {
                gStMotorRunState[motorNum].infraFaultTimeTotal = 0;
            }
            //ײ���ж�
            if(sys_para->CAR_RTinf.Link & LINK_PC_LOST) //pc���ݶ�ʧ������ײ���жϣ��������»ָ�ײ���ж���Ҫ�ȴ�
            {
                gStMotorRunState[motorNum].pcLostRecoveryTime = HAL_GetTick();
            }
            //pid���ִ���ִ�б�־��PC����ʱ�����Ҿ����ϴ�ͨ�Żָ�����500msʱ�Ž�����ײ���(��ֹʧ���ڼ����ݲ�׼ȷ)
            else if((lExeFlag & M_PID_EXE_FLAG) && (sys_para->CAR_RTinf.Link & LINK_PC)
                && (HAL_GetTick() - gStMotorRunState[motorNum].pcLostRecoveryTime >= 500))
            {
                //δ����ֹͣ�Լ�Ŀ���ٶȲ�Ϊ0��ʱ��Ž�����ײ���
                if(!(IS_EMERGENCY_STOP || (liTargetVelRef == 0)))
                {
                    //�Թߵ��ٶȻ�����Ϊ��ײ�������жϣ�Ŀ���ٶȴ���0.5m/s���˲��ٶȴ��ڵ���100
                    if((ABS_VALUE(liTargetVelRef) > 50) && (ABS_VALUE(gMotorVelFilter) >= 100))
                    {
                        //�ߵ��ٶȵ���Ŀ���ٶȼ�ȥ�ٶ���ֵ,�ߵ��ٶȼ��ٶȱ��趨���ٶȴ�����ֵ
                        if(((liTargetVelRef > 0) && (liCarVel + 100 < liTargetVelRef) && ((gStMotorRunState[motorNum].setAcc > gStMotorRunState[motorNum].curAcc + 250) && (gStMotorRunState[motorNum].curAcc < -100)))
                            || ((liTargetVelRef < 0) && (liCarVel - 100 > liTargetVelRef) && ((gStMotorRunState[motorNum].setAcc < gStMotorRunState[motorNum].curAcc - 250) && (gStMotorRunState[motorNum].curAcc > 100))))
                        {
                            rt_kprintf("collision-Tar:%d,car:%d,setAcc:%d,curAcc:%d.\r\n", liTargetVelRef, liCarVel,
                                gStMotorRunState[motorNum].setAcc, gStMotorRunState[motorNum].curAcc);
                            if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                            {
                                if(!(gStUfoData.flag & UFO_DISABLE_COLLISION_DETECT))   //��ֹ��ײ���
                                {
                                    if(IS_ENABLE_SET_WARNING_CODE)
                                    {
                                        SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_ABNORMAL_DECELERATION);
                                        SetErrorInfo(motorNum, WARNING_CODE_ABNORMAL_DECELERATION);
                                        gStErrorInfo.setAcc = gStMotorRunState[motorNum].setAcc;
                                        gStErrorInfo.curAcc = gStMotorRunState[motorNum].curAcc;
                                    }
                                }
                            }
                        }
                    }
                    //��ײ��ߵ��ٶ��쳣���жϣ��ߵ������ٶȲ65%���˲��ٶȴ��ڵ���100
                    if(CAR_MOTOR_VELERR_OUT_RANGE(65)
                        && (ABS_VALUE(gMotorAvaVelFilter) >= 100))
                    {
                        rt_kprintf("collision-Car:%d,mvel:%d.\r\n", gCarVelFilter, gMotorVelFilter);
                        if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                        {
                            if(!(gStUfoData.flag & UFO_DISABLE_COLLISION_DETECT))   //��ֹ��ײ���
                            {
                                if(IS_ENABLE_SET_WARNING_CODE)
                                {
                                    SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_NAV_ABNORMAL);
                                    SetErrorInfo(motorNum, WARNING_CODE_NAV_ABNORMAL);
                                    gStErrorInfo.carFilter = gCarVelFilter;
                                    gStErrorInfo.motorFilter = gMotorVelFilter;
                                    gStErrorInfo.ratioVelAuto = gAutoRatioVel;
                                }
                            }
                        }
                    }
                } 
            }
        }

        if(IS_UFOONE_FLAG_SET(UFOONE_BRAKE_DEC_TEST_FLAG))  //ɲ�����ٶȲ���
        {
            if(700 == gBrakeValue)  //ɲ��������
            {
                //��ѹ�ﵽһ����ֵ����ֹɲ��δ������ʱ���������0��ƽ��ֱ�����ɻ���
                if(AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO) >= ADC1_BRAKE_TEST_THRETHOLDS)  //������ֵ��ѹ(��λ0.1Mpa)
                {
                    gStMotorRunState[motorNum].setCurrent = 0;
                }
            }
        }

        //����ٶȳ�������
        if(IS_UFOONE_FLAG_SET(UFOONE_MOTION_SWITCH_DEAL))
        {
            if(SET_VEL_STATE_ACC_END == gStMotorRunState[motorNum].setVelState) //�����׶��趨����ֱ�Ӹ�0
            {
                gStMotorRunState[motorNum].setCurrent = 0;
            }
        }
        
        //�趨����ֵ
        CHANGE_MOTOR_TARGET_CURRENT(motorNum, gStMotorRunState[motorNum].setCurrent);

        //������ж�
        //MotorSkidJudge(motorNum, liRealVel, liTargetVel, gStMotorRunState[motorNum].setAcc, gStMotorRunState[motorNum].pidPeriodTime);

        //̨�ܲ��������ϴ�
        TestMotorDataUploadProcess(motorNum, RT_TRUE, liTargetVelRef, liRealVel);
        
        //��������ж�
        MotorOverloaderJudge(motorNum, RT_FALSE);
        
        if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_92)
        {
            luOilPressure = (uint32_t)(AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO)*10);  //��ȡ��ѹֵ0.01Mpa
            rt_kprintf("id-%d-t:%d,c:%d,m:%d,s:%d,tv:%d,sa:%d,ca:%d,mf:%d,r:%d,b:%d,sk:%d,oil:%d,wlf:%d,wrf:%d,t:%d.\r\n", gStMotorData[motorNum].idx,
                liTargetVelRef, liCarVel, liRealVel, gStMotorRunState[motorNum].setCurrent, liTargetVel, 
                gStMotorRunState[motorNum].setAcc, gStMotorRunState[motorNum].curAcc, gMotorAvaVelFilter,
                gAutoRatioVel, gBrakeValue, gStMotorRunState[motorNum].skidFlag, luOilPressure, liWheelVel[W_LEFT_FRONT], liWheelVel[W_RIGHT_FRONT], HAL_GetTick());
        }

        //gStMotorRunState[motorNum].infraTimeDelay = 0;
    }
    else if((lExeFlag & M_PID_EXE_FLAG) && (gStMotorRunState[motorNum].pidNoExeTime >= 500)) //��ʱ��pidδִ�У������ǻ�ȡ�ٶ�ֵʧ�ܵ���
    {
        gStMotorRunState[motorNum].pidNoExeTime = 0;
        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;//�ر�pid����
        rt_kprintf("M%d PID exe failed!\r\n", motorNum);
    }
    
    //����pid����
    if(M_PID_END_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag)
    {
        if(lExeFlag & M_R_ACCORD_L_FLAG)   //�ҵ������������־
        {
            if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            {
                gStMotorRunState[M_RIGHT].pidCurrentStartFlag = M_PID_END_FLAG; //�����ҵ��pid����
            }
        }
        else if(lExeFlag & M_FOUR_ALONE_FLAG)   //�ĵ��������־
        {
            if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            {
                gStMotorRunState[M_RIGHT].pidCurrentStartFlag = M_PID_END_FLAG; //�����ҵ��pid����
            }
            if(M_PID_RUN_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            {
                gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag = M_PID_END_FLAG; //������һ���pid����
            }
            if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)
            {
                gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag = M_PID_END_FLAG; //������һ���pid����
            }
        }
        gStMotorRunState[motorNum].setCurrent = 0;
        if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0))
        {
            gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_IDLE_FLAG;
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_IDLE;    //�趨�ٶ�״̬תΪ����
            if (DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_8A)
            {
                rt_kprintf("M%d:exit PID!\r\n", motorNum);
            }
            ClearPidMem_incres(&gStMotorRunState[motorNum].pidIncresParas);   //���pid����
            if(gStMotorData[motorNum].flag & MOTOR_TURN_PID_TO_ACC)   //ת��pid�������ٶȲ�����
            {
                ClearPidMem_incres(&gStMotorRunState[motorNum].pidIncresParasOne);  //���pid����
            }
            SET_BRAKE(0);
            ClearPidMem_incres(&gStMotorRunState[M_BRAKE].pidIncresParas);    //���pid����
            gMotorAvaVelFilter = 0;
        }
        else if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8A)
        {
            rt_kprintf("id-%d-c:%d stop failed!\r\n", gStMotorData[motorNum].idx, liRealVel); 
        }
    }
}
/*****************************************************************************
 ��������  : ɲ��pid����
 �������  : uint32_t motorNum       ������
             int32_t tar             Ŀ���ٶ�
             int32_t liSetTarAcc     Ŀ����ٶ�
             int32_t setCurrent      �趨����ֵ
             uint32_t processTimeMs  �߳���ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��8��9��
*****************************************************************************/
void PIDBrakeAdjust(uint32_t motorNum, int32_t tar, int32_t liSetTarAcc)
{
    static uint8_t brakeFilter = 0; //��ɲ����Ҫ�˲�
    int32_t setAccThresholds = 200;
    uint16_t advanceDecTime = 200;  //��ǰɲ��ʱ��

    //ɲ��������ʱ���趨���ٶ���ֵ
    if(gStMotorData[M_BRAKE].speedGrad > 0)
    {
        setAccThresholds = gStMotorData[M_BRAKE].speedGrad;
    }

    if((gStMotorData[M_BRAKE].start_fault_time > 0) && (gStMotorData[M_BRAKE].start_fault_time < 1000))
    {
        advanceDecTime = gStMotorData[M_BRAKE].start_fault_time;
    }
    
    //���ٶ�ɲ������
    if(IS_UFOONE_FLAG_SET(UFOONE_BRAKE_ADVANCE_START)   //ɲ����ǰ����
        && ((ABS_VALUE(gStMotorRunState[motorNum].setTargetVel) == ABS_VALUE(VelCm_sToRpm(sys_para->CAR_RTinf.max_vel)) && (0 != sys_para->CAR_RTinf.max_vel))) //���ٶ�
        && ((sys_para->PC_Remote.decRemainTime < advanceDecTime) && (0 != sys_para->PC_Remote.decRemainTime)) //������ǰ������ʱ��
        && (sys_para->PC_Remote.setDec < -setAccThresholds) //�趨���ٶȴ�����ֵ
        && (ABS_VALUE(tar) >= 200))
    {
        brakeFilter = 0;
        if(700 != gBrakeValue)    //ɲ��λ���и���
        {
            SET_BRAKE(700);       //��ɲ��
        }
    }
    else if(((tar >= 200) && (liSetTarAcc < -setAccThresholds))   //Ŀ���ٶȣ�=200�����趨Ŀ����ٶȣ���ֵ����ǰ�����ٶ��Ҽ��ٶȴ����趨��ֵ
    || ((tar <= -200) && (liSetTarAcc > setAccThresholds))    //Ŀ���ٶȣ�=200�����趨Ŀ����ٶȣ���ֵ�������˼��ٶ��Ҽ��ٶȴ����趨��ֵ
      )   
    {
        brakeFilter = 0;
        if(700 != gBrakeValue)    //ɲ��λ���и���
        {
            SET_BRAKE(700);       //��ɲ��
        }
    }
    else if (((tar > 0) && (liSetTarAcc > -(setAccThresholds >> 1)))//Ŀ���ٶȣ�0�����趨Ŀ����ٶȣ���ֵһ��ĸ�ֵ����ǰ�����ٶ�
          || ((tar < 0) && (liSetTarAcc < (setAccThresholds >> 1))) //Ŀ���ٶȣ�0�����趨Ŀ����ٶȣ���ֵһ�룬�����˼��ٶ�
            )
    {
        if(700 == gBrakeValue)    //����ɲ��״̬
        {
            brakeFilter++;
            if(brakeFilter >=  10)
            {
                SET_BRAKE(0);         //��ɲ��
            }
        }
    }
    else if(700 == gBrakeValue)    //����ɲ��״̬
    {
        brakeFilter = 0;
    }
}
/*****************************************************************************
 ��������  : ���λ��ģʽ����
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ������ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��17��
*****************************************************************************/
void ProcessMotorRunModePos(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    HAL_StatusTypeDef lResult;
	
    
    // 1.��ʱ����ֱ��ת����
    if(gStMotorRunState[motorNum].runTotalTime >= 30000)
    {
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
    }

    // 2.������ı䴦��
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED == gStMotorRunState[motorNum].change_flag)//�����ı�
        {
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_MODE_STEP_START);
        }
        else if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //��canopen������
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
        }
        else
        {
            ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_DISABLE_VOLTAGE); //�л���ֹͣģʽ
            return;
        }
    }

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if(gStMotorData[motorNum].flag & ENABLE_AUTO_HOMMING)    //תѰ��
    {
        if(HOME_FLAG_FINISH != gStMotorRunState[motorNum].homeFinishFlag)
        {
            if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
            {
                ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_DISABLE_VOLTAGE); //�л���ֹͣģʽ
            }
            else
            {
                //ɲ��δ�������ʱ��ת�������ܱ���
                if(!((M_TURN == motorNum) && (gStMotorData[M_BRAKE].flag & ENABLE_AUTO_HOMMING)
                    && (gStMotorData[M_BRAKE].driverType != DRIVER_TYPE_NONE) 
                    && (gStMotorRunState[M_BRAKE].homeFinishFlag != HOME_FLAG_FINISH)))
                {
                    ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_HOMMING, MOTOR_RUN_MODE_STEP_START); //�л���Ѱ��ģʽ
                }
            }
            return;
        }
    }
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)
    {
        if(gCurtRunMode[motorNum].target_value < 0)
        {
            gStMotorRunState[motorNum].targetPos = gStMotorRunState[motorNum].PosInit + gCurtRunMode[motorNum].target_value 
                * (gStMotorRunState[motorNum].PosInit - gStMotorRunState[motorNum].limitPos1)  / 700;
        }
        else
        {
            gStMotorRunState[motorNum].targetPos = gStMotorRunState[motorNum].PosInit + gCurtRunMode[motorNum].target_value 
                * (gStMotorRunState[motorNum].limitPos2 - gStMotorRunState[motorNum].PosInit)  / 700;
        }
        if(M_TURN == motorNum)
        {
            gRespondState.turnTargetPos = (short)CountsToRemteValue(motorNum, gStMotorRunState[motorNum].targetPos);
        }
        if(DEBUG_DATA_TYPE_2 || (DEBUG_DATA_TYPE_87 && (M_TURN == motorNum)) || DEBUG_DATA_TYPE_8F)
        {
            
            if(DEBUG_DATA_TYPE_87 || DEBUG_DATA_TYPE_8F)
            {
                rt_kprintf("M%d tPos: %d.\r\n", motorNum, gRespondState.turnTargetPos);
            }
            else
            {
                rt_kprintf("M%d tPos: %d.\r\n", motorNum, gStMotorRunState[motorNum].targetPos);
            }
        }
        if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //��canopen������
        {
            if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG) //תʹ��
            {
                if(DISABLE == gStMotorRunState[motorNum].enableFlag)
                {
                    GPIOEnableMotor(motorNum, ENABLE);
                }
                *pRunStep = MOTOR_RUN_COMMON_STEP3;
            }
            //��λ�����������������
            else if((gStMotorData[motorNum].flag & MOTOR_READ_POS_FLAG)
                && (gStMotorData[motorNum].flag & MOTOR_PWM_CONTRL_MODE))  //pwm���Ʒ�ʽ
            {
                gStMotorRunState[motorNum].runDelayTime = 200;
                gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_SET_FINISH;
                *pRunStep = MOTOR_RUN_COMMON_STEP6;
            }
            else
            {
                /*PWMUpdateMotorPulse(motorNum, RT_FALSE);  //���µ�ǰλ��
                //ת����
                if((!(gStMotorData[motorNum].flag & MOTOR_DIR_FLAG))
                    && (((DIR_CCW != gStMotorRunState[motorNum].dirFlag) && (gStMotorRunState[motorNum].curPos > gCurtRunMode[motorNum].target_value))
                    || ((DIR_CW != gStMotorRunState[motorNum].dirFlag) && (gStMotorRunState[motorNum].curPos < gCurtRunMode[motorNum].target_value))))
                {
                    if(gStMotorRunState[motorNum].curPos < gCurtRunMode[motorNum].target_value)
                    {
                        PWMSetMotorDir(motorNum, DIR_CW);
                    }
                    else
                    {
                        PWMSetMotorDir(motorNum, DIR_CCW);
                    }
                }*/
                lResult = CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(motorNum, gStMotorRunState[motorNum].targetPos);
                if((gStMotorData[motorNum].flag & MOTOR_PRESS_SENSOR_FLAG)
                    && (700 == gCurtRunMode[motorNum].target_value)) //תѹ���������ж�
                {
                    gStMotorRunState[motorNum].runDelayTime = 0;
                    *pRunStep = MOTOR_RUN_COMMON_STEP4;   //ѹ���ж�
                }
                else if(HAL_OK != lResult)
                {
                    gStMotorRunState[motorNum].runDelayTime = 0;
                    *pRunStep = MOTOR_RUN_COMMON_STEP5;   //�ȴ�����ִ�����
                    //*pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                }
                else
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                }
            }
        }
#ifdef ENABLE_CANOPEN_DRIVER
        else
        {
            if(POS_TYPE_TWO_ELECT == gCurtRunMode[motorNum].posType)
            {
                /*gCurtRunMode[motorNum].target_value = Limit(gCurtRunMode[motorNum].target_value,
                    gStMotorData[motorNum].infrared_limit1, gStMotorData[motorNum].infrared_limit2);*/
                /*if(DEBUG_DATA_TYPE_1)
                {
                    rt_kprintf("id-%d-real-pos: %d\r\n", gStMotorData[motorNum].idx, gCurtRunMode[motorNum].target_value);
                }*/
            }
            /*else if(DEBUG_DATA_TYPE_6)
            {
                rt_kprintf("id-%d-real-pos: %d\r\n", gStMotorData[motorNum].idx, gCurtRunMode[motorNum].target_value);
            }*/

            if(gStMotorData[motorNum].driverType == DRIVER_TYPE_QILING)
            {
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
            }
            else
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP1;
                if(IS_UFOONE_FLAG_SET(UFOONE_BRAKE_STOP_ACCORD_OIL) && (M_BRAKE == motorNum) && (700 == gBrakeValue))   ////ɲ��������ѹֹͣ
                {
                    *pRunStep = MOTOR_RUN_COMMON_STEP2;
                }
            }
            if(RT_FALSE == gStMotorRunState[motorNum].targetPosJudgeFlag)
            {
                if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &gStMotorRunState[motorNum].lastPosRecord))
                {
                    gStMotorRunState[motorNum].targetPosJudgeFlag = RT_TRUE;
                    gStMotorRunState[motorNum].posJudgeTimeCnt = 0;
                    gStMotorRunState[motorNum].posJudgeCnt = 0;
                }
            }
            ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PPM);
            CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(motorNum, 
                ConvertAngleToCounts(motorNum, gStMotorRunState[motorNum].targetPos, gCurtRunMode[motorNum].posType));
        }
#endif
    }
#ifdef ENABLE_CANOPEN_DRIVER
    else if((MOTOR_RUN_COMMON_STEP1 == *pRunStep) || (MOTOR_RUN_COMMON_STEP2 == *pRunStep))
    {
        if(MOTOR_RUN_COMMON_STEP2 == *pRunStep)   ////ɲ��������ѹֹͣ
        {
            if(AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO) >= ADC1_BRAKE_MAX_THRETHOLDS + 2)  //������ֵ��ѹ(��λ0.1Mpa)
            {
                if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &gStMotorRunState[motorNum].targetPos))
                {
                    CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(motorNum,
                        ConvertAngleToCounts(motorNum, gStMotorRunState[motorNum].targetPos, gCurtRunMode[motorNum].posType));
                    *pRunStep = MOTOR_RUN_COMMON_STEP1;
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("New brake pos:%d.\r\n", gStMotorRunState[motorNum].targetPos);
                    }
                }
            }
        }
        if(gStMotorRunState[motorNum].runDelayTime >= 500)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            if((OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep) 
                && (MOTOR_SET_NOTHING == gStMotorRunState[motorNum].targetPosVelSetFlag))
            {
                if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_STATUS_FLAG)
                    && (gStMotorRevData[motorNum].status & 0x0400))
                {
                    gStMotorRunState[motorNum].targetPosFlag = 1;
                    gStMotorRunState[motorNum].targetPosJudgeFlag = RT_FALSE;
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                }
                else
                {
                    MotorSendReadStatus(gStMotorData[motorNum].idx);
                }
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_STATUS_FLAG);
            }
            if(gStMotorData[motorNum].driverType == DRIVER_TYPE_PUSI)
            {
                MotorSendReadControlStatus(gStMotorData[motorNum].idx);
            }
        }
        if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_CTRL_STATUS_FLAG))
        {
            if(gStMotorRevData[motorNum].ctrlStatus & (DIVICE_CTROL_STATUS_EXT_STOP1 | DIVICE_CTROL_STATUS_EXT_STOP2))
            {
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                SetErrorCode(motorNum, ERROR_CODE_LIMIT_ABNORMAL, ERROR_L_NORMAL);
                if(gStMotorData[motorNum].flag & ENABLE_AUTO_HOMMING)    //����Ѱ��
                {
                    gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;
                }
            }
            else if(gStMotorRevData[motorNum].ctrlStatus & DIVICE_CTROL_STATUS_STALL)
            {
                gStMotorRunState[motorNum].errorCnt++;
                if(gStMotorRunState[motorNum].errorCnt > 3)
                {
                    gStMotorRunState[motorNum].errorCnt = 0;
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                    SetErrorCode(motorNum, ERROR_CODE_MOTOR_STALL, ERROR_L_HIHG);
                }
                else
                {
                    MotorSetMotorControlStatus(gStMotorData[motorNum].idx, DIVICE_CTROL_STATUS_STALL);  //���״̬λ
                }
            }
            CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
        }
    } 
#endif
    else if(MOTOR_RUN_COMMON_STEP3 == *pRunStep)
    {
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            if(gStMotorData[motorNum].flag & MOTOR_READ_POS_FLAG)  //��λ�����������������
            {
                if(DEBUG_DATA_TYPE_5)
                {
                    rt_kprintf("Cur:%d, new:%d.\r\n", gStMotorRunState[motorNum].curPos, gStMotorRunState[motorNum].lastPoweroffPos);
                }
                gStMotorRunState[motorNum].curPos = gStMotorRunState[motorNum].lastPoweroffPos;
            }
            else
            {
                PWMUpdateMotorPulse(motorNum, RT_FALSE);  //���µ�ǰλ��
            }
            //ת����
            if((gStMotorData[motorNum].flag & MOTOR_DIR_FLAG)
                && (((DIR_CCW != gStMotorRunState[motorNum].dirFlag) && (gStMotorRunState[motorNum].curPos > gStMotorRunState[motorNum].targetPos))
                || ((DIR_CW != gStMotorRunState[motorNum].dirFlag) && (gStMotorRunState[motorNum].curPos < gStMotorRunState[motorNum].targetPos))))
            {
                //PWMSetMotorOutputState(motorNum, DISABLE);    //��ֹpwm���
                if(gStMotorRunState[motorNum].curPos < gStMotorRunState[motorNum].targetPos)
                {
                    GPIOSetMotorDir(motorNum, DIR_CW);
                }
                else
                {
                    GPIOSetMotorDir(motorNum, DIR_CCW);
                }
            }
            //else
            {
                CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(motorNum, gStMotorRunState[motorNum].targetPos);
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
            }
        }
    }
    else if(MOTOR_RUN_COMMON_STEP4 == *pRunStep)    //ѹ���������궨��ѹ�������ж�
    {
        if(gStMotorRunState[motorNum].runDelayTime >= gStMotorData[motorNum].currentAdjustPeriod)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            gStMotorRunState[motorNum].infraPosRecord = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
            gStMotorRunState[motorNum].infraPosRecord = Limit(gStMotorRunState[motorNum].infraPosRecord, ADC1_IN4_LIMIT1, ADC1_IN4_LIMIT2);
            if(gStMotorRunState[motorNum].infraPosRecord < (gStUfoData.oilPressStand & 0xff))
            {
                gStMotorRunState[motorNum].limitPos2 += gStMotorData[motorNum].speedGrad * 3;
            }
            else if(gStMotorRunState[motorNum].infraPosRecord < ((gStUfoData.oilPressStand >> 8) & 0xff))
            {
                gStMotorRunState[motorNum].limitPos2 += gStMotorData[motorNum].speedGrad << 1;
            }
            else if(gStMotorRunState[motorNum].infraPosRecord < ((gStUfoData.oilPressStand >> 16) & 0xff))
            {
                gStMotorRunState[motorNum].limitPos2 += gStMotorData[motorNum].speedGrad;
            }
            else if(gStMotorRunState[motorNum].infraPosRecord >= ((gStUfoData.oilPressStand >> 24) & 0xff))
            {
                gStMotorRunState[motorNum].limitPos2 -= gStMotorData[motorNum].speedGrad;
            }
            if(gStMotorRunState[motorNum].limitPos2 >= gStMotorData[motorNum].pos_limit2)
            {
                gStMotorRunState[motorNum].limitPos2 = gStMotorData[motorNum].pos_limit2;
                if(gStMotorRunState[motorNum].infraPosRecord < ((gStUfoData.oilPressStand >> 8) & 0xff))
                {
                    gStMotorRunState[motorNum].infraPosCnt++;
                    if(gStMotorRunState[motorNum].infraPosCnt >= 5)
                    {
                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_BRAKE_ERROR, ERROR_L_NORMAL);
                    }
                }
                else
                {
                    gStMotorRunState[motorNum].infraPosCnt = 0;
                }
            }
            else if(gStMotorRunState[motorNum].limitPos2 <= gStMotorData[motorNum].initPos)
            {
                gStMotorRunState[motorNum].limitPos2 = gStMotorData[motorNum].initPos;
                SetErrorCode(motorNum, ERROR_CODE_MOTOR_BRAKE1_ERROR, ERROR_L_NORMAL);
            }
            else
            {
                gStMotorRunState[motorNum].infraPosCnt = 0;
            }
            if((gStMotorRunState[motorNum].infraPosRecord < ((gStUfoData.oilPressStand >> 16) & 0xff))
                || (gStMotorRunState[motorNum].infraPosRecord >= ((gStUfoData.oilPressStand >> 24) & 0xff)))
            {
                if(DEBUG_DATA_TYPE_5)
                {
                    rt_kprintf("Press(0.1MPa): %.1f, new pos:%d.\r\n", 
                        gStMotorRunState[motorNum].infraPosRecord, gStMotorRunState[motorNum].limitPos2);
                }
            }
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
        }
    }
    else if(MOTOR_RUN_COMMON_STEP5 == *pRunStep)
    {
        if(gStMotorRunState[motorNum].runDelayTime >= 10)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            lResult = CHANGE_MOTOR_TARGET_POS_WITH_JUDGE(motorNum, gStMotorRunState[motorNum].targetPos);
            if(HAL_OK == lResult)
            {
                gStMotorRunState[motorNum].errorCnt = 0;
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
            }
            else
            {
                gStMotorRunState[motorNum].errorCnt++;
                if(gStMotorRunState[motorNum].errorCnt > 50)
                {
                    gStMotorRunState[motorNum].errorCnt = 0;
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                    SetErrorCode(motorNum, ERROR_CODE_MOTOR_TURN_EXE_ERROR, ERROR_L_NORMAL);
                }
            }
        }
    }
    else if(MOTOR_RUN_COMMON_STEP6 == *pRunStep)
    {
        if(gStMotorRunState[motorNum].runDelayTime >= 10)
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
            PWMUpdateMotorPulse(motorNum, RT_FALSE);
            PWMSetMotorOutputState(motorNum, DISABLE);    //��ֹpwm���
            if(HAL_OK == Motor485ReadPos(motorNum, &gStMotorRunState[motorNum].lastPoweroffPos))
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP3;
            }
        }
    }

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
}
/*****************************************************************************
 ��������  : ���ֹͣ����
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ������ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��11��17��
*****************************************************************************/
void ProcessMotorRunModeStop(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;

    // 1.��ʱ����ֱ��ת����
    if(gStMotorRunState[motorNum].runTotalTime >= 30000)
    {
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
    }

    // 2.������ı䴦���ޣ�ֹͣ����
    /*if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        
    }*/

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if(OPERATION_DISABLE_VOLTAGE == *pRunStep)  //����������ѹֹͣ
    {
        *pRunStep = MOTOR_RUN_COMMON_STEP1;
        gStMotorRunState[motorNum].operStopStep = OPERATION_DISABLE_VOLTAGE;
        gStMotorRunState[motorNum].operTimedelay = 0x0fff;
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;//���ݷ��ص�״̬�Զ��л�
        }
    }
    else if(OPERATION_QUCIK_STOP == *pRunStep)  //����ֹͣ
    {
        *pRunStep = MOTOR_RUN_COMMON_STEP1;
        gStMotorRunState[motorNum].operStopStep = OPERATION_QUCIK_STOP;
        gStMotorRunState[motorNum].operTimedelay = 0x0fff;
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;//���ݷ��ص�״̬�Զ��л�
        }
    }
    else if(MOTOR_RUN_COMMON_STEP1 == *pRunStep)
    {
        if(OPERATION_STOP_FINISH == gStMotorRunState[motorNum].operStopStep)
        {
            *pRunStep = MOTOR_RUN_COMMON_STEP2;   //��Ŀ���ٶ�/�����л���0
        }
    }
    else if(MOTOR_RUN_COMMON_STEP2 == *pRunStep)
    {
#ifdef ENABLE_CANOPEN_DRIVER
        if((MOTOR_OPERATION_MODE_CST == gStMotorRunState[motorNum].operationMode)
            || (MOTOR_OPERATION_MODE_PTM == gStMotorRunState[motorNum].operationMode))
        {
            CHANGE_MOTOR_TARGET_CURRENT(motorNum, 0);
        }
        else
#endif
        {
            CHANGE_MOTOR_TARGET_VEL(motorNum, 0);
        }
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
    }
    else if(OPERATION_SLOW_STOP == *pRunStep) //����ֹͣ
    {
        #ifdef ENABLE_CANOPEN_DRIVER
        if((MOTOR_OPERATION_MODE_CSV == gStMotorRunState[motorNum].operationMode)
            || (MOTOR_OPERATION_MODE_PVM == gStMotorRunState[motorNum].operationMode))
        {
            gStMotorRunState[motorNum].setTargetVel = 0;
            if(OPERATION_MODE_SET_FINISH != gStMotorRunState[motorNum].operModeSetStep)
            {
                ChangeMotorControlMode(motorNum, gStMotorRunState[motorNum].operationMode);
            }
            *pRunStep = OPERATION_SLOW_STOP_WAIT;
        }
        else if((MOTOR_OPERATION_MODE_CST == gStMotorRunState[motorNum].operationMode)
            || (MOTOR_OPERATION_MODE_PTM == gStMotorRunState[motorNum].operationMode))
        {
            if(OPERATION_MODE_SET_FINISH != gStMotorRunState[motorNum].operModeSetStep)
            {
                ChangeMotorControlMode(motorNum, gStMotorRunState[motorNum].operationMode);
            }
            *pRunStep = OPERATION_CURRENT_SLOW_STOP_WAIT;
        }
        else
        #endif
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
        }
    }
    else if(OPERATION_STOP_ADJUST == *pRunStep) //����ģʽ�жϵ���Ƿ���ʵֹͣ
    {
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            if(MOTOR_SET_NOTHING == gStMotorRunState[motorNum].targetPosVelSetFlag)
            {
                if(gStMotorRunState[motorNum].operationStopJudgeFlag)
                {
                    gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;
                }
                else
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
                }
            }
        }
    }
    else if(OPERATION_SLOW_STOP_WAIT == *pRunStep) //����ֹͣ
    {
        if(gStMotorRunState[motorNum].runDelayTime >= gStMotorData[motorNum].speedAdjustPeriod)
        {
            if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
            {
                //CHANGE_MOTOR_TARGET_VEL(motorNum, GetSlopeSpeed(motorNum));
                if(0 == gStMotorRunState[motorNum].targetVel)
                {
                    CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, 0);
                    gStMotorRunState[motorNum].operationStopJudgeFlag = 1;  //��Ҫ�жϵ���Ƿ���ֹͣ
                    *pRunStep = OPERATION_STOP_ADJUST;   //�ж�ֹͣ
                }
                if(DEBUG_DATA_TYPE_5)
                {
                    rt_kprintf("id-%d-stop-vel: %d.\r\n", gStMotorData[motorNum].idx, 
                        gStMotorRunState[motorNum].targetVel);
                }
            }
            gStMotorRunState[motorNum].runDelayTime = 0;
        }
    } 
    else if(MOTOR_RUN_MODE_STEP_IDLE != *pRunStep)
    {
        if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG) //תʹ��
        {
            if(ENABLE == gStMotorRunState[motorNum].enableFlag)
            {
                CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, 0);
                GPIOEnableMotor(motorNum, DISABLE);
            }
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        }
        else
        {
            *pRunStep = OPERATION_SLOW_STOP;
        }
    }    

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
}
/*****************************************************************************
 ��������  : ����ٶ�ģʽ����
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ������ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2019��12��10��
*****************************************************************************/
void ProcessMotorRunModeSpeed(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    uint32_t lu32Temp;
    int32_t liTemp = 0, liTemp1, liTemp2;

    // 1.��ʱ������
    /*if(gStMotorRunState[motorNum].runTotalTime >= )
    {
        gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;     //�л�������ģʽ
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        ChangeMotorRunState(motorNum, MOTOR_STATE_ERROR);
        SetErrorCode(motorNum, );
        return;
    }*/

    // 2.������ı䴦����
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED != gStMotorRunState[motorNum].change_flag)//�����ı�
        {
            gCurtRunMode[motorNum].target_value = 0;
        }
        if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
        {
            lu32Temp = gStMotorRunState[motorNum].runDelayTime;
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_COMMON_STEP1);
            gStMotorRunState[motorNum].runDelayTime = lu32Temp;
        }
    }

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)  //�趨�ٶ�
    {        
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen������
        {
            ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PVM);
            CHANGE_MOTOR_TARGET_VEL(motorNum, 0);
        }
        else if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG)     //תʹ��
        {
            if(DISABLE == gStMotorRunState[motorNum].enableFlag)
            {
                GPIOEnableMotor(motorNum, ENABLE);
            }
        }
        gStMotorRunState[motorNum].targetVelOffset = 0;
        *pRunStep = MOTOR_RUN_COMMON_STEP1;
    } 
    else if((MOTOR_RUN_COMMON_STEP1 == *pRunStep) || (MOTOR_RUN_COMMON_STEP2 == *pRunStep)
         || (MOTOR_RUN_COMMON_STEP4 == *pRunStep))    //�Ӽ���
    {
        if(MOTOR_RUN_COMMON_STEP1 == *pRunStep)
        {
            gStMotorRunState[motorNum].setTargetVel = Limit(gCurtRunMode[motorNum].target_value, 
                -gStMotorData[motorNum].limitSpeed, gStMotorData[motorNum].limitSpeed);
            if(0 != gStMotorRunState[motorNum].setTargetVel)
            {
                //�޶�Ŀ���ٶ���СΪѰ���ٶ�
                if(ABS_VALUE(gStMotorRunState[motorNum].setTargetVel) < ABS_VALUE(gStMotorData[motorNum].homingSpeed))
                {
                    if(IS_SAME_SYMBLE(gStMotorRunState[motorNum].setTargetVel, gStMotorData[motorNum].homingSpeed))
                    {
                        gStMotorRunState[motorNum].setTargetVel = gStMotorData[motorNum].homingSpeed;
                    }
                    else
                    {
                        gStMotorRunState[motorNum].setTargetVel = -gStMotorData[motorNum].homingSpeed;
                    }
                }
            }

            if(DEBUG_DATA_TYPE_1)
            {
                rt_kprintf("id-%d-real-speed: %d\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].setTargetVel);
            }

            *pRunStep = MOTOR_RUN_COMMON_STEP2;
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
            {
                if(gStMotorData[motorNum].relatedMotor >= M_TOTAL_NUM)
                {
                    *pRunStep = MOTOR_RUN_COMMON_STEP3;
                }
            }
        }
        if((MOTOR_RUN_COMMON_STEP2 == *pRunStep) && (OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep))
        {
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //�ȴ�ͬ���������ʹ��
            {
                if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                {
                    if(OPERATION_MODE_SET_FINISH != gStMotorRunState[gStMotorData[motorNum].relatedMotor].operModeSetStep)
                    {
                        return;
                    }
                }
            }
            if(gStMotorRunState[motorNum].runDelayTime >= gStMotorData[motorNum].speedAdjustPeriod)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                //liTemp = GetSlopeSpeed(motorNum);
                //ת����
                if((gStMotorData[motorNum].flag & MOTOR_DIR_FLAG)
                    && (((DIR_CCW != gStMotorRunState[motorNum].dirFlag) && (liTemp < 0))
                    || ((DIR_CW != gStMotorRunState[motorNum].dirFlag) && (liTemp > 0))))
                {
                    if(liTemp > 0)
                    {
                        GPIOSetMotorDir(motorNum, DIR_CW);
                        if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                        {
                            if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                            {
                                GPIOSetMotorDir(gStMotorData[motorNum].relatedMotor, DIR_CW);
                            }
                        }
                    }
                    else
                    {
                        GPIOSetMotorDir(motorNum, DIR_CCW);
                        if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                        {
                            if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                            {
                                GPIOSetMotorDir(gStMotorData[motorNum].relatedMotor, DIR_CCW);
                            }
                        }
                    }
                }
                else
                {
                    *pRunStep = MOTOR_RUN_COMMON_STEP4;
                    if((0 == gStMotorRunState[motorNum].setTargetVel) && (0 == liTemp))
                    {
                        CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, liTemp);
                        *pRunStep = MOTOR_RUN_COMMON_STEP5;
                    }
                    else
                    {
                        CHANGE_MOTOR_TARGET_VEL(motorNum, liTemp);
                    }
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //�ȴ�ͬ���������ʹ��
                    {
                        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                        {
                            if((0 == gStMotorRunState[gStMotorData[motorNum].relatedMotor].setTargetVel) && (0 == liTemp))
                            {
                                CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(gStMotorData[motorNum].relatedMotor, liTemp);
                                *pRunStep = MOTOR_RUN_COMMON_STEP5;
                            }
                            else
                            {
                                if(IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_CURRENT_FLAG)
                                    && IS_REV_MOTOR_DATA(gStMotorData[motorNum].relatedMotor, REV_MOTOR_CURRENT_FLAG))
                                {
                                    liTemp1 = gStMotorRevData[motorNum].current;
                                    if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                                    {
                                        liTemp1 = -liTemp1;
                                    }
                                    liTemp2 = gStMotorRevData[gStMotorData[motorNum].relatedMotor].current;
                                    if(gStMotorData[gStMotorData[motorNum].relatedMotor].flag & MOTOR_DIR_INVERSE_FLAG) //�������
                                    {
                                        liTemp2 = -liTemp2;
                                    }
                                    if(liTemp >= 0)
                                    {
                                        if(liTemp1 >= liTemp2 + 5000)
                                        {
                                            gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset++;
                                        }
                                        else if(liTemp1 + 5000 <= liTemp2)
                                        {
                                            gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset--;
                                        }
                                        else
                                        {
                                            gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset = 0;
                                        }
                                    }
                                    else
                                    {
                                        if(liTemp1 <= liTemp2 - 5000)
                                        {
                                            gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset--;
                                        }
                                        else if(liTemp1 - 5000 >= liTemp2)
                                        {
                                            gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset++;
                                        }
                                        else
                                        {
                                            gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset = 0;
                                        }
                                    }
                                }

                                gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset = Limit(
                                    gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset, -5, 5);
                                liTemp += gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVelOffset;
                                CHANGE_MOTOR_TARGET_VEL(gStMotorData[motorNum].relatedMotor, liTemp);
                            }
                        }
                    }
                }
            }
        }
        if(MOTOR_RUN_COMMON_STEP4 == *pRunStep) //������
        {
            if(gStMotorRunState[motorNum].runDelayTime >= (gStMotorData[motorNum].speedAdjustPeriod >> 1))
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP2;
                if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen������
                {
                    MotorSendReadAvarageCurrentCmd(gStMotorData[motorNum].idx);
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                    {
                        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                        {
                            MotorSendReadAvarageCurrentCmd(gStMotorData[gStMotorData[motorNum].relatedMotor].idx);
                        }
                    }
                }
                if(DEBUG_DATA_TYPE_4)
                {
                    rt_kprintf("Vel1:%d,c1:%d.\r\n", gStMotorRunState[motorNum].targetVel,
                        IS_REV_MOTOR_DATA(motorNum, REV_MOTOR_CURRENT_FLAG) ? gStMotorRevData[motorNum].current : -1);
                }
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CURRENT_FLAG);
                if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
                {
                    if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                    {
                        if(DEBUG_DATA_TYPE_4)
                        {
                            rt_kprintf("Vel2:%d,c2:%d.\r\n", gStMotorRunState[gStMotorData[motorNum].relatedMotor].targetVel,
                                IS_REV_MOTOR_DATA(gStMotorData[motorNum].relatedMotor, REV_MOTOR_CURRENT_FLAG) ? gStMotorRevData[gStMotorData[motorNum].relatedMotor].current : -1);
                        }
                        CLEAR_MOTOR_REV_DATA_FLAG(gStMotorData[motorNum].relatedMotor, REV_MOTOR_CURRENT_FLAG);
                    }
                }
            }
        }
    }
    else if(MOTOR_RUN_COMMON_STEP3 == *pRunStep)    //�ȴ��л�ģʽ
    {
        
    }
    else if(MOTOR_RUN_COMMON_STEP5 == *pRunStep)    //�ȴ��л�ģʽ
    {
        if(MOTOR_SET_NOTHING == gStMotorRunState[motorNum].targetPosVelSetFlag)
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
        }
    }
    else
    {
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
    }    

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
} 
/*****************************************************************************
 ��������  : ����ٶ�ģʽ����
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ������ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2020��6��8��
*****************************************************************************/
void ProcessMotorRunModeCurrent(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    uint32_t lu32Temp;

    // 1.��ʱ����, ��
    /*if(gStMotorRunState[motorNum].runTotalTime >= MOTOR_PECT_CONTINUE_TIME_MAX)
    {
        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_SLOW_STOP); //�л���ֹͣģʽ
        return;
    }*/

    // 2.������ı䴦��
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED != gStMotorRunState[motorNum].change_flag)    //ģʽ�ı�
        {
            gCurtRunMode[motorNum].target_value = 0;    //ͣ���
        }
        if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
        {
            lu32Temp = gStMotorRunState[motorNum].runDelayTime;
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_COMMON_STEP1);
            gStMotorRunState[motorNum].runDelayTime = lu32Temp;
        }
    }

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)
    {
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen������
        {
            ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PTM);
            if(0 == gStMotorRunState[motorNum].setCurrent)
            {
                CHANGE_MOTOR_TARGET_CURRENT(motorNum, 0);
            }
        }        
        else if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG)     //תʹ��
        {
            if(DISABLE == gStMotorRunState[motorNum].enableFlag)
            {
                GPIOEnableMotor(motorNum, ENABLE);
            }
        }

        *pRunStep = MOTOR_RUN_COMMON_STEP1;
        //ClearSCurveMems(&gStMotorScurveParas);
        gStMotorRunState[motorNum].runDelayTime = gStMotorData[motorNum].speedAdjustPeriod;
    }
    else if(MOTOR_RUN_COMMON_STEP1 == *pRunStep)
    {
        gStMotorRunState[motorNum].setTargetVel = Limit(gCurtRunMode[motorNum].target_value, 
            -gStMotorData[motorNum].limitSpeed, gStMotorData[motorNum].limitSpeed);
        if(0 != gStMotorRunState[motorNum].setTargetVel)
        {
            //�޶�Ŀ���ٶ���СΪѰ���ٶ�
            /*if(ABS_VALUE(gStMotorRunState[motorNum].setTargetVel) < ABS_VALUE(gStMotorData[motorNum].homingSpeed))
            {
                if(IS_SAME_SYMBLE(gStMotorRunState[motorNum].setTargetVel, gStMotorData[motorNum].homingSpeed))
                {
                    gStMotorRunState[motorNum].setTargetVel = gStMotorData[motorNum].homingSpeed;
                }
                else
                {
                    gStMotorRunState[motorNum].setTargetVel = -gStMotorData[motorNum].homingSpeed;
                }
            }*/
        }

        if(DEBUG_DATA_TYPE_1)
        {
            rt_kprintf("id-%d-real-speed: %d\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].setTargetVel);
        }

        *pRunStep = MOTOR_RUN_COMMON_STEP2;
        if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //ͬ���������
        {
            if(gStMotorData[motorNum].relatedMotor >= M_TOTAL_NUM)
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP4;
            }
        }
    }
    else if(MOTOR_RUN_COMMON_STEP2 == *pRunStep)
    {
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //�ȴ�ͬ���������ʹ��
            {
                if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
                {
                    if(M_LEFT == motorNum)
                    {
                        if((gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                            (DRIVER_TYPE_NONE != gStMotorData[M_LEFT_ONE].driverType))
                        {
                            if(OPERATION_MODE_SET_FINISH != gStMotorRunState[M_LEFT_ONE].operModeSetStep)
                            {
                                return;
                            }
                        }
                    }
                }
                if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                {
                    if(OPERATION_MODE_SET_FINISH != gStMotorRunState[gStMotorData[motorNum].relatedMotor].operModeSetStep)
                    {
                        //ClearSCurveMems(&gStMotorScurveParas);
                        return;
                    }
                    if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //��������
                    {
                        if(M_RIGHT == gStMotorData[motorNum].relatedMotor)
                        {
                            if((gStMotorData[M_RIGHT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                                (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType))
                            {
                                if(OPERATION_MODE_SET_FINISH != gStMotorRunState[M_RIGHT_ONE].operModeSetStep)
                                {
                                    return;
                                }
                            }
                        }
                    }
                }
            }
            
            if(0 != gStMotorRunState[motorNum].setTargetVel)
            {
                if(M_PID_IDLE_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag)
                {
                    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //���ֶ�����־
                    {
                        //���е��ʹ�ú�������ͬ��pid����
                        if((M_RIGHT == motorNum) || (M_LEFT_ONE== motorNum) || (M_RIGHT_ONE == motorNum))
                        {
                            gStMotorRunState[motorNum].pidIncresParas.kp = gStMotorRunState[M_LEFT].pidIncresParas.kp;
                            gStMotorRunState[motorNum].pidIncresParas.ki = gStMotorRunState[M_LEFT].pidIncresParas.ki;
                            gStMotorRunState[motorNum].pidIncresParas.kd = gStMotorRunState[M_LEFT].pidIncresParas.kd;
                        }
                    }
                    ClearPidMem_incres(&gStMotorRunState[motorNum].pidIncresParas);   //���pid����
                    if(gStMotorData[motorNum].flag & MOTOR_TURN_PID_TO_ACC)           //ת��pid�������ٶȲ�����
                    {
                        InitPidParas_incres(&gStMotorRunState[motorNum].pidIncresParasOne,
                            gStMotorData[M_TURN].kp, gStMotorData[M_TURN].ki, 0, 0);
                        //ClearPidMem_incres(&gStMotorRunState[M_TURN].pidIncresParas); //���pid����
                    }
                    SET_BRAKE(0);
                    ClearPidMem_incres(&gStMotorRunState[M_BRAKE].pidIncresParas);    //���pid����
                    
                    if (DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_8A)
                    {
                        rt_kprintf("M%d:enter PID!\r\nVel-kp:%.1f, ki:%.1f, kd:%.1f, slopeLimit:%d.\r\n", motorNum, gStMotorRunState[motorNum].pidIncresParas.kp,
                            gStMotorRunState[motorNum].pidIncresParas.ki, gStMotorRunState[motorNum].pidIncresParas.kd, gStMotorData[motorNum].profileVelocity);
                        if(gStMotorData[motorNum].flag & MOTOR_TURN_PID_TO_ACC)           //ת��pid�������ٶȲ�����
                        {
                            rt_kprintf("Acc-kp:%.1f, ki:%.1f, kd:%.1f, startCurrent:%d, slopeLimit:%d.\r\n", gStMotorRunState[motorNum].pidIncresParasOne.kp,
                                gStMotorRunState[motorNum].pidIncresParasOne.ki, gStMotorRunState[motorNum].pidIncresParasOne.kd, gStMotorData[motorNum].startCurrent, gStMotorData[motorNum].pos_limit2);
                        }
                        if(IS_UFOONE_FLAG_SET(UFOONE_SET_ACC_OFFSET_VALID)) //�趨Ŀ����ٶȲ�����Ч�����������ٶ����
                        {
                            rt_kprintf("AccOffset-kp:%d.\r\n", gStMotorData[M_LEFT].speedGrad);
                        }
                        rt_kprintf("IIt time:%d, current_balance:%d.\r\n", gStMotorData[motorNum].overCurrentTime, gStMotorData[M_RIGHT].speedGrad);
                        if((M_LEFT == motorNum) && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))
                        {
                            rt_kprintf("Brake-current:%d, vel:%d, acc:%d, test(0.1Mpa):%d.\r\n", gStMotorData[M_BRAKE].homingSpeed, gStMotorData[M_BRAKE].profileVelocity, gStMotorData[M_BRAKE].profileAcc,
                                IS_UFOONE_FLAG_SET(UFOONE_BRAKE_DEC_TEST_FLAG) ? (int)ADC1_BRAKE_TEST_THRETHOLDS : -1);
                        }
                        PrintfUfoVerAndSn();    //��ӡ�汾��
                    }
                    gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_RUN_FLAG;  //����pid����
                    gStMotorRunState[motorNum].pidDelayTime = gStMotorData[motorNum].currentAdjustPeriod;
                    gStMotorRunState[motorNum].pidPeriodTime = gStMotorData[motorNum].currentAdjustPeriod;
                    gStMotorRunState[motorNum].pidNoExeTime = 0;
                    gStMotorRunState[motorNum].pidRunFlag = RT_FALSE;
                    gStMotorRunState[motorNum].infraFaultTimeTotal = 0;
                    gStMotorRunState[motorNum].lastSetAcc = 0;
                    //gStMotorRunState[motorNum].curMVelFilter = 0;
                    gStMotorRunState[motorNum].setAcc = 0;
                    gStMotorRunState[motorNum].curAcc = 0;
                    gStMotorRunState[motorNum].skidFlag = 0;
                    gStMotorRunState[motorNum].skidOffCnt = 0;
                    gStMotorRunState[motorNum].skidTotalTime = 0;
                    gStMotorRunState[motorNum].pcLostRecoveryTime = 0;
                    gStMotorRunState[motorNum].setCarVelValidNum = 1;
                    gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_START;   //�趨�ٶ�״̬תΪ����
                    gStMotorRunState[motorNum].accCurrent = 0;
                    if(M_LEFT == motorNum)
                    {
                        gCarNavCurAcc = 0;
                        memset((uint8_t*)gCarVel, 0, sizeof(int32_t) * NAV_CAR_VEL_RECORD_NUM);
                        gCarVelValidNum = 0;
                        SimpleInitCarNavAccKalmanData;
                        gMotorAvaVelValidNum = 0;
                        gMotorAvaVelFilter = 0;
                        gMotorAvaSetCurrentPositiveChangeIndex = MVEL_RECORD_NUM;
                        gMotorAvaSetCurrentNegativeChangeIndex = MVEL_RECORD_NUM;
                        memset((uint8_t*)gMotorAvaVel, 0, sizeof(int32_t) * MVEL_RECORD_NUM);
                        memset((uint8_t*)gMotorAvaSetCurrentRecord, 0, sizeof(int32_t) * MVEL_RECORD_NUM);
                        SimpleInitMotorAccKalmanData;
                        gMotorCurAcc = 0;
                        gMotorVelErrPeak = 0;
                        gMotorVelErrValley = 0;
                        gMotorVelErrPeakTime = 0;
                        gMotorVelErrValleyTime = 0;
                        gMotorLastLastVelErr = 0;
                        gMotorLastVelErr = 0;
                        gMotorVelOscillationFlag = RT_FALSE;
                        gMotorAccMax = 0;
                        gMotorAccMin = 0;
                        gMotorAccTemp = 0;
                        gMotorAccOscillationFlag = RT_FALSE;
                        gMotorAccPeriodAvaAcc = 0;
                        gMotorAccPeriodAvaCurrent = 0;
                        gMotorUniPeriodAvaAcc = 0;
                        gMotorUniPeriodAvaCurrent = 0;
                        gCarEnterUniTime = 0xffffffff;
                    }
                    SimpleInitSetAccKalmanData(motorNum);
                    MotorOverloaderJudge(motorNum, RT_TRUE);
                } 
            } 
            *pRunStep = MOTOR_RUN_COMMON_STEP3;
        }
    }
    else if(MOTOR_RUN_COMMON_STEP3 == *pRunStep)
    {
        if(0 == gStMotorRunState[motorNum].setTargetVel)
        {
            if(M_PID_IDLE_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag)
            {
                if(gStMotorRunState[motorNum].runDelayTime >= 200) //�ȴ�һС�����˳����̣���ֹ��������������̵���
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //ֹͣ����
                }
            }
            else
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
            }
        }
        else
        {
            gStMotorRunState[motorNum].runDelayTime = 0;
        }
    }

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
}
/*****************************************************************************
 ��������  : �������λ��ģʽ����
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  ������ѯ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��8��15��
*****************************************************************************/
void ProcessMotorRunModeCurrentPos(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    int32_t liTemp;

    // 1.��ʱ����, ��
    /*if(gStMotorRunState[motorNum].runTotalTime >= MOTOR_PECT_CONTINUE_TIME_MAX)
    {
        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_SLOW_STOP); //�л���ֹͣģʽ
        return;
    }*/

    // 2.������ı䴦��
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED == gStMotorRunState[motorNum].change_flag)//�����ı�
        {
            if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
            {
                ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_COMMON_STEP1);
            }
        }
        else if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //��canopen������
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //����
        }
        else
        {
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_MODE_STEP_END);
        }
    }

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)
    {
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen������
        {
            ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PTM);
            if(0 == gStMotorRunState[motorNum].setCurrent)
            {
                CHANGE_MOTOR_TARGET_CURRENT(motorNum, 0);
            }
        }

        *pRunStep = MOTOR_RUN_COMMON_STEP1;
    }
    else if(MOTOR_RUN_COMMON_STEP1 == *pRunStep)
    {
        if(gCurtRunMode[motorNum].target_value > 0)
        {
            *pRunStep = MOTOR_RUN_COMMON_STEP2;
            gStMotorRunState[motorNum].setCurrent = gStMotorData[motorNum].homingSpeed;
        }
        else
        {
            *pRunStep = MOTOR_RUN_COMMON_STEP3;
            gStMotorRunState[motorNum].setCurrent = -gStMotorData[motorNum].startCurrent;
            if(0 == gStMotorRunState[motorNum].setCurrent)
            {
                gStMotorRunState[motorNum].setCurrent = -(gStMotorData[motorNum].homingSpeed >> 1);
            }
        }
        gStMotorRunState[motorNum].setCurrent = Limit(gStMotorRunState[motorNum].setCurrent,
            -gStMotorData[motorNum].limitCurrent, gStMotorData[motorNum].limitCurrent);

        CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, gStMotorRunState[motorNum].setCurrent);
        gStMotorRunState[motorNum].runDelayTime = 0;

        if(DEBUG_DATA_TYPE_1)
        {
            rt_kprintf("id-%d-real-current: %d, ticks:%d.\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].setCurrent, HAL_GetTick());
        }
    }
    else if(MOTOR_RUN_COMMON_STEP2 == *pRunStep)    //�̶�����ɲ������ʱ����
    {
        if((gStMotorRunState[motorNum].runDelayTime >= 5000) ||
            ((gStMotorRunState[motorNum].runDelayTime >= gStMotorData[motorNum].overCurrentTime)
            && (0 != gStMotorData[motorNum].overCurrentTime)))
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_END;    //�趨����0����������
        }
    }
    else if(MOTOR_RUN_COMMON_STEP3 == *pRunStep)    //�����˻��ж�
    {
        if (gStMotorRunState[motorNum].runDelayTime >= 500)
        {
            gStMotorRunState[motorNum].runDelayTime = 450;  //�״ν�������Ϊ500ms��������������Ϊ50ms
            if(0 == MotorReadVelocity(gStMotorData[motorNum].idx, &liTemp))
            {
                if(DEBUG_DATA_TYPE_2)
                {
                    rt_kprintf("stall velocity:%d, ticks:%d.\r\n", liTemp, HAL_GetTick());
                }
                gStMotorRunState[motorNum].errorCnt = 0;
                if(ABS_VALUE(liTemp) < 10)
                {
                    gStMotorRunState[motorNum].limit1Time += 1;
                    if(gStMotorRunState[motorNum].limit1Time >= 3)  //����3������ת��С��10rpm
                    {
                        *pRunStep = MOTOR_RUN_MODE_STEP_END;    //�趨����0����������
                    }
                }
                else
                {
                    gStMotorRunState[motorNum].limit1Time = 0;
                }
            }
            else
            {
                gStMotorRunState[motorNum].errorCnt++;
                if(gStMotorRunState[motorNum].errorCnt > MOTOR_CAN_TIMEOUT_TIMES)
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //ֹͣ����
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("id-%d error exit current-speed-mode, ticks:%d.\r\n", gStMotorData[motorNum].idx, HAL_GetTick());
                    }
                }
            }
        }
    }
    else if(MOTOR_RUN_MODE_STEP_END == *pRunStep)
    {
        gStMotorRunState[motorNum].setCurrent = 0;
        if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0))
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //ֹͣ����
            if(DEBUG_DATA_TYPE_2)
            {
                rt_kprintf("id-%d exit current-speed-mode, ticks:%d.\r\n", gStMotorData[motorNum].idx, HAL_GetTick());
            }
        }
    }

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
}

/*void ProcessMotorRunModeCommon(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step; 

    // 1.��ʱ����
    if(gStMotorRunState[motorNum].runTotalTime >= )
    {
        gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;     //�л�������ģʽ
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        ChangeMotorRunState(motorNum, MOTOR_STATE_ERROR);
        SetErrorCode(motorNum, );
        return;
    }

    // 2.������ı䴦��
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        
    }

    // 3.��ز��裨��Ҫ��������MOTOR_RUN_MODE_STEP_FINISH��
    if( == *pRunStep)
    {
        *pRunStep = ;
    }
    else if( == *pRunStep)
    {
        *pRunStep = ;
    }
    else
    {
        *pRunStep = ;
    }    

    // 4.���н������л��¸�����ģʽ
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorRunState(motorNum, MOTOR_STATE_STOP);
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //�Զ��л�����ģʽ
    }
}*/
/*****************************************************************************
 ��������  : �͵�ѹ���
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��6��28��
*****************************************************************************/
void LowVoltageCheck(void)
{
    int i;
    static uint8_t lowVolCnt = 0, lowVolKphCnt = 0;
    uint16_t vol = 0;
    uint16_t volCnt = 0; 
    int16_t scenario_max_vel; //����ٶ�
        
    //�ж��Ƿ��ڵ������״̬(�Ƿ����pid)
    gStBatteryState.motorRunFlag = 0;
    gStBatteryState.motorIdleTime += LOW_VOLTAGE_CHECK_PERIOD;
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))  //�ж϶�����ѹ
        {
            if(M_PID_RUN_FLAG == gStMotorRunState[i].pidCurrentStartFlag)
            {
                gStBatteryState.motorRunFlag = 1;
                gStBatteryState.motorIdleTime = 0;
            }

            //�ۼ����ж�����������ѹֵ
            if((OPERATION_MODE_SET_FINISH == gStMotorRunState[i].operModeSetStep) 
                && (gStMotorRevData[i].vol > 0))
            {
                vol += gStMotorRevData[i].vol;
                volCnt++;
            }
        }
    }

    //����״̬�ж���������ѹ�Ƿ����
    if(volCnt)
    {
        vol = vol * 10 / volCnt;  //ȡ��ֵ��ת������λ0.1V
        if(gStBatteryState.motorRunFlag && IS_ENABLE_SET_WARNING_CODE)
        {
            if(vol <= gStUfoData.minVol)   //�����������͵�ѹ�ж�
            {
                SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_RUNNING_LOW_VOLTAGE);
                SetErrorInfo(M_TOTAL_NUM, WARNING_CODE_RUNNING_LOW_VOLTAGE);
                gStErrorInfo.driverVoltage[M_LEFT] = gStMotorRevData[M_LEFT].vol;
                gStErrorInfo.driverVoltage[M_RIGHT] = gStMotorRevData[M_RIGHT].vol;
                gStErrorInfo.driverVoltage[M_LEFT_ONE] = gStMotorRevData[M_LEFT_ONE].vol;
                gStErrorInfo.driverVoltage[M_RIGHT_ONE] = gStMotorRevData[M_RIGHT_ONE].vol;
            }
        }
        if((0 == gStBatteryState.validFlag) && (0 == gStBatteryState.motorRunFlag)) //����ʱ����ֵ����ֹ��������
        {
            gStBatteryState.voltage = vol;  //��ص�ѹ��Чʱ������������ѹ��ֵ�����
            gStBatteryState.driverVolValidFlag = 1;
        }
        else
        {
            gStBatteryState.driverVolValidFlag = 0;
        }
    }
    
    //�жϵ�ص�ѹ�Ƿ����
    if(IS_NO_WARNING_OR_ERROR_CODE)
    {
        if(gStBatteryState.validFlag)
        {
            if(gStBatteryState.voltage <= gStUfoData.minVol) //��ص͵�ѹ�ж�
            {
                lowVolCnt++;
                if(lowVolCnt > 3)
                {
                    SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_LOW_VOLTAGE);
                }
            }
            else
            {
                lowVolCnt = 0;
            }
        }
    }

    //��ֹ״̬�жϵ�ǰ��ѹ�Ƿ�֧�ֵ�ǰ�����ٶȲ���
    if(!IS_UFOONE_FLAG_SET(UFOONE_DISABLE_MIN_VOL_KPH_FLAG))    //δ��ֹ��С��ѹ��Ӧʱ�����Ʊ�־ʱ
    {
        if((0 == gStBatteryState.motorRunFlag) && (gStBatteryState.motorIdleTime >= 500))
        {
            if(gStBatteryState.validFlag)
            {
                scenario_max_vel = ABS_VALUE(sys_para->PC_Remote.max_vel * 36 / 1000);
                vol = gStBatteryState.voltage / 10;
                if((scenario_max_vel >= 19) && (vol < gStUfoData.minVol20kph))
                {
                    lowVolKphCnt++;
                }
                else if((scenario_max_vel >= 39) && (vol < gStUfoData.minVol40kph))
                {
                    lowVolKphCnt++;
                }
                else if((scenario_max_vel >= 59) && (vol < gStUfoData.minVol60kph))
                {
                    lowVolKphCnt++;
                }
                else if((scenario_max_vel >= 79) && (vol < gStUfoData.minVol80kph))
                {
                    lowVolKphCnt++;
                }
                else if((scenario_max_vel >= 99) && (vol < gStUfoData.minVol100kph))
                {
                    lowVolKphCnt++;
                }
                else if((scenario_max_vel >= 109) && (vol < gStUfoData.minVol110kph))
                {
                    lowVolKphCnt++;
                }
                else if(lowVolKphCnt)
                {
                    lowVolKphCnt--;
                }
                if(lowVolKphCnt >= 3)
                {
                    lowVolKphCnt = 3;
                    if(IS_NO_WARNING_OR_ERROR_CODE)
                    {
                        SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_LOW_VOLTAGE_TO_KPH);
                    }
                }
                //�������棬�л����ó�������Զ��������
                else if((0 == lowVolKphCnt) && (WARNING_CODE_LOW_VOLTAGE_TO_KPH == gErrorResult))
                {
                    ClearErrorCode(M_TOTAL_NUM);
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : ���λ�ü��㳵��λ��
 �������  : uint32_t motorNum
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2024��5��29��
*****************************************************************************/
void MotorPosCalCarPos(void)
{
    uint32_t totalCnt, revCnt, i;
    float posTemp;

    totalCnt = 0;
    revCnt = 0;
    posTemp = 0;

    //ȡ���ж������λ��ֵ���
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))
        {
            if(DRIVER_TYPE_NONE != gStMotorData[i].driverType)
            {
                totalCnt++;
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_POS_FLAG))
                {
                    revCnt++;
                    posTemp += gStMotorRevData[i].pos;
                }
            }
        }
    }
    //����������е��λ��ֵ���������ȡ��־�����ƽ��ֵ�������㵽����
    if((revCnt > 0) && (totalCnt == revCnt))
    {
        CLEAR_MOTOR_REV_DATA_FLAG(M_LEFT, REV_MOTOR_POS_FLAG);
        CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT, REV_MOTOR_POS_FLAG);
        CLEAR_MOTOR_REV_DATA_FLAG(M_LEFT_ONE, REV_MOTOR_POS_FLAG);
        CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT_ONE, REV_MOTOR_POS_FLAG);
        if((gStMotorData[M_LEFT].counts > 0) && (gStUfoData.ratioVel > 0))
        {
            posTemp = posTemp / revCnt / gStMotorData[M_LEFT].counts;   //Ȧ��
            gMotorAvaPos = (int32_t)(posTemp * NAV_RATIO_SCALE / gStUfoData.ratioVel * 600); //���㵽������
        }
    }
}
/*****************************************************************************
 ��������  : �����ȡ����
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��5��6��
*****************************************************************************/
void MotorReadStep(uint32_t processTimeMs)
{
    static uint32_t tmpReadTimeCnt = 0;
    static uint32_t volReadTimeCnt = 0;
    static uint8_t motorTempReadFlag = 0;
    uint8_t readFlag = 0;
    uint8_t readVolFlag = 0;
    int32_t i, liTemp;

    tmpReadTimeCnt += processTimeMs;
    if(tmpReadTimeCnt >= 500)
    {
        tmpReadTimeCnt = 0;
        motorTempReadFlag = 1 - motorTempReadFlag;
        readFlag = 1;
        if(DEBUG_DATA_TYPE_98)
        {
            rt_kprintf("Motor pos:%d cm, vel: %d cm/s.\r\n", gMotorAvaPos / 10, gMotorAvaVelFilter);
        }
    }
    volReadTimeCnt += processTimeMs;
    if(volReadTimeCnt >= LOW_VOLTAGE_CHECK_PERIOD)
    {
        volReadTimeCnt = 0;
        readVolFlag = 1;
        LowVoltageCheck();  //�͵�ѹ���
    }
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if(0 == gStMotorRunState[i].startRemoteFlag)    //���δ����������ȡ
        {
            if(MOTOR_READ_FINISH != gStMotorRunState[i].readStep)
            {
                gStMotorRunState[i].readStep = MOTOR_READ_FINISH;
            }
            continue;
        }
        else if(OPERATION_MODE_SET_FINISH == gStMotorRunState[i].operModeSetStep)
        {
            if((gStMotorData[i].driverType == DRIVER_TYPE_STAND)
                || (gStMotorData[i].driverType == DRIVER_TYPE_QILING)
                || (gStMotorData[i].driverType == DRIVER_TYPE_NIMOTION)
                || (gStMotorData[i].driverType == DRIVER_TYPE_COMPLEY)
                || (gStMotorData[i].driverType == DRIVER_TYPE_FDK)
                || (gStMotorData[i].driverType == DRIVER_TYPE_KINCO_CAN))
            {
                if(readFlag)    //�¶�500ms��һ�Σ�������������¶Ƚ����ȡ
                {
                    if((0 == motorTempReadFlag) || ((motorTempReadFlag && (gStMotorData[i].driverType == DRIVER_TYPE_FDK))))
                    {
                        if(DEBUG_DATA_TYPE_3 || (DEBUG_DATA_TYPE_4 && (M_PID_IDLE_FLAG != gStMotorRunState[M_LEFT].pidCurrentStartFlag)))
                        {
                            if((0 == motorTempReadFlag) && IS_REV_MOTOR_DATA(i, REV_MOTOR_DRIVER_TMP_FLAG))
                            {
                                rt_kprintf("M%d,DriverTmp:%d.\r\n", i, gStMotorRevData[i].tmp);
                            }
                            else if(motorTempReadFlag && IS_REV_MOTOR_DATA(i, REV_MOTOR_TMP_FLAG))
                            {
                                rt_kprintf("M%d,MotorTmp:%d.\r\n", i, gStMotorRevData[i].motorTmp);
                                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_TMP_FLAG);
                            }
                        }
                        if((0 == motorTempReadFlag) && IS_REV_MOTOR_DATA(i, REV_MOTOR_DRIVER_TMP_FLAG))
                        {
                            gRespondState.driverTmp[i] = Limit(gStMotorRevData[i].tmp + 40, 0, 255);
                            CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_DRIVER_TMP_FLAG);
                        }
                        gStMotorRunState[i].readStep |= MOTOR_READ_TMP;
                    }
                }
                if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //�޹ߵ���־ʱֻ�����������λ��
                {
                    if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))  //�ɼ�������ѹ
                    {
                        gStMotorRevData[i].timeCnt += processTimeMs;
                        if(gStMotorRevData[i].timeCnt >= 10)
                        {
                            gStMotorRevData[i].timeCnt = 0;
                            if(!IS_REV_MOTOR_DATA(i, REV_MOTOR_POS_FLAG))   //������յ�λ�ú�����½�������
                            {
                                gStMotorRunState[i].readStep |= MOTOR_READ_POS;
                            }
                            else
                            {
                                MotorPosCalCarPos();
                            }
                        }
                    }
                }
                else if(DEBUG_DATA_TYPE_3 || 
                    ((M_TURN == i) && ((MOTOR_RUN_MODE_STEP_IDLE != gStMotorRunState[i].run_step)
                        || (gStMotorData[i].driverType == DRIVER_TYPE_QILING))))
                {
                    gStMotorRevData[i].timeCnt += processTimeMs;
                    if((gStMotorRevData[i].timeCnt >= gStMotorData[i].currentAdjustPeriod)
                        && (gStMotorRevData[i].timeCnt >= 37))
                    {
                        gStMotorRevData[i].timeCnt = 0;
                        if(M_TURN == i)
                        {
                            if(0 == MotorReadPosition(gStMotorData[i].idx, &liTemp))
                            {
                                gRespondState.turnCurPos = (short)CountsToRemteValue(i, liTemp);
                            }
                            if(!(DEBUG_DATA_TYPE_3 || DEBUG_DATA_TYPE_87 || DEBUG_DATA_TYPE_8F))
                            {
                                continue;
                            }
                        }
                        rt_kprintf("M%d", i);
                        if(IS_REV_MOTOR_DATA(i, REV_MOTOR_POS_FLAG))
                        {
                            if(DEBUG_DATA_TYPE_87 || DEBUG_DATA_TYPE_8F)
                            {
                                rt_kprintf(",P:%d", CountsToRemteValue(i, gStMotorRevData[i].pos));
                            }
                            else
                            {
                                rt_kprintf(",P:%d", gStMotorRevData[i].pos);
                            }
                        }
                        if(IS_REV_MOTOR_DATA(i, REV_MOTOR_SPEED_FLAG))
                        {
                            rt_kprintf(",V:%d", gStMotorRevData[i].speed);
                        }
                        if(IS_REV_MOTOR_DATA(i, REV_MOTOR_CURRENT_FLAG))
                        {
                            rt_kprintf(",C:%d", gStMotorRevData[i].current);
                        }
                        if(IS_REV_MOTOR_DATA(i, REV_MOTOR_VOL_FLAG))
                        {
                            rt_kprintf(",vol:%d", gStMotorRevData[i].vol);
                        }
                        rt_kprintf(".\r\n");
                        CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_POS_FLAG);
                        CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_SPEED_FLAG);
                        CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_CURRENT_FLAG);
                        CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_VOL_FLAG);
                        gStMotorRunState[i].readStep |= MOTOR_READ_POS | MOTOR_READ_VEL | MOTOR_READ_CURRENT | MOTOR_READ_VOL;
                    }
                }
                else
                {
                    if((gStUfoData.flag & UFO_ENABLE_SEND_MOTOR_DATA)
                        && (M_TURN != i))  //ʹ�ܷ��͵������
                    {
                        gStMotorRevData[i].timeCnt += processTimeMs;
                        if((gStMotorRevData[i].timeCnt >= gStMotorData[i].currentAdjustPeriod)
                            && (gStMotorRevData[i].timeCnt >= 10))
                        {
                            gStMotorRevData[i].timeCnt = 0;                        
                            CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_SPEED_FLAG);
                            CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_CURRENT_FLAG);
                            gStMotorRunState[i].readStep |= MOTOR_READ_VEL | MOTOR_READ_CURRENT;
                        }
                    }
                    if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))  //�ɼ�������ѹ
                    {
                        if(readVolFlag) //��ѹ100ms��һ��
                        {
                            CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_VOL_FLAG);
                            gStMotorRunState[i].readStep |= MOTOR_READ_VOL;
                        }
                    }
                }
            }
        }
        if(MOTOR_READ_FINISH == gStMotorRunState[i].readStep)
        {
            continue;
        }
        //ȡһ�����ݽ��ж�ȡ
        if(!(MOTOR_READING & gStMotorRunState[i].readStep))
        {
            gStMotorRunState[i].readStep |= MOTOR_READING;
            if(MOTOR_READ_POS & gStMotorRunState[i].readStep)
            {
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_POS_FLAG);
                MotorSendReadPosCmd(gStMotorData[i].idx);
                gStMotorRunState[i].readTimeCnt = 0;
            }
            else if(MOTOR_READ_CURRENT & gStMotorRunState[i].readStep)
            {
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_CURRENT_FLAG);
                MotorSendReadAvarageCurrentCmd(gStMotorData[i].idx);
                gStMotorRunState[i].readTimeCnt = 0;
            }
            else if(MOTOR_READ_VEL & gStMotorRunState[i].readStep)
            {
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_SPEED_FLAG);
                MotorSendReadVelocity(gStMotorData[i].idx);
                gStMotorRunState[i].readTimeCnt = 0;
            }
            else if(MOTOR_READ_TMP & gStMotorRunState[i].readStep)
            {
                if(motorTempReadFlag)
                {
                    CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_TMP_FLAG);
                    MotorSendReadTmpCmd(gStMotorData[i].idx);
                }
                else
                {
                    CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_DRIVER_TMP_FLAG);
                    MotorSendReadDriverTmpCmd(gStMotorData[i].idx);
                }
                gStMotorRunState[i].readTimeCnt = 0;
            }
            else if(MOTOR_READ_VOL & gStMotorRunState[i].readStep)
            {
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_VOL_FLAG);
                MotorSendReadVolCmd(gStMotorData[i].idx);
                gStMotorRunState[i].readTimeCnt = 0;
            }
            else
            {
                gStMotorRunState[i].readStep = MOTOR_READ_FINISH;
                gStMotorRunState[i].readTimeCnt = 0;
            }
        }
        //�ж��Ƿ��յ���Ӧ����
        else
        {
            gStMotorRunState[i].readTimeCnt += processTimeMs;
            if(gStMotorRunState[i].readTimeCnt >= CAN_WRITE_TIMEOUT_MS)
            {
                gStMotorRunState[i].readTimeCnt = 0;
                gStMotorRunState[i].readStep &= ~MOTOR_READING;
            }
            if(MOTOR_READ_POS & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_POS_FLAG))    //���յ�λ��
                {
                    if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //�޹ߵ���־ʱ���ݵ��λ�ü��㵱ǰ����λ��
                    {
                        MotorPosCalCarPos();
                    }
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_POS;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_CURRENT & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_CURRENT_FLAG))    //���յ�����
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_CURRENT;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_VEL & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_SPEED_FLAG))    //���յ�ת��
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_VEL;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_TMP & gStMotorRunState[i].readStep)
            {
                if((IS_REV_MOTOR_DATA(i, REV_MOTOR_DRIVER_TMP_FLAG)
                    || (IS_REV_MOTOR_DATA(i, REV_MOTOR_TMP_FLAG)))) //���յ������������¶�
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_TMP;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_VOL & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_VOL_FLAG))    //���յ���������ѹ
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_VOL;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : �������Ŀ��λ���ж�
 �������  : uint32_t motorNum       ������
             uint32_t processTimeMs  �߳�ʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��6��17��
*****************************************************************************/
void MotorReachTargetPosJudge(uint32_t motorNum, uint32_t processTimeMs)
{
    int32_t liTemp;
    gStMotorRunState[motorNum].posJudgeTimeCnt += processTimeMs;
    if(gStMotorRunState[motorNum].posJudgeTimeCnt >= 70)
    {
        gStMotorRunState[motorNum].posJudgeTimeCnt = 0;
        if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
        {
            if(DEBUG_DATA_TYPE_8F)
            {
                rt_kprintf("M%d,tp:%d,cp:%d,rp:%d.\r\n", motorNum, gStMotorRunState[motorNum].targetPos, liTemp, gStMotorRunState[motorNum].lastPosRecord);
            }
            if(ABS_VALUE(gStMotorRunState[motorNum].targetPos - liTemp) >= 40000)
            {
                if(ABS_VALUE(gStMotorRunState[motorNum].lastPosRecord - liTemp) < 1000)
                {
                    gStMotorRunState[motorNum].posJudgeCnt++;
                    if(gStMotorRunState[motorNum].posJudgeCnt > 3)
                    {
                        gStMotorRunState[motorNum].targetPosJudgeFlag = RT_FALSE;
                        SetErrorCode(motorNum, ERROR_CODE_ZX_EXE_ERROR, ERROR_L_HIHG);
                    }
                }
                else
                {
                    gStMotorRunState[motorNum].lastPosRecord = liTemp;
                    if(gStMotorRunState[motorNum].posJudgeCnt)
                    {
                        gStMotorRunState[motorNum].posJudgeCnt--;
                    }
                }
            }
            else
            {
                gStMotorRunState[motorNum].lastPosRecord = liTemp;
                if(gStMotorRunState[motorNum].posJudgeCnt)
                {
                    gStMotorRunState[motorNum].posJudgeCnt--;
                }
            }
        }
    }
}
/*****************************************************************************
 ��������  : ������ƴ������
 �������  : void* parameter
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2018��11��21��
*****************************************************************************/
void MotorControlEntry(TickType_t curTime)
{
    static uint32_t l_lastTime = 0;
    static uint32_t l_tick_cnt_one = 0;
    int i, lUploadFlag = RT_FALSE;

    l_lastTime = curTime - l_lastTime;

    if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //�˳����Ͻ���ֹͣ״̬���ٽ��й����Իָ�
    {
        ErrorDealAndRecovery(l_lastTime);   //���ϴ�����ָ�
    }

    OldTestProcess(l_lastTime);     //�ϻ�����
    
#ifdef ENABLE_CANOPEN_DRIVER
    MotorReadProcess(); //�첽��ȡ�������
    MotorReadStep(l_lastTime);    //������ݶ�ȡ��������
#endif

    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        //�ȴ�����Զ��֡����
        if((0 == gStMotorRunState[i].startRemoteFlag)
            && (MOTOR_RUN_MODE_POWER_OFF != gCurtRunMode[i].run_mode))
        {
            continue;
        }
        //ת��λ�ж�
        if((gStMotorRunState[i].targetPosJudgeFlag) && (gStUfoData.flag & UFO_ENABLE_POS_JUDEGE))//ʹ�ܵ�λλ���ж�
        {
            MotorReachTargetPosJudge(i, l_lastTime);
        }
        //�ȴ�ת��ɲ��Ԥ�����
        if(gStMotorData[i].flag & MOTOR_USE_PI7_POWER)    //ʹ��PI7��Դ���ƶ˿ڣ�ԭ��Դ���ŵ���Ԥ������
        {
            if(MOTOR_POWER_FLAG_ON != gPreChargeStateOne)
            {
                continue;
            }
        }
        //ת��δ���Ѱ��ʱ����ִ������
        if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))
        {
            if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //Ԥ���ʹ��
            {
                if(MOTOR_POWER_FLAG_ON != gPreChargeState)
                {
                    continue;
                }
            }
            if((DRIVER_TYPE_NONE != gStMotorData[M_TURN].driverType) && (gStMotorData[M_TURN].flag & ENABLE_AUTO_HOMMING))    //ʹ��Ѱ��
            {
                if(HOME_FLAG_FINISH != gStMotorRunState[M_TURN].homeFinishFlag)
                {
                    if((MOTOR_RUN_MODE_POWER_OFF != gCurtRunMode[i].run_mode) 
                        && (0 != gCurtRunMode[i].target_value))
                    {
                        continue;
                    }
                }
            }
        }
        gStMotorRunState[i].runTotalTime += l_lastTime;
        gStMotorRunState[i].runDelayTime += l_lastTime;
        switch(gCurtRunMode[i].run_mode)
        {
            case MOTOR_RUN_MODE_POWER_OFF:
                ProcessMotorRunModePowerOff(i);
                break;
            case MOTOR_RUN_MODE_HOMMING:
                ProcessMotorRunModeHomming(i, l_lastTime);
                break;
            case MOTOR_RUN_MODE_STOP:
                ProcessMotorRunModeStop(i, l_lastTime);
                break;
            case MOTOR_RUN_MODE_POS:
                ProcessMotorRunModePos(i, l_lastTime);
                break;
            case MOTOR_RUN_MODE_SPEED:
                ProcessMotorRunModeSpeed(i, l_lastTime);
                break;
            case MOTOR_RUN_MODE_FLAP_FIX_CURRENT:
                ProcessMotorRunModeCurrent(i, l_lastTime);
                break;
            case MOTOR_RUN_MODE_CURRENT_SPEED:
                ProcessMotorRunModeCurrentPos(i, l_lastTime);
                break;
            default:
    		    break;
        }
        //�л��ٶȻ�λ��
#ifdef ENABLE_CANOPEN_DRIVER
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[i].operModeSetStep)
        {
            if(MOTOR_SET_NOTHING != gStMotorRunState[i].targetPosVelSetFlag)
            {
                SetDriveMotorTargetValue(i, l_lastTime);                
            }
        }
#endif
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[i].operModeSetStep)
        {
            if(M_PID_IDLE_FLAG != gStMotorRunState[i].pidCurrentStartFlag)    //����pid����
            {
                if(MOTOR_RUN_MODE_FLAP_FIX_CURRENT != gCurtRunMode[i].run_mode)
                {
                    gStMotorRunState[i].pidCurrentStartFlag = M_PID_END_FLAG;
                }
                PIDCurrentAdjust(i, l_lastTime);
                lUploadFlag = RT_TRUE;
            }
        }
    }

    if(!lUploadFlag)
    {
        TestMotorDataUploadProcess(M_LEFT, RT_FALSE, 0, 0); //�ϴ��������ݣ�����̨����λ������
    }
    
    ChangeDriveMotorControlMode(l_lastTime);    //�л�������ģʽ���ٶ�

    l_tick_cnt_one += l_lastTime;
    if(l_tick_cnt_one >= MOTORSTATUS_ANALYSIS_TIME)
    {            
        MotorsPowerCheck(l_tick_cnt_one);//��Դ���µ���
        GetMotorStatus();//���״̬����
        MotorOverLoaderWaitRecovery();  //������صȴ��ָ�(�ȵ����ȴ)
        l_tick_cnt_one = 0;
    }

    l_lastTime = curTime;
}
/*****************************************************************************
 ��������  : UFO����
 �������  : uint8_t* cmdData  ��������
             uint8_t size      �����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��18��
*****************************************************************************/
void TestUfoControl(uint8_t* cmdData, uint8_t size)
{
    int16_t i16Temp, i16Temp1;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode;

    if(cmdData[1] >= M_TOTAL_NUM)
    {
        rt_kprintf("Invalid motor num: %d\r\n", cmdData[1]);
        return;
    }
    
    switch(cmdData[0])
    {
        case 0x01:  //���λ��ģʽ����
            if(gStMotorData[cmdData[1]].flag & MOTOR_CURRENT_ADJUST_SPEED) //��������ת��
            {
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_CURRENT_SPEED;
            }
            else
            {
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
            }
            i16Temp = *(int16_t*)(&cmdData[2]);
            //WriteInfoToLogBuf(EVENT_WRITE_LOG, "Motor%d target pos: %d\r\n", cmdData[3], u16Temp);
            //PWMSetMotorValue(cmdData[3], (int32_t)u16Temp - 700);
            //PWMSetMotorOutputState(cmdData[3], ENABLE);
            lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
            lMotorRunMode.target_value = Limit(i16Temp, -700, 700);
            rt_kprintf("Motor%d target pos: %d\r\n", cmdData[1], lMotorRunMode.target_value);
            SetMotorRunModeData(cmdData[1], &lMotorRunMode);
            break;
        case 0x02:  //����ٶ�ģʽ����
            if(gStMotorData[cmdData[1]].flag & MOTOR_CURRENT_ADJUST_SPEED) //��������ת��
            {
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_FLAP_FIX_CURRENT;
            }
            else
            {
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_SPEED;
            }
            i16Temp = *(int16_t*)(&cmdData[2]);
            lMotorRunMode.target_value = Limit(i16Temp, -gStMotorData[cmdData[1]].limitSpeed, gStMotorData[cmdData[1]].limitSpeed);
            rt_kprintf("Motor%d target vel: %d\r\n", cmdData[1], lMotorRunMode.target_value);
            SetMotorRunModeData(cmdData[1], &lMotorRunMode);
            break;
        case 0x03:  //����ϵ�ģʽ����
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_POWER_OFF;
            lMotorRunMode.target_value = 0;
            rt_kprintf("Motor%d power off.\r\n", cmdData[1]);
            SetMotorRunModeData(cmdData[1], &lMotorRunMode);
            break;
        case 0x10:  //pid��������
            i16Temp = *(int16_t*)(&cmdData[2]);
            i16Temp1 = *(int16_t*)(&cmdData[4]);
            if(size > 7)
            {
                gStMotorRunState[cmdData[1]].pidIncresParas.kd = *(int16_t*)(&cmdData[6]);
            }
            rt_kprintf("Motor%d kp: %d, ki: %d, kd: %d.\r\n", cmdData[1], i16Temp, i16Temp1, (int16_t)gStMotorRunState[cmdData[1]].pidIncresParas.kd);
            gStMotorRunState[cmdData[1]].pidIncresParas.kp = i16Temp;
            gStMotorRunState[cmdData[1]].pidIncresParas.ki = i16Temp1;
            break;
    }
}
/*****************************************************************************
 ��������  : ������������ϴ�����
 �������  : rt_bool_t pidStartFlag  pid������־
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��7��10��
*****************************************************************************/
static void TestMotorDataUploadProcess(uint32_t motorNum, rt_bool_t pidStartFlag, int32_t tarVel, int32_t curVel)
{
    static uint32_t lastUploadTime = 0;
    uint32_t lCurTime;
    uint8_t lRespondMsg[32] = {0};

    lCurTime = HAL_GetTick();
    
    if(gMotorTestDataUploadFlag)
    {
        if(pidStartFlag || ((RT_FALSE == pidStartFlag) && (lCurTime - lastUploadTime >= 10) && (M_LEFT == motorNum)))
        {
            lastUploadTime = lCurTime;

            lRespondMsg[0] = '*';       //����ͷ
            lRespondMsg[1] = sizeof(lRespondMsg);//�����ݳ���
            lRespondMsg[2] = 0x81;      //������
            lRespondMsg[3] = gStBatteryState.voltage & 0xff;    //��ѹ
            lRespondMsg[4] = (gStBatteryState.voltage >> 8) & 0xff;
            lRespondMsg[5] = gStBatteryState.current & 0xff;    //����
            lRespondMsg[6] = (gStBatteryState.current >> 8) & 0xff;
            lRespondMsg[7] = tarVel & 0xff; //Ŀ���ٶ�
            lRespondMsg[8] = (tarVel >> 8) & 0xff;
            lRespondMsg[9] = curVel & 0xff; //��ǰ�ٶ�
            lRespondMsg[10] = (curVel >> 8) & 0xff;
            lRespondMsg[11] = gStMotorRunState[motorNum].setAcc & 0xff; //�趨���ٶ�
            lRespondMsg[12] = (gStMotorRunState[motorNum].setAcc >> 8) & 0xff;
            lRespondMsg[13] = gStMotorRunState[motorNum].curAcc & 0xff; //��ǰ���ٶ�
            lRespondMsg[14] = (gStMotorRunState[motorNum].curAcc >> 8) & 0xff;
            lRespondMsg[15] = (gStMotorRunState[motorNum].setCurrent / 100) & 0xff; //����趨����
            lRespondMsg[16] = ((gStMotorRunState[motorNum].setCurrent / 100) >> 8) & 0xff;
            lRespondMsg[17] = gErrorResult & 0xff;  //������
            lRespondMsg[18] = (gErrorResult >> 8) & 0xff;
            if((0 == gErrorResult) && (OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep))
            {
                lRespondMsg[19] = 1;    //���ʹ��
            }
            else
            {
                lRespondMsg[19] = 0;
            }
            lRespondMsg[20] = gLockFlag;    //��բ״̬
            lCurTime = lCurTime - gMotorTestDataUploadStartTime;
            lRespondMsg[27] = lCurTime & 0xff;  //���ʱ��
            lRespondMsg[28] = (lCurTime >> 8) & 0xff;
            lRespondMsg[29] = (lCurTime >> 16) & 0xff;
            lRespondMsg[30] = (lCurTime >> 24) & 0xff;
            lRespondMsg[31] = '#';      //����β

            rt_ksendData(lRespondMsg, sizeof(lRespondMsg));
        }
    }  
}
/*****************************************************************************
 ��������  : ��ӡ���״̬��Ϣ
 �������  : void
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��7��8��
*****************************************************************************/
void PrintfMotorStaus(void)
{
    int i;
    uint8_t ctrlStatus;
    
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if(DRIVER_TYPE_NONE != gStMotorData[i].driverType)
        {
            rt_kprintf("M%d pf:%d,cf:%d,rste:%d,sr:%d,ef:%d,tpos:%d,cPos:%d,posf:%d,res:%d,tpjf:%d,tpvsf:%d,pcsf:%d.\r\n", i, gStMotorRunState[i].powerFlag, gStMotorRunState[i].change_flag,
                gStMotorRunState[i].run_step, gStMotorRunState[i].startRemoteFlag,
                gStMotorRunState[i].enableFlag, gStMotorRunState[i].targetPos, gStMotorRunState[i].curPos,
                gStMotorRunState[i].posFlag, gStMotorRunState[i].readStep, gStMotorRunState[i].targetPosJudgeFlag, gStMotorRunState[i].targetPosVelSetFlag,
                gStMotorRunState[i].pidCurrentStartFlag);
            rt_kprintf("M%d om:%d,osjf:%d,omss:%d,osns:%d,oss:%d,nel:%d,el:%d.\r\n", i, gStMotorRunState[i].operationMode, gStMotorRunState[i].operationStopJudgeFlag, gStMotorRunState[i].operModeSetStep,
                gStMotorRunState[i].operSetNewStep, gStMotorRunState[i].operStopStep, gStMotorRunState[i].newErrorLevel, gStMotorRunState[i].errorLevel);
        }
    }
    ctrlStatus = GetUfoControlStatus();
    rt_kprintf("ufo pcs%d,pcs1%d,ls:%d,link:0x%x(%s,%s)(%s,%s).\r\n", gPreChargeState, gPreChargeStateOne, gLockFlag, sys_para->CAR_RTinf.Link, (sys_para->CAR_RTinf.Link & LINK_PC) ? "PC" : "Remote", (sys_para->CAR_RTinf.Link & LINK_POWER_ON) ? "PowerOn" : "PowerOff",
        (ctrlStatus & 0x01) ? "Mid" : ((ctrlStatus & 0x02) ? "High" : "Low"), (ctrlStatus & 0x10) ? "M-Enable" : "M-Disable");
}
/*****************************************************************************
 ��������  : �ϻ�ָ�������
 �������  : uint8_t* cmdData
             uint8_t size     
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��04��08��
*****************************************************************************/
void OldTestRevCmdAnalysis(uint8_t* pData, uint8_t size)
{
    uint8_t cmd;
    uint8_t *dataBuf;
    
    if(0 == size)
    {
        return;
    }
    cmd = pData[0];           //�ϻ�����
    dataBuf = &pData[1];      //�ϻ�ָ��
    
    if(0 == cmd)       //��ͣ�ϻ�
    {
        gOldTest.oldTestStep |= OLD_TEST_PAUSE;           //�����ϻ�������ͣ��־λ
        gOldTest.oldTestStep &= ~OLD_TEST_START;          //����ϻ����Կ�ʼ��־λ
        gOldTest.oldTestStep &= ~OLD_TEST_POWER_NEW_CMD;  //����ϻ����Զ�����ָ���־λ
        gOldTest.oldTestStep &= ~OLD_TEST_TURN_NEW_CMD;   //����ϻ�����ת����ָ���־λ
        gOldTest.oldTestStep &= ~OLD_TEST_BRAKE_NEW_CMD;  //����ϻ�����ɲ����ָ���־λ
        gOldTest.moduleAgingFlg = NONE_AGING;             //��ģ�����ϻ�
        rt_kprintf("Set Stress Test(10-start,0-pause,else-start_once): %d.\r\n", cmd);
    }
    else
    {
        gOldTest.oldTestStep &= ~OLD_TEST_PAUSE;    //����ϻ�������ͣ��־λ
        gOldTest.oldTestStep &= ~OLD_TEST_END;      //����ϻ����Խ�����־λ
        
        if(10 != cmd)
        {
            gOldTest.oldTestStep |= OLD_TEST_NEED_UPDATE_CMD;   //�ϻ����Թ����У���Ҫ���������ϻ�ָ��
        }
        else
        {
            gOldTest.oldTestStep &= ~OLD_TEST_NEED_UPDATE_CMD;  //�ϻ����Թ����У�����Ҫ���������ϻ�ָ��
        }
        gOldTest.moduleAgingFlg = dataBuf[0];                   //�ϻ�ģ��ѡ��
        
        if(!(gOldTest.oldTestStep & OLD_TEST_START))
        {
            gOldTest.oldTestStep |= OLD_TEST_START;      //�״ν��յ��ϻ���������ϻ����Կ�ʼ��־λ
            rt_kprintf("Set Stress Test(10-start,0-pause,else-start_once): %d.\r\n", cmd);
        }
        else
        {
            if (gOldTest.moduleAgingFlg & POWER_AGING)    
            {
                gOldTest.oldTestStep |= OLD_TEST_POWER_NEW_CMD;  //�ٴν��յ��ϻ���������ϻ����Զ����������־λ
            }
            if (gOldTest.moduleAgingFlg & TURN_AGING)    
            {
                gOldTest.oldTestStep |= OLD_TEST_TURN_NEW_CMD;   //�ٴν��յ��ϻ���������ϻ�����ת���������־λ
            }
            if (gOldTest.moduleAgingFlg & BRAKE_AGING)    
            {
                gOldTest.oldTestStep |= OLD_TEST_BRAKE_NEW_CMD;  //�ٴν��յ��ϻ���������ϻ�����ɲ���������־λ
            }
        }
    }
    
    //�����ϻ�
    if (gOldTest.moduleAgingFlg & POWER_AGING)    
    {
        if(0 != dataBuf[1])
        {
            gOldTest.forwardTime = dataBuf[1];    
        }
        if(0 != dataBuf[3])
        {
            gOldTest.backTime = dataBuf[3];
        }
        
        if((0 != dataBuf[5]) || (0 != dataBuf[6]))
        {
            gOldTest.forwardVel = *(int16_t*)(&dataBuf[5]);   //оƬ�ڲ�ΪС��ģʽ
        }
        if((0 != dataBuf[7]) || (0 != dataBuf[8]))
        {
            gOldTest.backVel = *(int16_t*)(&dataBuf[7]);
        }

        if(0 != dataBuf[9])
        {
            gOldTest.powerGapTime = dataBuf[9];
        }
        
        if(!(gOldTest.oldTestStep & OLD_TEST_POWER_NEW_CMD))
        {
            rt_kprintf("Forward: %dcm/s %ds, backward: %dcm/s %ds, gaptime: %ds.\r\n", 
                gOldTest.forwardVel, gOldTest.forwardTime, gOldTest.backVel, gOldTest.backTime, gOldTest.powerGapTime);
        }
    }
    //ת���ϻ�
    if (gOldTest.moduleAgingFlg & TURN_AGING)    
    {
        if((0 != dataBuf[11]) || (0 != dataBuf[12]))
        {
            gOldTest.turnVel = *(int16_t*)(&dataBuf[11]);
        }
        if((0 != dataBuf[13]) || (0 != dataBuf[14]))
        {
            gOldTest.turnRange = *(uint16_t*)(&dataBuf[13]);
            gOldTest.turnRange = Limit(gOldTest.turnRange, 0, CMD_VALUE_MAX);
        }
        
        if(0 != dataBuf[15])
        {
            gOldTest.turnGapTime = dataBuf[15];
        }
        
        if(!(gOldTest.oldTestStep & OLD_TEST_TURN_NEW_CMD))
        {
            rt_kprintf("turnVel: %drpm, turnRange: %d, gaptime: %ds.\r\n", 
                gOldTest.turnVel, gOldTest.turnRange, gOldTest.turnGapTime);
        }
    }
    //ɲ���ϻ�
    if (gOldTest.moduleAgingFlg & BRAKE_AGING)    
    {
        if((0 != dataBuf[17]) || (0 != dataBuf[18]))
        {
            gOldTest.brakeVel = *(int16_t*)(&dataBuf[17]);
        }
        if((0 != dataBuf[19]) || (0 != dataBuf[20]))
        {
            gOldTest.brakeInitPos = *(uint16_t*)(&dataBuf[19]);
            gOldTest.brakeInitPos = Limit(gOldTest.brakeInitPos, 0, CMD_VALUE_MAX);
        }
        if((0 != dataBuf[21]) || (0 != dataBuf[22]))
        {
            gOldTest.brakeLimitPos = *(uint16_t*)(&dataBuf[21]);
            gOldTest.brakeLimitPos = Limit(gOldTest.brakeLimitPos, 0, CMD_VALUE_MAX);
        }

        if(0 != dataBuf[23])
        {
            gOldTest.brakeGapTime = dataBuf[23];
        }
        
        if(!(gOldTest.oldTestStep & OLD_TEST_BRAKE_NEW_CMD))
        {
            rt_kprintf("brakeVel: %drpm, brakeInitPos: %d, brakeLimitPos: %d, gaptime: %ds.\r\n", 
                gOldTest.brakeVel, gOldTest.brakeInitPos, gOldTest.brakeLimitPos, gOldTest.brakeGapTime);
        }
    }
}

/*****************************************************************************
 ��������  : �ϻ����Խ���
 �������  : uint32_t processTimeMs
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2022��11��15��
*****************************************************************************/
static void OldTestProcess(uint32_t processTimeMs)
{
    static uint32_t lTimeCnt[MODULE_NUM] = {0, 0, 0};              //�ϻ���ʱ
    static uint32_t lOldTimesCnt[MODULE_NUM] = {0, 0, 0};          //�ϻ���������
    static AGING_STEP agingStep[MODULE_NUM] ={EXIT, EXIT, EXIT};   //�ϻ�����
    int32_t liTemp;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode = {0};      //��ʼ���ֲ���������ֹ����ֵ���ɿأ�����쳣
    float lfTemp;
    
    if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
    {
        if(!(gOldTest.oldTestStep & OLD_TEST_END))   
        {
            gOldTest.oldTestStep |= OLD_TEST_END;             //�����ϻ����Խ�����־λ��ȷ���ϻ���ʼ ת �ϻ���ͣʱ�Ž���
            gOldTest.oldTestStep &= ~OLD_TEST_START;          //����ϻ����Կ�ʼ��־λ
            gOldTest.oldTestStep &= ~OLD_TEST_POWER_NEW_CMD;  //����ϻ����Զ�����ָ���־λ
            gOldTest.oldTestStep &= ~OLD_TEST_TURN_NEW_CMD;   //����ϻ�����ת����ָ���־λ
            gOldTest.oldTestStep &= ~OLD_TEST_BRAKE_NEW_CMD;  //����ϻ�����ɲ����ָ���־λ
            gOldTest.moduleAgingFlg = NONE_AGING;             //��ģ�����ϻ�
            
            if (agingStep[POWER] != EXIT)                     //�����ϻ��ѿ���
            {
                agingStep[POWER] = EXIT;
                UfoLeftRightControl(0, &lMotorRunMode);       //�ϻ�������ת������Ϊ0
                rt_kprintf("Exit Power Stress Test, speed: 0 m/s.\r\n");
            }

            if (agingStep[TURN] != EXIT)                      //ת���ϻ��ѿ���
            {
                agingStep[TURN] = EXIT;
                memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = 0;
                SetMotorRunModeData(M_TURN, &lMotorRunMode);  //�ϻ�������ת��λ������Ϊ0(ת����λ)
                rt_kprintf("Exit Turn Stress Test, turnPos: 0.\r\n");
            }

            if (agingStep[BRAKE] != EXIT)                     //ɲ���ϻ��ѿ���
            {
                agingStep[BRAKE] = EXIT;
                memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = 0;
                SetMotorRunModeData(M_BRAKE, &lMotorRunMode); //�ϻ�������ɲ��λ������Ϊ0(ɲ����λ)
                rt_kprintf("Exit Brake Stress Test, brakePos: 0.\r\n");
            }
        }
        return;
    }
    
    if(gOldTest.oldTestStep & OLD_TEST_START)   //�ϻ����Կ�ʼ
    {
        //�����ϻ�
        if (gOldTest.moduleAgingFlg & POWER_AGING)
        {
            if (EXIT == agingStep[POWER])
            {
                lTimeCnt[POWER] = 0;
                agingStep[POWER] = START;
            }
            else if (START == agingStep[POWER])
            {
                if(lTimeCnt[POWER] >= gOldTest.powerGapTime * 1000)
                {
                    lTimeCnt[POWER] = 0;
                    lOldTimesCnt[POWER]++;
                    gSteeringAngleVelDiff = 0;
                    rt_kprintf("Power Stress Test Count: %d.\r\n", lOldTimesCnt[POWER]);
                    UfoLeftRightControl(VelCm_sToRpm(gOldTest.forwardVel), &lMotorRunMode);
                    agingStep[POWER] = STEP1;
                    rt_kprintf("Begin Forward, speed: %.2f m/s.\r\n", gOldTest.forwardVel / 100.0f);
                }
            }
            else if (STEP1 == agingStep[POWER])
            {
                if(lTimeCnt[POWER] >= gOldTest.forwardTime * 1000)
                {
                    lTimeCnt[POWER] = 0;
                    UfoLeftRightControl(0, &lMotorRunMode);
                    agingStep[POWER] = STEP2;
                    rt_kprintf("End Forward, speed: 0 m/s.\r\n");
                }
            }
            else if (STEP2 == agingStep[POWER])
            {
                if(lTimeCnt[POWER] >= gOldTest.powerGapTime * 1000)
                {
                    lTimeCnt[POWER] = 0;
                    UfoLeftRightControl(VelCm_sToRpm(gOldTest.backVel), &lMotorRunMode);
                    agingStep[POWER] = STEP3;
                    rt_kprintf("Begin Backward, speed: %.2f m/s.\r\n", gOldTest.backVel / 100.0f);
                }
            }
            else if (STEP3 == agingStep[POWER])
            {
                if(lTimeCnt[POWER] >= gOldTest.backTime * 1000)
                {
                    lTimeCnt[POWER] = 0;
                    UfoLeftRightControl(0, &lMotorRunMode);
                    rt_kprintf("End Backward, speed: 0 m/s.\r\n");
                    
                    if(gOldTest.oldTestStep & OLD_TEST_NEED_UPDATE_CMD)   //�´��ϻ���Ҫ�յ�����ı�־
                    {
                        if(gOldTest.oldTestStep & OLD_TEST_POWER_NEW_CMD) //�յ��µ�����
                        {
                            gOldTest.oldTestStep &= ~OLD_TEST_POWER_NEW_CMD;  //����ϻ����Զ�����ָ���־λ
                            agingStep[POWER] = START;                         //ת��һ����
                        }
                        else
                        {
                            gOldTest.oldTestStep |= OLD_TEST_PAUSE;       //ת��ͣ����Ϊ�����ϻ���ת���ϻ���ɲ���ϻ�����ͬһ��ָ����Ϣ�ڣ�����ͳһ��ͣ
                            agingStep[POWER] = EXIT;
                        }
                    }
                    else
                    {
                        agingStep[POWER] = START;                         //ת��һ����
                    }
                    
                    if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                    {
                        rt_kprintf("Power Stress Test End: net abnormal.\r\n");
                    }
                }
            }
            else
            {
                ;
            }
            
            lTimeCnt[POWER] += processTimeMs;
        }
        else
        {
            if (agingStep[POWER] != EXIT)
            {
                agingStep[POWER] = EXIT;
                UfoLeftRightControl(0, &lMotorRunMode);       //�ϻ�ȡ����ת������Ϊ0
                rt_kprintf("Power Stress Test Cancel, speed: 0 m/s.\r\n");     
            }
        }
        
        //ת���ϻ�
        if (gOldTest.moduleAgingFlg & TURN_AGING)
        {
            if (EXIT == agingStep[TURN])
            {
                lTimeCnt[TURN] = 0;
                agingStep[TURN] = START;
            }
            else if (START == agingStep[TURN])
            {
                if(lTimeCnt[TURN] >= gOldTest.turnGapTime * 1000)
                {
                    lTimeCnt[TURN] = 0;
                    lOldTimesCnt[TURN]++;
                    rt_kprintf("Turn Stress Test Count: %d.\r\n", lOldTimesCnt[TURN]);

                    //����ת���ٶ�
                    if(0 == MotorSetProfileVelocity(gStMotorData[M_TURN].idx, gOldTest.turnVel, RT_TRUE))
                    {
                        rt_kprintf("Turn speed: %d, ", gOldTest.turnVel);
                    }
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                    lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                    lMotorRunMode.target_value = -gOldTest.turnRange;
                    if(gStMotorData[M_TURN].flag & MOTOR_DIR_INVERSE_FLAG)  //�������
                    {
                        lMotorRunMode.target_value = -lMotorRunMode.target_value;
                    }
                    SetMotorRunModeData(M_TURN, &lMotorRunMode);                       //��ת
                    agingStep[TURN] = STEP1;
                    rt_kprintf("Begin Turn Left, pos: %d.\r\n", -gOldTest.turnRange);
                }
            }
            else if (STEP1 == agingStep[TURN])
            {
                if(0 == MotorReadPosition(gStMotorData[M_TURN].idx, &liTemp))
                {
                    if(ABS_VALUE(gStMotorRunState[M_TURN].targetPos - liTemp) <= 2000) //ת��λ�жϣ�counts
                    {
                        lTimeCnt[TURN] = 0;
                        rt_kprintf("End Turn Left.\r\n");
                        agingStep[TURN] = STEP2;
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_TURN, &lMotorRunMode);
                    }
                }
            }
            else if (STEP2 == agingStep[TURN])
            {
                if(lTimeCnt[TURN] >= gOldTest.turnGapTime * 1000)
                {
                    lTimeCnt[TURN] = 0;
                    rt_kprintf("Begin Turn Right, pos: %d.\r\n", gOldTest.turnRange);
                    
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                    lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                    lMotorRunMode.target_value = gOldTest.turnRange;
                    if(gStMotorData[M_TURN].flag & MOTOR_DIR_INVERSE_FLAG)  //�������
                    {
                        lMotorRunMode.target_value = -lMotorRunMode.target_value;
                    }
                    SetMotorRunModeData(M_TURN, &lMotorRunMode);                         //��ת
                    agingStep[TURN] = STEP3;
                }
            }
            else if (STEP3 == agingStep[TURN])
            {
                if(0 == MotorReadPosition(gStMotorData[M_TURN].idx, &liTemp))
                {
                    if(ABS_VALUE(gStMotorRunState[M_TURN].targetPos - liTemp) <= 2000)   //ת��λ�жϣ�counts
                    {
                        lTimeCnt[TURN] = 0;
                        rt_kprintf("End Turn Right.\r\n");
                        
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_TURN, &lMotorRunMode);
                        if(gOldTest.oldTestStep & OLD_TEST_NEED_UPDATE_CMD)    //�´��ϻ���Ҫ�յ�����ı�־
                        {
                            if(gOldTest.oldTestStep & OLD_TEST_TURN_NEW_CMD)   //�յ��µ�����
                            {
                                gOldTest.oldTestStep &= ~OLD_TEST_TURN_NEW_CMD; //����ϻ�����ת����ָ���־λ
                                agingStep[TURN] = START;                        //ת��һ����
                            }
                            else
                            {
                                gOldTest.oldTestStep |= OLD_TEST_PAUSE;       //ת��ͣ����Ϊ�����ϻ���ת���ϻ���ɲ���ϻ�����ͬһ����������Ϣ�ڣ�����ͳһ��ͣ
                                agingStep[TURN] = EXIT;
                            }
                        }
                        else
                        {
                            agingStep[TURN] = START; //ת��һ����
                        }
                        
                        if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                        {
                            rt_kprintf("Turn Stress Test End: net abnormal.\r\n");
                        }
                    }
                }
            }
            else
            {
                ;
            }
            
            lTimeCnt[TURN] += processTimeMs;
        }
        else
        {
            if (agingStep[TURN] != EXIT)
            {
                agingStep[TURN] = EXIT;
                memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = 0;
                SetMotorRunModeData(M_TURN, &lMotorRunMode);                             //�ϻ�ȡ����ת��λ������Ϊ0(ת����λ)
                rt_kprintf("Turn Stress Test Cancel, turnPos: 0.\r\n");
            }
        }
        
        //ɲ���ϻ�
        if (gOldTest.moduleAgingFlg & BRAKE_AGING)
        {
            if (EXIT == agingStep[BRAKE])
            {
                lTimeCnt[BRAKE] = 0;
                agingStep[BRAKE] = START;
            }
            else if (START == agingStep[BRAKE])
            {
                if(lTimeCnt[BRAKE] >= gOldTest.brakeGapTime * 1000)
                {
                    lTimeCnt[BRAKE] = 0;
                    lOldTimesCnt[BRAKE]++;
                    rt_kprintf("Brake Stress Test Count: %d.\r\n", lOldTimesCnt[BRAKE]);

                    //����ɲ���ٶ�
                    if(0 == MotorSetProfileVelocity(gStMotorData[M_BRAKE].idx, gOldTest.brakeVel, RT_TRUE))
                    {
                        rt_kprintf("Brake speed: %d, ", gOldTest.brakeVel);
                    }
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                    lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                    lMotorRunMode.target_value = gOldTest.brakeInitPos;
                    SetMotorRunModeData(M_BRAKE, &lMotorRunMode);                             //��ɲ��
                    agingStep[BRAKE] = STEP1;
                    rt_kprintf("Begin Release Brake, pos: %d.\r\n", gOldTest.brakeInitPos);
                }
            }
            else if (STEP1 == agingStep[BRAKE])
            {
                if(0 == MotorReadPosition(gStMotorData[M_BRAKE].idx, &liTemp))
                {
                    if(ABS_VALUE(gStMotorRunState[M_BRAKE].targetPos - liTemp) <= 1000)  //ɲ����λ�жϣ�counts
                    {
                        lTimeCnt[BRAKE] = 0;
                        lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                        rt_kprintf("End Release Brake, oil:%.1fMpa.\r\n", lfTemp / 10);
                        agingStep[BRAKE] = STEP2;
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_BRAKE, &lMotorRunMode);        //ɲ��λ������Ϊ0(ɲ����λ)
                    }
                }
            }
            else if (STEP2 == agingStep[BRAKE])
            {
                if(lTimeCnt[BRAKE] >= gOldTest.brakeGapTime * 1000)
                {
                    lTimeCnt[BRAKE] = 0;
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                    lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                    lMotorRunMode.target_value = gOldTest.brakeLimitPos;
                    SetMotorRunModeData(M_BRAKE, &lMotorRunMode);                             //��ɲ��
                    agingStep[BRAKE] = STEP3;
                    rt_kprintf("Begin Tighten Brake, pos: %d.\r\n", gOldTest.brakeLimitPos);
                }
            }
            else if (STEP3 == agingStep[BRAKE])
            {
                if(0 == MotorReadPosition(gStMotorData[M_BRAKE].idx, &liTemp))
                {
                    if((ABS_VALUE(gStMotorRunState[M_BRAKE].targetPos - liTemp) <= 1000) && (lTimeCnt[BRAKE] >= gOldTest.brakeGapTime * 1000))  //��ɲ����λ�ж�(counts)�������ֽ�ɲ��һ��ʱ��
                    {
                        lTimeCnt[BRAKE] = 0;
                        lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                        rt_kprintf("End Tighten Brake, oil:%.1fMpa.\r\n", lfTemp / 10);
                        
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_BRAKE, &lMotorRunMode);             //ɲ��λ������Ϊ0(ɲ����λ)
                        if(gOldTest.oldTestStep & OLD_TEST_NEED_UPDATE_CMD)       //�´��ϻ���Ҫ�յ�����ı�־
                        {
                            if(gOldTest.oldTestStep & OLD_TEST_BRAKE_NEW_CMD)     //�յ��µ�����
                            {
                                gOldTest.oldTestStep &= ~OLD_TEST_BRAKE_NEW_CMD;  //����ϻ�����ɲ����ָ���־λ
                                agingStep[BRAKE] = START;                         //ת��һ����
                            }
                            else
                            {
                                gOldTest.oldTestStep |= OLD_TEST_PAUSE;           //ת��ͣ����Ϊ�����ϻ���ת���ϻ���ɲ���ϻ�����ͬһ����������Ϣ�ڣ�����ͳһ��ͣ
                                agingStep[BRAKE] = EXIT;
                            }
                        }
                        else
                        {
                            agingStep[BRAKE] = START;                             //ת��һ����
                        }

                        if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                        {
                            rt_kprintf("Brake Stress Test End: net abnormal.\r\n");
                        }
                    }
                }
            }
            else
            {
                ;
            }
			//ɲ���ϻ�����ʱ��ʵʱ����ѹֵ
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_92)
            {
                lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);  //��ȡ��ѹֵ0.01Mpa
                rt_kprintf("id-%d-t:%d,c:%d,m:%d,s:%d,tv:%d,sa:%d,ca:%d,mf:%d,r:%d,b:%d,sk:%d,oil:%d,t:%d.\r\n", 1,
                    gOldTest.brakeVel, 0, gStMotorRevData[M_BRAKE].speed, gStMotorRevData[M_BRAKE].current, 0, 
                    gStMotorData[M_BRAKE].profileAcc, 0, 0, 0, ((gStMotorData[M_BRAKE].flag & MOTOR_DIR_INVERSE_FLAG) ? (700-gTargetRunMode[M_BRAKE].target_value):(gTargetRunMode[M_BRAKE].target_value)), 0, (uint32_t)(lfTemp*10), HAL_GetTick());
            }
            
            lTimeCnt[BRAKE] += processTimeMs;
        }
        else
        {
            if (agingStep[BRAKE] != EXIT)
            {
                agingStep[BRAKE] = EXIT;
                memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = 0;
                SetMotorRunModeData(M_BRAKE, &lMotorRunMode);                            //�ϻ�ȡ����ɲ��λ������Ϊ0(ɲ����λ)
                rt_kprintf("Brake Stress Test Cancel, brakePos: 0.\r\n");
            }
        }
    }
}
/*****************************************************************************
 ��������  : ģ����λ�� �򿪵�Դ��PVM�ٶ�ģʽ��ť
 �������  : ��
 �������  : ��
 ��    ��  : ������
 ��    ��  : 2024��11��25��
*****************************************************************************/
void MyMotorSet(void)
{
	int _motor_num;
	for(_motor_num = M_LEFT; _motor_num < M_TOTAL_NUM; _motor_num++)
    {
		gStMotorRunState[_motor_num].powerFlag = MOTOR_POWER_FLAG_TIMECNT;
		gStMotorRunState[_motor_num].powerTime = 0;
		gStMotorRunState[_motor_num].resetFlag = 0;

		ChangeMotorControlMode(_motor_num,MOTOR_OPERATION_MODE_PVM);
	}
}

void MyMotorVelSet_test(void)
{
	int32_t l_temp = 0;
	l_temp = 0;//rpm
	CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(M_LEFT,l_temp);
}
