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
uint16_t gErrorRunStateFlag = 0;  //故障出现在运行状态标志
uint32_t gErrorTime = 0;
uint32_t gErrorEmergencyResult = 0;
uint16_t gErrorEmergencyMotorNum = 0;
ST_ERROR_INFO gStErrorInfo = {0};

uint8_t  gErrorTimes = 0;                 //一个周期内的错误计数
uint32_t gErrorStartTick;                 //错误开始时的tick
uint32_t gErrorRecoveryTime;              //错误恢复计时

static ST_MOTOR_RUN_MODE_DATA gTargetRunMode[M_TOTAL_NUM];
static ST_MOTOR_RUN_MODE_DATA gCurtRunMode[M_TOTAL_NUM];
ST_MOTOR_RUN_STATE_DATA gStMotorRunState[M_TOTAL_NUM];

ST_MOTOR_REV_DATA gStMotorRevData[M_TOTAL_NUM] = {0};
BackDeffMode gBackWheelDeffDriverMode = BACK_DEFF_NONE;//后轮差速驱动模式标志

int32_t gCarVel[NAV_CAR_VEL_RECORD_NUM];//惯导速度值记录数组
uint16_t gCarVelValidNum;               //存储有效个数
uint32_t gCarVelTime[NAV_CAR_VEL_RECORD_NUM];   //车速记录时刻
int32_t gCarOverVel;                    //超调车速最大值
uint32_t gCarOverVelTime;               //超调车速最大值时刻
uint32_t gCarEnterUniTime;              //进入匀速阶段的时刻
int32_t gCarNavCurAcc;                  //惯导当前加速度
int32_t gWheelSpeed[W_TOTAL_NUM];       //轮速
int32_t gMotorAvaVel[MVEL_RECORD_NUM];  //电机和速度
uint16_t gMotorAvaVelValidNum;          //存储有效个数
uint32_t gMotorAvaVelTime[MACC_CAL_PERIOD_NUM]; //电机速度记录时刻
int32_t  gMotorAvaSetCurrentRecord[MVEL_RECORD_NUM];//设定电流记录
int32_t  gMotorAvaSetCurrentPositiveChangeIndex;    //设定电流正变化索引值
int32_t  gMotorAvaSetCurrentNegativeChangeIndex;    //设定电流负变化索引值
int32_t gMotorAvaVelFilter = 0;         //电机滤波和速度
int32_t gMotorAvaPos = 0;               //电机平均位置
int32_t gMotorCurAcc;                   //根据电机速度计算的当前加速度
int32_t gMotorAccPeriodAvaAcc;          //加速阶段平均加速度
int32_t gMotorAccPeriodAvaCurrent;      //加速阶段平均电流
int32_t gMotorUniPeriodAvaAcc;          //匀速阶段平均加速度
int32_t gMotorUniPeriodAvaCurrent;      //匀速阶段平均电流
int32_t gMotorAccCurrent = 0;           //1m/s2加速度对应电流学习值
int32_t gMotorAccCurrentRecord[MACC_CURRENT_LEARN_NUM];//1m/s2加速度对应电流学习值记录
int32_t gMotorAccCurrentValidNum = 0;   //1m/s2加速度对应电流学习值记录有效个数

rt_bool_t gNoLoadFlag = RT_FALSE;       //无负载标志
int32_t gMotorVelErrPeak;               //电机速度误差波峰值记录
int32_t gMotorVelErrValley;             //电机速度误差波谷值记录
uint32_t gMotorVelErrPeakTime;          //电机速度误差波峰值时刻记录
uint32_t gMotorVelErrValleyTime;        //电机速度误差波谷值时刻记录
int32_t gMotorLastLastVelErr;           //电机上上次速度误差
int32_t gMotorLastVelErr;               //电机上次速度误差
rt_bool_t gMotorVelOscillationFlag;     //电机速度快速振荡标志
int32_t gMotorAccMax;                   //电机最大加速度
int32_t gMotorAccMin;                   //电机最小加速度
int32_t gMotorAccTemp;                  //acc缓存
uint32_t gMotorAccMaxTime;              //电机最大加速度对应时刻
uint32_t gMotorAccMinTime;              //电机最小加速度对应时刻
int32_t gMotorAccMaxCurrent;            //电机最大加速度对应的电流值
int32_t gMotorAccMinCurrent;            //电机最小加速度对应的电流值
rt_bool_t gMotorAccOscillationFlag;     //电机加速度快速振荡标志

int32_t gMotorVelFilter;                //电机和惯导滤波出来的电机速度
int32_t gCarVelFilter;                  //电机和惯导滤波出来的惯导速度
int32_t gAutoRatioVel;                  //动态速比
int32_t gSteeringAngleVelDiff;          //差速转向时转角对应的速度差值cm/s

ST_KALMAN_DATA  gStCarNavAccKalmanData; //惯导加速度卡尔曼滤波结构体
ST_KALMAN_DATA  gStMotorAccKalmanData;  //电机加速度卡尔曼滤波结构体

uint8_t gCurrentBuffer[USART7_REV_BUFFER_LEN];
uint16_t gCurrentIndex = 0;
uint8_t gPreChargeState = MOTOR_POWER_FLAG_OFF;
uint8_t gPreChargeStateOne = MOTOR_POWER_FLAG_OFF;
BrakeState gBrakeFlag = BRAKE_OFF;              //刹车标志
BrakeWaitState gBrakeWaitOnFlag = BRAKE_WAIT_NONE;     //等待刹车上电标志

uint16_t gBrakeValue = 0;                       //刹车强度，0到700,    1000表示断电
LockState gLockFlag = LOCK_OFF;                 //抱闸标志
uint32_t gPreChargeTimeCnt;
uint32_t gPreChargeTimeCntOne;
//SCurveParas gStMotorScurveParas;

rt_bool_t gMotorTestDataUploadFlag = RT_FALSE;  //测试数据上传标志
uint32_t gMotorTestDataUploadStartTime = 0;     //开始上传时刻

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
 功能描述  : 电机数据初始化
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月27日
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
    //初始化电机驱动器电源，抱闸电源输出端口，光电传感器输入端口
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
        if(gStMotorData[i].flag & MOTOR_USE_PI7_POWER)    //使用PI7电源控制端口，原电源引脚当作预充引脚
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

    if(gStUfoData.flag & UFO_ENABLE_LOCK)  //抱闸使能
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

    gMotorAccCurrent = gStMotorData[M_LEFT].startCurrent;   //1m/s2加速度对应电流值
    //最大加速度暂定3，对应转速下的加速度即3000/(22/3)s，对应加加速度取时间1.2s，最大加速度/1.2s
    //InitSCurveParas(&gStMotorScurveParas, gStMotorData[M_LEFT].limitSpeed, 410.0f, 340.0f, 4.0f);
}
/*****************************************************************************
 功能描述  : 切换当前电机运行模式
 输入参数  : uint32_t motorNum  电机序号
             uint8_t runMode    指定运行模式，如果为空闲，则自动指定下一步
             uint8_t runStep    指定运行步骤
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月15日
*****************************************************************************/
void ChangeMotorCurRunMode(uint32_t motorNum, uint8_t runMode, uint8_t runStep)
{
    uint8_t lCurRunMode = gCurtRunMode[motorNum].run_mode;
    
    //指定下一模式
    if(MOTOR_RUN_MODE_STEP_IDLE != runMode)
    {
        memcpy(&gCurtRunMode[motorNum], &gTargetRunMode[motorNum], sizeof(ST_MOTOR_RUN_MODE_DATA));
        gCurtRunMode[motorNum].run_mode = runMode;
        gStMotorRunState[motorNum].run_step = runStep;
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
    }
    //模式执行完毕则自动切换至下一模式，否则需等待模式执行完
    else if(MOTOR_RUN_MODE_STEP_IDLE == gStMotorRunState[motorNum].run_step)
    {
        memcpy(&gCurtRunMode[motorNum], &gTargetRunMode[motorNum], sizeof(ST_MOTOR_RUN_MODE_DATA));
	    gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
        //命令执行完转入空闲
        if(lCurRunMode == gTargetRunMode[motorNum].run_mode)
        {
            gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;
        }
        //切换至对应模式
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

    /*//非匀速拍动模式进入匀速拍动模式时，需要先寻零
    if((lCurRunMode != MOTOR_RUN_MODE_FLAP_FIX) && (lCurRunMode != MOTOR_RUN_MODE_HOMMING) 
        && (gCurtRunMode[motorNum].run_mode == MOTOR_RUN_MODE_FLAP_FIX) && (gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM))
    {
        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;    //需要重新寻零
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
 功能描述  : 后轮差速模式处理
 输入参数  : ST_MOTOR_RUN_MODE_DATA* lMotorRunMode
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年12月12日
*****************************************************************************/
static void BackDeffModeDeal(ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    //大转向时切换至最大差速模式
    if((ABS_VALUE(lMotorRunMode->target_value) >= 600) && (ABS_VALUE(gRespondState.turnCurPos) >= 600)
        && (ABS_VALUE(gMotorAvaVelFilter) <= FOUR_TURN_FRONT_ONLY_VEL + 50))
    {
        gBackWheelDeffDriverMode = BACK_DEFF_MAX;
    }
    //低速时切换至依转向值而定的差速驱动模式
    else if(BACK_DEFF_ACCORD_TURN != gBackWheelDeffDriverMode)
    {
        if((ABS_VALUE(gMotorAvaVelFilter) <= FOUR_TURN_FRONT_ONLY_VEL - 100)
            && (ABS_VALUE(VelRpmToCm_s(gStMotorRunState[M_LEFT].setTargetVel)) <= FOUR_TURN_FRONT_ONLY_VEL - 100))
        {
            gBackWheelDeffDriverMode = BACK_DEFF_ACCORD_TURN;
        }
    }
    //速度大于阈值时切换至无差速模式
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
 功能描述  : 强制设置电机运行模式参数
 输入参数  : uint32_t motorNum
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月9日
*****************************************************************************/
static void SetMotorRunModeDataForce(uint32_t motorNum, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }

    if(motorNum == M_TURN)
    {
        if(IS_UFOONE_FLAG_SET(UFOONE_TWO_WHEL_DEFF_DRIV))   //两轮差速驱动
        {
            BackDeffModeDeal(lMotorRunMode);
        }
    }

    if((M_BRAKE == motorNum) && (gStMotorData[M_BRAKE].flag & MOTOR_DIR_INVERSE_FLAG)
        && (!(gStMotorData[M_BRAKE].flag & MOTOR_CURRENT_ADJUST_SPEED)))  //电机反向
    {
        lMotorRunMode->target_value = Limit(lMotorRunMode->target_value, 0, 700);
        lMotorRunMode->target_value = 700 - lMotorRunMode->target_value;
    }

    if((M_LEFT == motorNum) || (M_RIGHT == motorNum) || (M_LEFT_ONE == motorNum) || (M_RIGHT_ONE == motorNum))
    {
        if(sys_para->CAR_RTinf.Link & LINK_PC)   //PC控制
        {
            RecordCarSetVelAndAcc(motorNum, lMotorRunMode->target_value, HAL_GetTick());
        }
    }

    //比较命令是否发生改变，未改变则直接返回
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
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_PARA_CHANGED;   //参数发生改变
    }
    else
    {
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_CHANGED;   //模式发生改变
    }
    
    memcpy(&gTargetRunMode[motorNum], lMotorRunMode, sizeof(ST_MOTOR_RUN_MODE_DATA));

    if(DEBUG_DATA_TYPE_6 || DEBUG_DATA_TYPE_87)
    {
        rt_kprintf("motor%d mode: %d, value: %d, posType: %d.\r\n", motorNum, lMotorRunMode->run_mode, lMotorRunMode->target_value, lMotorRunMode->posType);
    }

    ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE);    //自动切换运行模式

    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机给定相同命令
    {
        if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
        {
            if(M_LEFT == motorNum)
            {
                if((gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                    (DRIVER_TYPE_NONE != gStMotorData[M_LEFT_ONE].driverType))   //同步关联电机给定相同命令
                {
                    SetMotorRunModeData(M_LEFT_ONE, lMotorRunMode);
                }
            }
        }
        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
        {
            SetMotorRunModeData(gStMotorData[motorNum].relatedMotor, lMotorRunMode);
            if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
            {
                if(M_RIGHT == gStMotorData[motorNum].relatedMotor)
                {
                    if((gStMotorData[M_RIGHT_ONE].flag & MOTOR_RELATED_SYNCHRO) && 
                        (DRIVER_TYPE_NONE != gStMotorData[M_RIGHT_ONE].driverType))   //同步关联电机给定相同命令
                    {
                        SetMotorRunModeData(M_RIGHT_ONE, lMotorRunMode);
                    }
                }
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 设置电机运行模式参数
 输入参数  : uint32_t motorNum
             ST_MOTOR_RUN_MODE_DATA* lMotorRunMode  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月14日
*****************************************************************************/
void SetMotorRunModeData(uint32_t motorNum, ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }
    else if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //出故障并停止后不执行外部命令(但警告可以遥控，前提是切换过停止)
    {
        if(gErrorResult && (!(IS_WARNING_CODE(gErrorMotorNum) && (!(sys_para->CAR_RTinf.Link & LINK_PC)) && (sys_para->CAR_RTinf.Link & LINK_REV_STOP_CMD))))
        {
            return;
        }
    }

    SetMotorRunModeDataForce(motorNum, lMotorRunMode);
}
/*****************************************************************************
 功能描述  : 电源抱闸上电检查
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月18日
*****************************************************************************/
static void MotorsPowerCheck(uint32_t processTimeMs)
{
    uint8_t i, k; 

    //上下电等待延时后才置标志位
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        //上电计时
        if(MOTOR_POWER_FLAG_TIMECNT == gStMotorRunState[i].powerFlag)
        {
            gStMotorRunState[i].powerTime += processTimeMs;
            if(IS_NOT_CANOPEN_DRIVER(gStMotorData[i].driverType))    //非canopen驱动器
            {
                if(gStMotorRunState[i].powerTime >= gStMotorData[i].powerDelayTime)
                {
                    gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_ON;//上电完成
                    gStMotorRunState[i].powerTime = 0;
                }
            }
            else
            {
                if(gStMotorRunState[i].powerTime >= MOTOR_ALL_POWERING_TIMEOUT)
                {
                    gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_ON;//上电完成
                    gStMotorRunState[i].powerTime = 0;
                }
                //检查电机启动事件
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
        //下电计时
        else if(MOTOR_POWEROFF_FLAG_TIMECNT == gStMotorRunState[i].powerFlag)
        {
            gStMotorRunState[i].powerTime += processTimeMs;
            if((gStMotorRunState[i].powerTime >= MOTOR_ALL_POWEROFF_TIME) 
                && (gStMotorRunState[i].powerTime >= gStMotorData[i].powerDelayTime))   //增加下电计时必须长于上电计时的判断，因大电容驱动器下电比较慢
            {
                gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_OFF;//下电完成
                gStMotorRunState[i].powerTime = 0;
            }
        }
    }

    //总电源打开则上电
    for(i = 0; i < M_TOTAL_NUM; i++)
    {
        //上电判断
        if(MOTOR_POWER_FLAG_OFF == gStMotorRunState[i].powerFlag)
        {
            //电机不存在则直接返回
            if((DRIVER_TYPE_NONE == gStMotorData[i].driverType) || (gStMotorData[i].flag & MOTOR_INVALID_FLAG))
            {
                continue;
            }
            if(MOTOR_RUN_MODE_POWER_OFF != gTargetRunMode[i].run_mode)    
        	{
                SetMotorPower(i, POWER_ON);        //若配置了预充功能，此处打开的是预充继电器
                gStMotorRunState[i].powerFlag = MOTOR_POWER_FLAG_TIMECNT;
                gStMotorRunState[i].powerTime = 0;
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_BOOT_UP_FLAG);

                //如果电机共用一个电源，则也上电
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

    //预充电处理
    if(MOTOR_POWER_FLAG_TIMECNT == gPreChargeState)
    {
        gPreChargeTimeCnt += processTimeMs;
        if(gPreChargeTimeCnt >= MOTOR_ALL_PRE_CHARGE_TIME)
        {
            gPreChargeState = MOTOR_POWER_FLAG_ON;
            SetMotorPower(M_TOTAL_NUM, POWER_ON);  //若配置了预充功能，此处打开的是直流接触器
        }
    }
    if(MOTOR_POWER_FLAG_TIMECNT == gPreChargeStateOne)
    {
        gPreChargeTimeCntOne += processTimeMs;
        if(gPreChargeTimeCntOne >= MOTOR_ALL_PRE_CHARGE_TIME)
        {
            gPreChargeStateOne = MOTOR_POWER_FLAG_ON;
            SetMotorPower(M_NUM_PI7, POWER_ON);  //若配置了预充功能，此处打开的是非预充接触器
        }
    }
}
/*****************************************************************************
 功能描述  : 各电机状态获取
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月23日
*****************************************************************************/
static void GetMotorStatus(void)
{
    static uint32_t motorNum = M_LEFT;
#ifdef ENABLE_CANOPEN_DRIVER
    uint16_t lResult = 0;
    uint16_t l_status;
    uint32_t l_tick_temp;
#endif

    if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //非canopen驱动器
    {
        if(MOTOR_POWER_FLAG_ON == gStMotorRunState[motorNum].powerFlag)
        {
            if(ERROR_NONE == gStMotorRunState[motorNum].newErrorLevel)
            {
                if(0 == gStMotorRunState[motorNum].startRemoteFlag)
                {
                    gStMotorRunState[motorNum].startRemoteFlag = 1;
                }
                else if(DRIVER_TYPE_KINCO == gStMotorData[motorNum].driverType) //步科驱动器
                {
                    if(HAL_OK == Motor485ReadErrorCode(motorNum, &lResult))
                    {
                        if(0 != lResult)
                        {
                            SetErrorCode(motorNum, lResult, ERROR_L_HIHG);
                        }
                    }
                }
                else if(DRIVER_TYPE_LEADSHINE == gStMotorData[motorNum].driverType) //雷赛驱动器
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
    //电机上电之后开始获取电机状态
    else if(MOTOR_POWER_FLAG_ON == gStMotorRunState[motorNum].powerFlag)
    {
        if(ERROR_NONE == gStMotorRunState[motorNum].newErrorLevel)
        {
            if(0 == gStMotorRunState[motorNum].startRemoteFlag)
            {
                gStMotorRunState[motorNum].startRemoteFlag = 1;
                SetEposNMTState(SET_NMT_STATE_OPERATIONAL, gStMotorData[motorNum].idx);//允许PDO传输
                if((DRIVER_TYPE_EPOS != gStMotorData[motorNum].driverType) && (DRIVER_TYPE_PUSI != gStMotorData[motorNum].driverType))//sdo模式
                {
                    MotorCanDisableTransimitPdo(gStMotorData[motorNum].idx);//关闭驱动器发送pdo，防止其发送多余数据
                }

                if (DRIVER_TYPE_STAND == gStMotorData[motorNum].driverType)
                {
                    MotorSetAbortConnectionOptionCode(gStMotorData[motorNum].idx, 0);        //CANopen总线通讯出现故障，驱动器不做动作
                }
                
                if(gStMotorRunState[motorNum].resetFlag)   //恢复断电前的位置
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
                            ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_HOMMING, HOME_FLAG_SET);    //设置零位
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
                    //电机发生故障
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
                    //无故障，但是电机异常退出使能，则报故障
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
                    //SetEposNMTState(SET_NMT_STATE_OPERATIONAL, gStMotorData[motorNum].idx);//允许PDO传输,出错后重启一次服务
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
    else if(MOTOR_POWER_FLAG_TIMECNT == gStMotorRunState[motorNum].powerFlag)//电机达到预上电时间时尝试着进行通信，以防止未收到启动信息时等太长时间
    {
        if(gStMotorRunState[motorNum].powerTime >= gStMotorData[motorNum].powerDelayTime)
        {
            l_tick_temp = xTaskGetTickCount() - gStMotorRunState[motorNum].offLineTime;
            if(l_tick_temp >= 1000)//每隔1s尝试一次
            {
                if(DEBUG_DATA_TYPE_6)
                {
                    rt_kprintf("Try communication, id = %d, time: %d.\r\n", gStMotorData[motorNum].idx, gStMotorRunState[motorNum].powerTime);
                }
                gStMotorRunState[motorNum].offLineTime = xTaskGetTickCount();
                SetEposNMTState(SET_NMT_STATE_OPERATIONAL, gStMotorData[motorNum].idx);//允许PDO传输
                if(0 == MotorReadStatus(gStMotorData[motorNum].idx, &l_status))
                {
                    gStMotorRunState[motorNum].powerTime = MOTOR_ALL_POWERING_TIMEOUT;//至满上电计时
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

    //9025异常去使能状态数据记录
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
                        if(DRIVER_TYPE_FDK == gStMotorData[motorNum].driverType)    //风得控驱动器读取工作状态
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
    

    //紧急故障赋值
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

    //用于判断时间是否准确
    /*if(DEBUG_DATA_TYPE_7)
    {
        rt_kprintf("%d ms\r\n", TickType_t());
    }*/
}
/*****************************************************************************
 功能描述  : 切换电机控制模式
 输入参数  : uint32_t motorNum       电机序号
             int8_t i8OperationMode  运行模式
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年12月1日
*****************************************************************************/
#ifdef ENABLE_CANOPEN_DRIVER
void ChangeMotorControlMode(uint32_t motorNum, int8_t i8OperationMode)
{
    if(motorNum < M_TOTAL_NUM)
    {
        //如果相同模式，则无需重新切换
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
                    gStMotorRunState[motorNum].operationStopJudgeFlag = 1;  //切换模式时需要先判断电机是否已停止
                }
                if(MOTOR_OPERATION_MODE_PPM != i8OperationMode)
                {
                    gStMotorRunState[motorNum].runParasSetFlag = RUN_PARAS_NOT_SET;     //切换模式后需要重新设定运行参数
                }
            }
            gStMotorRunState[motorNum].operationMode = i8OperationMode;
            gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_SET;
            gStMotorRunState[motorNum].targetVel = 0;
            gStMotorRunState[motorNum].setCurrent = 0;
        }
        else if(i8OperationMode == MOTOR_OPERATION_MODE_PPM)    //位置模式下新的位置
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
 功能描述  : 切换电机目标位置、速度或电流
 输入参数  : uint32_t motorNum   电机序号
             int32_t value       目标位置、速度或电流
             uint8_t changeFlag  切换标志
 输出参数  : HAL_StatusTypeDef
 作    者  : 刘鹏
 日    期  : 2018年12月1日
*****************************************************************************/
HAL_StatusTypeDef ChangeMotorTargetValue(uint32_t motorNum, int32_t value, uint8_t changeFlag)
{
    HAL_StatusTypeDef lResult = HAL_OK, lResult1 = HAL_OK;
    
    if(motorNum < M_TOTAL_NUM)
    {
        if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //非canopen驱动器
        {
            if((MOTOR_SET_TARGET_VEL == changeFlag) ||
                (MOTOR_SET_TARGET_VEL_NEED_JUDGE == changeFlag))
            {
                gStMotorRunState[motorNum].targetVel = value;
                if(DRIVER_TYPE_KINCO == gStMotorData[motorNum].driverType)
                {
                    //PWMSetMotorValue(motorNum, ABS_VALUE(value));
                    Motor485SetTargetVelocity(motorNum, value);
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
                    {
                        if(gStMotorData[motorNum].relatedMotor < M_TOTAL_NUM)
                        {
                            PWMSetMotorValue(gStMotorData[motorNum].relatedMotor, value);
                        }
                    }
                }
                else if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
                {
                    if(gStMotorData[motorNum].flag & MOTOR_PWM_CONTRL_MODE)  //pwm控制方式
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
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机给定相同目标值
            {
                if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
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
                            if(IS_UFOONE_FLAG_SET(UFOONE_DIFF_DRIVER_CURRENT))  //根据最大电流比值成比例放大缩小
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
                    if(IS_UFOONE_FLAG_SET(UFOONE_DIFF_DRIVER_CURRENT))  //根据最大电流比值成比例放大缩小
                    {
                        gStMotorRunState[gStMotorData[motorNum].relatedMotor].setCurrent = CalDifDriverCurrent(gStMotorData[motorNum].relatedMotor, motorNum);
                    }
                    if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
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
                                if(IS_UFOONE_FLAG_SET(UFOONE_DIFF_DRIVER_CURRENT))  //根据最大电流比值成比例放大缩小
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
 功能描述  : 设置驱动器目标位置、速度或电流
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年12月1日
*****************************************************************************/
#ifdef ENABLE_CANOPEN_DRIVER
static void SetDriveMotorTargetValue(uint32_t motorNum, uint32_t processTimeMs)
{
    if(motorNum >= M_TOTAL_NUM)
    {
        return;
    }

    gStMotorRunState[motorNum].operTimedelay += processTimeMs;
    
    //清除电机启动标志
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
    //切换速度或位置，不需要反馈
    else if(MOTOR_SET_TARGET_POS == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        MotorSetTargetPosition(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].targetPos, RT_FALSE);
        MotorDeviceControlCmd(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION1);//将电机设置成运行状态
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
                MotorDeviceControlCmd(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION);//开始运动
            }
        }
    }
    //切换速度，需要判断
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
                    MotorDeviceControlCmd(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION);//开始运动
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
    //判断电机是否执行目标速度
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
                    gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_TARGET_VEL_NEED_JUDGE;//未执行则需要重新设置一次速度
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
    //切换位置，需要判断
    else if(MOTOR_SET_TARGET_POS_NEED_JUDGE == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        if(OPERATION_MODE_SET == gStMotorRunState[motorNum].operSetNewStep)
        {
            gStMotorRunState[motorNum].operSetNewStep = OPERATION_MODE_UNKOWN;
            MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION);
        }
        if(0 == MotorSetTargetPosition(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].targetPos, RT_TRUE))
        {
            if(0 == MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_ENABLE_OPERATION1))//将电机设置成运行状态
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
    //判断电机是否执行目标位置
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
    //切换电流无需判断
    else if(MOTOR_SET_TARGET_CURRENT == gStMotorRunState[motorNum].targetPosVelSetFlag)
    {
        MotorSetTargetCurrent(gStMotorData[motorNum].idx, gStMotorRunState[motorNum].setCurrent, RT_FALSE);
        gStMotorRunState[motorNum].targetPosVelSetFlag = MOTOR_SET_NOTHING;
        if(DEBUG_DATA_TYPE_92)
        {
            rt_kprintf("M%d set current:%d.\r\n", motorNum, HAL_GetTick());
        }
    }
    //切换电流，需要判断
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
 功能描述  : 切换驱动器电机控制模式
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年12月1日
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
        //等待启动远程帧服务
        if(0 == gStMotorRunState[i].startRemoteFlag)
        {
            continue;
        }
        else if(gStMotorData[i].flag & MOTOR_ENABLE_FLAG) //转使能
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
        else if(IS_NOT_CANOPEN_DRIVER(gStMotorData[i].driverType))    //非canopen驱动器
        {
            continue;
        }
        //是否需要切换
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
        //电机故障时不处理
        if(ERROR_NONE != gStMotorRunState[i].newErrorLevel)
        {
            gStMotorRunState[i].operationMode = MOTOR_OPERATION_MODE_UNKOWN;
            continue;
        }
        //超时处理
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
        //停止电机
        if(OPERATION_STOP_FINISH != gStMotorRunState[i].operStopStep)
        {
            if((DRIVER_TYPE_PUSI ==  gStMotorData[i].driverType)
                || (DRIVER_TYPE_QILING ==  gStMotorData[i].driverType))
            {
                gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;//停止完成
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
                            gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;//停止完成
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
                    gStMotorRunState[i].operStopStep = OPERATION_STOP_FINISH;//停止完成
                }
                if(OPERATION_STOP_FINISH == gStMotorRunState[i].operStopStep)
                {
                    gStMotorRunState[i].operModeSetTimeOut = 0;
                    gStMotorRunState[i].retryTime = 0;
                    if(DRIVER_TYPE_STAND ==  gStMotorData[i].driverType)
                    {
                        gStMotorRunState[i].operationStopJudgeFlag = 1;  //停止完成后需要判断电机实际转速是否已停止
                    }
                }
            }
            continue;
        }
        //判断电机实际转速是否已停止
        if((gStMotorRunState[i].operationStopJudgeFlag) && (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operModeSetStep))
        {
            if(1 == gStMotorRunState[i].operationStopJudgeFlag) //获取实际转速
            {
                CLEAR_MOTOR_REV_DATA_FLAG(i, REV_MOTOR_SPEED_FLAG);
                MotorSendReadVelocity(gStMotorData[i].idx);
                gStMotorRunState[i].operationStopJudgeFlag = 2;
            }
            else if(2 == gStMotorRunState[i].operationStopJudgeFlag) //判断是否接收到实际转速
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
                    if(ABS_VALUE(gStMotorRevData[i].speed) <= 10)   //电机实际接近停止
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
        //设置电机运行参数
        if(gStMotorData[i].flag & MOTOR_NEED_SET_RUN_PARAS)
        {
            if(gStMotorRunState[i].operationMode == MOTOR_OPERATION_MODE_PPM)   //轮廓位置模式相关运行参数
            {
                if(!(gStMotorRunState[i].runParasSetFlag & RUN_PARAS_PROFILE_VEL_SET)) //设定轮廓速度
                {
                    if(0 == MotorSetProfileVelocity(gStMotorData[i].idx, gStMotorData[i].profileVelocity, RT_TRUE))
                    {
                        gStMotorRunState[i].runParasSetFlag |= RUN_PARAS_PROFILE_VEL_SET;
                    }
                    continue;
                }
                if(!(gStMotorRunState[i].runParasSetFlag & RUN_PARAS_PROFILE_ACC_SET)) //设定轮廓加速度
                {
                    if(0 == MotorSetProfileAcc(gStMotorData[i].idx, gStMotorData[i].profileAcc, RT_TRUE))
                    {
                        gStMotorRunState[i].runParasSetFlag |= RUN_PARAS_PROFILE_ACC_SET;
                    }
                    continue;
                }
                if(!(gStMotorRunState[i].runParasSetFlag & RUN_PARAS_PROFILE_DEC_SET)) //设定轮廓减速度
                {
                    if(0 == MotorSetProfileDec(gStMotorData[i].idx, gStMotorData[i].profileAcc, RT_TRUE)) //暂时和加速度值一致
                    {
                        gStMotorRunState[i].runParasSetFlag |= RUN_PARAS_PROFILE_DEC_SET;
                    }
                    continue;
                }
            }
        }
        //位置模式下准备设置新的位置
        if(OPERATION_MODE_UNKOWN != gStMotorRunState[i].operSetNewStep)
        {
            if(OPERATION_MODE_SET_FINISH != gStMotorRunState[i].operModeSetStep)
            {
                gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
            }
        }
        if(OPERATION_MODE_SET_NEW_POSITION == gStMotorRunState[i].operSetNewStep)
        {
            if(0 == MotorDeviceControlCmdWithJudge(gStMotorData[i].idx, DEVICE_CTROL_ENABLE_OPERATION))//使能电机运动
            {
                gStMotorRunState[i].operSetNewStep = OPERATION_MODE_AUTO_CHANGE;    //设置模式完成
            }
        }
        //开始切换模式
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
                            MotorSetOperationMode(gStMotorData[i].idx, gStMotorRunState[i].operationMode);//设置模式
                        }
                        else if(lOperationMode == MOTOR_OPERATION_MODE_HMM)
                        {
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_HOME_METHOD;//切换到设置回零方案
                        }
                        else
                        {
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SHUT_DOWN;//切换到准备状态
                        }
                    }
                }
                else//断开驱动器输出
                {
                    MotorDeviceControlCmdWithJudge(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);
                }
            }
        }
        //开始切换寻零方案
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
                    gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SHUT_DOWN;//切换到准备状态
                }
            }
        }
        //开始切换运行状态，根据Firmware-Specification文档中的2.2节设备状态传输
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
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;//切换模式完成
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
                    gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;//切换模式完成
                }
                else if(0 == MotorReadStatus(gStMotorData[i].idx, &l_status))
                {
                    l_status &= DEVICE_STATUS_MASK;
                    if(OPERATION_MODE_SHUT_DOWN == gStMotorRunState[i].operModeSetStep)
                    {
                        if(DEVICE_STATUS_QUICK_STOP == l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);//禁止电压输出
                        }
                        else if(DEVICE_STATUS_READY_SWITCH_ON != l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_SHUT_DOWN);//关断电机电源
                        }
                        else
                        {
                            gStMotorRunState[i].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;//根据返回的状态自动切换
                        }
                    }
                    if(((OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operModeSetStep)
                        || (OPERATION_MODE_AUTO_CHANGE == gStMotorRunState[i].operSetNewStep))
                        && (0 == gStMotorRunState[i].operationStopJudgeFlag))
                    {
                        if(DEVICE_STATUS_QUICK_STOP == l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_DISABLE_VOLTAGE);//禁止电压输出
                        }
                        else if(DEVICE_STATUS_SWITCH_ON_DISABLE & l_status)
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_SHUT_DOWN);//关断电机电源
                        }
                        else if((DEVICE_STATUS_READY_SWITCH_ON == l_status)
                            && (DRIVER_TYPE_FDK == gStMotorData[i].driverType))
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_SWITCH_ON);//等待电机使能
                        }
                        else if((DEVICE_STATUS_READY_SWITCH_ON == l_status)
                            || (DEVICE_STATUS_SWITCH_ON == l_status))
                        {
                            MotorDeviceControlCmd(gStMotorData[i].idx, DEVICE_CTROL_ENABLE_OPERATION);//使能电机运动
                        }
                        else if(DEVICE_STATUS_OPERATION_ENABLE == l_status)
                        {
                            //风得控驱动器还需要判断2001的状态
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
                            //使能完成
                            if(DEVICE_STATUS_OPERATION_ENABLE == l_status)
                            {
                                gStMotorRunState[i].retryTime = 0;
                                gStMotorRunState[i].operModeSetTimeOut = 0;
                                gStMotorRunState[i].operSetNewStep = OPERATION_MODE_UNKOWN;
                                gStMotorRunState[i].operModeSetStep = OPERATION_MODE_SET_FINISH;//切换模式完成
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
 功能描述  : 设置故障结构体信息，用于记录故障的过程数据
 输入参数  : uint16_t motorNum
             uint16_t lResult   
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年8月31日
*****************************************************************************/
void SetErrorInfo(uint16_t motorNum, uint16_t lResult)
{
    int pidInfoRecordFlag = 0;

    gStErrorInfo.errorMotorNum = motorNum;
    gStErrorInfo.errorResult = lResult;
    gStErrorInfo.errRecordTime = HAL_GetTick();
    
    //1003故障
    if(WARNING_CODE_NAV_ABNORMAL == lResult)
    {
        if(motorNum < M_TOTAL_NUM)
        {
            gStErrorInfo.setCurrentPositiveChangeIndex = gMotorAvaSetCurrentPositiveChangeIndex;
            gStErrorInfo.setCurrentNegativeChangeIndex = gMotorAvaSetCurrentNegativeChangeIndex;
        }
        pidInfoRecordFlag = 1;
    }
    //1004故障
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
 功能描述  : 打印故障详细信息
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年8月31日
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
 功能描述  : 设定错误标志，后续可在此增加特殊错误的处理
 输入参数  : uint16_t motorNum        电机序号
             uint16_t lResult         电机故障代码
             ERROR_LEVEL errorLevel   故障等级
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月12日
*****************************************************************************/
void SetErrorCode(uint16_t motorNum, uint16_t lResult, ERROR_LEVEL errorLevel)
{
    uint16_t motorNumBackup;

    motorNumBackup = motorNum;
    if(motorNum >= WARNING_MOTOR_NUM_OFFSET)
    {
        motorNum -= WARNING_MOTOR_NUM_OFFSET;
    }

    //是否运动过程中出现的故障
    if(!((M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT].pidCurrentStartFlag)
        && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
        && (M_PID_IDLE_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
        && (M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)))
    {
        gErrorRunStateFlag = 1;
    }

    //更新历史故障记录
    UpdateHistoryError((motorNum == M_TOTAL_NUM) ? (motorNumBackup - M_TOTAL_NUM) : (motorNumBackup + 1), lResult);
    
    sys_para->CAR_RTinf.Link |= LINK_ERROR_STOP; //进入故障紧急停止状态
    sys_para->CAR_RTinf.Link &= ~LINK_REV_STOP_CMD; //故障时清除收到停止命令的标志
    gErrorTime = HAL_GetTick(); //记录出故障的时刻
        
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
        if(gStUfoData.flag & UFO_ERROR_WAIT_FLAG)   //故障等待标志，不进行故障自恢复
        {
            gStMotorRunState[motorNum].errorLevel = ERROR_L_VERY_HIGH;
        }
        gStMotorRunState[motorNum].newErrorLevel = gStMotorRunState[motorNum].errorLevel;
    }
    else if(motorNum == M_TOTAL_NUM)
    {
        //上次非故障码的情况下才更新新的警告或故障码，防止警告码替换掉故障码
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
 功能描述  : 清除电机故障
 输入参数  : uint16_t motorNum  电机序号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月8日
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
        //软件可恢复故障，恢复至停止状态
        if(ERROR_L_NORMAL == gStMotorRunState[motorNum].errorLevel)
        {
            //gStMotorRunState[motorNum].runState = MOTOR_STATE_STOP;
            gStMotorRunState[motorNum].errorLevel = ERROR_NONE;
        }
        //断电才能恢复的故障，切换至断电状态
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
            if(ERROR_NONE != gStMotorRunState[i].errorLevel)   //故障状态
            {
                break;
            }
        }
        if((M_TOTAL_NUM == i) && (!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)))   //电机无故障并且故障停止状态时可清除故障
        {
            if(DEBUG_DATA_TYPE_6)
            {
                rt_kprintf("Error flag clear!\r\n");
            }
            gErrorResult = 0;   //清除故障标志
            gErrorRunStateFlag = 0;
        }
    }
}
/*****************************************************************************
 功能描述  : 故障自动停止命令
 输入参数  : ST_MOTOR_RUN_MODE_DATA* lMotorRunMode
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年7月19日
*****************************************************************************/
void ErrorAutoStopCmd(ST_MOTOR_RUN_MODE_DATA* lMotorRunMode)
{
    uint32_t motorNum;
    
    for(motorNum = M_LEFT; motorNum < M_TOTAL_NUM; motorNum++)
    {
        if(M_PID_IDLE_FLAG != gStMotorRunState[motorNum].pidCurrentStartFlag)
        {
            if(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP)  //因为是测试工具发的命令，故障时需要补发速度0的命令
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
            sys_para->CAR_RTinf.Link |= LINK_REV_STOP_CMD;  //接收到停止命令
        }
    }
}
/*****************************************************************************
 功能描述  : 故障恢复不同步骤时的电机命令
 输入参数  : int errorRecoveryStep  步骤
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月9日
*****************************************************************************/
static void ErrorRecoveryStepMotorCmd(int errorRecoveryStep)
{
    int i;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode;
    
    if((0 == errorRecoveryStep) || (1 == errorRecoveryStep))
    {
        //电机断电
        for(i = M_LEFT; i < M_TOTAL_NUM; i++)
        {
            if(0 == errorRecoveryStep)  //不需断电恢复的故障
            {
                ChangeMotorCurRunMode(i, gTargetRunMode[i].run_mode, MOTOR_RUN_MODE_STEP_START);
            }
            else    //需断电恢复的故障
            {
                ChangeMotorCurRunMode(i, MOTOR_RUN_MODE_POWER_OFF, MOTOR_RUN_MODE_STEP_START);
            }
        }
    }
    else if(2 == errorRecoveryStep)
    {
        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
        //转向居中
        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
        lMotorRunMode.target_value = 0;
        SetMotorRunModeDataForce(M_TURN, &lMotorRunMode);
        //驱动速度为0
        if(gStMotorData[M_LEFT].flag & MOTOR_CURRENT_ADJUST_SPEED) //电流调节转速
        {
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_FLAP_FIX_CURRENT;
        }
        else
	    {
	        lMotorRunMode.run_mode = MOTOR_RUN_MODE_SPEED;
	    }
        lMotorRunMode.target_value = 0;
        SetMotorRunModeDataForce(M_LEFT, &lMotorRunMode);
        if(gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)  //左右侧电机独立控制标志
        {
            SetMotorRunModeDataForce(M_RIGHT, &lMotorRunMode);
        }
        if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
        {
            SetMotorRunModeDataForce(M_LEFT_ONE, &lMotorRunMode);
            SetMotorRunModeDataForce(M_RIGHT_ONE, &lMotorRunMode);
        }
        //刹车到初始位置
        lMotorRunMode.target_value = 0;
        lMotorRunMode.homming_type = HOMMING_TYPE_NULL;
        if(gStMotorData[M_BRAKE].flag & MOTOR_CURRENT_ADJUST_SPEED) //电流调节转速
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
 功能描述  : 故障处理与恢复，恢复完以后等待上位机发送清除故障命令
 输入参数  : uint32_t processTimeMs  轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月8日
*****************************************************************************/
static void ErrorDealAndRecovery(uint32_t processTimeMs)
{
    static int errorRecoveryStep = 0;
    int errorFlag = 0;
    uint32_t lCurTick;
    int i;

    //故障搜寻
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if(DRIVER_TYPE_NONE == gStMotorData[i].driverType)
        {
            continue;
        }
        //软件可恢复故障
        if(ERROR_L_NORMAL == gStMotorRunState[i].newErrorLevel)
        {
            gStMotorRunState[i].newErrorLevel = ERROR_NONE;
            errorRecoveryStep = 0;  //有新故障，从第一步开始
            if(0 == errorFlag)
            {
                errorFlag = 1;
            }
        }
        //断电可恢复故障
        else if(ERROR_L_HIHG == gStMotorRunState[i].newErrorLevel)
        {
            gStMotorRunState[i].newErrorLevel = ERROR_NONE;
            errorRecoveryStep = 0;  //有新故障，从第一步开始
            if(3 != errorFlag)
            {
                errorFlag = 2;
            }
        }
        //不可恢复故障
        else if(ERROR_L_VERY_HIGH == gStMotorRunState[i].newErrorLevel)
        {
            errorFlag = 3;
            errorRecoveryStep = 0;  //有新故障，从第一步开始
            break;
        }
    }

    //故障处理与恢复
    if((1 == errorFlag) || (2 == errorFlag))
    {
        if(0 == errorRecoveryStep)  //第一步清除故障
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
                gErrorRunStateFlag = 1;//故障恢复失败则也至为运行过程中故障，需要上报
                gErrorTimes = 0;
                //至为严重故障
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
                errorRecoveryStep = 1;  //转到下一步，电机至为初始状态
            }
            if(lCurTick >= MOTOR_ERROR_RECOVERY_TIMEOUT)
            {
                gErrorTimes = 0;//转到下一周期
            }
        }
    }
    if(1 == errorRecoveryStep)  //第二步，电机至为初始状态
    {
        ErrorRecoveryStepMotorCmd(2);
        errorRecoveryStep = 2;
    }
    else if(2 == errorRecoveryStep)  //第三步，判断电机是否全部重新上电
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
        //全部上电
        if(M_TOTAL_NUM == i)
        {
            errorRecoveryStep = 3;
            gErrorRecoveryTime = 0;
        }
    }
    else if(3 == errorRecoveryStep)  //第四步，判断故障是否清除
    {
        //上电一定时间内无故障则清除故障
        gErrorRecoveryTime += processTimeMs;
        if(gErrorRecoveryTime > (MOTORSTATUS_TIMEOUT_LONGTIME << 1))
        {
            errorRecoveryStep = 0;
            //清除故障
            for(i = M_LEFT; i < M_TOTAL_NUM; i++)
            {
                if(ERROR_NONE != gStMotorRunState[i].errorLevel)
                {
                    ClearErrorCode(i);
                }
            }
            //如果是静止状态下的故障清除并且下发目标速度为0则自动清除故障码，可继续运行
            if(0 == gErrorRunStateFlag)
            {
                //pc控制下如果给了油门值，则切换至动态故障，需要上报
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
 功能描述  : 设置电机电源命令
 输入参数  : uint32_t motorNum
             PowerState powerState  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月27日
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
                if(gStMotorData[motorNum].flag & MOTOR_POWER_ON_INVERSE) //电源引脚反向
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
                if(gStMotorData[motorNum].flag & MOTOR_POWER_ON_INVERSE) //电源引脚反向
                {
                    lPinState = GPIO_PIN_SET;
                }
                HAL_GPIO_WritePin(GPIO_PORT(gStMotorData[motorNum].powerPort), GPIO_PIN(gStMotorData[motorNum].powerPin), lPinState);
                gStMotorRunState[motorNum].runParasSetFlag = RUN_PARAS_NOT_SET;     //断电需要重新设定运行参数
            }
        }

        //开抱闸
        /*if(POWER_ON == powerState)  
        {
            if((M_LEFT == motorNum) || (M_RIGHT == motorNum) || (M_LEFT_ONE == motorNum) || (M_RIGHT_ONE == motorNum))
            {
                OPEN_LOCK;
            }
        }*/

        if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //预充电使能
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
            if(gStMotorData[motorNum].flag & MOTOR_USE_PI7_POWER)    //使用PI7电源控制端口，原电源引脚当作预充引脚
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
        if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //预充电使能
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
                    if(gStUfoData.flag & UFO_PRECHARGE_INVERSE)  //预充反向
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
                    if(gStUfoData.flag & UFO_PRECHARGE_INVERSE)  //预充反向
                    {
                        lPinState = GPIO_PIN_SET;
                    }
                    HAL_GPIO_WritePin(GPIO_PORT(gStUfoData.preChargePort), GPIO_PIN(gStUfoData.preChargePin), lPinState);
                }
            }
        }
    }
    else if(M_NUM_PI7 == motorNum)  //新增转向刹车预充引脚，固定端口PI5
    {
        if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //预充电使能
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
 功能描述  : 设置电机抱闸命令
 输入参数  : uint32_t motorNum
             LockState lockState  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月27日
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
                /#SetPower24V(POWER_ON);   //打开抱闸前先打开24V总电源
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
        if(gStUfoData.flag & UFO_ENABLE_LOCK)  //抱闸使能
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
                    if(gStUfoData.flag & UFO_LOCK_INVERSE)  //预充反向
                    {
                        lPinState = GPIO_PIN_RESET;
                    }
                    HAL_GPIO_WritePin(GPIO_PORT(gStUfoData.lockPort), GPIO_PIN(gStUfoData.lockPin), lPinState);
                    
                    //如果抱闸使用刹车电源，则打开刹车电源
                    if(gStUfoData.flag & UFO_LOCK_TWO_CTRL)  //抱闸两个端口控制
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
                    if(gStUfoData.flag & UFO_LOCK_INVERSE)  //抱闸反向
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
 功能描述  : 将角度（单位为度）转换为驱动器对应的位置值
 输入参数  : uint32_t motorNum
             int32_t angle  角度
             PosType posType   位置类型
 输出参数  : int32_t 驱动器位置
 作    者  : 刘鹏
 日    期  : 2018年12月1日
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
 功能描述  : 获取斜坡加减速(注意调用此函数之前setTargetVel得预设定)
 输入参数  : uint32_t motorNum  电机序号
 输出参数  : int32_t    返回新的速度值
 作    者  : 刘鹏
 日    期  : 2019年11月17日
*****************************************************************************/
int32_t GetSlopeSpeed(uint32_t motorNum, int32_t setTarVel, int32_t curVel, int32_t carMotorVelOutRangeFlag, int32_t* liTempData, uint32_t processTimeMs)
{
    int32_t lTargetSpeed = gStMotorRunState[motorNum].targetVel;
    int32_t lSpeedGrad = 0, curAccLimit = 400, navAccLimit = 400, remoteAccLimit = 200;  
    uint32_t lCurTime;

    //遥控加速度限制
    if((gStMotorData[M_LEFT].profileAcc >= 100) && (gStMotorData[M_LEFT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        remoteAccLimit = gStMotorData[M_LEFT].profileAcc;
    }
    
    //惯导自动模式加速度限制
    if((gStMotorData[M_RIGHT].profileAcc >= 50) && (gStMotorData[M_RIGHT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        navAccLimit = gStMotorData[M_RIGHT].profileAcc;
    }
    lCurTime = HAL_GetTick();
    
    //限定紧急停止减速度为10
    if(sys_para->CAR_RTinf.Link & LINK_QUICK_STOP)
    {
        //lTempVel = OVER_VEL_MAX << 3;
        lSpeedGrad = 1000;
        curAccLimit = lSpeedGrad + 100;
        //增加设定加速度切换时延，防止来回切导致抖动
        //跨越正临界2时，设定加速度小于等于0时可以切负设定加速度，设定加速度大于0时，需先切换置0，再切换至负设定加速度
        if((curVel > setTarVel + 100) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = -lSpeedGrad;
        }
        //跨越正临界1时，设定加速度大于等于0时，切设定加速度为0
        else if((curVel > setTarVel) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        } 
        //跨越负临界2时，设定加速度大于等于0时可以切正设定加速度，设定加速度小于0时，需先切换置0，再切换至正设定加速度
        else if((curVel < setTarVel - 100) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = lSpeedGrad;
        }
        //跨越负临界1时，设定加速度小于等于0时，切设定加速度为0
        else if((curVel < setTarVel) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        }
    }
    //pc控制时加减速度和规划的一致
    else if(sys_para->CAR_RTinf.Link & LINK_PC)   //PC控制
    {
        if(gStUfoData.flag & UFO_NOT_USE_GET_SET_ACC) //不使用获取到的设定加速度
        {
            //滤波获得规划目标加速度
            /*for(k = 0; k < SET_ACC_RECORD_NUM; k++)
            {
                liTempData[k] = gStMotorRunState[motorNum].setCarAcc[k];
            }
            FilterMidAverage(liTempData, SET_ACC_RECORD_NUM, 1, gStMotorRunState[motorNum].setAcc, k, lTempVel);*/
            //设定加速度卡尔曼滤波
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
    //限定遥控加速度减速度为2，停止时减速度为6
    else
    {
        //lTempVel = OVER_VEL_MAX << 2;
        if((0 != setTarVel) && IS_SAME_SYMBLE(setTarVel, curVel))   //目标速度不为0，并且目标速度和当前速度方向一致，则按设定加速度规划，否则按停止减速度规划
        {
            if((remoteAccLimit < gRemoteStopDcc) && (gRemoteStopDcc == ABS_VALUE(gStMotorRunState[motorNum].setAcc)))   //上次用了较大的减速度
            {
                lTargetSpeed = curVel;  //目标规划速度更新至当前速度
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
        //增加设定加速度切换时延，防止来回切导致抖动
        //跨越正临界2时，设定加速度小于等于0时可以切负设定加速度，设定加速度大于0时，需先切换置0，再切换至负设定加速度
        if((curVel > setTarVel + 50) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = -lSpeedGrad;
        }
        //跨越正临界1时，设定加速度大于等于0时，切设定加速度为0
        else if((curVel > setTarVel) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        } 
        //跨越负临界2时，设定加速度大于等于0时可以切正设定加速度，设定加速度小于0时，需先切换置0，再切换至正设定加速度
        else if((curVel < setTarVel - 50) && (gStMotorRunState[motorNum].setAcc >= 0))
        {
            gStMotorRunState[motorNum].setAcc = lSpeedGrad;
        }
        //跨越负临界1时，设定加速度小于等于0时，切设定加速度为0
        else if((curVel < setTarVel) && (gStMotorRunState[motorNum].setAcc <= 0))
        {
            gStMotorRunState[motorNum].setAcc = 0;
        }
    }
    //根据加速度计算速度梯度值
    lSpeedGrad = lSpeedGrad * (int32_t)processTimeMs / 1000;
    if(0 == lSpeedGrad)
    {
        lSpeedGrad = 1; //限定最小速度梯度值
    }

    if(lTargetSpeed != setTarVel)
    {
        //判断是否到达设定目标速度
        if(ABS_VALUE(lTargetSpeed - setTarVel) <= lSpeedGrad)
        {
            lTargetSpeed = setTarVel;
        }
        //朝着目标速度加减速
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

    //限定最小速度为寻零速度
    /*if(0 != setTarVel)
    {
        lTempVel = ABS_VALUE(gStMotorData[motorNum].homingSpeed);
        if(ABS_VALUE(lTargetSpeed) < lTempVel)
        {
            lTargetSpeed = (setTarVel > 0) ? lTempVel : (-lTempVel);
        }
    }*/

    //滤波获得惯导加速度
    /*for(k = 0; k < NAV_ACC_RECORD_NUM; k++)
    {
        liTempData[k] = gCarNavAcc[k];
    }
    FilterMidAverage(liTempData, NAV_ACC_RECORD_NUM, 1, gCarNavCurAcc, k, lTempVel);*/

    //滤波获得当前电机算得的加速度
    /*for(k = 0; k < MACC_RECORD_NUM; k++)
    {
        liTempData[k] = gStMotorRunState[motorNum].mAcc[k];
    }
    FilterMidAverage(liTempData, MACC_RECORD_NUM, MACC_EXCLUDE_NUM, gStMotorRunState[motorNum].motorCurAcc, k, lTempVel);*/

    //实际加速度计算，紧急停止时不用惯导加速度
    if((!IS_EMERGENCY_STOP) && (sys_para->CAR_RTinf.Link & LINK_PC) && (!carMotorVelOutRangeFlag))   //PC控制，且惯导与电机的速度差＜50cm/s
    {
        gStMotorRunState[motorNum].curAcc = gCarNavCurAcc;  //惯导加速度
    }
    else
    {
        gStMotorRunState[motorNum].curAcc = gMotorCurAcc;   //电机加速度
    }
    gStMotorRunState[motorNum].curAcc = Limit(gStMotorRunState[motorNum].curAcc, -curAccLimit, curAccLimit);

    return lTargetSpeed;
}
/*****************************************************************************
 功能描述  : 电机断电处理
 输入参数  : uint32_t motorNum   电机序号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月15日
*****************************************************************************/
static void ProcessMotorRunModePowerOff(uint32_t motorNum)
{
    uint8_t k;
    
    if(MOTOR_POWER_FLAG_OFF != gStMotorRunState[motorNum].powerFlag)    //关电源，重置数据
    {
        if((M_LEFT == motorNum) || (M_RIGHT == motorNum) || (M_LEFT_ONE == motorNum) || (M_RIGHT_ONE == motorNum))
        {
            CLOSE_LOCK; //关闭抱闸
            CLOSE_BRAKE;//刹车断电
        }
        #ifdef ENABLE_CANOPEN_DRIVER
        if(gStMotorData[motorNum].flag & MOTOR_POWER_OFF_RECORDPOS) //断电记录位置
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
                    //gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;    //需要重新寻零
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
            if((DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)            //每次电机上电都会重新寻零
                || (DRIVER_TYPE_NIMOTION == gStMotorData[motorNum].driverType))  //每次电机上电都会重新寻零
            {
                gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;    //需要重新寻零
            }
        }
        SetMotorPower(motorNum, POWER_OFF);
        gStMotorRunState[motorNum].startRemoteFlag = 0;
        gStMotorRunState[motorNum].operationStopJudgeFlag = 0;  //断电后不需要判断电机实际转速是否已停止
        gStMotorRunState[motorNum].powerFlag = MOTOR_POWEROFF_FLAG_TIMECNT;
        gStMotorRunState[motorNum].powerTime = 0;
        gStMotorRunState[motorNum].targetPosJudgeFlag = RT_FALSE;
        ChangeMotorTargetValue(motorNum, 0, MOTOR_SET_NOTHING);
        #ifdef ENABLE_CANOPEN_DRIVER
        ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_UNKOWN); 
        #endif	

        if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG) //转使能
        {
            if(ENABLE == gStMotorRunState[motorNum].enableFlag)
            {
                CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, 0);
                GPIOEnableMotor(motorNum, DISABLE);
            }
        }

        //如果电机共用一个电源，则也断电
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
                        ChangeMotorCurRunMode(k, MOTOR_RUN_MODE_POWER_OFF, MOTOR_RUN_MODE_STEP_START);    //切换到断电模式
                        ProcessMotorRunModePowerOff(k);
                    }
                }
            }
        }
    } 

    gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_POWER_OFF;
    gStMotorRunState[motorNum].run_step = MOTOR_RUN_MODE_STEP_IDLE;

    ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
}
/*****************************************************************************
 功能描述  : 电机寻零处理
 输入参数  : rt_uint32_t motorNum      电机序号
             rt_uint32_t processTimeMs 函数轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月15日
*****************************************************************************/
void ProcessMotorRunModeHomming(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    uint16_t zeroState;
    int32_t liTemp;
    float lfTemp;

    // 1.超时处理
    if((gStMotorRunState[motorNum].runTotalTime >= MOTOR_PECT_HOMMING_TURN_TIMEOUT)
        || (gStMotorRunState[motorNum].errorCnt > MOTOR_CAN_TIMEOUT_TIMES))
    {
        gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;     //切换到空闲模式
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        SetErrorCode(motorNum, ERROR_CODE_HOMMING_TIMEOUT, ERROR_L_HIHG);
        return;
    }

    // 2.命令发生改变处理（无需处理）
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        gStMotorRunState[motorNum].change_flag = MOTOR_RUN_MODE_NO_CHANGE;
    }

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
    if((MOTOR_RUN_MODE_STEP_START == *pRunStep) || (HOME_FLAG_SET_MOVE == *pRunStep) || (HOME_FLAG_CLEAR_STOP == *pRunStep))
    {
        if(DRIVER_TYPE_HCX == gStMotorData[motorNum].driverType)
        {
            if(gStMotorRunState[motorNum].runDelayTime >= 20)
            {
                gStMotorRunState[motorNum].runDelayTime = 0;
                if(HAL_OK == Motor485SetMotorContrlmode(motorNum, MOTOR_MODE_IO))
                {
                    if(GPIO_PIN_RESET == GetMotorLimit1State(motorNum)) //被遮挡
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
                    MotorSetMotorControlStatus(gStMotorData[motorNum].idx, zeroState);  //清除状态位
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
                            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                            gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //寻零完成
                            if(DEBUG_DATA_TYPE_2)
                            {
                                rt_kprintf("Pos limit1: %d, limit2: %d\r\n", gStMotorRunState[motorNum].limitPos1, gStMotorRunState[motorNum].limitPos2);
                            }
                        }
                        else
                        {
                            *pRunStep = MOTOR_RUN_MODE_STEP_START;   //重新开始
                        }
                    }
                }
                CLEAR_MOTOR_REV_DATA_FLAG(motorNum, REV_MOTOR_CTRL_STATUS_FLAG);
            }
        }
        else
        {
            if(gStMotorData[motorNum].flag & MOTOR_ENABLE_STALL_HOMMING)    //堵转寻零
            {
                ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PTM);
                *pRunStep = HOME_FLAG_MOVE_CW;                //正向运行
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
                if(gStMotorRevData[motorNum].ctrlStatus & (DIVICE_CTROL_STATUS_EXT_STOP1 | DIVICE_CTROL_STATUS_EXT_STOP2))    //外部停止
                {
                    *pRunStep = HOME_FLAG_RECORD_POS;  //记录位置
                }
                else if(gStMotorRevData[motorNum].ctrlStatus & DIVICE_CTROL_STATUS_STALL)    //堵转
                {
                    if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
                    {
                        *pRunStep = HOME_FLAG_SET_MOVE;  //运行
                        gStMotorRunState[motorNum].reverseFlag = 1;
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;   //寻零未知
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
                        *pRunStep = HOME_FLAG_RECORD_POS;  //记录位置
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
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                    gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_UNKOWN; //发送快速停止时模式步骤恢复到未知，防止误报9025故障
                    gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //寻零完成
                }
                else//断开驱动器输出
                {
                    MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_QUICK_STOP);
                }
            }
        }
    }
    else if(HOME_FLAG_RECORD_INIT_POS == *pRunStep)                         //读初始位置
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
                
                if (gStMotorRunState[motorNum].PosInit > gStMotorRunState[motorNum].limitPos2)       //初始位置大于极限位置，需要交换
                {
                    gStMotorRunState[motorNum].PosInit = gStMotorRunState[motorNum].limitPos2;
                    gStMotorRunState[motorNum].limitPos2 = liTemp;
                }

                if(DEBUG_DATA_TYPE_2)
                {
                    //电机反馈的松刹车位置与紧刹车位置之差大于1500(单位:cnt)，则寻零成功；否则寻零失败
                    rt_kprintf("M%d homming %s, P-init:%d, P-limit2:%d, ticks:%d\r\n", motorNum, ((ABS_VALUE(gStMotorRunState[motorNum].limitPos2 - gStMotorRunState[motorNum].PosInit) > 1500) ? "succeed" : "fail"),
                                                gStMotorRunState[motorNum].PosInit, gStMotorRunState[motorNum].limitPos2, HAL_GetTick());
                }

                if (0 == MotorDeviceControlCmdWithJudge(gStMotorData[motorNum].idx, DEVICE_CTROL_QUICK_STOP))
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                    gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_UNKOWN; //发送快速停止时模式步骤恢复到未知，防止误报9025故障
                    lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                    if (lfTemp < 8)    //松刹车位置的油压＜0.8Mpa
                    {
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //寻零完成
                    }
                    else
                    {
                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_BRAKE_ERROR, ERROR_L_NORMAL); //松刹车位置仍有刹车力，报刹车故障
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
                    if(gStMotorRunState[motorNum].stallFlag)    //堵转后的处理
                    {
                        gStMotorRunState[motorNum].stallFlag = 0;
                        *pRunStep = HOME_FLAG_SET_MOVE;  //运行
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
                        *pRunStep = HOME_FLAG_SET_MOVE;  //运行
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
                        *pRunStep = HOME_FLAG_CLEAR_STOP;  //清除停止标志
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
                    *pRunStep = HOME_FLAG_STOP;     //停止电机
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
                if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, gStMotorData[motorNum].homingSpeed))//设置目标转矩(0.1%)
                {
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("limitPos2 setCurent:%d, ticks:%d\r\n", gStMotorData[motorNum].homingSpeed, HAL_GetTick());
                    }
                    *pRunStep = HOME_FLAG_WAIT_CW_STOP;          //等待正向运行停止
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
                if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, liTemp))//设置目标转矩(0.1%)，启动电流
                {
                    lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("posInit setCurent:%d,oil:%.1fMpa,setMaxOil:%.1fMpa,ticks:%d\r\n", liTemp, lfTemp / 10, ADC1_BRAKE_MAX_THRETHOLDS /10.0f, HAL_GetTick());
                    }
                    /*if(lfTemp < 2)  //油压小于0.2Mpa，则按照堵转逻辑退回，否则按照油压变小逻辑退回
                    {
                        *pRunStep = HOME_FLAG_WAIT_CCW_STALL_STOP;    //等待反向堵转停止
                    }
                    else
                    {
                        *pRunStep = HOME_FLAG_WAIT_CCW_STOP;          //等待反向运行停止
                    }*/
                    if (ADC1_BRAKE_MAX_THRETHOLDS > 0) 
                    {
                        *pRunStep = HOME_FLAG_RECORD_SETMAXOIL_POS;    //记录设置的最大油压对应的位置
                        gStMotorRunState[motorNum].limit1Time = 0;
                    }
                    else
                    {
                        *pRunStep = HOME_FLAG_WAIT_CCW_STALL_STOP;    //等待反向堵转停止
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
                if (lfTemp <= (IS_UFOONE_FLAG_SET(UFOONE_BRAKE_STOP_ACCORD_OIL) ? (ADC1_BRAKE_MAX_THRETHOLDS + 5) : ADC1_BRAKE_MAX_THRETHOLDS))    //当前油压值≤设定的最大油压值(0.1Mpa)
                {
                    if(0 == MotorReadPosition(gStMotorData[motorNum].idx, &liTemp))
                    {
                        gStMotorRunState[motorNum].errorCnt = 0;
                        gStMotorRunState[motorNum].limitPos2 = liTemp; //刹车最大位置
                        if(DEBUG_DATA_TYPE_2)
                        {
                            rt_kprintf("P-limit2:%d, oil:%.1fMpa, ticks:%d\r\n", gStMotorRunState[motorNum].limitPos2, lfTemp / 10, HAL_GetTick());
                        }

                        *pRunStep = HOME_FLAG_WAIT_CCW_STALL_STOP;    //等待反向堵转停止
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
                gStMotorRunState[motorNum].runDelayTime = 450;                  //首次进行周期为500ms，后续进入周期为50ms
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
                        if(gStMotorRunState[motorNum].limit1Time >= 3)  //连续3个周期转速小于10rpm
                        {
                            if(HOME_FLAG_WAIT_CW_STOP == *pRunStep)
                            {
                                *pRunStep = HOME_FLAG_SET_POS;  //读极限位置
                                gStMotorRunState[motorNum].runDelayTime = 0;
                            }
                            else if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0)) //设置转矩为0
                            {
                                *pRunStep = HOME_FLAG_RECORD_INIT_POS;   //读取初始位置
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
            if(GPIO_PIN_RESET == GetMotorLimit1State(motorNum)) //被遮挡
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
                if(FLT_EPSILON >= lfTemp)     //油压 ≤ 0MPa（读取油压值为float型）
                {
                    if(DEBUG_DATA_TYPE_2)
                    {
                        rt_kprintf("ADC1_%d: %.2f, ticks:%d\r\n", ADC1_FRONT_BRAKE_CHANNEL, lfTemp, HAL_GetTick());
                    }

                    if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0)) //设置转矩为0
                    {
                        *pRunStep = HOME_FLAG_RECORD_INIT_POS;   //读取初始位置
                    }
                }
            }
        }
        else
        {
            if(GPIO_PIN_SET == GetMotorLimit1State(motorNum)) //未被遮挡
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
                    gStMotorRunState[motorNum].limitPos2 = liTemp; //刹车最大位置
                    *pRunStep = HOME_FLAG_MOVE_CCW;                //反向运行

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
                    SetErrorCode(motorNum, ERROR_CODE_MOTOR_BRAKE1_ERROR, ERROR_L_NORMAL);   //刹车位置已到极限，但油压不正常，设置故障码
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
                    if(gStMotorData[motorNum].flag & MOTOR_PWM_CONTRL_MODE)  //pwm控制方式
                    {
                        *pRunStep = HOME_FLAG_SET_MODE_SPWM_DIR;    //切换到pwm控制模式
                    }
                    else
                    {
                        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //寻零完成
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
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_FINISH;   //寻零完成
            }
        }
    }

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
}
/*****************************************************************************
 功能描述  : 读取电机角度
 输入参数  : uint32_t motorNum  电机序号
             float *angle          电机角度值
 输出参数  : HAL_StatusTypeDef   读取结果
 作    者  : 刘鹏
 日    期  : 2020年6月2日
*****************************************************************************/
#ifdef ENABLE_CANOPEN_DRIVER
HAL_StatusTypeDef ReadMotorAngle(uint32_t motorNum, float *angle)
{
    HAL_StatusTypeDef lResult = HAL_ERROR;
    int32_t liTemp;

    /*if(gStMotorData[motorNum].flag & MOTOR_USE_RS485_ECODER) //使用外部485绝对值编码器
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
 功能描述  : 电流采集
 输入参数  : void
 输出参数  : int32_t    电流值  mA
 作    者  : 刘鹏
 日    期  : 2021年3月22日
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
 功能描述  : 查找最小值序号
 输入参数  : int32_t* pValue  数据
             int32_t len      长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年7月9日
*****************************************************************************/
int SearchMinValue(int32_t* pValue, int32_t len)
{
    int32_t i;
    int32_t min = 0;
    //寻找最小值
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
 功能描述  : 获得中间值
 输入参数  : int32_t* pValue
             int32_t len      
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年11月25日
*****************************************************************************/
int GetMidValue(int32_t* pValue, int32_t len)
{
    uint16_t i;
    int32_t lMin = pValue[0];
    int32_t lMax = lMin;
    int32_t lMid, lHighNum, lMinTemp, lMaxTemp;
    uint16_t lEndFlag = 0x02;
    
    //找最小最大值
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
    
    //寻找中值，二分查找
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
        //小于中值的较多，则下次在小于中值的数之间二分查找
        if(lHighNum + 1 < (len >> 1))
        {
            lMax = lMaxTemp;
        }
        //大于中值的较多，则下次在大于中值的数之间二分查找
        else if(lHighNum > (len >> 1) + 1)
        {
            lMin = lMinTemp;
        }
        else
        {
            break;
        }
    }
    
    lMid = (lMin + lMax) >> 1;    //该系统中，右移为算数右移

    return lMid;
}

/*****************************************************************************
 功能描述  : 获取相关电机速度值
 输入参数  : uint32_t motorNum
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年9月16日
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
    if(((gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO)//同步关联电机给定相同目标值
        && (gStUfoData.flag & UFO_FOUR_DRIVER_FLAG) //兼容四驱
        && (gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO))
        || (IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG)))    //四轮独立标志
    {
        if(M_LEFT == motorNum)
        {
            if(DRIVER_TYPE_NONE != gStMotorData[M_LEFT_ONE].driverType)
            {
                CLEAR_MOTOR_REV_DATA_FLAG(M_LEFT_ONE, REV_MOTOR_SPEED_FLAG);
                MotorSendReadVelocity(gStMotorData[M_LEFT_ONE].idx);
            }
            if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
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
        if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
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
        
        if((HAL_GetTick() - tickstart) > CAN_WRITE_TIMEOUT_MS)   //超时
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
             
            if(((gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO)//同步关联电机给定相同目标值
                && (gStUfoData.flag & UFO_FOUR_DRIVER_FLAG) //兼容四驱
                && (gStMotorData[M_LEFT_ONE].flag & MOTOR_RELATED_SYNCHRO))
                || (IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG)))    //四轮独立标志
            {
                if(IS_REV_MOTOR_DATA(M_LEFT_ONE, REV_MOTOR_SPEED_FLAG))
                {
                    if (!(liFlag & 0x2u))
                    {
                        liFlag |= 0x2u;
                        rotateSpeed[speedValidCnt] = CalRatioVelRpm(M_LEFT_ONE);
                        speedValidCnt++;
                    }
                    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
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
                if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
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

            if(gStUfoData.flag & UFO_FEEDBACK_MAXSPEED_FLAG) //反馈速度值取最大速度
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
                    *vel = sum >> 1u;                               /* 取平均值 */
                }
                else if (speedValidCnt > 2u)
                {
                    *vel = (sum - max - min) / (speedValidCnt - 2); /* 剔除最大最小值，取平均值 */
                }
                else
                {
                    ;
                } 
            }
            min = VelRpmToCm_s(min);
            max = VelRpmToCm_s(max); 
            if (max - min > 50)    //电机转速差值＞50cm/s
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
 功能描述  : 获取相关电机电流
 输入参数  : uint32_t motorNum
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年9月16日
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

    //所有电机目标电流给0
    gStMotorRunState[motorNum].setCurrent = 0;
    if(lExeFlag & M_R_ACCORD_L_FLAG)   //右电机依据左电机标志
    {
        if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
        {
            gStMotorRunState[M_RIGHT].setCurrent = 0;
        }
    }
    else if(lExeFlag & M_FOUR_ALONE_FLAG)   //四电机独立标志
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
        
        if((HAL_GetTick() - tickstart) > CAN_WRITE_TIMEOUT_MS)   //超时
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
                    ava = sum >> 1u;                               /* 取平均值 */
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
 功能描述  : 获取相关轮速值
 输入参数  : ST_TIM_DATA* tim            定时器数据
             uint8_t counts              车轮一转对应脉冲数
             uint32_t processTimeMs      函数调用周期(ms)
 输出参数  : lwheelSpeed 轮速(rpm)
 作    者  : 田忠
 日    期  : 2023年10月17日
*****************************************************************************/
static uint32_t GetWheelSpeed(ST_TIM_DATA* tim, uint8_t ch, uint8_t counts, uint32_t processTimeMs)
{
    int32_t lwheelSpeed;
    uint8_t lfactor;     //计算因数
    
    uint32_t tim_f0;     //定时器计数频率
    uint32_t tim_m0;     //定时器脉冲数
    uint32_t pluse_m1;   //编码器脉冲数

    if (counts == 0)      //车轮一转对应脉冲数不能为零
    {
        return 0;
    }

    tim_f0 = tim->cntFre;
    tim_m0 = tim->CCRRecord[ch] - tim->lastCCRRecord[ch] + (tim->overFlowCounts * tim->device->Init.Period);  //先减后加，以防止计算溢出
    pluse_m1 = tim->captureCounts[ch];

    //重新计数相对时间
    tim->lastCCRRecord[ch] = tim->CCRRecord[ch];
    tim->captureCounts[ch] = 0;
    
    
    if (tim_m0 == 0)     //定时器脉冲数不能为零
    {
        return 0;
    }
    
    //计算轮速(rpm)，M/T法测速理论公式n=(60*f0*M1)/(Z*M0),为防止计算溢出按以下公式计算n=60*f0/M0*M1/Z
    lfactor = tim_f0/108000000 + 1;  //通过增加因数值，在保证计算精度的情况下再次防止溢出,计数频率＜108MHz不需要分频计算
    lwheelSpeed = (int32_t)((60 / lfactor * tim_f0) / tim_m0 * pluse_m1 * lfactor / counts);
  
    return lwheelSpeed;
}
/*****************************************************************************
 功能描述  : ResetTimer(ST_TIM_DATA* tim)
 输入参数  : ST_TIM_DATA* tim            定时器数据
 输出参数  : lwheelSpeed 轮速(rpm)
 作    者  : 田忠
 日    期  : 2023年10月17日
*****************************************************************************/
static void ResetTimer(ST_TIM_DATA* tim)
{
    tim->overFlowCounts = 0;    
}
/*****************************************************************************
 功能描述  : 计算过去一段时间内的平均加速度和平均电流
 输入参数  : int32_t* accOut
             int32_t* currentOut  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2024年4月30日
*****************************************************************************/
void CalAvaAccAndCurrent(int32_t* accOut, int32_t* currentOut)
{
    int32_t liTemp, k;
    uint32_t time = 0;
    
    //记录满MVEL_RECORD_NUM个数据
    if((gCarVelValidNum >= MVEL_RECORD_NUM) && (gMotorAvaVelValidNum >= MVEL_RECORD_NUM))
    {
        liTemp = gCarVel[MVEL_RECORD_NUM - 1] - gCarVel[0];
        time = gCarVelTime[MVEL_RECORD_NUM - 1] - gCarVelTime[0];
        if(time > 0)
        {
            //计算平均惯导加速度
            liTemp = liTemp * 1000 / (int32_t)time;
            *accOut = Limit(liTemp, -CMD_ACC_VALUE_MAX, CMD_ACC_VALUE_MAX);
            //计算平均电流
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
 功能描述  : 自动学习1m/s2加速度对应电流
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2024年4月30日
*****************************************************************************/
void AutoLearnAccCurrent(void)
{
    int32_t k, liTemp;
    
    //加速阶段正加速度，并且加速度相差大于0.9m/s2时才认为有效
    if((gMotorAccPeriodAvaAcc > 0) && (gMotorAccPeriodAvaAcc > gMotorUniPeriodAvaAcc + 90)
        && (0 != gMotorAccPeriodAvaCurrent) && (0 != gMotorUniPeriodAvaCurrent))
    {
        liTemp = (gMotorAccPeriodAvaCurrent - gMotorUniPeriodAvaCurrent) * 100 / (gMotorAccPeriodAvaAcc - gMotorUniPeriodAvaAcc);
        if(!IsDiffOverPercent(liTemp, gStMotorData[M_LEFT].startCurrent, 30))
        {
            //记录学习值
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
 功能描述  : 设定速度状态切换处理
 输入参数  : uint32_t motorNum  电机序号
             int32_t* curSetCarVel当前设定速度
             uint32_t curTime 当前时刻
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年8月29日
*****************************************************************************/
void SetVelStateSwitchDeal(uint32_t motorNum, int32_t* curSetCarVel, uint32_t curTime)
{
    int16_t scenario_max_vel; //最大速度
    int32_t carVel;           //惯导速度
    int32_t liTemp;
    uint32_t time = 0;
    
    scenario_max_vel = ABS_VALUE(VelRpmToCm_s(VelCm_sToRpm(sys_para->CAR_RTinf.max_vel)));  //防止单位切换，数值整除后有-1的误差
    carVel = ABS_VALUE(sys_para->CAR_RTinf.vel);

    if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //无惯导标志时惯导速度直接赋值电机速度
    {
        carVel = ABS_VALUE(gMotorAvaVelFilter);
    }

    //////状态切换判定
    //启动时刻
    if(SET_VEL_STATE_START == gStMotorRunState[motorNum].setVelState)
    {
        gStMotorRunState[motorNum].setCarVelValidNum = 1;//只保留最近一个设定加速度值参与计算
        gStMotorRunState[motorNum].setAccKalmanData.LastP = QUICKLY_FOLLOW_LASTP;//设定加速度快速跟随
        gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_ACC;//转到匀加速状态
        if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
        {
            rt_kprintf("M%d-state-start,mxv:%d,carv:%d.\r\n", motorNum, scenario_max_vel, carVel);
        }
    }
    //加速阶段
    if(SET_VEL_STATE_ACC == gStMotorRunState[motorNum].setVelState)
    {
        //当前车速高于目标车速
        if(carVel > ABS_VALUE(*curSetCarVel))
        {
            //当前车速大于场景最大速度并且场景最大速度有效，或者目标速度不发生变化，进入加速结束阶段
            if(((carVel > scenario_max_vel) && (ABS_VALUE(*curSetCarVel) + 5 >= scenario_max_vel)
                && (ABS_VALUE(*curSetCarVel) <= scenario_max_vel))
                || (*curSetCarVel == gStMotorRunState[motorNum].setCarVel[0]))
            {
                gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_ACC_END;//转到加速结束状态
                if(M_LEFT == motorNum)  //计算加速末端加速度对应的电流值
                {
                    CalAvaAccAndCurrent(&gMotorAccPeriodAvaAcc, &gMotorAccPeriodAvaCurrent);
                    gCarOverVel = carVel;
                    gCarOverVelTime = curTime;
                }
                gStMotorRunState[motorNum].lastSetCurrent = gStMotorRunState[motorNum].setCurrent;  //记录此时电流值，便于误判进入加速结束时恢复之前电流值
                if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
                {
                    rt_kprintf("M%d-state-acc-to-acc-end,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
                }
            }
            //目标速度变小，进入到减速
            else if(ABS_VALUE(*curSetCarVel) < ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
            {
                gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC_START;   //转到减速开始时刻
                if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
                {
                    rt_kprintf("M%d-state-acc-to-dec,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
                }
            }
        } 
    }
    //加速结束状态，进行超调处理
    if(SET_VEL_STATE_ACC_END == gStMotorRunState[motorNum].setVelState)
    {
        //目标速度变大
        if(ABS_VALUE(*curSetCarVel) > ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
        {
            //目标速度大于场景速度，则可能处于后退过程中，场景速度值无效，回到匀加速状态
            if(ABS_VALUE(*curSetCarVel) > scenario_max_vel)
            {
                gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_ACC; //转到匀加速状态
                if(M_LEFT == motorNum)
                {
                    gMotorAccPeriodAvaAcc = 0;
                    gMotorAccPeriodAvaCurrent = 0;
                }
                gStMotorRunState[motorNum].setCurrent = gStMotorRunState[motorNum].lastSetCurrent;  //恢复之前的设定电流值
                if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
                {
                    rt_kprintf("M%d-state-acc-end-to-acc,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
                }
            }
        }
        //目标速度变小
        else if(ABS_VALUE(*curSetCarVel) < ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
        {
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC_START;   //转到减速开始时刻
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-acc-end-to-dec,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
            }
        }
        //目标速度不变，车速低于目标车速，进入匀速段
        else if(carVel + 2 < ABS_VALUE(*curSetCarVel))
        {
            gStMotorRunState[motorNum].setCarVelValidNum = 1;//只保留最近一个设定加速度值参与计算
            gStMotorRunState[motorNum].setAccKalmanData.LastP = IMMEDIATELY_FOLLOW_LASTP;//设定加速度立即跟随
            //计算超调阶段所有阻力平均减速度
            liTemp = gCarOverVel - carVel;
            time = curTime - gCarOverVelTime;
            if(time > 0)    //根据纯阻力减速度计算匀速时需要的电流值
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
                gStCarNavAccKalmanData.LastP = IMMEDIATELY_FOLLOW_LASTP;//当前加速度立即跟随
                if(*curSetCarVel > 0)   //前进时才计算
                {
                    gCarEnterUniTime = curTime;
                }
            }
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_UNI; //进入匀速阶段
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-acc-end-to-uni,mxv:%d,carv:%d,dec:%d,aCurrent:%d.\r\n", motorNum, scenario_max_vel, carVel, liTemp, gMotorAccCurrent);
            }
        }
        if(M_LEFT == motorNum)  //记录最大速度值及时刻
        {
            if(carVel > gCarOverVel)
            {
                gCarOverVel = carVel;
                gCarOverVelTime = curTime;
            }
        }
    }
    //匀速阶段
    if(SET_VEL_STATE_UNI == gStMotorRunState[motorNum].setVelState)
    {
        //目标速度变小
        if(ABS_VALUE(*curSetCarVel) < ABS_VALUE(gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1]))
        {
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC_START;   //转到减速开始时刻
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-uni-to-dec,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
            }
        }
        if(M_LEFT == motorNum)//进入匀速段一段时间后计算匀速段的平均加速度和平均电流值，并计算出1m/s2加速度对应电流学习值
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
    //减速开始时刻
    if(SET_VEL_STATE_DEC_START == gStMotorRunState[motorNum].setVelState)
    {
        gStMotorRunState[motorNum].setCarVelValidNum = 1;//只保留最近一个设定加速度值参与计算
        gStMotorRunState[motorNum].setAccKalmanData.LastP = QUICKLY_FOLLOW_LASTP;//设定加速度快速跟随
        gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_DEC;//转到减速阶段
    }
    //减速阶段
    if(SET_VEL_STATE_DEC == gStMotorRunState[motorNum].setVelState)
    {
        if(ABS_VALUE(*curSetCarVel) == scenario_max_vel)    //进入匀速阶段实际是收到异常的减速设定速度值时，重新恢复到匀速状态
        {
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_UNI;//转到匀速状态
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-dec-to-uni,mxv:%d,carv:%d,setv:%d.\r\n", motorNum, scenario_max_vel, carVel, *curSetCarVel);
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 记录电机合速度及合加速度
 输入参数  : int32_t curMotorAvaVel 当前电机合速度,单位rpm
             int32_t curMotorAvaCurrent 当前电机合电流，单位mA
             uint32_t curTime     当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年3月24日
*****************************************************************************/
void RecordMotorVelAndAcc(int32_t curMotorAvaVel, int32_t curMotorAvaCurrent, uint32_t curTime)
{
    uint32_t k, time = 0;
    int32_t velDif;
    uint16_t velValidNum;
    int32_t navAccLimit = 400;
    
    //惯导自动模式加速度限制
    if((gStMotorData[M_RIGHT].profileAcc >= 50) && (gStMotorData[M_RIGHT].profileAcc <= CMD_ACC_VALUE_MAX))
    {
        navAccLimit = gStMotorData[M_RIGHT].profileAcc;
    }
    
    curMotorAvaVel = VelRpmToCm_s(curMotorAvaVel);//转换单位到0.01m/s
    if(gMotorAvaVelValidNum > 0)
    {
        velValidNum = gMotorAvaVelValidNum;
        if(velValidNum > MACC_CAL_PERIOD_NUM)
        {
            velValidNum = MACC_CAL_PERIOD_NUM;
        }
        velDif = curMotorAvaVel - gMotorAvaVel[MVEL_RECORD_NUM - velValidNum];//计算加速度用的速度差
        time = curTime - gMotorAvaVelTime[MACC_CAL_PERIOD_NUM - velValidNum];//计算加速度用的时间差
    }

    //记录速度
    for(k = 1; k < MVEL_RECORD_NUM; k++)
    {
        gMotorAvaVel[k - 1] = gMotorAvaVel[k];
        gMotorAvaSetCurrentRecord[k - 1] = gMotorAvaSetCurrentRecord[k];
    }
    gMotorAvaVel[k - 1] = curMotorAvaVel;
    gMotorAvaSetCurrentRecord[MVEL_RECORD_NUM - 1] = curMotorAvaCurrent;
    
    //记录电流正切换时的索引
    if((gMotorAvaSetCurrentPositiveChangeIndex > -M_SET_CURRENT_EXE_DELAY_NUM)
        && (gMotorAvaSetCurrentPositiveChangeIndex < MVEL_RECORD_NUM))
    {
        gMotorAvaSetCurrentPositiveChangeIndex--; //随着新电流的更新，索引值往前移一
    }
    else if(gMotorAvaVelValidNum > M_SET_CURRENT_EXE_DELAY_NUM)
    {
        for(k = 1; k < MVEL_RECORD_NUM; k++)    //搜索第一个正电流切换点
        {
            if((gMotorAvaSetCurrentRecord[k - 1] < 0) && (gMotorAvaSetCurrentRecord[k] >= 0))
             {
                 gMotorAvaSetCurrentPositiveChangeIndex = k;
                 break;
             }
        }
    }
    //记录电流负切换时的索引
    if((gMotorAvaSetCurrentNegativeChangeIndex > -M_SET_CURRENT_EXE_DELAY_NUM)
        && (gMotorAvaSetCurrentNegativeChangeIndex < MVEL_RECORD_NUM))
    {
        gMotorAvaSetCurrentNegativeChangeIndex--; //随着新电流的更新，索引值往前移一
    }
    else if(gMotorAvaVelValidNum > M_SET_CURRENT_EXE_DELAY_NUM)
    {
        for(k = 1; k < MVEL_RECORD_NUM; k++)    //搜索第一个负电流切换点
        {
            if((gMotorAvaSetCurrentRecord[k - 1] >= 0) && (gMotorAvaSetCurrentRecord[k] < 0))
             {
                 gMotorAvaSetCurrentNegativeChangeIndex = k;
                 break;
             }
        }
    }

    //记录时刻
    for(k = 1; k < MACC_CAL_PERIOD_NUM; k++)
    {
        gMotorAvaVelTime[k - 1] = gMotorAvaVelTime[k];
    }
    gMotorAvaVelTime[k - 1] = curTime;
    
    //计算加速度并记录
    if(gMotorAvaVelValidNum > 0)
    {
        if(time > 0)
        {
            //计算加速度
            velDif = velDif * 1000 / (int32_t)time;
            velDif = Limit(velDif, -navAccLimit, navAccLimit);

            //电机加速度卡尔曼滤波
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
 功能描述  : 记录设定速度及加速度
 输入参数  : int32_t curSetCarVel  当前设定速度,单位rpm
             uint32_t curTime      当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年3月24日
*****************************************************************************/
void RecordCarSetVelAndAcc(uint32_t motorNum, int32_t curSetCarVel, uint32_t curTime)
{
    uint32_t k, time = 0;
    int32_t velDif = 0;

    curSetCarVel = VelRpmToCm_s(curSetCarVel);  //转换单位到0.01m/s

    //空闲状态下，目标速度从0切换成非0值时，加速度计算只保留最后一个值参与计算
    if(SET_VEL_STATE_IDLE == gStMotorRunState[motorNum].setVelState)
    {
        if((0 != curSetCarVel) && (ABS_VALUE(curSetCarVel) <= 6) && (0 == gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - 1])
            && (0 == gStMotorRunState[motorNum].setCarVel[0]))
        {
            gStMotorRunState[motorNum].setCarVelValidNum = 1;//只保留最近一个设定加速度值参与计算
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81)
            {
                rt_kprintf("M%d-state-idle-to-start,setv:%d.\r\n", motorNum, curSetCarVel);
            }
        }
    }

    //设定速度状态切换并且惯导有效时
    if(IS_UFOONE_FLAG_SET(UFOONE_MOTION_SWITCH_DEAL) && (!IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG)))
    {
        SetVelStateSwitchDeal(motorNum, &curSetCarVel, curTime);
    }
    
    if((gStMotorRunState[motorNum].setCarVelValidNum > 0) && (gStMotorRunState[motorNum].setCarVelValidNum <= SET_VEL_RECORD_NUM))
    {
        velDif = curSetCarVel - gStMotorRunState[motorNum].setCarVel[SET_VEL_RECORD_NUM - gStMotorRunState[motorNum].setCarVelValidNum];    //计算加速度用的速度差
        time = curTime - gStMotorRunState[motorNum].setCarVelTime[SET_VEL_RECORD_NUM - gStMotorRunState[motorNum].setCarVelValidNum];       //计算加速度用的时间差
    }

    //记录速度
    for(k = 1; k < SET_VEL_RECORD_NUM; k++)
    {
        gStMotorRunState[motorNum].setCarVel[k - 1] = gStMotorRunState[motorNum].setCarVel[k];
    }
    gStMotorRunState[motorNum].setCarVel[k - 1] = curSetCarVel;

    //记录时刻
    for(k = 1; k < SET_VEL_RECORD_NUM; k++)
    {
        gStMotorRunState[motorNum].setCarVelTime[k - 1] = gStMotorRunState[motorNum].setCarVelTime[k];
    }
    gStMotorRunState[motorNum].setCarVelTime[k - 1] = curTime;
    
    //计算加速度并记录
    if(gStMotorRunState[motorNum].setCarVelValidNum > 0)
    {
        if(time > 500) //长时间未更新则舍弃之前的数据，并设定目标加速度为0
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
 功能描述  : 记录惯导速度及加速度
 输入参数  : int32_t curCarVel  当前惯导速度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年3月24日
*****************************************************************************/
void RecordCarNavVelAndAcc(int32_t curCarVel, uint32_t curTime)
{
    uint32_t k, time = 0;
    int32_t velDif;
    uint16_t velValidNum;
    int32_t navAccLimit = 400;
    
    //惯导自动模式加速度限制
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
        velDif = curCarVel - gCarVel[NAV_CAR_VEL_RECORD_NUM - velValidNum]; //计算加速度用的速度差
        time = curTime - gCarVelTime[NAV_CAR_VEL_RECORD_NUM - velValidNum]; //计算加速度用的时间差
    }

    //记录速度和时刻
    for(k = 1; k < NAV_CAR_VEL_RECORD_NUM; k++)
    {
        gCarVel[k - 1] = gCarVel[k];
        gCarVelTime[k - 1] = gCarVelTime[k];
    }
    gCarVel[k - 1] = curCarVel;
    gCarVelTime[k - 1] = curTime; 
    
    //计算加速度并记录
    if(gCarVelValidNum > 0)
    {
        if(time > 0)
        {
            //计算惯导加速度
            velDif = velDif * 1000 / (int32_t)time;
            velDif = Limit(velDif, -navAccLimit, navAccLimit);

            //惯导加速度卡尔曼滤波
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
 功能描述  : 电机打滑判断
 输入参数  : uint32_t motorNum       电机序号
             int32_t motorVel        电机速度，单位cm/s
             int32_t targetVel       实际目标速度，单位cm/s
             int32_t setAcc          设定加速度，单位cm/s^2
             uint32_t processTimeMs  处理周期
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年5月29日
*****************************************************************************/
void MotorSkidJudge(uint32_t motorNum, int32_t motorVel, int32_t targetVel, int32_t setAcc, uint32_t processTimeMs)
{
    if (processTimeMs == 0)
    {
        processTimeMs = 10;   //打滑累积时间基数不能为零
    }
    
    if (((motorVel - targetVel > 150) && (IS_ACC_GTE_0(targetVel, setAcc)))        //加速段或匀速段，电机反馈速度远大于实际目标速度
	 || ((targetVel - motorVel > 150) && (!(IS_ACC_GTE_0(targetVel, setAcc))))     //或减速段，电机反馈速度远小于实际目标速度
	   )
    {
        gStMotorRunState[motorNum].skidFlag = 1;            //电机打滑标志
        gStMotorRunState[motorNum].skidOffCnt= 0;
        
    }
    else
    {
        if ((ABS_VALUE(motorVel - targetVel) < 50)                                   //电机反馈速度贴合实际目标速度
         || ((targetVel - motorVel >= 80) && (IS_ACC_GTE_0(targetVel, setAcc)))    //或 加速段或匀速段，电机性能不够，未打滑情况下无法跟随实际目标速度
           )
        {
            if (gStMotorRunState[motorNum].skidOffCnt >= 5) //不满足打滑条件，且连续5次进入
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
    if(gStMotorRunState[motorNum].skidTotalTime > 5000)    //持续打滑5000ms
    {
        gStMotorRunState[motorNum].skidTotalTime = 0;   //防止连续写故障
        //SetErrorCode(motorNum, ERROR_CODE_CONTINUOUS_SKID, ERROR_L_NORMAL);
        return;
    }
}
/*****************************************************************************
 功能描述  : 电机过载值记录更新缓存指针，指向下个缓冲位置
 输入参数  : uint32_t motorNum
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年7月14日
*****************************************************************************/
void MotorOverloaderUpdateIndex(uint32_t motorNum)
{
    uint32_t currentLimitAmp, currentLimitTime0_01s;

    //最大1.2倍过载以内不进行过载判断
    if((gStMotorData[motorNum].normalCurrent * 6 / 5)  > gStMotorData[motorNum].limitCurrent)
    {
        IIt_CLEAR_FLAG(motorNum, IIt_JUDGE_VALID);
        return;
    }
    
    //mA转换到单位A
    currentLimitAmp = gStMotorData[motorNum].limitCurrent / 1000;
    if((0 == currentLimitAmp) || (currentLimitAmp > 500))   //目前只支持500A电流范围内的计算
    {
        IIt_CLEAR_FLAG(motorNum, IIt_JUDGE_VALID);
        return;
    }

    //ms转换到单位0.01s
    currentLimitTime0_01s = gStMotorData[motorNum].overCurrentTime / 10;
    if((0 == currentLimitTime0_01s) || (currentLimitTime0_01s > 15000)) //目前只支持150s过载时间内的计算
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

    //过载限制值计算，电流单位A，时间单位0.01s
    gStMotorRunState[motorNum].IItLimit = currentLimitAmp * currentLimitAmp * currentLimitTime0_01s;

    if(DEBUG_DATA_TYPE_96)
    {
        rt_kprintf("M%d overcurrent%d limit:%d.\r\n", motorNum, gStMotorRunState[motorNum].IItIndex, gStMotorRunState[motorNum].IItLimit);
    }
}
/*****************************************************************************
 功能描述  : 电机过载判断
 输入参数  : uint32_t motorNum       电机序号
             rt_bool_t motorStartFlag电机启动标志
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年7月13日
*****************************************************************************/
void MotorOverloaderJudge(uint32_t motorNum, rt_bool_t motorStartFlag)
{
    uint32_t curTime, temp;
    uint32_t setCurrentAbs, IItTemp;
    
    if(gStMotorRunState[motorNum].IItIndex >= M_BUFFER_IIt_LEN)
    {
        gStMotorRunState[motorNum].IItIndex = 0;
    }

    //启动时更新缓存索引指针
    if(motorStartFlag)
    {
        MotorOverloaderUpdateIndex(motorNum);
        if(0 != gStMotorRunState[motorNum].IItOverTimeRecord)   //记录过过载时刻，则重新启动电机时设为等待恢复状态，不允许再次过载
        {
            IIt_SET_FLAG(motorNum, IIt_ENTER_WAIT_RECOVERY_FLAG);
        }
    }
    
    if(IIt_IS_SET_FLAG(motorNum, IIt_JUDGE_VALID))
    {
        curTime = HAL_GetTick();

        //获取设定电流绝对值
        setCurrentAbs = ABS_VALUE(gStMotorRunState[motorNum].setCurrent);
        
        //单次过载值计算
        if(setCurrentAbs > gStMotorData[motorNum].normalCurrent)
        {
            temp = curTime - gStMotorRunState[motorNum].IItOverLastTime;
            if(0 == gStMotorRunState[motorNum].IItOverLastTime)
            {
                gStMotorRunState[motorNum].IItOverLastTime = curTime;
            }
            else if(temp >= 100)    //0.1s计算1次
            {
                //检查是否切换至反向过载，如切换则存入下个缓冲位置
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

                //如果处于等待恢复期又进入到过载状态，则报警告
                if(IIt_IS_SET_FLAG(motorNum, IIt_ENTER_WAIT_RECOVERY_FLAG))
                {
                    if(IS_ENABLE_SET_WARNING_CODE)
                    {
                        SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_OVER_LOADER_NEED_REST);
                    }
                }
                
                setCurrentAbs /= 1000;  //mA转换到单位A
                temp = temp / 10;   //过载时间单位0.01s
                //IIt累加
                gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex] += setCurrentAbs * setCurrentAbs * temp;
                if(0 == gStMotorRunState[motorNum].IItStartTime[gStMotorRunState[motorNum].IItIndex])
                {
                    gStMotorRunState[motorNum].IItStartTime[gStMotorRunState[motorNum].IItIndex] = curTime;
                }
                gStMotorRunState[motorNum].IItOverLastTime = curTime;

                //单次过载报警告
                if(gStMotorRunState[motorNum].IItBuffer[gStMotorRunState[motorNum].IItIndex] >= gStMotorRunState[motorNum].IItLimit)
                {
                    if(IS_ENABLE_SET_WARNING_CODE)
                    {
                        SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_OVER_LOADER_SINGLE);
                    }
                    if(0 == gStMotorRunState[motorNum].IItOverTimeRecord)   //记录过载时刻，需休息3分钟
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
    
        //1分半钟内累计过载值计算(包括加速和减速)
        IItTemp = 0;
        for(temp = gStMotorRunState[motorNum].IItIndex + 1; ; temp++)
        {
            if(temp >= M_BUFFER_IIt_LEN)
            {
                temp = 0;
            }
            if(curTime - gStMotorRunState[motorNum].IItStartTime[temp] < 90000) //一分半钟内的有效数据
            {
                IItTemp += gStMotorRunState[motorNum].IItBuffer[temp];
            }
            if(temp == gStMotorRunState[motorNum].IItIndex)
            {
                break;
            }
        }
        //超过2倍过载时间时报电机累计过载警告
        if((IItTemp >> 1) >= gStMotorRunState[motorNum].IItLimit)
        {
            if(IS_ENABLE_SET_WARNING_CODE)
            {
                SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_OVER_LOADER_ONE_MINUTE);
            }
            if(0 == gStMotorRunState[motorNum].IItOverTimeRecord)   //记录过载时刻，需休息3分钟
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
 功能描述  : 电机过载等待恢复(等电机冷却)
 输入参数  :
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年7月14日
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
            if(0 != gStMotorRunState[motorNum].IItOverTimeRecord)   //记录过过载时刻
            {
                if(curTime - gStMotorRunState[motorNum].IItOverTimeRecord >= 180000)   //距离上次过载时刻超过3分钟时可退出过载报警状态
                {
                    gStMotorRunState[motorNum].IItOverTimeRecord = 0;
                    
                    IIt_CLEAR_FLAG(motorNum, IIt_ENTER_WAIT_RECOVERY_FLAG); //清除等待恢复标志
                    
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
 功能描述  : 计算pid一些运行参数，包括双电机同步参数，周期，pid启动标志，速度值，惯导速比
 输入参数  : uint32_t motorNum
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年10月13日
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
    
    //周期计时
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
        //获取电机速度
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen驱动器
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
            //电机速度记录
            if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
            {
                gStMotorRunState[motorNum].motorVelRecord = CalRatioVelRpm(motorNum);
                gStMotorRunState[M_RIGHT].motorVelRecord = CalRatioVelRpm(M_RIGHT);
                gStMotorRunState[M_LEFT_ONE].motorVelRecord = CalRatioVelRpm(M_LEFT_ONE);
                gStMotorRunState[M_RIGHT_ONE].motorVelRecord = CalRatioVelRpm(M_RIGHT_ONE);
                gStMotorRunState[motorNum].pidRunFlag = RT_TRUE;    //开启pid运算
                gStMotorRunState[M_RIGHT].pidRunFlag = RT_TRUE;    //开启pid运算
                gStMotorRunState[M_LEFT_ONE].pidRunFlag = RT_TRUE;    //开启pid运算
                gStMotorRunState[M_RIGHT_ONE].pidRunFlag = RT_TRUE;    //开启pid运算
                //记录电机转速和电流
                liTemp = (gStMotorRunState[motorNum].setCurrent + gStMotorRunState[M_RIGHT].setCurrent
                    + gStMotorRunState[M_LEFT_ONE].setCurrent + gStMotorRunState[M_RIGHT_ONE].setCurrent) >> 2;
                RecordMotorVelAndAcc(liMotorVel, liTemp, HAL_GetTick());
            }
            else if(lExeFlag & M_R_ACCORD_L_FLAG)   //左右侧电机独立控制标志
            {
                gStMotorRunState[motorNum].motorVelRecord = liMotorVel;
                if(DEBUG_DATA_TYPE_92)
                {
                    rt_kprintf("M%d get speed:%d.\r\n", M_RIGHT, HAL_GetTick());
                }
                //获取电机速度
                if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[M_RIGHT].driverType))    //canopen驱动器
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
                    gStMotorRunState[motorNum].pidRunFlag = RT_TRUE;    //开启pid运算
                    gStMotorRunState[M_RIGHT].pidPeriodTime = gStMotorRunState[motorNum].pidPeriodTime;
                    gStMotorRunState[M_RIGHT].motorVelRecord = liMotorVel;
                    gStMotorRunState[M_RIGHT].pidRunFlag = RT_TRUE;     //开启pid运算
                    liMotorVel = (gStMotorRunState[motorNum].motorVelRecord + liMotorVel) >> 1;  //左右侧合速度
                    //记录电机转速和电流
                    liTemp = (gStMotorRunState[motorNum].setCurrent + gStMotorRunState[M_RIGHT].setCurrent) >> 1;
                    RecordMotorVelAndAcc(liMotorVel, liTemp, HAL_GetTick());
                }
            }
            else
            {
                gStMotorRunState[motorNum].motorVelRecord = liMotorVel;
                gStMotorRunState[motorNum].pidRunFlag = RT_TRUE;    //开启pid运算
                //记录电机转速和电流
                RecordMotorVelAndAcc(liMotorVel, gStMotorRunState[motorNum].setCurrent, HAL_GetTick());
            }
        }
    }

    //轮速计算
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

    //计算惯导速比(非紧急停止下)
    if((gStUfoData.flag & UFO_MOTOR_AS_FEEDBACK_FLAG)    //电机速度作为pid反馈标志
        && (sys_para->CAR_RTinf.Link & LINK_PC)              //PC控制时才动态计算速比
        && (M_LEFT == motorNum)
        && (RT_TRUE == gStMotorRunState[motorNum].pidRunFlag)
        && (!IS_EMERGENCY_STOP))
    {
        lTempVel = VelRpmToCm_s(gStMotorRunState[motorNum].setTargetVel);
        if((ABS_VALUE(gCarVel[NAV_CAR_VEL_RECORD_NUM - 1]) > 20)   //惯导速度不为0
            &&(ABS_VALUE(gCarVel[0]) > 20)
            && (ABS_VALUE(lTempVel) > 30)       //油门值大于30cm/s
            && (gMotorAvaVelValidNum >= CALRATIOL_FILTER_NUM)
            && (gCarVelValidNum >= CALRATIOL_FILTER_NUM)    //存满一组数据
            )
        {
            //取惯导速度滤波
            for(k = 1; k <= CALRATIOL_FILTER_NUM; k++)
            {
                liTempData[k] = gCarVel[NAV_CAR_VEL_RECORD_NUM - k];
            }
            FilterMidAverage(liTempData, CALRATIOL_FILTER_NUM, CALRATIOL_EXCLUDE_NUM, lTempVel, k, liTemp);
            lTempVel = ABS_VALUE(lTempVel);
            //取电机速度滤波
            for(k = 1; (k <= CALRATIOL_FILTER_NUM) && (k <= MVEL_RECORD_NUM); k++)
            {
                liTempData[k] = gMotorAvaVel[MVEL_RECORD_NUM - k];
            }
            FilterMidAverage(liTempData, CALRATIOL_FILTER_NUM, CALRATIOL_EXCLUDE_NUM, liMotorVel, k, liTemp);
            liMotorVel = ABS_VALUE(liMotorVel);
            //计算速比
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
 功能描述  : 惯导速度反方向判定(双重标准滤波，一个松一些，一个严格一些，如果松些判断有正负值突变太大，则按严格一些的判断)
 输入参数  : int32_t setTargetVel  设定速度
             int32_t mVelFilter    电机滤波速度
 输出参数  : rt_bool_t  为真表示反向
 作    者  : 刘鹏
 日    期  : 2023年7月20日
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

    //依次对电机速度和惯导速度绝对值进行比较，如果接近则有效值+1，有MVEL_CAR_DIR_JUDGE_TOTAL_NUM个值接近则判断方向有效
    if(gMotorAvaVelValidNum >= MVEL_CAR_DIR_JUDGE_TOTAL_NUM)
    {
        for(k = MVEL_RECORD_NUM - 1; k >= 0; k--)
        {
            //电机速度绝对值与惯导速度绝对值差值的绝对值
            lTemp = ABS_VALUE(gMotorAvaVel[k]);
            if(0 != lTemp)
            {
                lTemp = ABS_VALUE(lTemp - ABS_VALUE(gCarVel[k])) * 100 / lTemp; //差值占电机的百分比
                if(k == MVEL_RECORD_NUM - 1)
                {
                    lTempOne = lTemp;
                }
                if(lTemp < 30)  //30%以内都认为接近
                {
                    lDirJudgeValidCnt++;
                    if(lDirJudgeValidCnt <= MVEL_CAR_DIR_JUDGE_NUM)    //参与方向判断的个数内进行正负方向判定
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
                    if(lDirJudgeValidCnt >= lDirJudgeTotalNum)  //找到满足要求的数据则退出
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
    //宽松判断惯导方向，如果电机判定有效，则依据电机方向而定
    if((lDirJudgeValidCnt >= lDirJudgeTotalNum)
        && ((positiveCnt > lTemp) || (negativeCnt > lTemp)))
    {
        if(negativeCnt > lTemp) //电机反向
        {
            lbNegativeDirFlag = RT_TRUE;
        }

        //宽松判断存在大的正负值突变则进行严格判断
        if(((gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] > 50) && (RT_TRUE == lbNegativeDirFlag))
            || ((gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] < -50) && (RT_FALSE == lbNegativeDirFlag)))
        {
            lbNegativeDirFlag = RT_FALSE;
            /////严格判断惯导方向
            lTemp = lDirJudgeTotalNum - 1;
            //如果电机判定有效，则依据电机方向而定
            if((lDirJudgeValidCnt >= lDirJudgeTotalNum)
                && ((strictPositiveCnt > lTemp) || (strictNegativeCnt > lTemp)))
            {
                if(strictNegativeCnt > lTemp)   //电机反向
                {
                    lbNegativeDirFlag = RT_TRUE;
                }
                if(DEBUG_DATA_TYPE_90)
                {
                    rt_kprintf("Car-strict-dir-judge-valid:p:%d,n:%d,nDir:%d.\r\n", strictPositiveCnt, strictNegativeCnt, lbNegativeDirFlag);
                }
                return lbNegativeDirFlag;   //严格判定正确则直接返回惯导判定的方向
            }
        }
        else    //宽松判定正确则直接返回惯导判定的方向
        {
            return lbNegativeDirFlag;
        }
    }

    /////宽松或严格判定未通过，则进行如下判定
    lbNegativeDirFlag = RT_FALSE;
    //电机滤波速度为0或上次惯导速度为0或者数据有效个数不够时依据目标速度而定
    if((0 == mVelFilter) || (0 == gCarVel[NAV_CAR_VEL_RECORD_NUM - 1])
        || (gMotorAvaVelValidNum < lDirJudgeTotalNum))
    {
        if(setTargetVel < 0)
        {
            lbNegativeDirFlag = RT_TRUE;
        }
    }
    //否则保留上次的方向
    else if(gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] < 0)
    {
        lbNegativeDirFlag = RT_TRUE;
    }

    return lbNegativeDirFlag;
}
/*****************************************************************************
 功能描述  : 获取电机和惯导滤波速度
 输入参数  : int32_t* liTempData        临时变量
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年7月21日
*****************************************************************************/
void GetMotorAndNavFilterVel(int32_t* liTempData)
{
    int32_t liTemp, liTempVel = 0, positiveCurrentNum;
    int8_t i, k, li8Temp, liPositiveFilterNum, liNegativeFilterNum, li8Index[CAR_MOTOR_MAX_OR_MIN_FILTER_NUM];
    
    //统计正电流个数
    positiveCurrentNum = 0;
    for(i = 0; i < MVEL_RECORD_NUM; i++)
    {
        if(gMotorAvaSetCurrentRecord[i] >= 0)
        {
            positiveCurrentNum++;
        }
        if(i < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM)
        {
            li8Index[i] = 0;    //赋初值
        }
    }

    liPositiveFilterNum = (positiveCurrentNum + 2) * CAR_MOTOR_MAX_OR_MIN_FILTER_NUM / MVEL_RECORD_NUM;
    liNegativeFilterNum = CAR_MOTOR_MAX_OR_MIN_FILTER_NUM - liPositiveFilterNum;
    //正电流时取速度最小值滤波时
    if(liPositiveFilterNum > 0)
    {
        //先预存入
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
                    liTempData[k] = gCarVel[i]; //延迟周期内取惯导速度
                }
                else
                {
                    liTempData[k] = gMotorAvaVel[i];
                }
                li8Index[k] = i;    //记录序号，用于和惯导速度值对应位置对应
                k++;
            }
        }

        //查找后续队列，用小值替换掉之前队列里的最大值
        liTemp = 1; //需要搜寻最大值
        for(; i < MVEL_RECORD_NUM; i++)
        {
            //找出最大的值的序号
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
            //如果当前查找的值小于队列里的最大的值则进行替换
            if(gMotorAvaSetCurrentRecord[i] >= 0)
            {
                if((i >= gMotorAvaSetCurrentPositiveChangeIndex)
                    && (i < gMotorAvaSetCurrentPositiveChangeIndex + M_SET_CURRENT_EXE_DELAY_NUM))
                {
                    liTempVel = gCarVel[i]; //延迟周期内取惯导速度
                }
                else
                {
                    liTempVel = gMotorAvaVel[i];
                }
                if(liTempVel < liTempData[li8Temp])
                {
                    liTemp = 1; //需要重新搜寻最大值
                    liTempData[li8Temp] = liTempVel;
                    li8Index[li8Temp] = i;  //记录序号，用于和惯导速度值对应位置对应
                }
            }
        }
    }
    //负电流时取速度最大值滤波时
    if((liNegativeFilterNum > 0) && (liPositiveFilterNum < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM))
    {
        //先预存入
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
                    liTempData[k] = gCarVel[i]; //延迟周期内取惯导速度
                }
                else
                {
                    liTempData[k] = gMotorAvaVel[i];
                }
                li8Index[k] = i;    //记录序号，用于和惯导速度值对应位置对应
                k++;
            }
        }

        //查找后续队列，用大值替换掉之前队列里的最小值
        liTemp = 1; //需要搜寻最小值
        for(; i < MVEL_RECORD_NUM; i++)
        {
            //找出最小的值的序号
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
            //如果当前查找的值大于队列里的最小的值则进行替换
            if(gMotorAvaSetCurrentRecord[i] < 0)
            {
                if((i >= gMotorAvaSetCurrentNegativeChangeIndex)
                    && (i < gMotorAvaSetCurrentNegativeChangeIndex + M_SET_CURRENT_EXE_DELAY_NUM))
                {
                    liTempVel = gCarVel[i]; //延迟周期内取惯导速度
                }
                else
                {
                    liTempVel = gMotorAvaVel[i];
                }
                if(liTempVel > liTempData[li8Temp])
                {
                    liTemp = 1; //需要重新搜寻最小值
                    liTempData[li8Temp] = liTempVel;
                    li8Index[li8Temp] = i;  //记录序号，用于和惯导速度值对应位置对应
                }
            }
        } 
    } 

    //电机速度打印
    if(DEBUG_DATA_TYPE_90)
    {
        rt_kprintf("Filter-motorvel");
        for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
        {
            rt_kprintf(",%d", liTempData[k]);
        }
        rt_kprintf(".\r\n");
    }
    
    //惯导速度打印
    if(DEBUG_DATA_TYPE_90)
    {
        rt_kprintf("Filter-navvel");
        for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
        {
            rt_kprintf(",%d", gCarVel[li8Index[k]]);
        }
        rt_kprintf(".\r\nFilter-index");
    }

    //将最大差值的n个放在最后
    liTemp = 0;
    while(liTemp < CAR_MOTOR_FILTER_EXCLUDE_NUM)
    {
        li8Temp = -1;
        for(k = 0; k < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM; k++)
        {
            if(-1 == li8Index[k])
            {
                continue;       //已经无效的数据跳过
            }
            else if(-1 == li8Temp)
            {
                li8Temp = k;    //li8Temp赋初值
            }
            else if(ABS_VALUE(liTempData[li8Temp] - gCarVel[li8Index[li8Temp]]) < ABS_VALUE(liTempData[k] - gCarVel[li8Index[k]]))
            {
                li8Temp = k;    //li8Temp替换成最大差值索引号
            }
        }
        if((li8Temp >= 0) && (li8Temp < CAR_MOTOR_MAX_OR_MIN_FILTER_NUM))
        {
            li8Index[li8Temp] = -1; //查找到的最大差值赋值索引为-1
        }
        liTemp++;   //搜索下个次大值
    }

    //未赋值最大差值的求平均值，并将索引打印出来
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
    //求最终滤波结果

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
 功能描述  : 电流自动均衡
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年12月14日
*****************************************************************************/
static void CurrentBanlance(void)
{
    int32_t lBanlanceCurrent = 0;

    if(RT_FALSE == gNoLoadFlag) //有负载时才进行电流均衡
    {
        if((gStMotorData[M_RIGHT].speedGrad > 0) && (gStMotorData[M_RIGHT].speedGrad < 3000))
        {
            lBanlanceCurrent = gStMotorData[M_RIGHT].speedGrad;
        }
    }
    if(!(sys_para->CAR_RTinf.Link & LINK_PC))   //非PC控制均衡电流小一半
    {
        lBanlanceCurrent >>= 1;
    }

    //差速驱动时不进行电流均衡
    if(BACK_DEFF_NONE != gBackWheelDeffDriverMode)
    {
        lBanlanceCurrent = 0;
    }
    
    //四轮独立控制
    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))
    {
        //右侧电流大
        if(gStMotorRunState[M_RIGHT].setCurrent + gStMotorRunState[M_RIGHT_ONE].setCurrent >= 
            gStMotorRunState[M_LEFT].setCurrent + gStMotorRunState[M_LEFT_ONE].setCurrent)
        {
            //右边减电流
            if(gStMotorRunState[M_RIGHT].setCurrent >= gStMotorRunState[M_RIGHT_ONE].setCurrent)
            {
                gStMotorRunState[M_RIGHT].setCurrent -= lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_RIGHT_ONE].setCurrent -= lBanlanceCurrent;
            }
            //左边加电流
            if(gStMotorRunState[M_LEFT].setCurrent >= gStMotorRunState[M_LEFT_ONE].setCurrent)
            {
                gStMotorRunState[M_LEFT_ONE].setCurrent += lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_LEFT].setCurrent += lBanlanceCurrent;
            }
        }
        //左侧电流大
        else
        {
            //右边加电流
            if(gStMotorRunState[M_RIGHT].setCurrent >= gStMotorRunState[M_RIGHT_ONE].setCurrent)
            {
                gStMotorRunState[M_RIGHT_ONE].setCurrent += lBanlanceCurrent;
            }
            else
            {
                gStMotorRunState[M_RIGHT].setCurrent += lBanlanceCurrent;
            }
            //左边减电流
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
    //后两轮控制
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
 功能描述  : 差速转向目标轮速计算
 输入参数  : uint32_t motorNum  电机序号
             int32_t tarSetVel  目标合速度
 输出参数  : int32_t    返回差速后的目标速度
 作    者  : 刘鹏
 日    期  : 2023年12月26日
*****************************************************************************/
int32_t WheelDeffTarSetVelCal(uint32_t motorNum, int32_t tarSetVel)
{
    float lfTemp, lfTempOne;
    
    if(ABS_VALUE(gSteeringAngleVelDiff) >= 5)   //转角大时才计算
    {
        lfTemp = 0.67f / tan(ABS_VALUE(gSteeringAngleVelDiff) * DEF_PI / 180.0f);    //计算转向原点到中轴线的距离，轴距1340mm，一半是0.67m
        lfTempOne = 0.67f * 0.67f;
        if((((M_LEFT == motorNum) || (M_LEFT_ONE == motorNum)) && (gSteeringAngleVelDiff < 0))
            || (((M_RIGHT == motorNum) || (M_RIGHT_ONE == motorNum)) && (gSteeringAngleVelDiff > 0)))
        {
            //计算左轮与中轴线的转弯半径比值，轮距1480mm，一半是0.74m
            lfTemp = sqrt(((lfTemp - 0.74f) * (lfTemp - 0.74f) + lfTempOne) / (lfTemp * lfTemp + lfTempOne));
            tarSetVel = lfTemp * tarSetVel; //求取目标速度
        }
        else
        {
            //计算右轮与中轴线的转弯半径比值，轮距1480mm，一半是0.74m
            lfTemp = sqrt(((lfTemp + 0.74f) * (lfTemp + 0.74f) + lfTempOne) / (lfTemp * lfTemp + lfTempOne));
            tarSetVel = lfTemp * tarSetVel; //求取目标速度
        }
    }

    return tarSetVel;
}
/*****************************************************************************
 功能描述  : 电机速度波形分析（波峰波谷）
 输入参数  : int32_t velErr 当前速度和目标速度速度误差值
             uint32_t curTime      当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2024年1月18日
*****************************************************************************/
static void MotorVelWaveAnalysis(int32_t velErr, uint32_t curTime)
{
    static uint8_t lExitLoadCnt = 0;
    uint32_t lTime = 0;
    int32_t liTemp = 0, lCurrentIndex = MVEL_RECORD_NUM - MACC_CAL_PERIOD_NUM;
    
    //波峰值判断
    if((gMotorLastVelErr > 0) && (gMotorLastLastVelErr < gMotorLastVelErr) && (velErr < gMotorLastVelErr))
    {
        if(gMotorVelErrPeakTime > gMotorVelErrValleyTime)   //上次更新的波峰
        {
            if(gMotorLastVelErr > gMotorVelErrPeak) //新的波峰值大，则替代上次的波峰值
            {
                gMotorVelErrPeak = gMotorLastVelErr;
                gMotorVelErrPeakTime = curTime;
                lTime = gMotorVelErrValleyTime;
            }
        }
        else    //上次更新的波谷值，则直接更新新的波峰值
        {
            gMotorVelErrPeak = gMotorLastVelErr;
            gMotorVelErrPeakTime = curTime;
            lTime = gMotorVelErrValleyTime;
        }
    }
    //波谷值判断
    else if((gMotorLastVelErr < 0) && (gMotorLastLastVelErr > gMotorLastVelErr) && (velErr > gMotorLastVelErr))
    {
        if(gMotorVelErrPeakTime > gMotorVelErrValleyTime)   //上次更新的波峰，则直接更新新的波谷值
        {
            gMotorVelErrValley = gMotorLastVelErr;
            gMotorVelErrValleyTime = curTime;
            lTime = gMotorVelErrPeakTime;
        }
        else    //上次更新的波谷值
        {
            if(gMotorLastVelErr < gMotorVelErrValley)   //新的波谷值小，则替代上次的波谷值
            {
                gMotorVelErrValley = gMotorLastVelErr;
                gMotorVelErrValleyTime = curTime;
                lTime = gMotorVelErrPeakTime;
            }
        }
    }

    if(velErr != gMotorLastVelErr)    //不相等才赋值，防止判断不了波峰和波谷
    {
        gMotorLastLastVelErr = gMotorLastVelErr;
        gMotorLastVelErr = velErr;
    }

    //快速振荡判断
    if(0 != lTime)
    {
        if(DEBUG_DATA_TYPE_90)
        {
            rt_kprintf("New p:%d,%lu,v:%d,%lu.\r\n", gMotorVelErrPeak, gMotorVelErrPeakTime,
                gMotorVelErrValley, gMotorVelErrValleyTime);
        }
        //波峰波谷振荡超过0.4m/s，且时间在快速振荡周期内
        if((gMotorVelErrPeak >= 40) && (gMotorVelErrValley <= -40))
        {
            lTime = curTime - lTime;
            if((lTime > MOTOR_NO_LOAD_JUDGE_MIN_HALF_PERIOD) && (lTime < MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD))  //快速振荡认为无负载
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
        gMotorVelOscillationFlag = RT_FALSE;    //超出振荡周期清除振荡标志
    }

    lTime = 0;
    //加速度振荡判断
    if(gMotorAvaVelValidNum >= MACC_CAL_PERIOD_NUM)
    {
        //峰值加速度判断
        if(gMotorCurAcc > 0)
        {
            if(gMotorAvaSetCurrentRecord[lCurrentIndex] > 0)    //只判断电流大于0的情况，防止减速段打滑
            {
                if(gMotorCurAcc < gMotorAccTemp)    //加速度变小，找到新的加速度峰值
                {
                    if(gMotorAccMaxTime > gMotorAccMinTime)   //上次更新的波峰
                    {
                        if(gMotorAccTemp > gMotorAccMax) //新的波峰值大，则替代上次的波峰值
                        {
                            gMotorAccMax = gMotorAccTemp;
                            gMotorAccMaxCurrent = gMotorAvaSetCurrentRecord[lCurrentIndex];
                            gMotorAccMaxTime = curTime;
                            lTime = gMotorAccMinTime;
                        }
                    }
                    else    //上次更新的波谷值，则直接更新新的波峰值
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
        //谷值减速度判断
        else
        {
            if(gMotorAvaSetCurrentRecord[lCurrentIndex] < 0)    //只判断电流小于0的情况，防止加速段打滑
            {
                if(gMotorCurAcc > gMotorAccTemp)    //加速度变大，找到新的速度谷值
                {
                    if(gMotorAccMaxTime > gMotorAccMinTime)   //上次更新的波峰，则直接更新新的波谷值
                    {
                        gMotorAccMin = gMotorAccTemp;
                        gMotorAccMinCurrent = gMotorAvaSetCurrentRecord[lCurrentIndex];
                        gMotorAccMinTime = curTime;
                        lTime = gMotorAccMaxTime;
                    }
                    else    //上次更新的波谷值
                    {
                        if(gMotorAccTemp > gMotorAccMin) //新的波谷值小，则替代上次的波谷值
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

    //加速度判断
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
        //波峰波谷振荡超过1m/s2，且时间在快速振荡周期内，且电流较小
        if((gMotorAccMax >= 100) && (gMotorAccMin <= -100)
            && gMotorAccMaxCurrent < (gMotorAccMax * liTemp / 100)
            && gMotorAccMinCurrent > (gMotorAccMin * liTemp / 100))
        {
            lTime = curTime - lTime;
            if((lTime > MOTOR_NO_LOAD_JUDGE_MIN_HALF_PERIOD) && (lTime < MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD))  //快速振荡认为无负载
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
        gMotorAccOscillationFlag = RT_FALSE;    //超出振荡周期清除振荡标志
    }
    
    //无负载判断
    if(sys_para->CAR_RTinf.Link & LINK_PC)  //PC控制时直接认为是非空载
    {
        gNoLoadFlag = RT_FALSE;
    }
    else if(gMotorVelOscillationFlag && gMotorAccOscillationFlag)   //速度和加速度同时振荡，则认为是空载
    {
        gNoLoadFlag = RT_TRUE;
    }

    //有负载判断
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
        //加速度超过0.5m/s2，且电流较大，则计数加一
        if(((gMotorCurAcc >= 50) && (gMotorAvaSetCurrentRecord[lCurrentIndex] > liTemp))
            || ((gMotorCurAcc <= -50) && (gMotorAvaSetCurrentRecord[lCurrentIndex] < liTemp)))
        {
            lExitLoadCnt++;
            if(lExitLoadCnt >= 10)  //连续10次电流和加速度匹配则认为有负载
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
 功能描述  : pid电流转速调节
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年7月15日
*****************************************************************************/
static void PIDCurrentAdjust(uint32_t motorNum, uint32_t processTimeMs)
{
    int32_t liTemp, liRealVel = 0, liMotorVelCal, liMotorLRVel, liCarVel = 0, liSetTargetVelRef, liTargetVel, liTargetVelRef, k, liWheelVel[W_TOTAL_NUM];
    int32_t liTempData[MAX(MVEL_FILTER_NUM, MAX(NAV_CAR_VEL_RECORD_NUM, NAV_ACC_RECORD_NUM))];
    uint8_t lExeFlag = 0;
    uint32_t luOilPressure = 0;

    //判断某些代码是否执行，左右是否同步标志
    //如果左右侧电机独立控制，则右侧电机参数在左侧电机已计算完毕
    if(!((M_LEFT != motorNum)
        && (gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG)))  //左右侧电机独立控制标志
    {
        lExeFlag |= M_PID_EXE_FLAG;
    }
    
    if((M_LEFT == motorNum) && IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
    {
        lExeFlag |= M_FOUR_ALONE_FLAG;
        //等待4个电机pid都已启动
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
        && (gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG))  //左右侧电机独立控制标志
    {
        lExeFlag |= M_R_ACCORD_L_FLAG;
        //等待左右pid都已启动
        if((M_PID_IDLE_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            && (M_PID_RUN_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag))
        {
            return;
        }
    }

    if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
    {
        CalPidRunParas(motorNum, lExeFlag, liTempData, processTimeMs);
    }
    
    //成功取到当前速度值
    gStMotorRunState[motorNum].pidNoExeTime += processTimeMs;
    if((gStMotorRunState[motorNum].pidRunFlag) && (M_PID_RUN_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag))
    {
        gStMotorRunState[motorNum].pidRunFlag = RT_FALSE;
        gStMotorRunState[motorNum].pidNoExeTime = 0;
        //设定的参考目标车速，单位0.01m/s
        liSetTargetVelRef = VelRpmToCm_s(gStMotorRunState[motorNum].setTargetVel);
        //轮速和车速速度单位转换
        liMotorVelCal = VelRpmToCm_s(gStMotorRunState[motorNum].motorVelRecord);   //电机速度转换成车速单位0.01m/s
        liWheelVel[W_LEFT_FRONT] = WheelRpmToCm_s(gWheelSpeed[W_LEFT_FRONT]);      //轮速转换成车速单位0.01m/s
        liWheelVel[W_RIGHT_FRONT] = WheelRpmToCm_s(gWheelSpeed[W_RIGHT_FRONT]);    //轮速转换成车速单位0.01m/s
        
        //左右电机和速度，用于一些故障的判断速度
        liMotorLRVel = liMotorVelCal;
        if(lExeFlag & M_R_ACCORD_L_FLAG)   //右电机依据左电机标志
        {
            liMotorLRVel = (gStMotorRunState[motorNum].motorVelRecord + gStMotorRunState[M_RIGHT].motorVelRecord) >> 1;
            liMotorLRVel = VelRpmToCm_s(liMotorLRVel);
        }
        else if(lExeFlag & M_FOUR_ALONE_FLAG)   //四电机独立标志
        {
            liMotorLRVel = (gStMotorRunState[motorNum].motorVelRecord + gStMotorRunState[M_RIGHT].motorVelRecord
                + gStMotorRunState[M_LEFT_ONE].motorVelRecord + gStMotorRunState[M_RIGHT_ONE].motorVelRecord) >> 2;
            liMotorLRVel = VelRpmToCm_s(liMotorLRVel);
        }
        
        //当前惯导实际车速赋值及方向判断
        if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
        {
            //滤波计算当前电机实际和速度
            for(k = 1; (k <= MVEL_FILTER_NUM) && (k <= MVEL_RECORD_NUM); k++)
            {
                liTempData[k] = gMotorAvaVel[MVEL_RECORD_NUM - k];
            }
            FilterMidAverage(liTempData, MVEL_FILTER_NUM, MVEL_EXCLUDE_NUM, gMotorAvaVelFilter, k, liTemp); //滤波防止惯导速度方向误判

            liCarVel = sys_para->CAR_RTinf.vel;
            if(NavNegativeDirJudge(liSetTargetVelRef, gMotorAvaVelFilter))  //反向判定
            {
                liCarVel = -liCarVel;   //反向
            }
            if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //无惯导标志时惯导速度直接赋值电机速度
            {
                liCarVel = gMotorAvaVelFilter;
            }
        }
        
        //计算限制电流
        if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
        {
            if (!((0 == liSetTargetVelRef) && (ABS_VALUE(gMotorAvaVelFilter) < 150)))   //不满足 设定速度为0且滤波后的电机速度＜150cm/s
            {
                gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].limitCurrent;
            }
        }
        
        //记录惯导车速及加速度
        if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
        {
            RecordCarNavVelAndAcc(liCarVel, HAL_GetTick());
            //计算电机速度和惯导速度的差值绝对值，用于判断惯导准确性
            GetMotorAndNavFilterVel(liTempData);
        }
        
        //liMotorVelFilter = liMotorVelFilter * (int32_t)gStUfoData.ratioVel / gStMotorRunState[motorNum].ratioVel;
        
        //当前实际目标车速(设定加速度大于允许的最大加速度下会与设定的目标车速不一致)，单位0.01m/s
        liTargetVelRef = GetSlopeSpeed(motorNum, liSetTargetVelRef, gMotorAvaVelFilter, CAR_MOTOR_VELERR_OUT_RANGE(25), liTempData, gStMotorRunState[motorNum].pidPeriodTime);
        //计算差速转向时的目标速度的差速值
        liTargetVel = liTargetVelRef;
        if(IS_UFOONE_FLAG_SET(UFOONE_TWO_WHEL_DEFF_DRIV))   //两轮差速驱动
        {
            if(((gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG) && (BACK_DEFF_NONE != gBackWheelDeffDriverMode))  //左右侧电机独立控制标志
                || IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
            {
                liTargetVel = WheelDeffTarSetVelCal(motorNum, liTargetVel);
            }
        }
        //计算动态速比下的实际目标速度
        liTargetVel = liTargetVel * gAutoRatioVel / (int32_t)gStUfoData.ratioVel;
        
        //记录设定速度值
        for(k = 1; k < MVEL_RECORD_NUM; k++)
        {
            gStMotorRunState[motorNum].setVelRecord[k - 1] = gStMotorRunState[motorNum].setVelRecord[k];
        }
        gStMotorRunState[motorNum].setVelRecord[MVEL_RECORD_NUM - 1] = liTargetVelRef;
        //判断停止
        if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
        {
            if(0 == liSetTargetVelRef)    //目标速度为0，则此时判断停止，退出pid调节
            {
                liTemp = 0;
                for(k = MVEL_RECORD_NUM - MVEL_STOP_FILTER_NUM; k < MVEL_RECORD_NUM; k++)
                {
                    if(k >= 0)
                    {
                        liTemp += gMotorAvaVel[k];
                        //增加速度阈值限制，要求滤波数组内所有速度值都在阈值范围内，不在范围内不能判停
                        if(ABS_VALUE(gMotorAvaVel[k]) >= 100)
                        {
                            liTemp = 500;
                            break;
                        }
                    }
                }
                if(ABS_VALUE(liTemp) < (IsBrakeLock ? 200 : 20))    //多次累积之和小于阈值时判断为停止，抱闸的时候放宽条件
                {
                    if(HAL_OK == IsMotorCurrentBelowStartCurrent(motorNum, lExeFlag))
                    {
                        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;    //关闭pid调节
                    }
                }
                else
                {
                    liTemp = 0;
                    //计算电流负切正的个数，正切负也存在同样个数
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
                    //电流存在4次负切正，且速度滤波后值较小则认为是空载抖动，也判断为停止
                    if((liTemp >= 4) && (ABS_VALUE(gMotorAvaVelFilter) < 60))
                    {
                        if(DEBUG_DATA_TYPE_4)
                        {
                            rt_kprintf("Judge-stop n-to-p:%d, vel:%d.\r\n", liTemp, gMotorAvaVelFilter);
                        }
                        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;//关闭pid调节
                    }
                }
                if((ABS_VALUE(gMotorAvaVelFilter) < 40)   //速度接近完全停止时，限定电流为启动电流，防止停止过冲
                    && (gStMotorData[motorNum].startCurrent < gStMotorData[motorNum].normalCurrent)
                    && (gStMotorData[motorNum].startCurrent >= (gStMotorData[motorNum].normalCurrent >> 2)))    
                {
                    gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].startCurrent;
                }
                else if(ABS_VALUE(gMotorAvaVelFilter) < 100) //速度接近停止时，限定电流为额定电流，防止停止过冲
                {
                    gStMotorRunState[motorNum].limitCurrent = gStMotorData[motorNum].normalCurrent;
                }
            }
        }
        //赋值电机当前速度值
        liRealVel = liMotorVelCal;
        //赋值用于加速度补偿的速度误差值，自动控制时以惯导值为准，紧急停止时以电机为准，因为惯导速度已滤波，波动不大，但是增加惯导速度与电机速度差的判断，防止惯导数据异常导致失控
        if((!IS_EMERGENCY_STOP) && (sys_para->CAR_RTinf.Link & LINK_PC) && (!((CAR_MOTOR_VELERR_OUT_RANGE(25)) || (IS_CAR_DECELERATION_AND_SKID(motorNum, liSetTargetVelRef)))))   //PC控制
        {
            liTemp = liTargetVelRef - liCarVel;
            gStMotorRunState[motorNum].skidFlag = 0;
        }
        else
        {
            liTemp = liTargetVel - gMotorAvaVelFilter;
            gStMotorRunState[motorNum].skidFlag = 1;
        }
        //电机速度波形分析，主要获取波峰波谷信息，用于有无负载判断
        if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
        {
            MotorVelWaveAnalysis(gMotorAvaVelFilter - liTargetVel, HAL_GetTick());
        }
        //pid电流调节转速
        CalcPid_incres_driver(motorNum, liTargetVel, liRealVel, liTemp, gStMotorRunState[motorNum].pidPeriodTime);
        //两侧电流均衡
        if(!(lExeFlag & M_PID_EXE_FLAG))   //pid部分代码不执行标志，代表右电机
        {
            CurrentBanlance();
        }
        //电流限制
        /*if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
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
        if(lExeFlag & M_R_ACCORD_L_FLAG)   //右电机依据左电机标志
        {
            gStMotorRunState[M_RIGHT].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
        }
        else if(lExeFlag & M_FOUR_ALONE_FLAG)   //四电机独立标志
        {
            gStMotorRunState[M_RIGHT].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
            gStMotorRunState[M_LEFT_ONE].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
            gStMotorRunState[M_RIGHT_ONE].limitCurrent = gStMotorRunState[M_LEFT].limitCurrent;
        }
        
        gStMotorRunState[motorNum].setCurrent = Limit(gStMotorRunState[motorNum].setCurrent,
            -gStMotorRunState[motorNum].limitCurrent, gStMotorRunState[motorNum].limitCurrent);
        gStMotorRunState[motorNum].setCurrent = Limit(gStMotorRunState[motorNum].setCurrent,
            -gStMotorData[motorNum].limitCurrent, gStMotorData[motorNum].limitCurrent);
        if(lExeFlag & M_PID_EXE_FLAG)   //pid部分代码执行标志
        {
            //启动故障判断
            if(ABS_VALUE(liMotorLRVel) <= 5)
            {
                if(liSetTargetVelRef >= 20)
                {
                    gStMotorRunState[motorNum].infraFaultTimeTotal += gStMotorRunState[motorNum].pidPeriodTime;
                    if(gStMotorRunState[motorNum].infraFaultTimeTotal >= gStMotorData[motorNum].start_fault_time)
                    {
                        gStMotorRunState[motorNum].infraFaultTimeTotal = 0;
                        SetErrorCode(motorNum, ERROR_CODE_MOTOR_START_ERROR, ERROR_L_NORMAL);
                        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;//关闭pid调节
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
            //撞击判断
            if(sys_para->CAR_RTinf.Link & LINK_PC_LOST) //pc数据丢失不进行撞击判断，并且重新恢复撞击判断需要等待
            {
                gStMotorRunState[motorNum].pcLostRecoveryTime = HAL_GetTick();
            }
            //pid部分代码执行标志及PC控制时，并且距离上次通信恢复超过500ms时才进行碰撞检测(防止失联期间数据不准确)
            else if((lExeFlag & M_PID_EXE_FLAG) && (sys_para->CAR_RTinf.Link & LINK_PC)
                && (HAL_GetTick() - gStMotorRunState[motorNum].pcLostRecoveryTime >= 500))
            {
                //未紧急停止以及目标速度不为0的时候才进行碰撞检测
                if(!(IS_EMERGENCY_STOP || (liTargetVelRef == 0)))
                {
                    //以惯导速度回落作为碰撞条件的判断，目标速度大于0.5m/s，滤波速度大于等于100
                    if((ABS_VALUE(liTargetVelRef) > 50) && (ABS_VALUE(gMotorVelFilter) >= 100))
                    {
                        //惯导速度低于目标速度减去速度阈值,惯导速度减速度比设定减速度大于阈值
                        if(((liTargetVelRef > 0) && (liCarVel + 100 < liTargetVelRef) && ((gStMotorRunState[motorNum].setAcc > gStMotorRunState[motorNum].curAcc + 250) && (gStMotorRunState[motorNum].curAcc < -100)))
                            || ((liTargetVelRef < 0) && (liCarVel - 100 > liTargetVelRef) && ((gStMotorRunState[motorNum].setAcc < gStMotorRunState[motorNum].curAcc - 250) && (gStMotorRunState[motorNum].curAcc > 100))))
                        {
                            rt_kprintf("collision-Tar:%d,car:%d,setAcc:%d,curAcc:%d.\r\n", liTargetVelRef, liCarVel,
                                gStMotorRunState[motorNum].setAcc, gStMotorRunState[motorNum].curAcc);
                            if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                            {
                                if(!(gStUfoData.flag & UFO_DISABLE_COLLISION_DETECT))   //禁止碰撞检测
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
                    //碰撞后惯导速度异常的判断，惯导与电机速度差超65%，滤波速度大于等于100
                    if(CAR_MOTOR_VELERR_OUT_RANGE(65)
                        && (ABS_VALUE(gMotorAvaVelFilter) >= 100))
                    {
                        rt_kprintf("collision-Car:%d,mvel:%d.\r\n", gCarVelFilter, gMotorVelFilter);
                        if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
                        {
                            if(!(gStUfoData.flag & UFO_DISABLE_COLLISION_DETECT))   //禁止碰撞检测
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

        if(IS_UFOONE_FLAG_SET(UFOONE_BRAKE_DEC_TEST_FLAG))  //刹车减速度测试
        {
            if(700 == gBrakeValue)  //刹车起作用
            {
                //油压达到一定阈值，防止刹车未起作用时，误给电流0，平板直接自由滑行
                if(AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO) >= ADC1_BRAKE_TEST_THRETHOLDS)  //大于阈值油压(单位0.1Mpa)
                {
                    gStMotorRunState[motorNum].setCurrent = 0;
                }
            }
        }

        //电机速度超调处理
        if(IS_UFOONE_FLAG_SET(UFOONE_MOTION_SWITCH_DEAL))
        {
            if(SET_VEL_STATE_ACC_END == gStMotorRunState[motorNum].setVelState) //超调阶段设定电流直接给0
            {
                gStMotorRunState[motorNum].setCurrent = 0;
            }
        }
        
        //设定电流值
        CHANGE_MOTOR_TARGET_CURRENT(motorNum, gStMotorRunState[motorNum].setCurrent);

        //电机打滑判断
        //MotorSkidJudge(motorNum, liRealVel, liTargetVel, gStMotorRunState[motorNum].setAcc, gStMotorRunState[motorNum].pidPeriodTime);

        //台架测试数据上传
        TestMotorDataUploadProcess(motorNum, RT_TRUE, liTargetVelRef, liRealVel);
        
        //电机过载判断
        MotorOverloaderJudge(motorNum, RT_FALSE);
        
        if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_92)
        {
            luOilPressure = (uint32_t)(AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO)*10);  //读取油压值0.01Mpa
            rt_kprintf("id-%d-t:%d,c:%d,m:%d,s:%d,tv:%d,sa:%d,ca:%d,mf:%d,r:%d,b:%d,sk:%d,oil:%d,wlf:%d,wrf:%d,t:%d.\r\n", gStMotorData[motorNum].idx,
                liTargetVelRef, liCarVel, liRealVel, gStMotorRunState[motorNum].setCurrent, liTargetVel, 
                gStMotorRunState[motorNum].setAcc, gStMotorRunState[motorNum].curAcc, gMotorAvaVelFilter,
                gAutoRatioVel, gBrakeValue, gStMotorRunState[motorNum].skidFlag, luOilPressure, liWheelVel[W_LEFT_FRONT], liWheelVel[W_RIGHT_FRONT], HAL_GetTick());
        }

        //gStMotorRunState[motorNum].infraTimeDelay = 0;
    }
    else if((lExeFlag & M_PID_EXE_FLAG) && (gStMotorRunState[motorNum].pidNoExeTime >= 500)) //长时间pid未执行，可能是获取速度值失败导致
    {
        gStMotorRunState[motorNum].pidNoExeTime = 0;
        gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_END_FLAG;//关闭pid调节
        rt_kprintf("M%d PID exe failed!\r\n", motorNum);
    }
    
    //结束pid调节
    if(M_PID_END_FLAG == gStMotorRunState[motorNum].pidCurrentStartFlag)
    {
        if(lExeFlag & M_R_ACCORD_L_FLAG)   //右电机依据左电机标志
        {
            if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            {
                gStMotorRunState[M_RIGHT].pidCurrentStartFlag = M_PID_END_FLAG; //结束右电机pid运算
            }
        }
        else if(lExeFlag & M_FOUR_ALONE_FLAG)   //四电机独立标志
        {
            if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT].pidCurrentStartFlag)
            {
                gStMotorRunState[M_RIGHT].pidCurrentStartFlag = M_PID_END_FLAG; //结束右电机pid运算
            }
            if(M_PID_RUN_FLAG == gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag)
            {
                gStMotorRunState[M_LEFT_ONE].pidCurrentStartFlag = M_PID_END_FLAG; //结束左一电机pid运算
            }
            if(M_PID_RUN_FLAG == gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag)
            {
                gStMotorRunState[M_RIGHT_ONE].pidCurrentStartFlag = M_PID_END_FLAG; //结束右一电机pid运算
            }
        }
        gStMotorRunState[motorNum].setCurrent = 0;
        if(HAL_OK == CHANGE_MOTOR_TARGET_CURRENT_WITH_JUDGE(motorNum, 0))
        {
            gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_IDLE_FLAG;
            gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_IDLE;    //设定速度状态转为空闲
            if (DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_8A)
            {
                rt_kprintf("M%d:exit PID!\r\n", motorNum);
            }
            ClearPidMem_incres(&gStMotorRunState[motorNum].pidIncresParas);   //清除pid数据
            if(gStMotorData[motorNum].flag & MOTOR_TURN_PID_TO_ACC)   //转向pid当作加速度补偿用
            {
                ClearPidMem_incres(&gStMotorRunState[motorNum].pidIncresParasOne);  //清除pid数据
            }
            SET_BRAKE(0);
            ClearPidMem_incres(&gStMotorRunState[M_BRAKE].pidIncresParas);    //清除pid数据
            gMotorAvaVelFilter = 0;
        }
        else if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8A)
        {
            rt_kprintf("id-%d-c:%d stop failed!\r\n", gStMotorData[motorNum].idx, liRealVel); 
        }
    }
}
/*****************************************************************************
 功能描述  : 刹车pid控制
 输入参数  : uint32_t motorNum       电机序号
             int32_t tar             目标速度
             int32_t liSetTarAcc     目标加速度
             int32_t setCurrent      设定电流值
             uint32_t processTimeMs  线程轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年8月9日
*****************************************************************************/
void PIDBrakeAdjust(uint32_t motorNum, int32_t tar, int32_t liSetTarAcc)
{
    static uint8_t brakeFilter = 0; //松刹车需要滤波
    int32_t setAccThresholds = 200;
    uint16_t advanceDecTime = 200;  //提前刹车时间

    //刹车起作用时的设定加速度阈值
    if(gStMotorData[M_BRAKE].speedGrad > 0)
    {
        setAccThresholds = gStMotorData[M_BRAKE].speedGrad;
    }

    if((gStMotorData[M_BRAKE].start_fault_time > 0) && (gStMotorData[M_BRAKE].start_fault_time < 1000))
    {
        advanceDecTime = gStMotorData[M_BRAKE].start_fault_time;
    }
    
    //减速段刹车调节
    if(IS_UFOONE_FLAG_SET(UFOONE_BRAKE_ADVANCE_START)   //刹车提前启动
        && ((ABS_VALUE(gStMotorRunState[motorNum].setTargetVel) == ABS_VALUE(VelCm_sToRpm(sys_para->CAR_RTinf.max_vel)) && (0 != sys_para->CAR_RTinf.max_vel))) //匀速段
        && ((sys_para->PC_Remote.decRemainTime < advanceDecTime) && (0 != sys_para->PC_Remote.decRemainTime)) //到达提前启动的时间
        && (sys_para->PC_Remote.setDec < -setAccThresholds) //设定减速度大于阈值
        && (ABS_VALUE(tar) >= 200))
    {
        brakeFilter = 0;
        if(700 != gBrakeValue)    //刹车位置有更新
        {
            SET_BRAKE(700);       //紧刹车
        }
    }
    else if(((tar >= 200) && (liSetTarAcc < -setAccThresholds))   //目标速度＞=200，且设定目标加速度＜阈值，即前进减速段且减速度大于设定阈值
    || ((tar <= -200) && (liSetTarAcc > setAccThresholds))    //目标速度＜=200，且设定目标加速度＞阈值，即后退减速段且减速度大于设定阈值
      )   
    {
        brakeFilter = 0;
        if(700 != gBrakeValue)    //刹车位置有更新
        {
            SET_BRAKE(700);       //紧刹车
        }
    }
    else if (((tar > 0) && (liSetTarAcc > -(setAccThresholds >> 1)))//目标速度＞0，且设定目标加速度＞阈值一半的负值，即前进加速段
          || ((tar < 0) && (liSetTarAcc < (setAccThresholds >> 1))) //目标速度＜0，且设定目标加速度＜阈值一半，即后退加速段
            )
    {
        if(700 == gBrakeValue)    //处于刹车状态
        {
            brakeFilter++;
            if(brakeFilter >=  10)
            {
                SET_BRAKE(0);         //松刹车
            }
        }
    }
    else if(700 == gBrakeValue)    //处于刹车状态
    {
        brakeFilter = 0;
    }
}
/*****************************************************************************
 功能描述  : 电机位置模式处理
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  函数轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月17日
*****************************************************************************/
void ProcessMotorRunModePos(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    HAL_StatusTypeDef lResult;
	
    
    // 1.超时处理，直接转结束
    if(gStMotorRunState[motorNum].runTotalTime >= 30000)
    {
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
    }

    // 2.命令发生改变处理
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED == gStMotorRunState[motorNum].change_flag)//参数改变
        {
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_MODE_STEP_START);
        }
        else if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //非canopen驱动器
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
        }
        else
        {
            ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_DISABLE_VOLTAGE); //切换到停止模式
            return;
        }
    }

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
    if(gStMotorData[motorNum].flag & ENABLE_AUTO_HOMMING)    //转寻零
    {
        if(HOME_FLAG_FINISH != gStMotorRunState[motorNum].homeFinishFlag)
        {
            if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
            {
                ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_DISABLE_VOLTAGE); //切换到停止模式
            }
            else
            {
                //刹车未标零完成时，转向电机不能标零
                if(!((M_TURN == motorNum) && (gStMotorData[M_BRAKE].flag & ENABLE_AUTO_HOMMING)
                    && (gStMotorData[M_BRAKE].driverType != DRIVER_TYPE_NONE) 
                    && (gStMotorRunState[M_BRAKE].homeFinishFlag != HOME_FLAG_FINISH)))
                {
                    ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_HOMMING, MOTOR_RUN_MODE_STEP_START); //切换到寻零模式
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
        if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //非canopen驱动器
        {
            if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG) //转使能
            {
                if(DISABLE == gStMotorRunState[motorNum].enableFlag)
                {
                    GPIOEnableMotor(motorNum, ENABLE);
                }
                *pRunStep = MOTOR_RUN_COMMON_STEP3;
            }
            //读位置来决定相对脉冲数
            else if((gStMotorData[motorNum].flag & MOTOR_READ_POS_FLAG)
                && (gStMotorData[motorNum].flag & MOTOR_PWM_CONTRL_MODE))  //pwm控制方式
            {
                gStMotorRunState[motorNum].runDelayTime = 200;
                gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_SET_FINISH;
                *pRunStep = MOTOR_RUN_COMMON_STEP6;
            }
            else
            {
                /*PWMUpdateMotorPulse(motorNum, RT_FALSE);  //更新当前位置
                //转方向
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
                    && (700 == gCurtRunMode[motorNum].target_value)) //转压力传感器判断
                {
                    gStMotorRunState[motorNum].runDelayTime = 0;
                    *pRunStep = MOTOR_RUN_COMMON_STEP4;   //压力判断
                }
                else if(HAL_OK != lResult)
                {
                    gStMotorRunState[motorNum].runDelayTime = 0;
                    *pRunStep = MOTOR_RUN_COMMON_STEP5;   //等待命令执行完成
                    //*pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                }
                else
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
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
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
            }
            else
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP1;
                if(IS_UFOONE_FLAG_SET(UFOONE_BRAKE_STOP_ACCORD_OIL) && (M_BRAKE == motorNum) && (700 == gBrakeValue))   ////刹车依据油压停止
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
        if(MOTOR_RUN_COMMON_STEP2 == *pRunStep)   ////刹车依据油压停止
        {
            if(AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO) >= ADC1_BRAKE_MAX_THRETHOLDS + 2)  //大于阈值油压(单位0.1Mpa)
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
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
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
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                SetErrorCode(motorNum, ERROR_CODE_LIMIT_ABNORMAL, ERROR_L_NORMAL);
                if(gStMotorData[motorNum].flag & ENABLE_AUTO_HOMMING)    //重新寻零
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
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                    SetErrorCode(motorNum, ERROR_CODE_MOTOR_STALL, ERROR_L_HIHG);
                }
                else
                {
                    MotorSetMotorControlStatus(gStMotorData[motorNum].idx, DIVICE_CTROL_STATUS_STALL);  //清除状态位
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
            if(gStMotorData[motorNum].flag & MOTOR_READ_POS_FLAG)  //读位置来决定相对脉冲数
            {
                if(DEBUG_DATA_TYPE_5)
                {
                    rt_kprintf("Cur:%d, new:%d.\r\n", gStMotorRunState[motorNum].curPos, gStMotorRunState[motorNum].lastPoweroffPos);
                }
                gStMotorRunState[motorNum].curPos = gStMotorRunState[motorNum].lastPoweroffPos;
            }
            else
            {
                PWMUpdateMotorPulse(motorNum, RT_FALSE);  //更新当前位置
            }
            //转方向
            if((gStMotorData[motorNum].flag & MOTOR_DIR_FLAG)
                && (((DIR_CCW != gStMotorRunState[motorNum].dirFlag) && (gStMotorRunState[motorNum].curPos > gStMotorRunState[motorNum].targetPos))
                || ((DIR_CW != gStMotorRunState[motorNum].dirFlag) && (gStMotorRunState[motorNum].curPos < gStMotorRunState[motorNum].targetPos))))
            {
                //PWMSetMotorOutputState(motorNum, DISABLE);    //禁止pwm输出
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
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
            }
        }
    }
    else if(MOTOR_RUN_COMMON_STEP4 == *pRunStep)    //压力传感器标定与压力故障判断
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
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
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
                *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
            }
            else
            {
                gStMotorRunState[motorNum].errorCnt++;
                if(gStMotorRunState[motorNum].errorCnt > 50)
                {
                    gStMotorRunState[motorNum].errorCnt = 0;
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
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
            PWMSetMotorOutputState(motorNum, DISABLE);    //禁止pwm输出
            if(HAL_OK == Motor485ReadPos(motorNum, &gStMotorRunState[motorNum].lastPoweroffPos))
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP3;
            }
        }
    }

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
}
/*****************************************************************************
 功能描述  : 电机停止处理
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  函数轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年11月17日
*****************************************************************************/
void ProcessMotorRunModeStop(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;

    // 1.超时处理，直接转结束
    if(gStMotorRunState[motorNum].runTotalTime >= 30000)
    {
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
    }

    // 2.命令发生改变处理，无，停止优先
    /*if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        
    }*/

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
    if(OPERATION_DISABLE_VOLTAGE == *pRunStep)  //关驱动器电压停止
    {
        *pRunStep = MOTOR_RUN_COMMON_STEP1;
        gStMotorRunState[motorNum].operStopStep = OPERATION_DISABLE_VOLTAGE;
        gStMotorRunState[motorNum].operTimedelay = 0x0fff;
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;//根据返回的状态自动切换
        }
    }
    else if(OPERATION_QUCIK_STOP == *pRunStep)  //快速停止
    {
        *pRunStep = MOTOR_RUN_COMMON_STEP1;
        gStMotorRunState[motorNum].operStopStep = OPERATION_QUCIK_STOP;
        gStMotorRunState[motorNum].operTimedelay = 0x0fff;
        if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
        {
            gStMotorRunState[motorNum].operModeSetStep = OPERATION_MODE_AUTO_CHANGE;//根据返回的状态自动切换
        }
    }
    else if(MOTOR_RUN_COMMON_STEP1 == *pRunStep)
    {
        if(OPERATION_STOP_FINISH == gStMotorRunState[motorNum].operStopStep)
        {
            *pRunStep = MOTOR_RUN_COMMON_STEP2;   //将目标速度/电流切换至0
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
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
    }
    else if(OPERATION_SLOW_STOP == *pRunStep) //缓慢停止
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
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
        }
    }
    else if(OPERATION_STOP_ADJUST == *pRunStep) //电流模式判断电机是否真实停止
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
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
                }
            }
        }
    }
    else if(OPERATION_SLOW_STOP_WAIT == *pRunStep) //缓慢停止
    {
        if(gStMotorRunState[motorNum].runDelayTime >= gStMotorData[motorNum].speedAdjustPeriod)
        {
            if(OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep)
            {
                //CHANGE_MOTOR_TARGET_VEL(motorNum, GetSlopeSpeed(motorNum));
                if(0 == gStMotorRunState[motorNum].targetVel)
                {
                    CHANGE_MOTOR_TARGET_VEL_WITH_JUDGE(motorNum, 0);
                    gStMotorRunState[motorNum].operationStopJudgeFlag = 1;  //需要判断电机是否已停止
                    *pRunStep = OPERATION_STOP_ADJUST;   //判断停止
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
        if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG) //转使能
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

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
}
/*****************************************************************************
 功能描述  : 电机速度模式处理
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  函数轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2019年12月10日
*****************************************************************************/
void ProcessMotorRunModeSpeed(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    uint32_t lu32Temp;
    int32_t liTemp = 0, liTemp1, liTemp2;

    // 1.超时处理，无
    /*if(gStMotorRunState[motorNum].runTotalTime >= )
    {
        gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;     //切换到空闲模式
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        ChangeMotorRunState(motorNum, MOTOR_STATE_ERROR);
        SetErrorCode(motorNum, );
        return;
    }*/

    // 2.命令发生改变处理，无
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED != gStMotorRunState[motorNum].change_flag)//参数改变
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

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)  //设定速度
    {        
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen驱动器
        {
            ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PVM);
            CHANGE_MOTOR_TARGET_VEL(motorNum, 0);
        }
        else if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG)     //转使能
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
         || (MOTOR_RUN_COMMON_STEP4 == *pRunStep))    //加减速
    {
        if(MOTOR_RUN_COMMON_STEP1 == *pRunStep)
        {
            gStMotorRunState[motorNum].setTargetVel = Limit(gCurtRunMode[motorNum].target_value, 
                -gStMotorData[motorNum].limitSpeed, gStMotorData[motorNum].limitSpeed);
            if(0 != gStMotorRunState[motorNum].setTargetVel)
            {
                //限定目标速度最小为寻零速度
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
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
            {
                if(gStMotorData[motorNum].relatedMotor >= M_TOTAL_NUM)
                {
                    *pRunStep = MOTOR_RUN_COMMON_STEP3;
                }
            }
        }
        if((MOTOR_RUN_COMMON_STEP2 == *pRunStep) && (OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep))
        {
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //等待同步关联电机使能
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
                //转方向
                if((gStMotorData[motorNum].flag & MOTOR_DIR_FLAG)
                    && (((DIR_CCW != gStMotorRunState[motorNum].dirFlag) && (liTemp < 0))
                    || ((DIR_CW != gStMotorRunState[motorNum].dirFlag) && (liTemp > 0))))
                {
                    if(liTemp > 0)
                    {
                        GPIOSetMotorDir(motorNum, DIR_CW);
                        if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
                        if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //等待同步关联电机使能
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
                                    if(gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
                                    {
                                        liTemp1 = -liTemp1;
                                    }
                                    liTemp2 = gStMotorRevData[gStMotorData[motorNum].relatedMotor].current;
                                    if(gStMotorData[gStMotorData[motorNum].relatedMotor].flag & MOTOR_DIR_INVERSE_FLAG) //电机反向
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
        if(MOTOR_RUN_COMMON_STEP4 == *pRunStep) //读电流
        {
            if(gStMotorRunState[motorNum].runDelayTime >= (gStMotorData[motorNum].speedAdjustPeriod >> 1))
            {
                *pRunStep = MOTOR_RUN_COMMON_STEP2;
                if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen驱动器
                {
                    MotorSendReadAvarageCurrentCmd(gStMotorData[motorNum].idx);
                    if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
                if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
    else if(MOTOR_RUN_COMMON_STEP3 == *pRunStep)    //等待切换模式
    {
        
    }
    else if(MOTOR_RUN_COMMON_STEP5 == *pRunStep)    //等待切换模式
    {
        if(MOTOR_SET_NOTHING == gStMotorRunState[motorNum].targetPosVelSetFlag)
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
        }
    }
    else
    {
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
    }    

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
} 
/*****************************************************************************
 功能描述  : 电机速度模式处理
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  函数轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2020年6月8日
*****************************************************************************/
void ProcessMotorRunModeCurrent(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    uint32_t lu32Temp;

    // 1.超时处理, 无
    /*if(gStMotorRunState[motorNum].runTotalTime >= MOTOR_PECT_CONTINUE_TIME_MAX)
    {
        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_SLOW_STOP); //切换到停止模式
        return;
    }*/

    // 2.命令发生改变处理
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED != gStMotorRunState[motorNum].change_flag)    //模式改变
        {
            gCurtRunMode[motorNum].target_value = 0;    //停电机
        }
        if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
        {
            lu32Temp = gStMotorRunState[motorNum].runDelayTime;
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_COMMON_STEP1);
            gStMotorRunState[motorNum].runDelayTime = lu32Temp;
        }
    }

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)
    {
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen驱动器
        {
            ChangeMotorControlMode(motorNum, MOTOR_OPERATION_MODE_PTM);
            if(0 == gStMotorRunState[motorNum].setCurrent)
            {
                CHANGE_MOTOR_TARGET_CURRENT(motorNum, 0);
            }
        }        
        else if(gStMotorData[motorNum].flag & MOTOR_ENABLE_FLAG)     //转使能
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
            //限定目标速度最小为寻零速度
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
        if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //同步关联电机
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
            if(gStMotorData[motorNum].flag & MOTOR_RELATED_SYNCHRO) //等待同步关联电机使能
            {
                if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
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
                    if(gStUfoData.flag & UFO_FOUR_DRIVER_FLAG)  //兼容四驱
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
                    if(IS_UFOONE_FLAG_SET(UFOONE_FOUR_ALONE_FLAG))  //四轮独立标志
                    {
                        //所有电机使用和左电机相同的pid参数
                        if((M_RIGHT == motorNum) || (M_LEFT_ONE== motorNum) || (M_RIGHT_ONE == motorNum))
                        {
                            gStMotorRunState[motorNum].pidIncresParas.kp = gStMotorRunState[M_LEFT].pidIncresParas.kp;
                            gStMotorRunState[motorNum].pidIncresParas.ki = gStMotorRunState[M_LEFT].pidIncresParas.ki;
                            gStMotorRunState[motorNum].pidIncresParas.kd = gStMotorRunState[M_LEFT].pidIncresParas.kd;
                        }
                    }
                    ClearPidMem_incres(&gStMotorRunState[motorNum].pidIncresParas);   //清除pid数据
                    if(gStMotorData[motorNum].flag & MOTOR_TURN_PID_TO_ACC)           //转向pid当作加速度补偿用
                    {
                        InitPidParas_incres(&gStMotorRunState[motorNum].pidIncresParasOne,
                            gStMotorData[M_TURN].kp, gStMotorData[M_TURN].ki, 0, 0);
                        //ClearPidMem_incres(&gStMotorRunState[M_TURN].pidIncresParas); //清除pid数据
                    }
                    SET_BRAKE(0);
                    ClearPidMem_incres(&gStMotorRunState[M_BRAKE].pidIncresParas);    //清除pid数据
                    
                    if (DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_8A)
                    {
                        rt_kprintf("M%d:enter PID!\r\nVel-kp:%.1f, ki:%.1f, kd:%.1f, slopeLimit:%d.\r\n", motorNum, gStMotorRunState[motorNum].pidIncresParas.kp,
                            gStMotorRunState[motorNum].pidIncresParas.ki, gStMotorRunState[motorNum].pidIncresParas.kd, gStMotorData[motorNum].profileVelocity);
                        if(gStMotorData[motorNum].flag & MOTOR_TURN_PID_TO_ACC)           //转向pid当作加速度补偿用
                        {
                            rt_kprintf("Acc-kp:%.1f, ki:%.1f, kd:%.1f, startCurrent:%d, slopeLimit:%d.\r\n", gStMotorRunState[motorNum].pidIncresParasOne.kp,
                                gStMotorRunState[motorNum].pidIncresParasOne.ki, gStMotorRunState[motorNum].pidIncresParasOne.kd, gStMotorData[motorNum].startCurrent, gStMotorData[motorNum].pos_limit2);
                        }
                        if(IS_UFOONE_FLAG_SET(UFOONE_SET_ACC_OFFSET_VALID)) //设定目标加速度补偿有效，用于消除速度误差
                        {
                            rt_kprintf("AccOffset-kp:%d.\r\n", gStMotorData[M_LEFT].speedGrad);
                        }
                        rt_kprintf("IIt time:%d, current_balance:%d.\r\n", gStMotorData[motorNum].overCurrentTime, gStMotorData[M_RIGHT].speedGrad);
                        if((M_LEFT == motorNum) && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))
                        {
                            rt_kprintf("Brake-current:%d, vel:%d, acc:%d, test(0.1Mpa):%d.\r\n", gStMotorData[M_BRAKE].homingSpeed, gStMotorData[M_BRAKE].profileVelocity, gStMotorData[M_BRAKE].profileAcc,
                                IS_UFOONE_FLAG_SET(UFOONE_BRAKE_DEC_TEST_FLAG) ? (int)ADC1_BRAKE_TEST_THRETHOLDS : -1);
                        }
                        PrintfUfoVerAndSn();    //打印版本号
                    }
                    gStMotorRunState[motorNum].pidCurrentStartFlag = M_PID_RUN_FLAG;  //开启pid调节
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
                    gStMotorRunState[motorNum].setVelState = SET_VEL_STATE_START;   //设定速度状态转为启动
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
                if(gStMotorRunState[motorNum].runDelayTime >= 200) //等待一小会再退出流程，防止大电流立即断主继电器
                {
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //停止结束
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

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
}
/*****************************************************************************
 功能描述  : 电机电流位置模式处理
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  函数轮询周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年8月15日
*****************************************************************************/
void ProcessMotorRunModeCurrentPos(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step;
    int32_t liTemp;

    // 1.超时处理, 无
    /*if(gStMotorRunState[motorNum].runTotalTime >= MOTOR_PECT_CONTINUE_TIME_MAX)
    {
        gStMotorRunState[motorNum].homeFinishFlag = HOME_FLAG_UNDEF;
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STOP, OPERATION_SLOW_STOP); //切换到停止模式
        return;
    }*/

    // 2.命令发生改变处理
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        if(MOTOR_RUN_PARA_CHANGED == gStMotorRunState[motorNum].change_flag)//参数改变
        {
            if(MOTOR_RUN_MODE_STEP_START != *pRunStep)
            {
                ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_COMMON_STEP1);
            }
        }
        else if(IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //非canopen驱动器
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //结束
        }
        else
        {
            ChangeMotorCurRunMode(motorNum, gCurtRunMode[motorNum].run_mode, MOTOR_RUN_MODE_STEP_END);
        }
    }

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
    if(MOTOR_RUN_MODE_STEP_START == *pRunStep)
    {
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[motorNum].driverType))    //canopen驱动器
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
    else if(MOTOR_RUN_COMMON_STEP2 == *pRunStep)    //固定电流刹车，定时结束
    {
        if((gStMotorRunState[motorNum].runDelayTime >= 5000) ||
            ((gStMotorRunState[motorNum].runDelayTime >= gStMotorData[motorNum].overCurrentTime)
            && (0 != gStMotorData[motorNum].overCurrentTime)))
        {
            *pRunStep = MOTOR_RUN_MODE_STEP_END;    //设定电流0并结束流程
        }
    }
    else if(MOTOR_RUN_COMMON_STEP3 == *pRunStep)    //反向退回判断
    {
        if (gStMotorRunState[motorNum].runDelayTime >= 500)
        {
            gStMotorRunState[motorNum].runDelayTime = 450;  //首次进行周期为500ms，后续进入周期为50ms
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
                    if(gStMotorRunState[motorNum].limit1Time >= 3)  //连续3个周期转速小于10rpm
                    {
                        *pRunStep = MOTOR_RUN_MODE_STEP_END;    //设定电流0并结束流程
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
                    *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //停止结束
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
            *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;   //停止结束
            if(DEBUG_DATA_TYPE_2)
            {
                rt_kprintf("id-%d exit current-speed-mode, ticks:%d.\r\n", gStMotorData[motorNum].idx, HAL_GetTick());
            }
        }
    }

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
}

/*void ProcessMotorRunModeCommon(uint32_t motorNum, uint32_t processTimeMs)
{
    uint8_t* pRunStep = &gStMotorRunState[motorNum].run_step; 

    // 1.超时处理
    if(gStMotorRunState[motorNum].runTotalTime >= )
    {
        gCurtRunMode[motorNum].run_mode = MOTOR_RUN_MODE_STEP_IDLE;     //切换到空闲模式
        *pRunStep = MOTOR_RUN_MODE_STEP_IDLE;
        ChangeMotorRunState(motorNum, MOTOR_STATE_ERROR);
        SetErrorCode(motorNum, );
        return;
    }

    // 2.命令发生改变处理
    if(MOTOR_RUN_MODE_NO_CHANGE != gStMotorRunState[motorNum].change_flag)
    {
        
    }

    // 3.相关步骤（需要结束条件MOTOR_RUN_MODE_STEP_FINISH）
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

    // 4.运行结束，切换下个运行模式
    if(MOTOR_RUN_MODE_STEP_IDLE == *pRunStep)
    {
        ChangeMotorRunState(motorNum, MOTOR_STATE_STOP);
        ChangeMotorCurRunMode(motorNum, MOTOR_RUN_MODE_STEP_IDLE, MOTOR_RUN_MODE_STEP_IDLE); //自动切换运行模式
    }
}*/
/*****************************************************************************
 功能描述  : 低电压检测
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年6月28日
*****************************************************************************/
void LowVoltageCheck(void)
{
    int i;
    static uint8_t lowVolCnt = 0, lowVolKphCnt = 0;
    uint16_t vol = 0;
    uint16_t volCnt = 0; 
    int16_t scenario_max_vel; //最大速度
        
    //判断是否处于电机运行状态(是否进入pid)
    gStBatteryState.motorRunFlag = 0;
    gStBatteryState.motorIdleTime += LOW_VOLTAGE_CHECK_PERIOD;
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))  //判断动力电压
        {
            if(M_PID_RUN_FLAG == gStMotorRunState[i].pidCurrentStartFlag)
            {
                gStBatteryState.motorRunFlag = 1;
                gStBatteryState.motorIdleTime = 0;
            }

            //累加所有动力驱动器电压值
            if((OPERATION_MODE_SET_FINISH == gStMotorRunState[i].operModeSetStep) 
                && (gStMotorRevData[i].vol > 0))
            {
                vol += gStMotorRevData[i].vol;
                volCnt++;
            }
        }
    }

    //运行状态判断驱动器电压是否过低
    if(volCnt)
    {
        vol = vol * 10 / volCnt;  //取均值并转换到单位0.1V
        if(gStBatteryState.motorRunFlag && IS_ENABLE_SET_WARNING_CODE)
        {
            if(vol <= gStUfoData.minVol)   //动力驱动器低电压判断
            {
                SET_WARNING_CODE(M_TOTAL_NUM, WARNING_CODE_RUNNING_LOW_VOLTAGE);
                SetErrorInfo(M_TOTAL_NUM, WARNING_CODE_RUNNING_LOW_VOLTAGE);
                gStErrorInfo.driverVoltage[M_LEFT] = gStMotorRevData[M_LEFT].vol;
                gStErrorInfo.driverVoltage[M_RIGHT] = gStMotorRevData[M_RIGHT].vol;
                gStErrorInfo.driverVoltage[M_LEFT_ONE] = gStMotorRevData[M_LEFT_ONE].vol;
                gStErrorInfo.driverVoltage[M_RIGHT_ONE] = gStMotorRevData[M_RIGHT_ONE].vol;
            }
        }
        if((0 == gStBatteryState.validFlag) && (0 == gStBatteryState.motorRunFlag)) //运行时不赋值，防止电量波动
        {
            gStBatteryState.voltage = vol;  //电池电压无效时，将驱动器电压赋值给电池
            gStBatteryState.driverVolValidFlag = 1;
        }
        else
        {
            gStBatteryState.driverVolValidFlag = 0;
        }
    }
    
    //判断电池电压是否过低
    if(IS_NO_WARNING_OR_ERROR_CODE)
    {
        if(gStBatteryState.validFlag)
        {
            if(gStBatteryState.voltage <= gStUfoData.minVol) //电池低电压判断
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

    //静止状态判断当前电压是否支持当前场景速度测试
    if(!IS_UFOONE_FLAG_SET(UFOONE_DISABLE_MIN_VOL_KPH_FLAG))    //未禁止最小电压对应时速限制标志时
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
                //场景警告，切换可用场景后可自动清除警告
                else if((0 == lowVolKphCnt) && (WARNING_CODE_LOW_VOLTAGE_TO_KPH == gErrorResult))
                {
                    ClearErrorCode(M_TOTAL_NUM);
                }
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 电机位置计算车子位置
 输入参数  : uint32_t motorNum
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2024年5月29日
*****************************************************************************/
void MotorPosCalCarPos(void)
{
    uint32_t totalCnt, revCnt, i;
    float posTemp;

    totalCnt = 0;
    revCnt = 0;
    posTemp = 0;

    //取所有动力电机位置值求和
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
    //如果读到所有电机位置值，则清除读取标志，求得平均值，并换算到距离
    if((revCnt > 0) && (totalCnt == revCnt))
    {
        CLEAR_MOTOR_REV_DATA_FLAG(M_LEFT, REV_MOTOR_POS_FLAG);
        CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT, REV_MOTOR_POS_FLAG);
        CLEAR_MOTOR_REV_DATA_FLAG(M_LEFT_ONE, REV_MOTOR_POS_FLAG);
        CLEAR_MOTOR_REV_DATA_FLAG(M_RIGHT_ONE, REV_MOTOR_POS_FLAG);
        if((gStMotorData[M_LEFT].counts > 0) && (gStUfoData.ratioVel > 0))
        {
            posTemp = posTemp / revCnt / gStMotorData[M_LEFT].counts;   //圈数
            gMotorAvaPos = (int32_t)(posTemp * NAV_RATIO_SCALE / gStUfoData.ratioVel * 600); //换算到近距离
        }
    }
}
/*****************************************************************************
 功能描述  : 电机读取步骤
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年5月6日
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
        LowVoltageCheck();  //低电压检测
    }
    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        if(0 == gStMotorRunState[i].startRemoteFlag)    //电机未启动，不读取
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
                if(readFlag)    //温度500ms读一次，电机和驱动器温度交替读取
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
                if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //无惯导标志时只读动力电机的位置
                {
                    if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))  //采集动力电压
                    {
                        gStMotorRevData[i].timeCnt += processTimeMs;
                        if(gStMotorRevData[i].timeCnt >= 10)
                        {
                            gStMotorRevData[i].timeCnt = 0;
                            if(!IS_REV_MOTOR_DATA(i, REV_MOTOR_POS_FLAG))   //清除接收到位置后才重新接收数据
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
                        && (M_TURN != i))  //使能发送电机数据
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
                    if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))  //采集动力电压
                    {
                        if(readVolFlag) //电压100ms读一次
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
        //取一个数据进行读取
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
        //判断是否收到对应数据
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
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_POS_FLAG))    //接收到位置
                {
                    if(IS_UFOONE_FLAG_SET(UFOONE_NO_NAV_FLAG))  //无惯导标志时根据电机位置计算当前车子位置
                    {
                        MotorPosCalCarPos();
                    }
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_POS;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_CURRENT & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_CURRENT_FLAG))    //接收到电流
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_CURRENT;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_VEL & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_SPEED_FLAG))    //接收到转速
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_VEL;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_TMP & gStMotorRunState[i].readStep)
            {
                if((IS_REV_MOTOR_DATA(i, REV_MOTOR_DRIVER_TMP_FLAG)
                    || (IS_REV_MOTOR_DATA(i, REV_MOTOR_TMP_FLAG)))) //接收到驱动器或电机温度
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_TMP;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
            if(MOTOR_READ_VOL & gStMotorRunState[i].readStep)
            {
                if(IS_REV_MOTOR_DATA(i, REV_MOTOR_VOL_FLAG))    //接收到驱动器电压
                {
                    gStMotorRunState[i].readStep &= ~MOTOR_READ_VOL;
                    gStMotorRunState[i].readStep &= ~MOTOR_READING;
                }
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 电机到达目标位置判断
 输入参数  : uint32_t motorNum       电机序号
             uint32_t processTimeMs  线程时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年6月17日
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
 功能描述  : 电机控制处理进程
 输入参数  : void* parameter
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2018年11月21日
*****************************************************************************/
void MotorControlEntry(TickType_t curTime)
{
    static uint32_t l_lastTime = 0;
    static uint32_t l_tick_cnt_one = 0;
    int i, lUploadFlag = RT_FALSE;

    l_lastTime = curTime - l_lastTime;

    if(!(sys_para->CAR_RTinf.Link & LINK_ERROR_STOP))   //退出故障紧急停止状态后再进行故障自恢复
    {
        ErrorDealAndRecovery(l_lastTime);   //故障处理与恢复
    }

    OldTestProcess(l_lastTime);     //老化测试
    
#ifdef ENABLE_CANOPEN_DRIVER
    MotorReadProcess(); //异步读取电机数据
    MotorReadStep(l_lastTime);    //电机数据读取各个步骤
#endif

    for(i = M_LEFT; i < M_TOTAL_NUM; i++)
    {
        //等待启动远程帧服务
        if((0 == gStMotorRunState[i].startRemoteFlag)
            && (MOTOR_RUN_MODE_POWER_OFF != gCurtRunMode[i].run_mode))
        {
            continue;
        }
        //转向到位判断
        if((gStMotorRunState[i].targetPosJudgeFlag) && (gStUfoData.flag & UFO_ENABLE_POS_JUDEGE))//使能到位位置判断
        {
            MotorReachTargetPosJudge(i, l_lastTime);
        }
        //等待转向刹车预充完成
        if(gStMotorData[i].flag & MOTOR_USE_PI7_POWER)    //使用PI7电源控制端口，原电源引脚当作预充引脚
        {
            if(MOTOR_POWER_FLAG_ON != gPreChargeStateOne)
            {
                continue;
            }
        }
        //转向未完成寻零时，不执行驱动
        if((M_LEFT == i) || (M_RIGHT == i) || (M_LEFT_ONE == i) || (M_RIGHT_ONE == i))
        {
            if(gStUfoData.flag & UFO_PRE_CHARGE_FLAG)  //预充电使能
            {
                if(MOTOR_POWER_FLAG_ON != gPreChargeState)
                {
                    continue;
                }
            }
            if((DRIVER_TYPE_NONE != gStMotorData[M_TURN].driverType) && (gStMotorData[M_TURN].flag & ENABLE_AUTO_HOMMING))    //使能寻零
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
        //切换速度或位置
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
            if(M_PID_IDLE_FLAG != gStMotorRunState[i].pidCurrentStartFlag)    //电流pid调节
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
        TestMotorDataUploadProcess(M_LEFT, RT_FALSE, 0, 0); //上传测试数据，用于台架上位机测试
    }
    
    ChangeDriveMotorControlMode(l_lastTime);    //切换驱动器模式及速度

    l_tick_cnt_one += l_lastTime;
    if(l_tick_cnt_one >= MOTORSTATUS_ANALYSIS_TIME)
    {            
        MotorsPowerCheck(l_tick_cnt_one);//电源上下电检查
        GetMotorStatus();//电机状态分析
        MotorOverLoaderWaitRecovery();  //电机过载等待恢复(等电机冷却)
        l_tick_cnt_one = 0;
    }

    l_lastTime = curTime;
}
/*****************************************************************************
 功能描述  : UFO控制
 输入参数  : uint8_t* cmdData  命令数据
             uint8_t size      命令长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月18日
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
        case 0x01:  //电机位置模式控制
            if(gStMotorData[cmdData[1]].flag & MOTOR_CURRENT_ADJUST_SPEED) //电流调节转速
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
        case 0x02:  //电机速度模式控制
            if(gStMotorData[cmdData[1]].flag & MOTOR_CURRENT_ADJUST_SPEED) //电流调节转速
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
        case 0x03:  //电机断电模式控制
            lMotorRunMode.run_mode = MOTOR_RUN_MODE_POWER_OFF;
            lMotorRunMode.target_value = 0;
            rt_kprintf("Motor%d power off.\r\n", cmdData[1]);
            SetMotorRunModeData(cmdData[1], &lMotorRunMode);
            break;
        case 0x10:  //pid参数设置
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
 功能描述  : 电机测试数据上传进程
 输入参数  : rt_bool_t pidStartFlag  pid启动标志
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年7月10日
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

            lRespondMsg[0] = '*';       //数据头
            lRespondMsg[1] = sizeof(lRespondMsg);//总数据长度
            lRespondMsg[2] = 0x81;      //命令字
            lRespondMsg[3] = gStBatteryState.voltage & 0xff;    //电压
            lRespondMsg[4] = (gStBatteryState.voltage >> 8) & 0xff;
            lRespondMsg[5] = gStBatteryState.current & 0xff;    //电流
            lRespondMsg[6] = (gStBatteryState.current >> 8) & 0xff;
            lRespondMsg[7] = tarVel & 0xff; //目标速度
            lRespondMsg[8] = (tarVel >> 8) & 0xff;
            lRespondMsg[9] = curVel & 0xff; //当前速度
            lRespondMsg[10] = (curVel >> 8) & 0xff;
            lRespondMsg[11] = gStMotorRunState[motorNum].setAcc & 0xff; //设定加速度
            lRespondMsg[12] = (gStMotorRunState[motorNum].setAcc >> 8) & 0xff;
            lRespondMsg[13] = gStMotorRunState[motorNum].curAcc & 0xff; //当前加速度
            lRespondMsg[14] = (gStMotorRunState[motorNum].curAcc >> 8) & 0xff;
            lRespondMsg[15] = (gStMotorRunState[motorNum].setCurrent / 100) & 0xff; //电机设定电流
            lRespondMsg[16] = ((gStMotorRunState[motorNum].setCurrent / 100) >> 8) & 0xff;
            lRespondMsg[17] = gErrorResult & 0xff;  //故障字
            lRespondMsg[18] = (gErrorResult >> 8) & 0xff;
            if((0 == gErrorResult) && (OPERATION_MODE_SET_FINISH == gStMotorRunState[motorNum].operModeSetStep))
            {
                lRespondMsg[19] = 1;    //电机使能
            }
            else
            {
                lRespondMsg[19] = 0;
            }
            lRespondMsg[20] = gLockFlag;    //抱闸状态
            lCurTime = lCurTime - gMotorTestDataUploadStartTime;
            lRespondMsg[27] = lCurTime & 0xff;  //相对时间
            lRespondMsg[28] = (lCurTime >> 8) & 0xff;
            lRespondMsg[29] = (lCurTime >> 16) & 0xff;
            lRespondMsg[30] = (lCurTime >> 24) & 0xff;
            lRespondMsg[31] = '#';      //数据尾

            rt_ksendData(lRespondMsg, sizeof(lRespondMsg));
        }
    }  
}
/*****************************************************************************
 功能描述  : 打印电机状态信息
 输入参数  : void
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年7月8日
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
 功能描述  : 老化指令令解析
 输入参数  : uint8_t* cmdData
             uint8_t size     
 输出参数  : 无
 作    者  : 田忠
 日    期  : 2023年04月08日
*****************************************************************************/
void OldTestRevCmdAnalysis(uint8_t* pData, uint8_t size)
{
    uint8_t cmd;
    uint8_t *dataBuf;
    
    if(0 == size)
    {
        return;
    }
    cmd = pData[0];           //老化功能
    dataBuf = &pData[1];      //老化指令
    
    if(0 == cmd)       //暂停老化
    {
        gOldTest.oldTestStep |= OLD_TEST_PAUSE;           //设置老化测试暂停标志位
        gOldTest.oldTestStep &= ~OLD_TEST_START;          //清除老化测试开始标志位
        gOldTest.oldTestStep &= ~OLD_TEST_POWER_NEW_CMD;  //清除老化测试动力新指令标志位
        gOldTest.oldTestStep &= ~OLD_TEST_TURN_NEW_CMD;   //清除老化测试转向新指令标志位
        gOldTest.oldTestStep &= ~OLD_TEST_BRAKE_NEW_CMD;  //清除老化测试刹车新指令标志位
        gOldTest.moduleAgingFlg = NONE_AGING;             //无模块需老化
        rt_kprintf("Set Stress Test(10-start,0-pause,else-start_once): %d.\r\n", cmd);
    }
    else
    {
        gOldTest.oldTestStep &= ~OLD_TEST_PAUSE;    //清除老化测试暂停标志位
        gOldTest.oldTestStep &= ~OLD_TEST_END;      //清除老化测试结束标志位
        
        if(10 != cmd)
        {
            gOldTest.oldTestStep |= OLD_TEST_NEED_UPDATE_CMD;   //老化测试过程中，需要持续发送老化指令
        }
        else
        {
            gOldTest.oldTestStep &= ~OLD_TEST_NEED_UPDATE_CMD;  //老化测试过程中，不需要持续发送老化指令
        }
        gOldTest.moduleAgingFlg = dataBuf[0];                   //老化模块选择
        
        if(!(gOldTest.oldTestStep & OLD_TEST_START))
        {
            gOldTest.oldTestStep |= OLD_TEST_START;      //首次接收到老化命令，设置老化测试开始标志位
            rt_kprintf("Set Stress Test(10-start,0-pause,else-start_once): %d.\r\n", cmd);
        }
        else
        {
            if (gOldTest.moduleAgingFlg & POWER_AGING)    
            {
                gOldTest.oldTestStep |= OLD_TEST_POWER_NEW_CMD;  //再次接收到老化命令，设置老化测试动力新命令标志位
            }
            if (gOldTest.moduleAgingFlg & TURN_AGING)    
            {
                gOldTest.oldTestStep |= OLD_TEST_TURN_NEW_CMD;   //再次接收到老化命令，设置老化测试转向新命令标志位
            }
            if (gOldTest.moduleAgingFlg & BRAKE_AGING)    
            {
                gOldTest.oldTestStep |= OLD_TEST_BRAKE_NEW_CMD;  //再次接收到老化命令，设置老化测试刹车新命令标志位
            }
        }
    }
    
    //动力老化
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
            gOldTest.forwardVel = *(int16_t*)(&dataBuf[5]);   //芯片内部为小端模式
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
    //转向老化
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
    //刹车老化
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
 功能描述  : 老化测试进程
 输入参数  : uint32_t processTimeMs
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年11月15日
*****************************************************************************/
static void OldTestProcess(uint32_t processTimeMs)
{
    static uint32_t lTimeCnt[MODULE_NUM] = {0, 0, 0};              //老化计时
    static uint32_t lOldTimesCnt[MODULE_NUM] = {0, 0, 0};          //老化次数计算
    static AGING_STEP agingStep[MODULE_NUM] ={EXIT, EXIT, EXIT};   //老化步骤
    int32_t liTemp;
    ST_MOTOR_RUN_MODE_DATA lMotorRunMode = {0};      //初始化局部变量，防止变量值不可控，造成异常
    float lfTemp;
    
    if(gOldTest.oldTestStep & OLD_TEST_PAUSE)
    {
        if(!(gOldTest.oldTestStep & OLD_TEST_END))   
        {
            gOldTest.oldTestStep |= OLD_TEST_END;             //设置老化测试结束标志位，确保老化开始 转 老化暂停时才进入
            gOldTest.oldTestStep &= ~OLD_TEST_START;          //清除老化测试开始标志位
            gOldTest.oldTestStep &= ~OLD_TEST_POWER_NEW_CMD;  //清除老化测试动力新指令标志位
            gOldTest.oldTestStep &= ~OLD_TEST_TURN_NEW_CMD;   //清除老化测试转向新指令标志位
            gOldTest.oldTestStep &= ~OLD_TEST_BRAKE_NEW_CMD;  //清除老化测试刹车新指令标志位
            gOldTest.moduleAgingFlg = NONE_AGING;             //无模块需老化
            
            if (agingStep[POWER] != EXIT)                     //动力老化已开启
            {
                agingStep[POWER] = EXIT;
                UfoLeftRightControl(0, &lMotorRunMode);       //老化结束，转速设置为0
                rt_kprintf("Exit Power Stress Test, speed: 0 m/s.\r\n");
            }

            if (agingStep[TURN] != EXIT)                      //转向老化已开启
            {
                agingStep[TURN] = EXIT;
                memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = 0;
                SetMotorRunModeData(M_TURN, &lMotorRunMode);  //老化结束，转向位置设置为0(转向零位)
                rt_kprintf("Exit Turn Stress Test, turnPos: 0.\r\n");
            }

            if (agingStep[BRAKE] != EXIT)                     //刹车老化已开启
            {
                agingStep[BRAKE] = EXIT;
                memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                lMotorRunMode.target_value = 0;
                SetMotorRunModeData(M_BRAKE, &lMotorRunMode); //老化结束，刹车位置设置为0(刹车零位)
                rt_kprintf("Exit Brake Stress Test, brakePos: 0.\r\n");
            }
        }
        return;
    }
    
    if(gOldTest.oldTestStep & OLD_TEST_START)   //老化测试开始
    {
        //动力老化
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
                    
                    if(gOldTest.oldTestStep & OLD_TEST_NEED_UPDATE_CMD)   //下次老化需要收到命令的标志
                    {
                        if(gOldTest.oldTestStep & OLD_TEST_POWER_NEW_CMD) //收到新的命令
                        {
                            gOldTest.oldTestStep &= ~OLD_TEST_POWER_NEW_CMD;  //清除老化测试动力新指令标志位
                            agingStep[POWER] = START;                         //转下一周期
                        }
                        else
                        {
                            gOldTest.oldTestStep |= OLD_TEST_PAUSE;       //转暂停，因为动力老化、转向老化、刹车老化是在同一条指令信息内，所以统一暂停
                            agingStep[POWER] = EXIT;
                        }
                    }
                    else
                    {
                        agingStep[POWER] = START;                         //转下一周期
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
                UfoLeftRightControl(0, &lMotorRunMode);       //老化取消，转速设置为0
                rt_kprintf("Power Stress Test Cancel, speed: 0 m/s.\r\n");     
            }
        }
        
        //转向老化
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

                    //设置转向速度
                    if(0 == MotorSetProfileVelocity(gStMotorData[M_TURN].idx, gOldTest.turnVel, RT_TRUE))
                    {
                        rt_kprintf("Turn speed: %d, ", gOldTest.turnVel);
                    }
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                    lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                    lMotorRunMode.target_value = -gOldTest.turnRange;
                    if(gStMotorData[M_TURN].flag & MOTOR_DIR_INVERSE_FLAG)  //电机反向
                    {
                        lMotorRunMode.target_value = -lMotorRunMode.target_value;
                    }
                    SetMotorRunModeData(M_TURN, &lMotorRunMode);                       //左转
                    agingStep[TURN] = STEP1;
                    rt_kprintf("Begin Turn Left, pos: %d.\r\n", -gOldTest.turnRange);
                }
            }
            else if (STEP1 == agingStep[TURN])
            {
                if(0 == MotorReadPosition(gStMotorData[M_TURN].idx, &liTemp))
                {
                    if(ABS_VALUE(gStMotorRunState[M_TURN].targetPos - liTemp) <= 2000) //转向到位判断，counts
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
                    if(gStMotorData[M_TURN].flag & MOTOR_DIR_INVERSE_FLAG)  //电机反向
                    {
                        lMotorRunMode.target_value = -lMotorRunMode.target_value;
                    }
                    SetMotorRunModeData(M_TURN, &lMotorRunMode);                         //右转
                    agingStep[TURN] = STEP3;
                }
            }
            else if (STEP3 == agingStep[TURN])
            {
                if(0 == MotorReadPosition(gStMotorData[M_TURN].idx, &liTemp))
                {
                    if(ABS_VALUE(gStMotorRunState[M_TURN].targetPos - liTemp) <= 2000)   //转向到位判断，counts
                    {
                        lTimeCnt[TURN] = 0;
                        rt_kprintf("End Turn Right.\r\n");
                        
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_TURN, &lMotorRunMode);
                        if(gOldTest.oldTestStep & OLD_TEST_NEED_UPDATE_CMD)    //下次老化需要收到命令的标志
                        {
                            if(gOldTest.oldTestStep & OLD_TEST_TURN_NEW_CMD)   //收到新的命令
                            {
                                gOldTest.oldTestStep &= ~OLD_TEST_TURN_NEW_CMD; //清除老化测试转向新指令标志位
                                agingStep[TURN] = START;                        //转下一周期
                            }
                            else
                            {
                                gOldTest.oldTestStep |= OLD_TEST_PAUSE;       //转暂停，因为动力老化、转向老化、刹车老化是在同一条命令令信息内，所以统一暂停
                                agingStep[TURN] = EXIT;
                            }
                        }
                        else
                        {
                            agingStep[TURN] = START; //转下一周期
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
                SetMotorRunModeData(M_TURN, &lMotorRunMode);                             //老化取消，转向位置设置为0(转向零位)
                rt_kprintf("Turn Stress Test Cancel, turnPos: 0.\r\n");
            }
        }
        
        //刹车老化
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

                    //设置刹车速度
                    if(0 == MotorSetProfileVelocity(gStMotorData[M_BRAKE].idx, gOldTest.brakeVel, RT_TRUE))
                    {
                        rt_kprintf("Brake speed: %d, ", gOldTest.brakeVel);
                    }
                    memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                    lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                    lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                    lMotorRunMode.target_value = gOldTest.brakeInitPos;
                    SetMotorRunModeData(M_BRAKE, &lMotorRunMode);                             //松刹车
                    agingStep[BRAKE] = STEP1;
                    rt_kprintf("Begin Release Brake, pos: %d.\r\n", gOldTest.brakeInitPos);
                }
            }
            else if (STEP1 == agingStep[BRAKE])
            {
                if(0 == MotorReadPosition(gStMotorData[M_BRAKE].idx, &liTemp))
                {
                    if(ABS_VALUE(gStMotorRunState[M_BRAKE].targetPos - liTemp) <= 1000)  //刹车到位判断，counts
                    {
                        lTimeCnt[BRAKE] = 0;
                        lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                        rt_kprintf("End Release Brake, oil:%.1fMpa.\r\n", lfTemp / 10);
                        agingStep[BRAKE] = STEP2;
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_BRAKE, &lMotorRunMode);        //刹车位置设置为0(刹车零位)
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
                    SetMotorRunModeData(M_BRAKE, &lMotorRunMode);                             //紧刹车
                    agingStep[BRAKE] = STEP3;
                    rt_kprintf("Begin Tighten Brake, pos: %d.\r\n", gOldTest.brakeLimitPos);
                }
            }
            else if (STEP3 == agingStep[BRAKE])
            {
                if(0 == MotorReadPosition(gStMotorData[M_BRAKE].idx, &liTemp))
                {
                    if((ABS_VALUE(gStMotorRunState[M_BRAKE].targetPos - liTemp) <= 1000) && (lTimeCnt[BRAKE] >= gOldTest.brakeGapTime * 1000))  //紧刹车到位判断(counts)，并保持紧刹车一段时间
                    {
                        lTimeCnt[BRAKE] = 0;
                        lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
                        rt_kprintf("End Tighten Brake, oil:%.1fMpa.\r\n", lfTemp / 10);
                        
                        memset((uint8_t*)(&lMotorRunMode), 0, sizeof(ST_MOTOR_RUN_MODE_DATA));
                        lMotorRunMode.run_mode = MOTOR_RUN_MODE_POS;
                        lMotorRunMode.posType = POS_TYPE_MULTI_ABSOLUTE;
                        lMotorRunMode.target_value = 0;
                        SetMotorRunModeData(M_BRAKE, &lMotorRunMode);             //刹车位置设置为0(刹车零位)
                        if(gOldTest.oldTestStep & OLD_TEST_NEED_UPDATE_CMD)       //下次老化需要收到命令的标志
                        {
                            if(gOldTest.oldTestStep & OLD_TEST_BRAKE_NEW_CMD)     //收到新的命令
                            {
                                gOldTest.oldTestStep &= ~OLD_TEST_BRAKE_NEW_CMD;  //清除老化测试刹车新指令标志位
                                agingStep[BRAKE] = START;                         //转下一周期
                            }
                            else
                            {
                                gOldTest.oldTestStep |= OLD_TEST_PAUSE;           //转暂停，因为动力老化、转向老化、刹车老化是在同一条命令令信息内，所以统一暂停
                                agingStep[BRAKE] = EXIT;
                            }
                        }
                        else
                        {
                            agingStep[BRAKE] = START;                             //转下一周期
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
			//刹车老化测试时，实时读油压值
            if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_90 || DEBUG_DATA_TYPE_81 || DEBUG_DATA_TYPE_8A || DEBUG_DATA_TYPE_92)
            {
                lfTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO);  //读取油压值0.01Mpa
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
                SetMotorRunModeData(M_BRAKE, &lMotorRunMode);                            //老化取消，刹车位置设置为0(刹车零位)
                rt_kprintf("Brake Stress Test Cancel, brakePos: 0.\r\n");
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 模拟上位机 打开电源、PVM速度模式按钮
 输入参数  : 无
 输出参数  : 无
 作    者  : 刘子雄
 日    期  : 2024年11月25日
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
