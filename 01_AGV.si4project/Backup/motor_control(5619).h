#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include <preDef.h>
#include "motor_pidctrl.h"
#include "pid_fuzzy.h"
#include "math.h"
#include "kinematics.h"


#define MVEL_RECORD_NUM                     50     //惯导和电机速度差值记录个数，至少大于4
#define MVEL_STOP_FILTER_NUM                10     //电机停止判断滤波个数，必须小于等于MVEL_RECORD_NUM
#define MVEL_FILTER_NUM                     7      //速度滤波个数，用于计算当前电机实际速度，必须小于等于MVEL_RECORD_NUM
#define MVEL_CAR_DIR_JUDGE_TOTAL_NUM        10     //标示惯导速度方向的电机总有效个数，必须小于等于MVEL_RECORD_NUM
#define MVEL_CAR_DIR_JUDGE_NUM              3      //参与计算惯导速度方向的电机个数，必须小于等于MVEL_RECORD_NUM
#define MVEL_EXCLUDE_NUM                    2      //计算电机实际速度滤波排除最大最小值个数
#define MACC_EXCLUDE_NUM                    3      //计算电机加速度滤波排除最大最小值个数
#define MACC_CAL_PERIOD_NUM                 10     //电机加速度计算跨周期个数，必须小于等于MVEL_RECORD_NUM
#define MACC_CURRENT_LEARN_NUM              3      //电流学习记录个数

#define SET_VEL_RECORD_NUM                  6      //设定速度记录个数
#define M_SET_CURRENT_EXE_DELAY_NUM         5      //电机设定电流执行滞后的周期个数

#define NAV_CAR_VEL_RECORD_NUM              MVEL_RECORD_NUM //惯导车速记录个数
#define CALRATIOL_FILTER_NUM                15     //计算速比滤波个数，必须小于等于MVEL_RECORD_NUM
#define CALRATIOL_EXCLUDE_NUM               3      //计算速比滤波排除最大最小值个数
#define NAV_ACC_RECORD_NUM                  5      //惯导加速度数据记录个数
#define NAV_ACC_CAL_PERIOD_NUM              6      //惯导加速度计算跨周期个数，必须小于等于CALRATIOL_FILTER_NUM

#define CAR_MOTOR_FILTER_EXCLUDE_NUM        5      //计算惯导车速和电机速度差滤波排除最大差值个数
#define CAR_MOTOR_MAX_OR_MIN_FILTER_NUM     10     //惯导与电机速度差判断时，取最小或最大值滤波的个数，需小于MVEL_RECORD_NUM的一半
#define CAR_MOTOR_VELERR_OUT_RANGE(percent)  ((ABS_VALUE((gCarVelFilter - gMotorVelFilter) * 100) > ABS_VALUE(gMotorVelFilter * percent)) && (ABS_VALUE(gMotorVelFilter) >= 100)) //惯导与电机速度超限百分比
#define IS_CAR_DECELERATION_AND_SKID(motorNum, tarVel)  ((((tarVel >= 0) && (gStMotorRunState[motorNum].setAcc < 0)) || ((tarVel <= 0) && (gStMotorRunState[motorNum].setAcc > 0))) /*目标速度＞=0，且设定目标加速度＜0，即前进减速段,目标速度＜=0，且设定目标加速度＞0，即后退减速段*/\
            && (ABS_VALUE((gCarVel[NAV_CAR_VEL_RECORD_NUM - 1] - gMotorAvaVelFilter) * 100) > ABS_VALUE(gCarVel[NAV_CAR_VEL_RECORD_NUM - 1]) * 150))  //轮子打滑，惯导与电机速度差值百分比阈值判定
#define IsBrakeValid    ((DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType) && (sys_para->CAR_RTinf.Link & LINK_PC))    //(PC控制下才生效)

//电机控制线程相关时间定义
#define	MOTORSTATUS_ANALYSIS_TIME		    (20)   //电机状态检测轮询时间ms
#define	MOTORSTATUS_TIMEOUT_TIME		    (500)  //电机状态检测超时时间ms
#define	MOTORSTATUS_TIMEOUT_LONGTIME		(3000) //电机状态检测长超时时间ms

#define MOTOR_ALL_POWERING_TIMEOUT          (10000) //所有电机上电总超时时间10s，一旦到达这个时间，直接判断为已上电
#define MOTOR_LOCK_OFF_DELAY_TIME           (25000) //驱动器断电后，抱闸未关的情况下，延时关抱闸的时间
#define MOTOR_ALL_POWEROFF_TIME             (5000)  //所有电机下电时间
#define MOTOR_ALL_PRE_CHARGE_TIME           (1800)  //预充电时间

#define	MOTOR_CAN_TIMEOUT_TIMES             (5)     //CAN通信超时次数限制
#define	MOTOR_PECT_HOMMING_TURN_TIMEOUT	    (30000) //寻零转圈超时时间ms
#define	MOTOR_AUTO_HOMMING_TIME	            (7200000)//自动寻零间隔时间ms

#define MOTOR_NO_LOAD_JUDGE_MIN_HALF_PERIOD 50      //无负载判断最小振荡半周期
#define MOTOR_NO_LOAD_JUDGE_MAX_HALF_PERIOD 750     //无负载判断最大振荡半周期

//高优先级线程相关时间定义
#define MOTOR_CONTROL_THREAD_BASE_TIME		(2)     //电机控制线程基准轮询时间ms
#define LOW_VOLTAGE_CHECK_PERIOD		    (100)   //低压检测周期时间ms

#define	OPERATION_MODE_SET_TIMEOUT		    (10000) //模式切换超时时间ms

#define MOTOR_ERROR_RECOVERY_TIMEOUT	    (180000)//电机故障自恢复的超时时间ms
#define MOTOR_ERROR_RECOVERY_TIMES	        (4)     //电机故障自恢复周期内多少次故障才进行上报，否则继续进行自恢复

#define MOTOR_RUN_MODE_STEP_IDLE    100     //电机运行空闲
#define MOTOR_RUN_MODE_STEP_START   101     //电机开始运行
#define MOTOR_RUN_COMMON_STEP1      102     //通用步骤1
#define MOTOR_RUN_COMMON_STEP2      103     //通用步骤2
#define MOTOR_RUN_COMMON_STEP3      104     //通用步骤3
#define MOTOR_RUN_COMMON_STEP4      105     //通用步骤4
#define MOTOR_RUN_COMMON_STEP5      106     //通用步骤5
#define MOTOR_RUN_COMMON_STEP6      107     //通用步骤6
#define MOTOR_RUN_MODE_STEP_END     110     //电机结束运行

#define M_BUFFER_IIt_LEN            10      //过载时间缓存长度
#define IIt_JUDGE_VALID             0x01u   //过载判断有效标志
#define IIt_ENTER_WAIT_RECOVERY_FLAG 0x02u  //过载进入等待恢复标志，此时不允许再次进行过载测试
#define IIt_POSITIVE_CURRENT_FLAG   0x04u   //过载正电流标志，正电流过载切换至负电流过载时需要存入下个缓冲区
#define IIt_SET_FLAG(motorNum, x)   gStMotorRunState[motorNum].IItFlag |= x
#define IIt_CLEAR_FLAG(motorNum, x) gStMotorRunState[motorNum].IItFlag &= (~x)
#define IIt_IS_SET_FLAG(motorNum, x) (gStMotorRunState[motorNum].IItFlag & x)

#define M_PID_EXE_FLAG              0x01    //pid部分代码执行标志
#define M_R_ACCORD_L_FLAG           0x02    //右电机依据左电机标志
#define M_FOUR_ALONE_FLAG           0x04    //四电机独立标志

#define M_PID_IDLE_FLAG             0       //PID空闲标志
#define M_PID_RUN_FLAG              1       //PID运行标志
#define M_PID_END_FLAG              2       //PID结束标志

//设定速度状态切换定义
enum{
    SET_VEL_STATE_IDLE,             //设定速度空闲状态
    SET_VEL_STATE_START,            //设定速度开始启动状态
    SET_VEL_STATE_ACC,              //设定速度加速阶段
    SET_VEL_STATE_ACC_END,          //设定速度加速结束状态
    SET_VEL_STATE_UNI,              //设定速度匀速状态
    SET_VEL_STATE_DEC_START,        //设定速度减速开始状态
    SET_VEL_STATE_DEC,              //设定速度减速阶段
};

//电机运行模式标志定义
enum{
    MOTOR_RUN_MODE_POWER_OFF,       //断电模式
    MOTOR_RUN_MODE_STOP,            //停止模式
    MOTOR_RUN_MODE_FLAP_SIN,        //正弦拍动模式
    MOTOR_RUN_MODE_FLAP_FIX,        //匀速拍动模式
    MOTOR_RUN_MODE_POS,             //位置模式
    MOTOR_RUN_MODE_POS_INFRARED,    //红外位置模式
    MOTOR_RUN_MODE_HOMMING,         //寻零模式
    MOTOR_RUN_MODE_SPEED,           //速度模式
    MOTOR_RUN_MODE_FLAP_FIX_CURRENT,//匀速电流拍动模式
    MOTOR_RUN_MODE_CURRENT_SPEED,   //电流速度模式
};

enum{
    MOTOR_RUN_MODE_NO_CHANGE,       //模式无改变
    MOTOR_RUN_MODE_CHANGED,         //模式发生改变
    MOTOR_RUN_PARA_CHANGED,         //参数发生改变
};

//电机运行状态标志定义
enum{
    MOTOR_STATE_RUN,    //运行状态
    MOTOR_STATE_STOP,   //停止/抱闸状态
    MOTOR_STATE_POWER_OFF,   //断电状态
    MOTOR_STATE_ERROR   //故障状态
};

//电机上电标志定义
enum{
    MOTOR_POWER_FLAG_OFF,   //断电状态
    MOTOR_POWEROFF_FLAG_TIMECNT,   //下电计时状态
    MOTOR_POWER_FLAG_TIMECNT,   //上电计时状态
    MOTOR_POWER_FLAG_ON,   //上电计时完成
}; 

//胸鳍拍动标志定义
enum{ 
    PECT_STATE_NORMAL,                      //正在拍动
    PECT_STATE_WAIT_RUN_TO_ZERO_PHASE,      //停止后等待运动置零相位
    PECT_STATE_START,                       //开始拍动
};
//寻零标志定义
enum{
    HOME_FLAG_UNDEF,  //未知位置
    HOME_FLAG_FINISH,   //零位设置完成标志
    HOME_FLAG_SET_MOVE,    //设定运行
    HOME_FLAG_RECORD_POS,  //记录位置
    HOME_FLAG_CLEAR_STOP,   //清除停止标志
    HOME_FLAG_WAIT_STOP,    //等待停止标志
    HOME_FLAG_MOVE_CW,      //正向运行
    HOME_FLAG_MOVE_CCW,     //反向运行
    HOME_FLAG_WAIT_CW_STOP, //等待正向停止
    HOME_FLAG_WAIT_CCW_STOP,//等待反向停止
    HOME_FLAG_WAIT_CCW_STALL_STOP,//等待反向堵转停止
    HOME_FLAG_SET_POS,      //设定位置
    HOME_FLAG_SET_MODE_IO,  //设定io控制模式
    HOME_FLAG_SET_MODE_SPWM_DIR,  //设定pwm+方向控制模式
    HOME_FLAG_STOP,         //停止电机
    HOME_FLAG_RECORD_INIT_POS,  //记录初始位置
    HOME_FLAG_RECORD_SETMAXOIL_POS,   //记录设置的最大油压对应的位置
};
//位置标志定义
enum{
    POS_FLAG_TARGET,   //目标位置
    POS_FLAG_UNKOWN,   //未知位置
    POS_FLAG_LIMIT_UP,   //上限位置
    POS_FLAG_LIMIT_DOWN,   //下限位置
    POS_FLAG_LIMIT_UP_MAX,   //上限极限位置，有两种限位处理才有效
    POS_FLAG_LIMIT_DOWN_MAX,   //下限极限位置，有两种限位处理才有效
};
//读取数据步骤定义
enum{
    MOTOR_READ_FINISH   = 0,        //读取完毕
    MOTOR_READING       = 0x01,     //正在读取标志
    MOTOR_READ_CURRENT  = 0x02,     //读取电流
    MOTOR_READ_VEL      = 0x04,     //读取速度
    MOTOR_READ_POS      = 0x08,     //读取位置
    MOTOR_READ_TMP      = 0x10,     //读取温度
    MOTOR_READ_VOL      = 0x20,     //读取电压
};
//电机运行模式切换步骤定义
enum{
    OPERATION_MODE_UNKOWN,//未知
    OPERATION_MODE_SET,   //设置模式
    OPERATION_MODE_SET_HOME_METHOD,   //设置回零方案
    OPERATION_MODE_SHUT_DOWN,   //切换到准备状态
    OPERATION_MODE_AUTO_CHANGE,   //根据返回的状态自动切换
    OPERATION_MODE_SET_NEW_POSITION,   //位置模式下设置新的位置
    OPERATION_MODE_SET_FINISH,   //切换模式完成
};
//电机运行参数设置标志
enum{
    RUN_PARAS_NOT_SET = 0,  //运行参数未设置
    RUN_PARAS_PROFILE_VEL_SET = 0x01,  //轮廓速度已设定
    RUN_PARAS_PROFILE_ACC_SET = 0x02,  //轮廓加速度已设定
    RUN_PARAS_PROFILE_DEC_SET = 0x04,  //轮廓减速度已设定
};
//电机停止步骤定义
enum{
    OPERATION_STOP_FINISH,   //停止完成
    OPERATION_DISABLE_VOLTAGE,    //关断驱动器电压
    OPERATION_QUCIK_STOP,   //快速停止
    OPERATION_QUCIK_STOP_WAIT,   //等待快速停止
    OPERATION_HALT,   //刹车停止
    OPERATION_HALT_WAIT,   //等待刹车停止
    OPERATION_SLOW_STOP,   //缓慢停止
    OPERATION_SLOW_STOP_WAIT,   //等待缓慢停止
    OPERATION_CURRENT_SLOW_STOP_WAIT,   //电流模式等待缓慢停止
    OPERATION_STOP_ADJUST,  //判断电机是否真实停止
};
//电机目标位置、速度切换宏定义
enum{
    MOTOR_SET_NOTHING,      //不设置
    MOTOR_SET_TARGET_POS,   //只设置目标位置
    MOTOR_SET_TARGET_VEL,   //只设置目标速度
    MOTOR_SET_TARGET_CURRENT,   //只设置目标电流
    MOTOR_SET_TARGET_POS_NEED_JUDGE,   //只设置目标位置，并且需要判断驱动器是否执行
    MOTOR_SET_TARGET_VEL_NEED_JUDGE,   //只设置目标速度，并且需要判断驱动器是否执行
    MOTOR_SET_TARGET_CURRENT_NEED_JUDGE,   //只设置目标电流，并且需要判断驱动器是否执行
    MOTOR_SET_ZEOR_VEL_LOCKOFF_NEED_JUDGE, //停电机并关抱闸
    MOTOR_SET_TARGET_VEL_JUDGE,     //开始判断电机是否跟随目标速度
    MOTOR_SET_TARGET_POS_JUDGE,     //开始判断电机是否跟随目标位置
    MOTOR_SET_TARGET_POS_CLEAR_START_FLAG,  //启动位置运行后清除运行标志
};		
//电源状态
typedef enum
{
	POWER_OFF = 0,
	POWER_ON =1
} PowerState; 
//刹车状态
typedef enum
{
	BRAKE_OFF = 0,      //刹车断电状态
	BRAKE_ON  = 1,      //刹车上电状态
	BRAKE_OFF_CMD = 2,  //刹车断电命令状态
	BRAKE_POS_CMD = 3,  //刹车位置命令状态
	BRAKE_OFF_FORCE = 4,//刹车强制断电状态
	BRAKE_OFF_FORCE_CMD = 5,    //刹车强制断电命令状态
} BrakeState;
//刹车等待状态
typedef enum
{
	BRAKE_WAIT_NONE = 0,//刹车无需等待
	BRAKE_WAIT_PRE  = 1,//刹车预等待
	BRAKE_WAIT = 2,     //刹车等待
} BrakeWaitState;

//抱闸状态
typedef enum
{
	LOCK_OFF = 0,       //抱闸断电状态
	LOCK_ON  = 1,       //抱闸上电状态
} LockState;
//位置类型
typedef enum
{
    POS_TYPE_NULL = 0,
	POS_TYPE_SINGLE_ABSOLUTE = 1,   //单圈绝对位置
	POS_TYPE_SINGLE_RELATE = 2,     //单圈相对位置
	POS_TYPE_MULTI_ABSOLUTE = 3,    //多圈绝对位置
	POS_TYPE_TWO_ELECT = 4          //两端光电位置值
} PosType; 
//寻零类型
typedef enum
{
    HOMMING_TYPE_NULL = 0,
	HOMMING_TYPE_STAND = 1,         //标准寻零，光电被挡后设置零位
	HOMMING_TYPE_OUT_IN_OFFSET = 2, //出进加偏移寻零，因为挡片是有厚度的，如果当前已在光电传感器位置，为保证寻零准度，需先离开再进入，再次进入时，再运行一个偏移位置
	HOMMING_TYPE_TWO_ELECT = 3      //两端光电寻零
} HommingType; 

/**
 * Motor Runmode Data
 */
typedef struct
{
    uint8_t              run_mode;                       //电机运行模式
    HommingType          homming_type;                   //寻零需要先离开光电传感器标志，因为挡片是有厚度的，如果当前已在光电传感器位置，为保证寻零准度，需先离开再进入
    int16_t              target_value;                   //目标位置或目标速度
    PosType              posType;                        //位置类型
    //int8_t               flap_amp;                       //拍动幅度
    //float                flap_fre;                       //拍动频率
}ST_MOTOR_RUN_MODE_DATA;

/**
 * Motor Runstate Data
 */
typedef struct
{
    //电机运行状态相关
    uint8_t              change_flag;                    //变化标志
    uint8_t              run_step;                       //当前运行步骤
    uint32_t             runTotalTime;                   //运行计时
    uint32_t             runDelayTime;                   //运行延时
    uint32_t             pidDelayTime;                   //pid运行延时
    uint32_t             pidNoExeTime;                   //pid未执行计时
    uint32_t             pidPeriodTime;                  //pid运行周期
    uint8_t              pidRunFlag;                     //pid运行标志
    uint8_t              homeFinishFlag;                 //寻零完成标志
    uint8_t              startRemoteFlag;                //启动远程帧标志，允许PDO和SDO传输
    uint8_t              skidFlag;                       //电机打滑标志
    uint8_t              skidOffCnt;                     //电机打滑关闭计数
    uint32_t             skidTotalTime;                  //打滑计时
    //电源、抱闸、限位相关
    uint8_t              powerFlag;                      //电机上电完成标志（包含上电时间等待）
    uint32_t             powerTime;                      //电机上电时间
    uint8_t              enableFlag;                     //使能标志
    ENUM_DIR             dirFlag;                        //方向标志
    uint32_t             limit1Time;                     //限位1滤波时间累计
    uint32_t             limit2Time;                     //限位2滤波时间累计
    //速度位置电流相关
    int32_t              lastPoweroffPos;                //上次断电时的位置
    int32_t              targetPos;                      //目标位置
    int32_t              curPos;                         //当前位置
    int32_t              limitPos1;                      //限位1值
    int32_t              limitPos2;                      //限位2值
    int32_t              PosInit;                        //初始值
    int32_t              setTargetVel;                   //设定的目标速度
    int32_t              lastSetTargetVel;               //上次设定的目标速度
    int32_t              targetVel;                      //目标速度
    int32_t              targetVelOffset;                //目标速度偏移值
    int32_t              motorVelRecord;                 //电机速度记录
    int32_t              setCurrent;                     //设定的电流
    int32_t              lastSetCurrent;                 //上次设定的电流值
    int32_t              accCurrent;                     //加速度补偿的电流值
    int32_t              limitCurrent;                   //限定电流值
    uint8_t              posFlag;                        //电机位置标志
    uint8_t              readStep;                       //读取步骤
    uint8_t              targetPosJudgeFlag;             //位置到位判断
    uint8_t              targetPosFlag;                  //位置或寻零模式时，到达目标位置标志
    uint8_t              targetPosVelSetFlag;            //设定目标位置和目标速度标志
    uint8_t              pidCurrentStartFlag;            //电流pid调节标志
    uint8_t              clearStartMoveFlag;             //清除开始运行标志
    uint32_t             readTimeCnt;                    //读取计时
    uint32_t             posJudgeTimeCnt;                //位置判断计时
    uint32_t             posJudgeCnt;                    //位置判断计次
    int32_t              lastPosRecord;                  //上次位置记录
    //红外相关
    uint32_t             infraFaultTimeTotal;            //红外故障时间累计
    uint32_t             infraTimeDelay;                 //红外速度调节延时
    float                infraPosRecord;                 //红外位置记录
    float                infraPosTotal;                  //红外位置值累计
    uint32_t             infraPosCnt;                    //红外位置值计数
	//电机驱动模式切换相关
    int8_t               operationMode;                  //电机运行模式
    int8_t               operationStopJudgeFlag;         //电机停止判断标志，判断电机实际转速是否已停止，停止才切换模式，使能电机
    uint16_t             operModeSetStep;                //电机运行模式设置步骤
    uint16_t             operSetNewStep;                 //新位置设置步骤
    uint16_t             operStopStep;                   //电机运行时停止步骤
    uint32_t             operTimedelay;                  //时间延迟
    uint32_t             operModeSetTimeOut;             //电机运行模式设置超时计算
    uint32_t             retryTime;                      //电机运行模式设置失败重试的次数
    uint32_t             retrySetTargetTime;             //电机目标值设置失败重试的次数
    uint32_t             runParasSetFlag;                //电机运行参数设置标志
    //故障判断相关
    uint8_t              resetFlag;                      //复位标志
    uint8_t              errorCnt;                       //故障计数
    uint8_t              stallFlag;                      //堵转标志
    uint8_t              reverseFlag;                    //反向标志
    uint8_t              offLineCnt;                     //掉线计数
    uint8_t              abnormalOperationCnt;           //异常去使能计数
    ERROR_LEVEL          newErrorLevel;                  //新故障等级
    ERROR_LEVEL          errorLevel;                     //故障等级
    uint32_t             offLineTime;                    //电机上电后，检测电机状态离线时间
    //pid参数
    PIDParas_incres      pidIncresParas;                 //增量式pid参数
    PIDParas_incres      pidIncresParasOne;              //增量式pid参数1
    PID                  pidFuzzyParas;                  //模糊pid参数
    //int32_t              curMVelFilter;                  //当前电机滤波完的速度值
    int32_t              setAcc;                         //设定加速度
    int32_t              curAcc;                         //本次加速度
    ST_KALMAN_DATA       setAccKalmanData;               //设定加速度卡尔曼滤波结构体
    int32_t              lastSetAcc;                     //上一次设定加速度
    uint32_t             pcLostRecoveryTime;             //失联恢复时刻
    int32_t              setVelRecord[MVEL_RECORD_NUM];  ////设定速度记录
    int32_t              setCarVel[SET_VEL_RECORD_NUM];  //规划速度值记录数组
    uint16_t             setCarVelValidNum;              //规划速度值有效个数
    uint16_t             setVelState;                    //设定速度状态
    uint32_t             setCarVelTime[SET_VEL_RECORD_NUM];//规划车速记录时刻
    int32_t              setCarAcc;                     //规划加速度计算值
    uint32_t             IItBuffer[M_BUFFER_IIt_LEN];   //过载值缓存
    uint32_t             IItStartTime[M_BUFFER_IIt_LEN];//每次过载时的开始时间
    uint32_t             IItIndex;                      //过载缓存指针
    uint32_t             IItOverLastTime;               //上次过载时刻
    uint32_t             IItOverTimeRecord;             //过载时间记录
    uint32_t             IItLimit;                      //过载限制值
    uint32_t             IItFlag;                       //过载标志位
}ST_MOTOR_RUN_STATE_DATA; 
#pragma pack (1)  //1字节对齐
/**
 * Motor Data
 */
typedef struct
{
    uint8_t                 idx;                            //can通信id->1
    uint8_t                 relatedMotor;                   //关联的电机序号->2
    uint8_t                 powerPort;                      //电源控制端口->3
    uint8_t                 powerPin;                       //电源控制引脚->4
    DRIVER_TYPE             driverType;                     //驱动器类型->8
    uint32_t                flag;                           //标志位->12
    uint32_t                powerDelayTime;                 //电机上电延时时间->16
    int32_t                 counts;                         //电机一转对应脉冲数->20
    float                   ratio;                          //电机减速比->24
    int32_t                 limitSpeed;                     //限制转速rpm->28
    int32_t                 speedGrad;                      //加减速梯度rpm->32
    uint32_t                speedAdjustPeriod;              //速度调节周期ms->36
    int32_t                 homingSpeed;                    //寻零速度->40
    int32_t                 normalCurrent;                  //额定电流mA->44
    uint32_t                enableDelayTime;                //使能延时时间->48
    float                   kp;                             //kp->52
    float                   ki;                             //ki->56
    uint32_t                currentAdjustPeriod;            //电流调节周期ms->60
    int32_t                 limitCurrent;                   //限制电流mA->64
    int32_t                 startCurrent;                   //起动电流值mA->68
    uint32_t                start_fault_time;               //启动判故障时间->72
    int32_t                 pos_limit1;                     //位置限制1->76
    int32_t                 pos_limit2;                     //位置限制2->80
    uint32_t                profileVelocity ;               //轮廓速度rpm->84
    uint32_t                profileAcc;                     //轮廓加速度rpm/s2 ->88
    int32_t                 initPos;                        //标定位置->92
    uint32_t                overCurrentTime;                //过流时间->96
    uint8_t                 reserved[32];                   //预留->128
}ST_MOTOR_DATA;
#pragma pack ()  //1字节对齐

//故障结构体信息，用于分析故障原因
#pragma pack (1)  //1字节对齐
typedef struct
{
    //故障通用信息
    uint16_t                errorResult;                    //故障码
    uint16_t                errorMotorNum;                  //故障序号
    uint32_t                errRecordTime;                  //故障记录时间
    //1003故障相关信息
    int32_t                 setCurrentPositiveChangeIndex;  //设定电流正变化索引值
    int32_t                 setCurrentNegativeChangeIndex;  //设定电流负变化索引值
    int32_t                 carFilter;                      //惯导滤波值
    int32_t                 motorFilter;                    //电机滤波值
    int16_t                 ratioVelAuto;                   // 速度转电机转速百分比值(实时计算)
    //1004故障相关信息
    int32_t                 setAcc;                         //设定加速度
    int32_t                 curAcc;                         //本次加速度
    //pid数据信息记录
    uint8_t                 idx;                            //can通信id
    int32_t                 setVelRecord[MVEL_RECORD_NUM];  ////设定速度记录
    int32_t                 carVelRecord[MVEL_RECORD_NUM];  //惯导速度记录
    int32_t                 mVel[MVEL_RECORD_NUM];          //电机速度值记录数组
    int32_t                 setCurrentRecord[MVEL_RECORD_NUM];//设定电流记录
    //1002故障相关信息
    uint16_t                driverVoltage[M_TOTAL_NUM];     //驱动器电压值记录
    //9025故障相关信息
    uint16_t                preState;
    uint16_t                afterIndex;
    uint16_t                afterState[6];
    uint32_t                afterTime[6];
    uint16_t                afterWorkState[5];
}ST_ERROR_INFO;
#pragma pack ()  //1字节对齐


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
    uint32_t     timeCnt;                   //读取计时
    uint32_t     rev_flag;                  //接收到数据标志
    uint16_t     status;                    //电机状态信息
    int32_t      current;                   //电机电流
    int32_t      speed;                     //电机转速
    int32_t      pos;                       //位置数据
    int16_t      tmp;                       //驱动器温度数据
    int16_t      motorTmp;                  //电机温度数据
    uint16_t     vol;                       //电压数据
    uint32_t     eMergencyErrorCode;        //紧急故障码
    uint8_t      ctrlStatus;                //电机控制器状态信息
    uint32_t     lastSendTime;              //上次发送时间
}ST_MOTOR_REV_DATA;

extern ST_MOTOR_REV_DATA gStMotorRevData[M_TOTAL_NUM];
extern uint16_t gErrorResult;
extern uint16_t gErrorMotorNum;
extern uint16_t gErrorRunStateFlag;
extern uint32_t gErrorTime;
extern ST_ERROR_INFO gStErrorInfo;
extern BrakeState gBrakeFlag;                           //刹车标志
extern BrakeWaitState gBrakeWaitOnFlag;
extern uint16_t gBrakeValue; 
extern LockState gLockFlag;                             //抱闸标志

//开关刹车命令
#define SET_BRAKE(value)    if((BRAKE_OFF_FORCE != gBrakeFlag) && (BRAKE_OFF_FORCE_CMD != gBrakeFlag))\
                            {\
                                gBrakeFlag = BRAKE_POS_CMD; \
                                gBrakeValue = value;\
                            }
#define CLOSE_BRAKE         if((BRAKE_OFF != gBrakeFlag) && (BRAKE_OFF_FORCE != gBrakeFlag) && (BRAKE_OFF_FORCE_CMD != gBrakeFlag)) gBrakeFlag = BRAKE_OFF_CMD
//强制关刹车
#define CLOSE_BRAKE_FORCE   if((BRAKE_OFF_FORCE != gBrakeFlag) && (gStUfoData.flag & UFO_ENABLE_LOCK)\
                            && (gStMotorData[M_BRAKE].powerPort == gStUfoData.lockPort)\
                            && (gStMotorData[M_BRAKE].powerPin == gStUfoData.lockPin)\
                            && (DRIVER_TYPE_NONE != gStMotorData[M_BRAKE].driverType))\
                            {\
                                gBrakeFlag = BRAKE_OFF_FORCE_CMD;\
                            }

//开关抱闸命令，如果刹车因抱闸强行关闭，则刹车上电
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

//电机接收数据标志定义
#define REV_MOTOR_BOOT_UP_FLAG      0x01u   //接收到启动信息
#define REV_MOTOR_STATUS_FLAG       0x02u   //接收到状态信息
#define REV_MOTOR_CURRENT_FLAG      0x04u   //接收到电流数据
#define REV_MOTOR_SPEED_FLAG        0x08u   //接收到速度信息
#define REV_MOTOR_EMERGENCY_FLAG    0x10u   //接收到紧急故障消息
#define REV_MOTOR_CTRL_STATUS_FLAG  0x20u   //接收到控制器状态信息
#define REV_MOTOR_POS_FLAG          0x40u   //接收到位置信息
#define REV_MOTOR_TMP_FLAG          0x80u   //接收到温度信息
#define REV_MOTOR_VOL_FLAG          0x100u  //接收到电压信息
#define REV_MOTOR_DRIVER_TMP_FLAG   0x200u  //接收到驱动器温度信息

#define SET_MOTOR_REV_DATA_FLAG(motorNum, x)        gStMotorRevData[motorNum].rev_flag |= x
#define CLEAR_MOTOR_REV_DATA_FLAG(motorNum, x)      gStMotorRevData[motorNum].rev_flag &= (~x)
#define IS_REV_MOTOR_DATA(motorNum, x)  (gStMotorRevData[motorNum].rev_flag & x)


//AGV小车轮子物理参数
#define WHEEL_RADIUS          0.09f   	//轮子半径m
#define GEAR_RATIO            16.0f     //齿轮比
#define WHEEL_DIST_X          0.00f     //轮子X方向间距
#define WHEEL_DIST_Y          0.00f   	//轮子Y方向间距
#define WHEEL_COUNT 			2

//浮点型比较
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

