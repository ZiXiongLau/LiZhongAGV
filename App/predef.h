#ifndef __PREDEF_H
#define __PREDEF_H

#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "FilterDef.h"

#define CODE_VERSION        "V1.002B077"

//硬件版本标志位
#define UFO_PRE_CHARGE_FLAG             0x01        //预充标志
#define UFO_FOUR_DRIVER_FLAG            0x02        //四驱动同步标志
#define UFO_CAN_RX_PI9                  0x04        //CAN引脚更换
#define UFO_FEEDBACK_MAXSPEED_FLAG      0x08        //速度反馈值取电机最大速度标志(未配置则取平均速度)
#define UFO_PC_CAN_FLAG                 0x10        //工控机can通信标志
#define UFO_BATTERY_CAN2_FLAG           0x20        //电池使用can2通信标志
#define UFO_ERROR_WAIT_FLAG             0x40        //故障等待标志，不进行故障自恢复
#define UFO_ENABLE_REMOTE_HIGH_VEL_FLAG 0x80        //允许遥控高速标志
#define UFO_NEW_PROTOCOL                0x100       //新通信协议
#define UFO_ENABLE_LOCK                 0x200       //使能抱闸
#define UFO_ENABLE_POS_JUDEGE           0x400       //使能到位位置判断
#define UFO_LOCK_TWO_CTRL               0x800       //抱闸两个端口控制
#define UFO_ENABLE_SEND_MOTOR_DATA      0x1000      //使能电机数据上发
#define UFO_LEFT_RIGHT_ALONE_FLAG       0x2000      //左右侧电机独立控制标志
#define UFO_MOTOR_AS_FEEDBACK_FLAG      0x4000      //电机速度作为pid反馈标志
#define UFO_ENABLE_REMOTE_QUCIK_STOP    0x8000      //使能遥控紧急停止
#define UFO_BMS_JBD                     0x10000     //JBD品牌485通信bms
#define UFO_BMS_JIAYUAN                 0x20000     //佳源品牌485通信bms
#define UFO_BMS_YBT                     0x40000     //优倍特can通信bms
#define UFO_PRECHARGE_INVERSE           0x80000     //预充引脚反向
#define UFO_BMS_JIKONG                  0x100000    //极空can通信bms
#define UFO_LOCK_INVERSE                0x200000    //抱闸引脚反向
#define UFO_BMS_BAIWEI                  0x400000    //百维can通信BMS
#define UFO_BMS_YBT_v2                  0x800000    //优倍特v2.0 can通信bms
#define UFO_DISABLE_COLLISION_DETECT    0x1000000   //禁止碰撞检测
#define UFO_KINCO_NEW_PCB               0x2000000   //步科驱动器使用新pcb控制
#define UFO_NOT_USE_GET_SET_ACC         0x4000000   //不使用获取到的设定加速度
#define UFO_BMS_YBT_v3                  0x8000000   //优倍特v3.0 can通信bms
#define UFO_BMS_YBT_THREE_LARGE         0x10000000  //优倍特三轮大平板电池
#define UFO_BMS_JIUPU_FOUR_LARGE        0x20000000  //四轮大平板久普电池
#define UFO_ENABLE_FLAG_ONE             0x80000000  //使能标志位1的相关功能，此位由程序自动至1，强制不可清除

//标志位1相关定义
#define IS_UFOONE_FLAG_SET(flag)        (gStUfoData.flagOne & (flag))
#define UFOONE_SET_ACC_OFFSET_VALID     0x01        //设定目标加速度补偿有效，用于消除速度误差
#define UFOONE_MOTION_SWITCH_DEAL       0x02        //设定动作状态切换处理，用于消除速度过冲
#define UFOONE_PC_USART_FLAG            0x04        //工控机串口通信标志(当前使用为串口5，若后续使用其它串口，程序要做相应更改)
#define UFOONE_SOC_FROM_VOL             0x08        //电量值来源于电压换算
#define UFOONE_DIFF_DRIVER_CURRENT      0x10        //不同驱动电流标志，允许动力电机功率不一致
#define UFOONE_BMS_500K                 0x20        //BMS波特率指定500k
#define UFOONE_TWO_WHEL_DEFF_DRIV       0x40        //两轮差速驱动
#define UFOONE_FOUR_ALONE_FLAG          0x80        //四轮独立标志
#define UFOONE_BRAKE_DEC_TEST_FLAG      0x100       //刹车减速度测试
#define UFOONE_FOUR_TURN_MOTOR          0x200       //四个转向电机标志
#define UFOONE_USE_MOTOR_RATIO          0x400       //速度换算使用电机速比
#define UFOONE_BRAKE_ADVANCE_START      0x800       //刹车提前启动
#define UFOONE_BRAKE_STOP_ACCORD_OIL    0x1000      //刹车依据油压停止
#define UFOONE_MOTOR_STATE_DUAL_CTRL    0x2000      //电机上电和使能分开控制
#define UFOONE_NO_NAV_FLAG              0x4000      //无惯导标志
#define UFOONE_DISABLE_MIN_VOL_KPH_FLAG 0x8000      //禁止最小电压对应时速限制标志
#define UFOONE_BMS_SMTK_FOUR_MID        0x1000000   //四轮中平板思玛泰克电池

#pragma pack (1)  //1字节对齐
/**
 * UFO Data
 */
typedef struct
{
    uint16_t                minVol;                 // 电池最低电压->2
    uint8_t                 fullVol;                // 电池满电电压->3
    uint8_t                 preChargePort;          // 预充电控制端口->4
    uint8_t                 preChargePin;           // 预充电控制引脚->5
    int16_t                 ratioVel;               // 速度转电机转速百分比值[v(m/s)=n(rpm)/i/60*2π*R(mm)/1000 --> ratioVel=1/(1/i/60*2π*R(mm)/1000)]->7
    uint32_t                flag;                   // 标志位->11
    uint32_t                oilPressStand;          // 油压标准值->15
    uint8_t                 lockPort;               // 抱闸控制端口->16
    uint8_t                 lockPin;                // 抱闸控制引脚->17
    uint32_t                flagOne;                // 标志位1->21
    uint8_t                 ipLastAdr;              // ip最后一个字节地址->22
    uint8_t                 minVol20kph;            // 20kph以上时速要求的最低电压->23
    uint8_t                 minVol40kph;            // 40kph以上时速要求的最低电压->24
    uint8_t                 minVol60kph;            // 60kph以上时速要求的最低电压->25
    uint8_t                 minVol80kph;            // 80kph以上时速要求的最低电压->26
    uint8_t                 minVol100kph;           // 100kph以上时速要求的最低电压->27
    uint8_t                 minVol110kph;           // 110kph以上时速要求的最低电压->28
    uint8_t                 reserved[4];            // 预留->32
}ST_UFO_DATA;
#pragma pack ()  //1字节对齐 

//电机序号定义
enum
{
    M_LEFT = 0,                         //左驱动电机
    M_RIGHT,                            //右驱动电机
    M_TURN,                             //转向电机
    M_BRAKE,                            //刹车电机
    M_LEFT_ONE,                         //左驱动电机1
    M_RIGHT_ONE,                        //右驱动电机1
    M_TOTAL_NUM,
    M_NUM_PI7
};

//前后轮序号定义
typedef enum
{
    W_LEFT_FRONT = 0,                   //左前轮
    W_RIGHT_FRONT,                      //右前轮
    W_LEFT_REAR,                        //左后轮
    W_RIGHT_REAR,                       //右后轮
    W_TOTAL_NUM,
}EN_WHEEL_ORDER;

//驱动器类型
#define ENABLE_CANOPEN_DRIVER           //使能canopen驱动器
enum
{
    DRIVER_TYPE_NONE,       //无驱动器类型，表示电机不存在
    DRIVER_TYPE_STAND,      //标准驱动器类型，如ELMO驱动器
    DRIVER_TYPE_EPOS,       //EPOS驱动器
    DRIVER_TYPE_COMPLEY,    //COMPLEY驱动器
    DRIVER_TYPE_NIMOTION,   //立迈胜驱动器
    DRIVER_TYPE_PUSI,       //谱思驱动器
    DRIVER_TYPE_QILING,     //麒麟驱动器
    DRIVER_TYPE_NOT_CANOPEN,//非canopen驱动器
    DRIVER_TYPE_KINCO,      //步科驱动器
    DRIVER_TYPE_HCX,        //宏创兴驱动器
    DRIVER_TYPE_LEADSHINE,  //雷赛驱动器
    DRIVER_TYPE_DUOJI_GDW,  //GDW舵机8.4V 1520us/333Hz
    DRIVER_TYPE_NOT_CANOPEN_END = 30,//非canopen驱动器结束段
    DRIVER_TYPE_FDK = 31,   //风得控驱动器
    DRIVER_TYPE_KINCO_CAN = 32,//步科can接口驱动器
};
typedef uint32_t DRIVER_TYPE;
#define IS_NOT_CANOPEN_DRIVER(driver)   ((driver >= DRIVER_TYPE_NOT_CANOPEN) && (driver <= DRIVER_TYPE_NOT_CANOPEN_END))

//电机标志位
#define ENABLE_AUTO_HOMMING             0x01        //使能自动寻零
//#define ENABLE_STO                      0x02        //使能STO
#define MOTOR_USE_PI7_POWER             0x04        //使用PI7电源控制端口
#define MOTOR_NOT_JUDGE_BOOTUP          0x08        //上电不检查启动事件，因为收到启动信号后不代表电机可以运行
#define MOTOR_NEED_SET_RUN_PARAS        0x10        //需要设定运行参数
//#define MOTOR_FIX_DIRECT_POSOTIVE       0x20        //电机固定正方向
//#define MOTOR_HOMING_RECORD_POS         0x40        //电机寻零记录位置，以防止有飞轮的情况下电机不能及时停下来
//#define MOTOR_REACH_POS_NEED_RETURN     0x80        //电机到位后需要反转，主要是针对油泵电机，防止负压
#define MOTOR_CURRENT_ADJUST_SPEED      0x100       //电流调节转速
#define MOTOR_RELATED_SYNCHRO           0x200       //关联电机完全同步标志
#define MOTOR_ENABLE_FLAG               0x400       //电机使能标志
#define MOTOR_DIR_FLAG                  0x800       //电机方向标志
#define MOTOR_INVALID_FLAG              0x1000      //电机无效标志
#define MOTOR_DIR_INVERSE_FLAG          0x2000      //电机反向标志
//#define MOTOR_LAXIAN_FLAG               0x4000      //拉线标定标志
#define MOTOR_PRESS_SENSOR_FLAG         0x8000      //压力传感器标志
#define MOTOR_POWER_OFF_RECORDPOS       0x10000     //断电记录位置
#define MOTOR_READ_POS_FLAG             0x20000     //读取位置标志
#define MOTOR_PWM_CONTRL_MODE           0x40000     //电机pwm控制模式
#define MOTOR_POWER_ON_INVERSE          0x80000     //电源引脚反向
#define MOTOR_DUAL_POWER                0x100000    //双电源供电
#define MOTOR_TURN_PID_TO_ACC           0x200000    //转向pid当作加速度补偿用
#define MOTOR_ENABLE_STALL_HOMMING      0x400000    //使能堵转寻零

/* boolean type definitions */
typedef int                             rt_bool_t;      /**< boolean type */
#define RT_TRUE                         1               /**< boolean true  */
#define RT_FALSE                        0               /**< boolean fails */

#define FLT_EPSILON                     1e-5f

//LED序号定义
enum
{
    LED0, LED1, LED2, LED3
};

//LED状态定义
typedef enum
{
    LED_ON,     //打开led
    LED_OFF,    //关闭led
}ENUM_LED_STATE;

//正反转方向定义
typedef enum
{
    DIR_CW,     //正转
    DIR_CCW,    //反转
    DIR_STOP,   //停止
}ENUM_DIR;

//故障类型定义
typedef enum
{
    ERROR_NONE,          //无故障
    ERROR_L_NORMAL,      //正常故障，可尝试通过发命令恢复，如果多次恢复失败将转变成高等级故障
    ERROR_L_HIHG,        //高等级故障，只能通过断电尝试恢复，如果多次恢复失败将转变成最高等级故障
    ERROR_L_VERY_HIGH,   //最高等级故障，无法恢复
}ERROR_LEVEL;

//后差速转向模式
typedef enum
{
    BACK_DEFF_NONE = 0,         //后轮无差速
	BACK_DEFF_ACCORD_TURN = 1,  //后轮依据转向值大小进行差速
	BACK_DEFF_MAX = 2,          //后轮最大差速
} BackDeffMode;

//电机警告码定义
#define WARNING_MOTOR_NUM_OFFSET        20          //警告模块序号偏移值
#define IS_WARNING_CODE(motorNum)       (motorNum >= WARNING_MOTOR_NUM_OFFSET)  //判断是否是警告码
#define IS_NO_WARNING_OR_ERROR_CODE     (0 == gErrorResult) //无警告和故障
#define IS_ENABLE_SET_WARNING_CODE      (IS_NO_WARNING_OR_ERROR_CODE || (IS_WARNING_CODE(gErrorMotorNum) && (sys_para->CAR_RTinf.Link & LINK_REV_STOP_CMD)))
#define SET_WARNING_CODE(motorNum, lResult) SetErrorCode((motorNum + WARNING_MOTOR_NUM_OFFSET), lResult, ERROR_NONE)
#define WARNING_CODE_BASE               0x1000
#define WARNING_CODE_LOW_VOLTAGE                    (WARNING_CODE_BASE + 0x01)  //空闲状态时低电压报警
#define WARNING_CODE_RUNNING_LOW_VOLTAGE            (WARNING_CODE_BASE + 0x02)  //运行过程中低电压报警(请更换低速低加速度场景测试)
#define WARNING_CODE_NAV_ABNORMAL                   (WARNING_CODE_BASE + 0x03)  //运行过程中惯导速度值异常
#define WARNING_CODE_ABNORMAL_DECELERATION          (WARNING_CODE_BASE + 0x04)  //运行过程中异常减速(碰撞、摩地、碾压)
#define WARNING_CODE_OVER_LOADER_SINGLE             (WARNING_CODE_BASE + 0x05)  //电机单次过载
#define WARNING_CODE_OVER_LOADER_ONE_MINUTE         (WARNING_CODE_BASE + 0x06)  //电机一分半钟累计过载
#define WARNING_CODE_OVER_LOADER_NEED_REST          (WARNING_CODE_BASE + 0x07)  //电机已过载需停止冷却几分钟再进行下一次过载测试
#define WARNING_CODE_LOW_VOLTAGE_TO_KPH             (WARNING_CODE_BASE + 0x08)  //当前电量较低不支持当前场景测试(请更换低速场景测试)

//电机故障代码定义
#define ERROR_CODE_BASE                 0x9000
#define ERROR_CODE_CAN_LINE_OFF                     (ERROR_CODE_BASE + 0x01)    //can通信离线
#define ERROR_CODE_OPERATRION_MODE_SET_FAILED       (ERROR_CODE_BASE + 0x02)    //运行模式设置失败
#define ERROR_CODE_HOMMING_TIMEOUT                  (ERROR_CODE_BASE + 0x03)    //寻零超时
#define ERROR_CODE_FLAP_SIN_TIMEOUT                 (ERROR_CODE_BASE + 0x04)    //正弦拍动超时
#define ERROR_CODE_MASS_POS_TIMEOUT                 (ERROR_CODE_BASE + 0x05)    //重心电机运行至指定位置超时
#define ERROR_CODE_MAIN_POWER_NOT_OEPN              (ERROR_CODE_BASE + 0x06)    //总电源未打开
#define ERROR_CODE_MAIN_POWER_CLOSE_ABNORMAL        (ERROR_CODE_BASE + 0x07)    //总电源中途异常关闭
#define ERROR_CODE_ECoder11M_BASE_START             (ERROR_CODE_BASE + 0x08)    //绝对值编码器故障码起始地址
#define ERROR_CODE_ECoder11M_BASE_END               (ERROR_CODE_BASE + 0x0C)    //绝对值编码器故障码结束地址
#define ERROR_CODE_MEASURE_SENSOR                   (ERROR_CODE_BASE + 0x0D)    //电机测量传感器故障
#define ERROR_CODE_MOTOR_START_ERROR                (ERROR_CODE_BASE + 0x0E)    //电机启动故障
#define ERROR_CODE_MOTOR_STALL                      (ERROR_CODE_BASE + 0x0F)    //电机堵转
#define ERROR_CODE_LIMIT_ABNORMAL                   (ERROR_CODE_BASE + 0x10)    //电机误触发限位
#define ERROR_CODE_MOTOR_ERROR_CHECK                (ERROR_CODE_BASE + 0x11)    //电机IO口检测到故障
#define ERROR_CODE_MOTOR_BRAKE_ERROR                (ERROR_CODE_BASE + 0x12)    //刹车故障
#define ERROR_CODE_MOTOR_BRAKE1_ERROR               (ERROR_CODE_BASE + 0x13)    //刹车故障1
//#define ERROR_CODE_MOTOR_STALL1                     (ERROR_CODE_BASE + 0x14)    //检测到设备减速，碰撞、摩地或惯导数据异常
#define ERROR_CODE_MOTOR_STALL2                     (ERROR_CODE_BASE + 0x15)    //电机堵转2执行错误
#define ERROR_CODE_MOTOR_SKID                       (ERROR_CODE_BASE + 0x16)    //电机打滑或惯导数据故障
#define ERROR_CODE_MOTOR_TURN_EXE_ERROR             (ERROR_CODE_BASE + 0x17)    //转向电机
#define ERROR_CODE_485_LINE_OFF                     (ERROR_CODE_BASE + 0x18)    //485通信离线
#define ERROR_CODE_REMOTE_LINE_OFF                  (ERROR_CODE_BASE + 0x19)    //遥控通信超时
#define ERROR_CODE_PC_LINE_OFF                      (ERROR_CODE_BASE + 0x20)    //主控卡通信超时
#define ERROR_CODE_ZX_EXE_ERROR                     (ERROR_CODE_BASE + 0x21)    //转向无法执行
#define ERROR_CODE_WRONG_PARAS                      (ERROR_CODE_BASE + 0x22)    //错误的参数配置
#define ERROR_CODE_ROTATION_SYNC_ERROR              (ERROR_CODE_BASE + 0x23)    //电机转速同步故障
#define ERROR_CODE_CONTINUOUS_SKID                  (ERROR_CODE_BASE + 0x24)    //电机长时间打滑
#define ERROR_CODE_MOTOR_ABNORMAL_OPERATION         (ERROR_CODE_BASE + 0x25)    //电机异常去使能

//#define ERROR_CODE_NAV_DATA_INVALID                 (ERROR_CODE_BASE + 0x30)    //导航数据无效
//#define ERROR_CODE_NAV_LANE_DEPARTURE1              (ERROR_CODE_BASE + 0x31)    //车道偏离报警1，距上次距离偏移超过1m
//#define ERROR_CODE_NAV_LANE_DEPARTURE2              (ERROR_CODE_BASE + 0x32)    //车道偏离报警2，距上次距离偏移超过3s
//#define ERROR_CODE_NAV_LANE_DEPARTURE3              (ERROR_CODE_BASE + 0x33)    //车道偏离报警3，稳定后，发散偏离出限制范围

//遥控或pc链接状态定义
#define LINK_NONE           0
#define LINK_REMOTE_DATA    0x01        //接收到遥控数据
#define LINK_PC_DATA        0x02        //接收到PC数据
#define LINK_REMOTE         0x04        //遥控控制 
#define LINK_HAND_OVER      0x08        //遥控器移交控制
#define LINK_PC             0x10        //PC控制
#define LINK_AUTO_NAV       0x20        //自动导航控制
#define LINK_REMOT_OFF      0x40        //遥控关闭或者失联状态
#define LINK_PC_LOST        0x80        //PC数据丢失
#define LINK_QUICK_STOP     0x100       //紧急停止
#define LINK_POWER_ON       0x200       //上电状态
#define LINK_OUT_OF_CTRL    0x400       //失控状态
#define LINK_ERROR_STOP     0x1000      //故障停机
#define LINK_REMOTE_STOP    0x2000      //遥控停机
#define LINK_REV_STOP_CMD   0x4000      //接收到速度为0的停止命令，故障后会清除此标志位

#define IS_EMERGENCY_STOP   (sys_para->CAR_RTinf.Link & (LINK_QUICK_STOP | LINK_ERROR_STOP | LINK_REMOTE_STOP | LINK_OUT_OF_CTRL))
#define IS_EMERGENCY_STOP_NOT_ERR   (sys_para->CAR_RTinf.Link & (LINK_QUICK_STOP | LINK_REMOTE_STOP | LINK_OUT_OF_CTRL))

#define CMD_VALUE_OFFSET    1000        //命令偏移值
#define CMD_VALUE_MAX       700         //命令最大值
#define CMD_VEL_VALUE_MAX   3400        //速度命令最大值0.01m/s
#define CMD_ACC_VALUE_MAX   900         //加速度命令最大值0.01m/s^2
#define FOUR_TURN_FRONT_ONLY_VEL 300    //仅前轮转向切换速度阈值0.01m/s

//Modbus寄存器地址范围宏定义 
#define	REG_HOLD_START	1
#define REG_HOLD_NREGS	500

//Modbus寄存器地址定义 
extern unsigned short   usRegHoldBuf[REG_HOLD_NREGS];

#define SBUS_RX_pack_LEN    sizeof(SBUS_RX_pack)
#pragma pack (1)  //1字节对齐
typedef struct
{	
	uint16_t pack_Mark;//数据包标志
	uint16_t Link; //连接标志
    uint16_t CH[50];//数据通道
} SBUS_RX_pack;

#define PC_Remote_pack_LEN    sizeof(PC_Remote_pack)
typedef struct
{	
    uint16_t Link; //连接标志
    uint16_t YM;   //油门/刹车 -写
    uint16_t ZX;   //转向  -写
    uint16_t max_vel;  //最大速度
    uint16_t VEL;  //惯导速度值
    uint16_t SetAcc; //设定加速度
    uint16_t decRemainTime; //距离减速点剩余时间，单位ms，最大限制65000ms，大于65s时则赋值等于65s
    int16_t setDec; //场景减速段设定平均减速度，单位0.01m/s2
    uint16_t reserved[8]; //预留空间
} PC_Remote_pack;

#define CAR_RTinf_pack_LEN    sizeof(CAR_RTinf_pack)
typedef struct
{	
	uint16_t Link; //连接标志 遥控器或者PC控制标志
    int16_t YM;   //油门/刹车
	int16_t ZX;   //转向
	int16_t vel;  //惯导速度
	int16_t SetAcc;  //设定加速度
	int16_t max_vel; //最大速度
} CAR_RTinf_pack;


#define  SYS_PARA_LEN    sizeof(SYS_PARA)  //注意该结构体长度不能大于usRegHoldBuf数组的长度，否则可能会寻址越界
typedef struct
{
    uint16_t            reserved[104];  //预留
	SBUS_RX_pack        SBUS_rx;        //SBUS收到数据	地址偏移量 104(Modbus寄存器地址，单位uint16) 该结构体长度 52*2 字节
	PC_Remote_pack      PC_Remote;      //PC遥控数据	地址偏移量 156(Modbus寄存器地址，单位uint16) 该结构体长度 16*2 字节
	CAR_RTinf_pack      CAR_RTinf;      //车底运行信息	地址偏移量 172(Modbus寄存器地址，单位uint16) 该结构体长度 6*2 字节
	
} SYS_PARA;

//(2)状态信息上传，主动上发
#pragma pack (1)  //1字节对齐
typedef struct
{
    //id-0x991
    unsigned short  batteryVoltage; //电池电压，单位0.1V
    short           batteryCurrent; //电池电流，单位0.1A
    unsigned char   batterySoc;     //电池电量百分比值，0到100
    unsigned char   faultModule; 	//故障模块，0-整机故障，1-左1电机，2-右1电机，3-转向，4-刹车，5-左2电机，6-右2电机，10-电池
    unsigned short  errorCode;      //故障码	
    //id-0x992
    unsigned char   batteryTmp[8];  //电池1到8温度，单位偏移-40℃，实际2个
    //id-0x993
    unsigned char   motorTmp[8];    //电机1到8温度，单位偏移-40℃，实际6个
    //id-0x994
    unsigned char   driverTmp[8];   //驱动器1到8温度，单位偏移-40℃，实际6个
    //id-0x995
    unsigned char   moduleTmp[6];   //特定位置温度探头1到8温度，单位偏移-40℃，实际预留4个,0-工控卡,1-交换机，2-电源板
    //id-0x996
    unsigned short  linkState;      //遥控状态信息，pc控制 or 遥控控制
    unsigned char   ctrlState;      //控制状态信息，电机使能状态，遥控高中低速挡位
    short           turnTargetPos;      //转向目标位置
    short           turnCurPos;         //转向当前位置
}RESPOND_STATE;
#pragma pack ()  //1字节对齐

/*构造发送结构体*/
#pragma pack (1)  //1字节对齐
typedef struct
{
    unsigned char header1;  //头部1
    unsigned char header2;  //头部2
    unsigned short functional_parameter_len;    //功能码长度
    unsigned short functional_code;             //功能码
    RESPOND_STATE RESPOND_STATE_DATA;           //发送的数据
    unsigned short crc16;                       //crc校验
}SEND_RESPOND_STATE;
#pragma pack ()

//老化测试结构体
//老化步骤
#define OLD_TEST_PAUSE            0x01   //老化暂停
#define OLD_TEST_END              0x02   //老化结束
#define OLD_TEST_START            0x04   //老化测试开始
#define OLD_TEST_NEED_UPDATE_CMD  0x08   //需要更新命令
#define OLD_TEST_POWER_NEW_CMD    0x10   //收到新的动力老化命令
#define OLD_TEST_TURN_NEW_CMD     0x20   //收到新的转向老化命令
#define OLD_TEST_BRAKE_NEW_CMD    0x40   //收到新的刹车老化命令

//老化模块标志
#define NONE_AGING                0x00   //无模块需老化
#define POWER_AGING               0x01   //动力老化
#define TURN_AGING                0x02   //转向老化
#define BRAKE_AGING               0x04   //刹车老化

typedef struct
{
    uint16_t oldTestStep;       //老化测试运行步骤
    uint16_t moduleAgingFlg;    //模块老化标志
    uint16_t forwardTime;       //前进时间(单位:s)
    uint16_t backTime;          //后退时间(单位:s)
    int16_t  forwardVel;        //前进速度(单位:cm/s)
    int16_t  backVel;           //后退速度(单位:cm/s)
    uint16_t powerGapTime;      //前进与后退转换间隔时间(单位:s)
    int16_t  turnVel;           //转向速度(单位:rpm)
    uint16_t turnRange;         //转向范围(0-700)
    uint16_t turnGapTime;       //左转与右转转换间隔时间(单位:s)
    int16_t  brakeVel;          //刹车速度(单位:rpm)
    uint16_t brakeInitPos;      //松刹车位置(0-700)
    uint16_t brakeLimitPos;     //紧刹车位置(0-700)
    uint16_t brakeGapTime;      //松刹车与紧刹车转换间隔时间(单位:s)  
}OLD_TEST;

//老化测试运行步骤
typedef enum
{
    EXIT=0,       //退出老化
    START,        //开始老化
    STEP1,        //步骤1
    STEP2,        //步骤2
    STEP3         //步骤3
}AGING_STEP;
typedef enum
{
    POWER=0,      //动力
    TURN,         //转向
    BRAKE,        //刹车
    MODULE_NUM   //老化模块数量
}AGING_MODULE;



#define GPIO_PORT_TOTOAL_NUM    9
#define GPIO_PIN_MAX_NUM        16

extern SYS_PARA       *sys_para; 
extern GPIO_TypeDef* gGpioPortArray[GPIO_PORT_TOTOAL_NUM];

//通用公式定义
#define DEF_PI  3.14159265358979f
#define ABS_VALUE(x)        (((x) >= 0) ? (x) : (-(x)))     //取绝对值
#define Limit(x, low, up)   ((x) <= (low) ? (low) : ((x) >= (up) ? (up) : (x))) //取限位后的值
#define IS_SAME_SYMBLE(x, y) ((((x) >= 0) && ((y) >= 0)) || (((x) <= 0) && ((y) <= 0))) //判断两个变量符号是否相同
#define IS_ACC_GTE_0(x, y)   ((((x) > 0) && ((y) >= 0)) || (((x) < 0) && ((y) <= 0))) //判断加速或匀速
#define MAX(x, y)           (x > y ? x : y)                 //取较大值
#define CountsToAngle(motorNum, lCounts) fmod((float)(lCounts) * 360.0f / gStMotorData[motorNum].ratio / gStMotorData[motorNum].counts, 360.0f)
#define AngleToCounts(motorNum, lAngle) (lAngle * gStMotorData[motorNum].ratio / 360.0f * gStMotorData[motorNum].counts)
#define GPIO_PORT(x)    ((x < GPIO_PORT_TOTOAL_NUM) ? gGpioPortArray[x] : gGpioPortArray[0])
#define GPIO_PIN(x)     ((uint16_t)0x01 << x)
#define CountsToRemteValue(motorNum, cur) (((gStMotorData[motorNum].flag & MOTOR_DIR_INVERSE_FLAG) ? (-1) : 1) * ((cur >= gStMotorRunState[motorNum].PosInit) \
    ? ((cur - gStMotorRunState[motorNum].PosInit) * 700 / (gStMotorRunState[motorNum].limitPos2 - gStMotorRunState[motorNum].PosInit)) \
    : ((cur - gStMotorRunState[motorNum].PosInit) * 700 / (gStMotorRunState[motorNum].PosInit - gStMotorRunState[motorNum].limitPos1))) * 10 / 7) 
#define NAV_RATIO_SCALE         1000        //惯导速比换算比例
#define VelRpmToCm_s(velRpm)    ((velRpm) * NAV_RATIO_SCALE / gStUfoData.ratioVel + (((ABS_VALUE(velRpm) * NAV_RATIO_SCALE) % gStUfoData.ratioVel >= (gStUfoData.ratioVel >> 1)) ? ((velRpm > 0) ? 1 : (-1)) : 0))   //速度单位换算，rpm转cm/s
#define VelCm_sToRpm(velCm_s)   ((int32_t)(velCm_s) * (int32_t)gStUfoData.ratioVel / NAV_RATIO_SCALE + (((ABS_VALUE((int32_t)(velCm_s)) * (int32_t)gStUfoData.ratioVel) % NAV_RATIO_SCALE >= (NAV_RATIO_SCALE >> 1)) ? ((velCm_s > 0) ? 1 : (-1)) : 0)) //速度单位换算，cm/s转rpm
#define CalDifDriverCurrent(tarM, curM) ((gStMotorData[tarM].limitCurrent == gStMotorData[curM].limitCurrent) ? gStMotorRunState[curM].setCurrent : (((gStMotorRunState[curM].setCurrent << 11)  / ((gStMotorData[curM].limitCurrent == 0) ? 1 : gStMotorData[curM].limitCurrent) * gStMotorData[tarM].limitCurrent) >> 11)) //用当前电机电流推算目标电机设定电流，根据最大电流比值成比例放大缩小，为防止电流乘积超出int32位最大值，故先左移11位放大，然后先除后乘，最后算完右移11位切换回来
#define WheelRpmToCm_s(Rpm)     ((Rpm) * NAV_RATIO_SCALE / gStUfoData.ratioVel)    //速度单位换算，rpm转cm/s
#define IsBrakeLock ((gStUfoData.flag & UFO_ENABLE_LOCK) && (gStUfoData.lockPin < GPIO_PIN_MAX_NUM) && (LOCK_OFF == gLockFlag)) //抱闸关闭标志
#define CalRatioVelRpm(motorNum)  (IS_UFOONE_FLAG_SET(UFOONE_USE_MOTOR_RATIO) ? (gStMotorRevData[motorNum].speed * gStMotorData[motorNum].ratio) : gStMotorRevData[motorNum].speed) //计算减速比后的电机转速
#define IsDiffOverPercent(val, standVal, percent)  ((ABS_VALUE((val - standVal) * 100) > ABS_VALUE(standVal * percent)) && (standVal != 0)) //差值对比是否超过标准值的百分比

#define EVENT_READ_LOG          (0x01 << 0) //设置事件掩码的位 0
#define EVENT_WRITE_LOG         (0x01 << 1) //设置事件掩码的位 1
#define EVENT_PRINTF            (0x01 << 2) //设置事件掩码的位 2

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


//调试数据记录类型定义
#define DEBUG_DATA_TYPE_1       ((gFlashData.enableDebugDataSend & 0x81) == 0x01)   //一般数据
#define DEBUG_DATA_TYPE_2       ((gFlashData.enableDebugDataSend & 0x82) == 0x02)   //更多数据
#define DEBUG_DATA_TYPE_3       ((gFlashData.enableDebugDataSend & 0x84) == 0x04)   //电机数据
#define DEBUG_DATA_TYPE_4       ((gFlashData.enableDebugDataSend & 0x88) == 0x08)   //PID调节
#define DEBUG_DATA_TYPE_5       ((gFlashData.enableDebugDataSend & 0x90) == 0x10)   //显示详细信息
#define DEBUG_DATA_TYPE_6       ((gFlashData.enableDebugDataSend & 0xa0) == 0x20)   
#define DEBUG_DATA_TYPE_7       ((gFlashData.enableDebugDataSend & 0xc0) == 0x40)   //错误详细数据
#define DEBUG_DATA_TYPE_81      (gFlashData.enableDebugDataSend == 0x81)            //主控卡can命令129
#define DEBUG_DATA_TYPE_82      (gFlashData.enableDebugDataSend == 0x82)            //遥控数据130
#define DEBUG_DATA_TYPE_83      (gFlashData.enableDebugDataSend == 0x83)            //驱动器485通信数据131
#define DEBUG_DATA_TYPE_84      (gFlashData.enableDebugDataSend == 0x84)            //电池数据132
#define DEBUG_DATA_TYPE_85      (gFlashData.enableDebugDataSend == 0x85)            //can收发数据133
#define DEBUG_DATA_TYPE_86      (gFlashData.enableDebugDataSend == 0x86)            //电流设置数据134
#define DEBUG_DATA_TYPE_87      (gFlashData.enableDebugDataSend == 0x87)            //主控卡can命令及转向数据135
#define DEBUG_DATA_TYPE_88      (gFlashData.enableDebugDataSend == 0x88)            //遥控原始数据136
#define DEBUG_DATA_TYPE_89      (gFlashData.enableDebugDataSend == 0x89)            //遥控解析数据137
#define DEBUG_DATA_TYPE_8A      (gFlashData.enableDebugDataSend == 0x8a)            //网络接收异常打印138
#define DEBUG_DATA_TYPE_8B      (gFlashData.enableDebugDataSend == 0x8b)            //网络接收详细打印139
#define DEBUG_DATA_TYPE_8C      (gFlashData.enableDebugDataSend == 0x8C)            //can1数据及速度数据140
#define DEBUG_DATA_TYPE_8D      (gFlashData.enableDebugDataSend == 0x8D)            //网络发送数据打印141
#define DEBUG_DATA_TYPE_8E      (gFlashData.enableDebugDataSend == 0x8E)            //断电确认相关数据142
#define DEBUG_DATA_TYPE_8F      (gFlashData.enableDebugDataSend == 0x8F)            //转向及速度数据143
#define DEBUG_DATA_TYPE_90      (gFlashData.enableDebugDataSend == 0x90)            //电机与惯导速度差值数据144
#define DEBUG_DATA_TYPE_91      (gFlashData.enableDebugDataSend == 0x91)            //定时器中断调试145
#define DEBUG_DATA_TYPE_92      (gFlashData.enableDebugDataSend == 0x92)            //pid运行时间测试146
#define DEBUG_DATA_TYPE_93      (gFlashData.enableDebugDataSend == 0x93)            //fishComDebug数据147
#define DEBUG_DATA_TYPE_94      (gFlashData.enableDebugDataSend == 0x94)            //can3数据148
#define DEBUG_DATA_TYPE_95      (gFlashData.enableDebugDataSend == 0x95)            //udp调试数据串口发送149
#define DEBUG_DATA_TYPE_96      (gFlashData.enableDebugDataSend == 0x96)            //电机过载判断数据打印150
#define DEBUG_DATA_TYPE_97      (gFlashData.enableDebugDataSend == 0x97)            //flash读写打印151
#define DEBUG_DATA_TYPE_98      (gFlashData.enableDebugDataSend == 0x98)            //电机平均速度及换算到车载位置值打印152

extern ST_UFO_DATA  gStUfoData;
extern uint32_t gUartNumWifiPrint;
extern int udp_print_connected_flag;
extern RESPOND_STATE gRespondState;                    //响应状态信息
extern int32_t gRemoteStopDcc;
extern OLD_TEST gOldTest;                               //老化测试相关变量
extern uint8_t gU8DownloadType;                         //程序下载类型
extern uint8_t wdgDisableFlag;                          //禁止看门狗喂狗标志
extern uint8_t gPrintfTestDataShowFlag;
extern rt_bool_t gMotorTestDataUploadFlag;              //测试数据上传标志
extern uint32_t gMotorTestDataUploadStartTime;          //开始上传时刻
extern int32_t gAutoRatioVel;                           //动态速比
extern int32_t gSteeringAngleVelDiff;                   //差速转向时转角对应的速度差值cm/s
extern rt_bool_t gNoLoadFlag;                           //无负载标志
extern BackDeffMode gBackWheelDeffDriverMode;           //后轮差速驱动模式标志
extern int32_t gMotorAvaVelFilter;                      //电机滤波和速度
extern int32_t gMotorAvaPos;                            //电机平均位置

#endif

