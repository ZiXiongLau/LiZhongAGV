#ifndef _NAV_H
#define _NAV_H

#include <preDef.h>

//导航控制命令
enum
{
    NAV_CMD_STOP,       //无命令
    NAV_CMD_START,      //启动
    NAV_CMD_RETURN,     //回退
    NAV_CMD_NUM         //命令个数
};

#define IS_NAV_STATE_OK(status, posType)    ((3 == status) && (56 == posType))
#define IS_NAV_STATE_OK1(status, posType) (((3 == status) || (7 == status)) && ((53 == posType) || (54 == posType) || (55 == posType) || (56 == posType)))

#define NAV_Ea      6378137     //   赤道半径m
#define NAV_Eb      6356725     //   极半径m

#define NAV_PI      3.14159265358979

/**
 * Nav Data
 */
typedef struct
{
    double              Lat;                        //纬度（°）
    double              Lon;                        //经度（°）
}ST_NAV_POS;

/**
 * Nav Data
 */
typedef struct
{
    uint8_t             validFlag;                  //数据有效标志
    uint32_t            lastRevTime;                //上次接收到数据的时间
    uint32_t            INS_Status;                 //INS 解算状态，表 4-8 惯性导航状态说明
    uint32_t            Pos_Type;                   //位置信息类型，见表 4-2 定位状态描述说明
    ST_NAV_POS          pos;                        //位置
    double              North_Veloci;               //北向速度（m/s）
    double              East_Velocit;               //东向速度（m/s）
    double              Up_Velocity;                //天向速度（m/s）
    double              Azimuth;                    //航向角（取值范围 0°~360°）
}ST_NAV_DATA;

/**
 * Nav Control
 */
typedef struct
{
    uint8_t             controlCmd;                 //控制命令
    uint8_t             oldTestFlag;                //老化测试
    uint8_t             oldExeCmd;
    uint8_t             curExeCmd;                  //当前执行的命令
    uint16_t            targetIndex;                //目标点索引
    uint16_t            preIndex1;                  //预瞄点1
    uint16_t            preIndex2;                  //预瞄点2
    uint8_t             backFlag;                   //后退标志
    uint8_t             lastValidFlag;              //上次值有效标志
    float               targetDistance;             //目标距离
    float               targetAngle;                //目标角度
    float               preAngle;                   //预瞄角度
    float               lastHengxiangDis;           //上一次横向距离
    float               curVel;                     //当前速度
    float               curHengxiangVel;            //当前横向速度
    double              AzimuthCal;                 //航向修正角（取值范围 0°~360°）
    ST_NAV_POS          lastPos;                    //上一次位置
    ST_NAV_POS          recordPos;                  //位置记录
    int16_t             YM;                         //油门值
    int16_t             ZX;                         //转向值
    uint32_t            recordTime;                 //距离记录时间点
    float               recordDis;                  //距离记录
    float               recordAngle;                //方位角记录
    uint32_t            recordCnt;                  //记录次数
}ST_NAV_CONTROL;


void NavDataInit(void);
void NavDataBinRev(TickType_t curTime);
void NavPrintData(TickType_t curTime);
void NavControlCmd(uint8_t* cmdData, uint8_t size);
void NavProcess(TickType_t curTime);

extern ST_NAV_DATA gStNavData; //导航数据
extern ST_NAV_CONTROL gStNavControl;   //导航控制数据

#endif /* _NAV_H */

