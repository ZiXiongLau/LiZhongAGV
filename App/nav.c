/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "usart.h"
#include "math.h"
#include "motor_control.h"
#include <motor_driven.h>
#include <s_curve.h>

#define NAV_CONTROL_PERIOD_MS       50      //导航控制周期
#define NAV_BIG_RADIUS_M            1.0f    //大圆半径
#define NAV_SMALL_RADIUS_M          0.1f    //小圆半径
#define NAV_AUTO_CALIBRATION_TOTALDIS_M  4.0f//自动校准总距离
#define NAV_AUTO_CALIBRATION_DIS_M  1.0f    //自动校准截取距离
#define NAV_AUTO_CALIBRATION_ANGLE  0.2f    //自动校准角度精度

#define NAV_LANE_DEPARTURE_DIS      1.0f    //车道偏离报警距离

#define NAV_HENGXIANG_VEL_MAX       0.3f    //横向最大速度
#define NAV_HENGXIANG_ACC_MAX       0.1f    //横向最大加速度
#define NAV_HENGXIANG_ACCTIME       8.0f    //横向加加速时间s

#define NAV_ZONGXIANG_VEL_MAX       23.0f   //横向最大速度
#define NAV_ZONGXIANG_ACC_MAX       2.0f    //纵向最大加速度
#define NAV_ZONGXIANG_ACCTIME       1.0f    //纵向加加速时间s

#define NAV_BUFFER_LEN              256
#define NAV_DATA_INVALID_TIMEOUT_MS 200     //惯导多久未收到数据判为数据无效

#define IS_AUTO_CALIBRATION_Azimuth (3 == gStNavControl.oldTestFlag)    //自动校准航向角


static uint8_t gNavBuffer[NAV_BUFFER_LEN];
static uint16_t gNavIndex = 0;

ST_NAV_DATA gStNavData; //导航数据
ST_NAV_CONTROL gStNavControl;   //导航控制数据

PIDParas_incres     gZxPidIncParas;         //转向增量式pid参数
float   gHXVel = NAV_HENGXIANG_VEL_MAX;
float   gHXAcc = NAV_HENGXIANG_ACC_MAX;
float   gHXAccTime = NAV_HENGXIANG_ACCTIME;
float   gZXAcc = NAV_ZONGXIANG_ACC_MAX;
float   gZXAccTime = NAV_ZONGXIANG_ACCTIME; 
float   gZXCurVel = 0;
SCurveParas gStNavScurveParas;
SCurveParas gStNavZXScurveParas;

#define CRC32_POLYNOMIAL 0xEDB88320L

/*****************************************************************************
 功能描述  : 导航数据初始化
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月25日
*****************************************************************************/
void NavDataInit(void)
{
    gStNavData.validFlag = 0;
    gStNavControl.controlCmd = NAV_CMD_STOP;
    gStNavControl.curExeCmd = NAV_CMD_STOP;
    gStNavControl.targetIndex = 0;
    gStNavControl.preIndex1 = 0;
    gStNavControl.preIndex2 = 0;
    gStNavControl.YM = 0;
    gStNavControl.ZX = 0;
    InitPidParas_incres(&gZxPidIncParas, 50, 0.6, 0, 0);
    InitSCurveParas(&gStNavScurveParas, gHXVel, gHXAcc, gHXAcc / gHXAccTime, 1.0f);
    InitSCurveParas(&gStNavZXScurveParas, NAV_ZONGXIANG_VEL_MAX, gZXAcc, gZXAcc / gZXAccTime, 1.0f);
    ClearSCurveMems(&gStNavZXScurveParas);
}
/* --------------------------------------------------------------------------
Calculate a CRC value
value: Value
-------------------------------------------------------------------------- */
unsigned long CalcCRC32Value(int value)
{
    int i;
    unsigned long ulCRC;
    ulCRC = value;
    for ( i = 8 ; i > 0; --i ) {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}
/* --------------------------------------------------------------------------
Calculates the CRC-32 of a data block
ulCount: Number of bytes in the data block
ucBuff: Data block
-------------------------------------------------------------------------- */
unsigned long CalcBlockCRC32( unsigned long ulCount, unsigned char *ucBuff )
{
    unsigned long ulTmp1;
    unsigned long ulTmp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 ) {
        ulTmp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTmp2 = CalcCRC32Value( ((int) ulCRC ^ *ucBuff++ ) & 0xFF );
        ulCRC = ulTmp1 ^ ulTmp2;
    }
    return ulCRC;
}
/*****************************************************************************
 功能描述  : 导航数据对齐同步字节到第1个字节
 输入参数  : uint16_t findStartIndex  查找起始序号
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月24日
*****************************************************************************/
void NavDataAlignSync(uint16_t findStartIndex)
{
    uint16_t i;
    
    if(gNavIndex >= findStartIndex)
    {
        for(i = findStartIndex; i + 3 <= gNavIndex; i++)
        {
            if(0xaa == gNavBuffer[i])
            {
                if(0x44 == gNavBuffer[i + 1])
                {
                    if(0x12 == gNavBuffer[i + 2])
                    {
                        break;  //数据头校验成功
                    }
                }
            }
        }
        if((findStartIndex != i) || (0 != findStartIndex))  //删除前面无效字节
        {
            findStartIndex = i;                    
            for(; i < gNavIndex; i++)
            {
                gNavBuffer[i - findStartIndex] = gNavBuffer[i];
            }
            gNavIndex -= findStartIndex;
        }
    }
}
/*****************************************************************************
 功能描述  : 数据接收完后解析数据
 输入参数  : uint16_t datalen  数据包长度
             TickType_t curTime当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月24日
*****************************************************************************/
void NavDataAnalysis(uint16_t datalen, TickType_t curTime)
{
    uint16_t lu16Temp;
    unsigned long ulCRC;

    ulCRC = gNavBuffer[datalen - 4];
    ulCRC += gNavBuffer[datalen - 3] << 8;
    ulCRC += gNavBuffer[datalen - 2] << 16;
    ulCRC += gNavBuffer[datalen - 1] << 24;
    if(ulCRC != CalcBlockCRC32(datalen- 4, gNavBuffer))
    {
        NavDataAlignSync(3);
        if(DEBUG_DATA_TYPE_5)
        {
            rt_kprintf("Crc err.\r\n", datalen);
        }
    }
    else
    {
        lu16Temp = gNavBuffer[4] + (gNavBuffer[5] << 8);   //消息id
        if(1465 == lu16Temp)   //需要的消息id,INSPVAX*
        {
            if(datalen < gNavBuffer[3] + 126)   //消息长度错误
            {
                NavDataAlignSync(datalen);
                if(DEBUG_DATA_TYPE_5)
                {
                    rt_kprintf("Msg len err: %d.\r\n", datalen);
                }
            }
            else
            {
                gStNavData.validFlag = 1;
                gStNavData.lastRevTime = curTime;
                gStNavData.INS_Status = *((uint32_t*)(gNavBuffer + gNavBuffer[3]));
                gStNavData.Pos_Type = *((uint32_t*)(gNavBuffer + gNavBuffer[3] + 4));
                gStNavData.pos.Lat = *((double*)(gNavBuffer + gNavBuffer[3] + 8));
                gStNavData.pos.Lon = *((double*)(gNavBuffer + gNavBuffer[3] + 16));
                gStNavData.North_Veloci = *((double*)(gNavBuffer + gNavBuffer[3] + 36));
                gStNavData.East_Velocit = *((double*)(gNavBuffer + gNavBuffer[3] + 44));
                gStNavData.Azimuth = *((double*)(gNavBuffer + gNavBuffer[3] + 76));
                gStNavControl.AzimuthCal = gStNavData.Azimuth + gFlashData.calibrationAngle;   //航向修正角
                if(gStNavControl.AzimuthCal >= 360.0f) gStNavControl.AzimuthCal -= 360.0f;
                else if(gStNavControl.AzimuthCal < 0.0f) gStNavControl.AzimuthCal += 360.0f;
                NavDataAlignSync(datalen);
            }
        }
        else
        {
            NavDataAlignSync(datalen);
            if(DEBUG_DATA_TYPE_5)
            {
                rt_kprintf("Noneed id: %d.\r\n", lu16Temp);
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 导航数据二进制接收
 输入参数  : TickType_t curTime 当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月21日
*****************************************************************************/
void NavDataBinRev(TickType_t curTime)
{
    uint16_t lu16Temp, lIndex;
    
    lIndex = UsartDeviceRead(USART6_DEVICE, &gNavBuffer[gNavIndex], NAV_BUFFER_LEN - gNavIndex);

    if(lIndex > 0)  //接收到数据
    {
        if(gNavIndex < 3)  //判断数据头
        {
            gNavIndex += lIndex;
            NavDataAlignSync(0);
        }
        else
        {
            gNavIndex += lIndex;
            if(gNavIndex > 8)  //接收到数据头长度及消息长度
            {
                if(gNavBuffer[3] < 10)   //header长度错误
                {
                    NavDataAlignSync(3);
                    if(DEBUG_DATA_TYPE_5)
                    {
                        rt_kprintf("Nav head len err: %d.\r\n", gNavBuffer[3], gStNavData.INS_Status);
                    }
                }
                else
                {
                    lu16Temp = gNavBuffer[8] + (gNavBuffer[9] << 8) + gNavBuffer[3] + 4;    //总长度
                    if((lu16Temp > NAV_BUFFER_LEN) || (lu16Temp < 12))   //总长度错误
                    {
                        NavDataAlignSync(3);
                        if(DEBUG_DATA_TYPE_5)
                        {
                            rt_kprintf("Nav total len err: %d.\r\n", lu16Temp);
                        }
                    }
                    else if(gNavIndex >= lu16Temp)  //接收完数据
                    {
                        NavDataAnalysis(lu16Temp, curTime);
                    }
                }
            }
        }
    }

    //长时间为收到数据，至数据无效标志
    if(gStNavData.validFlag)
    {
        if(curTime - gStNavData.lastRevTime > NAV_DATA_INVALID_TIMEOUT_MS)
        {
            gStNavData.validFlag = 0;
        }
    }
}
/*****************************************************************************
 功能描述  : 打印惯导数据
 输入参数  : TickType_t curTime  当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月24日
*****************************************************************************/
void NavPrintData(TickType_t curTime)
{
    static TickType_t lastTime = 0;
    
    if(curTime - lastTime >= 1000)
    {
        lastTime = curTime;
        if(gStNavData.validFlag)
        {
            rt_kprintf("Nav s:%d,p:%d,lo:%.7lf,la:%.7lf,a:%.2lf.\r\n", gStNavData.INS_Status
                , gStNavData.Pos_Type, gStNavData.pos.Lon, gStNavData.pos.Lat, gStNavData.Azimuth);
        }
        else
        {
            rt_kprintf("Nav invalid!\r\n");
        }
    }
}
/*****************************************************************************
 功能描述  : 计算两点之间的距离
 输入参数  : ST_NAV_POS* pos1  点1
             ST_NAV_POS* pos2  点2
             float* outAngle   输出方位角
 输出参数  : float  距离m
 作    者  : 刘鹏
 日    期  : 2021年5月26日
*****************************************************************************/
float NavCalDistance(ST_NAV_POS* pos1, ST_NAV_POS* pos2, float* outAngle)
{
    float dy = fabs((pos1->Lat - pos2->Lat) * NAV_PI / 180.0 * NAV_Ea);  
    float dx = fabs((pos1->Lon- pos2->Lon) * NAV_PI / 180.0 * NAV_Ea);
    float xRatio = sqrt(fabs(cos(pos1->Lat * NAV_PI / 180.0) * cos(pos2->Lat * NAV_PI / 180.0)));
    dx *= xRatio;   //椭圆修正因子
    if(dx <= 0.001f) dx = 0.001f;
    if(dy <= 0.001f) dy = 0.001f;
    *outAngle = atan(dx / dy);
    float distance = fabs(dx / sin(*outAngle));
    *outAngle *= 180.0 / NAV_PI;
    if(pos2->Lon >= pos1->Lon)
    {
        if(pos2->Lat < pos1->Lat)
        {
            *outAngle = 180.0f - *outAngle;
        }
    }
    else
    {
        if(pos2->Lat >= pos1->Lat)
        {
            *outAngle = 360.0f - *outAngle;
        }
        else
        {
            *outAngle = 180.0f + *outAngle;
        }
    }
    return distance;
}
/*****************************************************************************
 功能描述  : 导航控制命令
 输入参数  : uint8_t* cmdData  命令数据
             uint8_t size      命令长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年3月18日
*****************************************************************************/
void NavControlCmd(uint8_t* cmdData, uint8_t size)
{
    uint32_t i, lu32Temp = 0;
    int16_t i16Temp = 0, i16Temp1 = 0;
    float dis, angle;
    
    switch(cmdData[0])
    {
        case 0x01:  //读取目标点信息
            for(i = 0; i < NAV_TARGET_POINT_MAX_NUM; i++)
            {
                if(gFlashData.targetValid[i])
                {
                    if(!lu32Temp)
                    {
                        i16Temp = i;
                        rt_kprintf("\r\nRead target info:\r\n");
                    }
                    else
                    {
                        i16Temp1 = i;
                    }
                    lu32Temp = 1;                    
                    rt_kprintf("Target%d: lon:%.7lf, lat:%.7lf, vel:%.3f!\r\n", i, gFlashData.targetPos[i].Lon, gFlashData.targetPos[i].Lat, gFlashData.targetVel[i]);
                }
            }
            if(!lu32Temp)
            {
                rt_kprintf("No target info!\r\n");
            }
            else if(i16Temp != i16Temp1)
            {
                dis = NavCalDistance(&(gFlashData.targetPos[i16Temp]), 
                    &(gFlashData.targetPos[i16Temp1]), &angle);
                rt_kprintf("T%d-T%d dis:%.3f,ang:%.2f.\r\n", i16Temp, i16Temp1, dis, angle);
            }
            break;
        case 0x02:  //导航指令
            if(100 == cmdData[1])
            {
                gStNavControl.oldTestFlag = 1;
                gStNavControl.oldExeCmd = NAV_CMD_STOP;
            }  
            else if(101 == cmdData[1])  //打印正向数据不动作
            {
                gStNavControl.oldTestFlag = 2;
                gStNavControl.controlCmd = NAV_CMD_START;
                gStNavControl.preIndex1 = gStNavControl.preIndex2;
            }
            else if(102 == cmdData[1])  //打印反向数据不动作
            {
                gStNavControl.oldTestFlag = 2;
                gStNavControl.controlCmd = NAV_CMD_RETURN;
                gStNavControl.preIndex1 = gStNavControl.preIndex2;
            }
            else if(103 == cmdData[1])  //自动校准惯导正前方与车身正前方
            {
                gStNavControl.oldTestFlag = 3;
                gStNavControl.controlCmd = NAV_CMD_START;
                gStNavControl.lastValidFlag = 0;
                gStNavControl.preIndex1 = gStNavControl.preIndex2;
            }
            else if(cmdData[1] < NAV_CMD_NUM)
            {
                gStNavControl.oldTestFlag = 0;
                gStNavControl.controlCmd = cmdData[1];
                rt_kprintf("Nav cmd: %d (0-stop,1-start,2-return)!\r\n", cmdData[1]);
            }
            else
            {
                rt_kprintf("Nav cmd invalid: %d!\r\n", cmdData[1]);
            }
            break;
        case 0x03:  //pid参数设置
            i16Temp = *(int16_t*)(&cmdData[2]);
            i16Temp1 = *(int16_t*)(&cmdData[4]);
            if(0 == cmdData[1])
            {
                gZxPidIncParas.kp = i16Temp;
                gZxPidIncParas.ki = i16Temp1;
                gZxPidIncParas.ki /= 100;
                rt_kprintf("ZX_PID kp: %.1f, ki: %.2f.\r\n", gZxPidIncParas.kp, gZxPidIncParas.ki);
            }
            break;
        case 0x04:
            if(0 == cmdData[1]) //横向参数设置
            {
                i16Temp = *(int16_t*)(&cmdData[2]);
                gHXVel = i16Temp / 100.0f;
                i16Temp = *(int16_t*)(&cmdData[4]);
                gHXAcc = i16Temp / 1000.0f;
                i16Temp = *(int16_t*)(&cmdData[6]);
                gHXAccTime = i16Temp / 10.0f;
                if(gHXAccTime < 0.1f) gHXAccTime = 0.1f;
                InitSCurveParas(&gStNavScurveParas, gHXVel, gHXAcc, gHXAcc / gHXAccTime, 1.0f);
                rt_kprintf("HX_PARA vel: %.2f, acc: %.3f, aacc: %.4f, accvel: %.3f, aadis: %.2f, aadis: %.2f.\r\n", gStNavScurveParas.maxVel,
                    gStNavScurveParas.accMax, gStNavScurveParas.aacc, gStNavScurveParas.aaccVel, gStNavScurveParas.aaccDis, gStNavScurveParas.accDis);
            }
            else if(1 == cmdData[1]) //纵向参数设置
            {
                i16Temp = *(int16_t*)(&cmdData[4]);
                gZXAcc = i16Temp / 1000.0f;
                i16Temp = *(int16_t*)(&cmdData[6]);
                gZXAccTime = i16Temp / 10.0f;
                if(gZXAccTime < 0.1f) gZXAccTime = 0.1f;
                InitSCurveParas(&gStNavZXScurveParas, (float)(*(int16_t*)(&cmdData[2])) / 100.0f, gZXAcc, gZXAcc / gZXAccTime, 1.0f);
                rt_kprintf("ZX_PARA vel: %.2f, acc: %.3f, aacc: %.4f, accvel: %.3f, aadis: %.2f, aadis: %.2f.\r\n", gStNavZXScurveParas.maxVel,
                    gStNavZXScurveParas.accMax, gStNavZXScurveParas.aacc, gStNavZXScurveParas.aaccVel, gStNavZXScurveParas.aaccDis, gStNavZXScurveParas.accDis);
            }
            break;
    }
}
/*****************************************************************************
 功能描述  : 获取目标点个数
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月27日
*****************************************************************************/
uint16_t GetTargetPosNum(void)
{
    uint32_t i, num = 0;

    for(i = 0; i < NAV_TARGET_POINT_MAX_NUM; i++)
    {
        if(gFlashData.targetValid[i])
        {
            num++;
        }
    }

    return num;
}
/*****************************************************************************
 功能描述  : 判断当前目标点是否是最后一个目标点
 输入参数  : void  
 输出参数  : uint32_t   1表示是
 作    者  : 刘鹏
 日    期  : 2021年5月28日
*****************************************************************************/
uint32_t NavIsLastTarget(void)
{
    uint32_t i;

    for(i = gStNavControl.targetIndex + 1; i < NAV_TARGET_POINT_MAX_NUM; i++)
    {
        if(gFlashData.targetValid[i])
        {
            return 0;
        }
    }

    return 1;
}
/*****************************************************************************
 功能描述  : 切换目标点
 输入参数  : uint8_t  切换成功返回0，不成功返回1表示完成
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月27日
*****************************************************************************/
void NavChangeTargetPos(void)
{
    int32_t i, preIndexFlag = 0;
    float targetAngle, preDistance;
    
    if(NAV_CMD_START == gStNavControl.controlCmd)
    {
        for(i = gStNavControl.targetIndex + 1; i < NAV_TARGET_POINT_MAX_NUM; i++)
        {
            if(gFlashData.targetValid[i])
            {
                gStNavControl.targetIndex = i;
                gStNavControl.preIndex2 = i;
                preIndexFlag = 1;
                break;
            }
        }
        if(0 == preIndexFlag)
        {
            gStNavControl.preIndex2 = gStNavControl.targetIndex;
            preIndexFlag = 1;
        }
        for(i = gStNavControl.targetIndex - 1; i >= 0; i--)
        {
            if(gFlashData.targetValid[i])
            {
                //预瞄距离太短，重新找点
                preDistance = NavCalDistance(&(gFlashData.targetPos[i]), 
                    &(gFlashData.targetPos[gStNavControl.preIndex2]), &(gStNavControl.preAngle));
                if(preDistance >= 0.01f)
                {
                    gStNavControl.preIndex1 = i;
                    preIndexFlag = 2;
                    break;
                }
            }
        }
    }
    else if(NAV_CMD_RETURN == gStNavControl.controlCmd)
    {
        for(i = 0; i < NAV_TARGET_POINT_MAX_NUM; i++)
        {
            if(gFlashData.targetValid[i])
            {
                if(preIndexFlag)
                {
                    //预瞄距离太短，重新找点
                    preDistance = NavCalDistance(&(gFlashData.targetPos[gStNavControl.preIndex1]), 
                        &(gFlashData.targetPos[i]), &(gStNavControl.preAngle));
                    if(preDistance >= 0.01f)
                    {
                        gStNavControl.preIndex2 = i;
                        preIndexFlag = 2;
                        break;
                    }
                }
                else
                {
                    gStNavControl.targetIndex = i;
                    gStNavControl.preIndex1 = i;
                    preIndexFlag = 1;
                }
            }
        }
    }
    if(2 != preIndexFlag)   //未找到预瞄线段
    {
        gStNavControl.controlCmd = NAV_CMD_STOP;
        rt_kprintf("Nav not find pre_line, change to stop!\r\n");
    }
    else if(NAV_CMD_RETURN == gStNavControl.controlCmd)    //判断是正向回去还是倒退回去
    {
        NavCalDistance(&(gStNavData.pos), &(gFlashData.targetPos[gStNavControl.targetIndex]), &targetAngle);
        targetAngle = fabs(targetAngle - gStNavControl.preAngle);
        if((targetAngle >= 90.0f) && (targetAngle <= 270.0f))
        {
            gStNavControl.backFlag = 1; //退回去
        }
        else
        {
            gStNavControl.backFlag = 0;
        }
    }
    else
    {
        gStNavControl.backFlag = 0; //前进时不能倒退
    }

    gStNavControl.lastValidFlag = 0;

    if(DEBUG_DATA_TYPE_3)
    {
        rt_kprintf("Nav t:%d,p1:%d,p2:%d,back:%d,pAng:%.2f.\r\n", 
            gStNavControl.targetIndex, gStNavControl.preIndex1, gStNavControl.preIndex2, 
            gStNavControl.backFlag, gStNavControl.preAngle);
    }
}
/*****************************************************************************
 功能描述  : 获取横向偏移距离，顺时针偏移为正，逆时针偏移为负，后退时相反
 输入参数  : float  横向偏移距离
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月28日
*****************************************************************************/
float NavGetHengxiangDis(void) 
{
    float lfDistance;
    
    lfDistance = gStNavControl.targetDistance \
    * sin((double)(gStNavControl.targetAngle - gStNavControl.preAngle) * NAV_PI / 180.0);

    if(gStNavControl.backFlag)
    {
        lfDistance = -lfDistance;
    }

    return lfDistance;
}
/*****************************************************************************
 功能描述  : 导航目标点判断
 输入参数  : TickType_t curTime  当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月27日
*****************************************************************************/
void NavTargetJudge(TickType_t curTime)
{    
    float lfTemp;
    uint8_t lu8Temp = 0;
    
    //第一次切点和算预瞄点及速度
    if(gStNavControl.preIndex1 == gStNavControl.preIndex2)
    {
        ClearPidMem_incres(&gZxPidIncParas);
        NavChangeTargetPos();
    }
    //目标距离及航向角计算
    gStNavControl.targetDistance = NavCalDistance(&(gStNavData.pos), 
        &(gFlashData.targetPos[gStNavControl.targetIndex]), &(gStNavControl.targetAngle));
    if(gStNavControl.backFlag)
    {
        gStNavControl.targetAngle += 180.0f;
        if(gStNavControl.targetAngle >= 360.0f)
        {
            gStNavControl.targetAngle -= 360.0f;
        }
    }
    //到目标点判断
    if(gStNavControl.targetDistance < NAV_SMALL_RADIUS_M)   //进入小圆半径
    {
        lu8Temp = 1;
        rt_kprintf("Nav reach point.\r\n");
    }
    else if(gStNavControl.targetDistance < NAV_BIG_RADIUS_M)  //进入大圆半径可切点
    {
        lu8Temp = 2;
    }
    else    //越过目标垂直线判断
    {
        lfTemp = fabs(gStNavControl.targetAngle - gStNavControl.preAngle);
        if((lfTemp >= 90.0f) && (lfTemp <= 270.0f))
        {
            lu8Temp = 1;
            rt_kprintf("Nav over point.\r\n");
        }
    }
    //切点判断
    if(lu8Temp && (NAV_CMD_RETURN != gStNavControl.controlCmd))
    {
        if(!NavIsLastTarget())
        {
            lu8Temp = 0;
            NavChangeTargetPos();   //切换至下一个目标点
        }
    }
    //到达当前目标点，选择切点或停止
    if(1 == lu8Temp)
    {
        gStNavControl.controlCmd = NAV_CMD_STOP;
    }
    //赋当前值给上一次
    if(!gStNavControl.lastValidFlag)
    {
        gStNavControl.lastValidFlag = 1;
        gStNavControl.lastHengxiangDis = NavGetHengxiangDis();
        gStNavControl.recordTime = curTime;
        gStNavControl.recordDis = gStNavControl.lastHengxiangDis;
        gStNavControl.lastPos.Lat = gStNavData.pos.Lat;
        gStNavControl.lastPos.Lon= gStNavData.pos.Lon;
        InitSCurveParas(&gStNavZXScurveParas, gFlashData.targetVel[gStNavControl.targetIndex], gZXAcc, gZXAcc / gZXAccTime, 1.0f);
        rt_kprintf("Hd: %.2f, ZX_PARA vel: %.2f, acc: %.3f, aacc: %.4f, accvel: %.3f, aadis: %.2f, aadis: %.2f.\r\n", gStNavControl.lastHengxiangDis, gStNavZXScurveParas.maxVel,
            gStNavZXScurveParas.accMax, gStNavZXScurveParas.aacc, gStNavZXScurveParas.aaccVel, gStNavZXScurveParas.aaccDis, gStNavZXScurveParas.accDis);
    }
}
/*****************************************************************************
 功能描述  : 导航自动校准目标点判断
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年6月19日
*****************************************************************************/
void NavAutoCalibrationTargetJudge(void)
{
    float lfTemp, lfDis;
    
    //首次进入
    if(!gStNavControl.lastValidFlag)
    {
        gStNavControl.lastValidFlag = 1;
        gStNavControl.backFlag = 0;
        ClearPidMem_incres(&gZxPidIncParas);
        gStNavControl.recordPos.Lat = gStNavData.pos.Lat;
        gStNavControl.recordPos.Lon = gStNavData.pos.Lon;
        gStNavControl.lastPos.Lat = gStNavData.pos.Lat;
        gStNavControl.lastPos.Lon = gStNavData.pos.Lon;
        gStNavControl.targetAngle = gStNavControl.AzimuthCal; //目标方位为当前方位
        gStNavControl.recordDis = 0;
        gStNavControl.recordCnt = 0;
        gStNavControl.recordAngle = 0;
        gStNavControl.preAngle = 0;
        gFlashData.calibrationAngle = 0;
    }
    //目标距离计算
    gStNavControl.targetDistance = NavCalDistance(&(gStNavData.pos), 
        &(gStNavControl.recordPos), &lfTemp);
    //校准完成判断
    if(gStNavControl.targetDistance > NAV_AUTO_CALIBRATION_TOTALDIS_M) //超出校准距离
    {
        gStNavControl.controlCmd = NAV_CMD_STOP;
        rt_kprintf("Nav over dis.\r\n");
    }
    else
    {
        if(fabs(gStNavControl.targetAngle - gStNavControl.AzimuthCal) > NAV_AUTO_CALIBRATION_ANGLE)   //超出精度则重新取数据
        {
            gStNavControl.lastPos.Lat = gStNavData.pos.Lat;
            gStNavControl.lastPos.Lon = gStNavData.pos.Lon;
            gStNavControl.recordCnt = 0;
            gStNavControl.recordAngle = 0;
            gStNavControl.preAngle = 0;
        }
        else    //计算截取段距离
        {
            lfDis = NavCalDistance(&(gStNavData.pos), 
                &(gStNavControl.lastPos), &lfTemp);
            if(lfDis > NAV_AUTO_CALIBRATION_DIS_M)   //已经获取到有效的校准数据
            {
                if(gStNavControl.recordCnt > 0)
                {
                    gStNavControl.controlCmd = NAV_CMD_STOP;
                    gStNavControl.preAngle /= gStNavControl.recordCnt;
                    gStNavControl.recordAngle /= gStNavControl.recordCnt;
                    gFlashData.calibrationAngle = gStNavControl.recordAngle - gStNavControl.preAngle;
                    rt_kprintf("Nav auto cal angle: %.2f, times: %d.\r\n", gFlashData.calibrationAngle, gStNavControl.recordCnt);
                    FlashWriteConfigurePara();
                    FlashReadConfigurePara();//写完后读取一遍
                }
            }
            else if(lfDis > 0.1f)   //记录数据
            {
                gStNavControl.preAngle += gStNavData.Azimuth;
                gStNavControl.recordAngle += lfTemp;
                gStNavControl.recordCnt++;
            }
        }
    }
}
/*****************************************************************************
 功能描述  : 导航控制
 输入参数  : TickType_t curTime        当前时间
             TickType_t controlPeriod  控制周期
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月27日
*****************************************************************************/
void NavControl(TickType_t curTime, TickType_t controlPeriod)
{
    float lfTemp, lHxTarvel, lfTarVel, lfHengxiangDis, lTargetAngle;
    int32_t liTemp = 0;

    //当前切线速度计算
    gStNavControl.curVel = sqrt(gStNavData.North_Veloci * gStNavData.North_Veloci + gStNavData.East_Velocit * gStNavData.East_Velocit);

    if(IS_AUTO_CALIBRATION_Azimuth)
    {
        lfTarVel = 2.0f;
        gZXCurVel = lfTarVel;
        lfHengxiangDis = 0.0f;
        lHxTarvel = 0.0f;
        lTargetAngle = gStNavControl.targetAngle;
    }
    else
    {
        //油门pi调节
        lfTarVel = gFlashData.targetVel[gStNavControl.targetIndex];
        //最后一个目标点，并且到达减速距离，则按照距离减速
        if(((NAV_CMD_RETURN == gStNavControl.controlCmd) || NavIsLastTarget())
            && (gStNavControl.targetDistance < gStNavZXScurveParas.aaccDis + gStNavZXScurveParas.accDis))
        {
            gZXCurVel = GetSCurveSpeedFromDis(&gStNavZXScurveParas, gStNavControl.targetDistance);
            gStNavZXScurveParas.timeRecord = curTime;
        }
        //否则按s曲线进行速度规划
        else
        {
            gZXCurVel = GetSCurveSpeed(&gStNavZXScurveParas, curTime, lfTarVel, gZXCurVel);
        }

        /*lfCurVel = NavCalDistance(&gStNavControl.lastPos, &gStNavData.pos, &lfTemp);
        lfCurVel = lfCurVel * 1000.0f / controlPeriod;
        gStNavControl.lastPos.Lat = gStNavData.pos.Lat;
        gStNavControl.lastPos.Lon= gStNavData.pos.Lon;
        if(lfCurVel <= 0.001f)
        {
            return;
        }*/
        //横向偏移距离计算
        lfHengxiangDis = NavGetHengxiangDis();
        //横向规划速度计算
        lHxTarvel = GetSCurveSpeedFromDis(&gStNavScurveParas, fabs(lfHengxiangDis));
        /*lfTemp = gHXVel * gHXVel / gHXAcc / 2;    //求减速距离，Vt^2-V0^2 = 2as
        if(lfTemp >= fabs(lfHengxiangDis))   //只够减速的距离
        {
            lHxTarvel = sqrt(2 * gHXAcc * fabs(lfHengxiangDis));
        }
        else
        {
            lHxTarvel = gHXVel;
        }*/
        //目标方位角计算
        lfTemp = lfTarVel;
        if(lfTemp < lHxTarvel * 1.414f) lfTemp = lHxTarvel * 1.414f;
        if(lfTemp < 0.1f) lfTemp = 0.1f;
        lTargetAngle = asin(lHxTarvel / lfTemp);
        lTargetAngle *= 180.0 / NAV_PI;
        if(lTargetAngle > 45.0f) lTargetAngle = 45.0f;
        if(gStNavControl.backFlag)
        {
            lTargetAngle = -lTargetAngle;
        }
        if(lfHengxiangDis >= 0.0f)
        {
            lTargetAngle = gStNavControl.preAngle + lTargetAngle;
        }
        else
        {
            lTargetAngle = gStNavControl.preAngle - lTargetAngle;
        }
        if(lTargetAngle >= 360.0f) lTargetAngle -= 360.0f;
        else if(lTargetAngle < 0.0f) lTargetAngle += 360.0f;
    }
    
    lfTemp = gZXCurVel * 700.0f / 15.0f;
    if(gStNavControl.backFlag)
    {
        gStNavControl.YM = lfTemp;
    }
    else
    {
        gStNavControl.YM = -lfTemp;
    }
    
    //转向pi调节
    lfTemp = lTargetAngle - gStNavControl.AzimuthCal;
    if(lfTemp >= 180.0f) lfTemp -= 360.0f;
    else if(lfTemp <= -180.0f) lfTemp += 360.0f;
    if(gStNavControl.backFlag)
    {
        lfTemp = -lfTemp;
    }    
    gStNavControl.ZX += CalcPid_incres(&gZxPidIncParas,
        lfTemp, 0);
    gStNavControl.ZX = Limit(gStNavControl.ZX, -CMD_VALUE_MAX, CMD_VALUE_MAX);
    if(DEBUG_DATA_TYPE_3)
    {
        if(!IS_NOT_CANOPEN_DRIVER(gStMotorData[M_TURN].driverType))
        {
            LockThread();
            if(0 == MotorReadPosition(gStMotorData[M_TURN].idx, &liTemp))
            {
                if(liTemp < gStMotorRunState[M_TURN].PosInit)
                {
                    liTemp = (liTemp - gStMotorRunState[M_TURN].PosInit) * 700 
                        / (gStMotorRunState[M_TURN].PosInit - gStMotorRunState[M_TURN].limitPos1);
                }
                else
                {
                    liTemp = (liTemp - gStMotorRunState[M_TURN].PosInit) * 700 
                        / (gStMotorRunState[M_TURN].limitPos2 - gStMotorRunState[M_TURN].PosInit);
                }
            }
            UnLockThread();
        }
        rt_kprintf("N,%d,\t%d,\t%.2f,\t%.3f,\t%.2f,\t%.2f,\t%.2f,\t%.2f,\t%.2f.\r\n", gStNavControl.ZX, -liTemp, gStNavControl.targetDistance, lfHengxiangDis, lHxTarvel, gStNavControl.curVel, lTargetAngle, gStNavControl.AzimuthCal, gZXCurVel);
    }
    gStNavControl.lastHengxiangDis = lfHengxiangDis;
}
/*****************************************************************************
 功能描述  : 导航异常判断
 输入参数  : TickType_t curTime  当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年6月19日
*****************************************************************************/
void NavAbnormalJudge(TickType_t curTime)
{
    float lfTemp;
    
    //车道偏离判断
    lfTemp = fabs(gStNavControl.lastHengxiangDis);
    if(lfTemp > NAV_LANE_DEPARTURE_DIS)
    {
        //从未偏离状态进入偏离状态，则判为车道偏离
        if(gStNavControl.recordDis < 0.001f)
        {
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_LANE_DEPARTURE3, ERROR_L_NORMAL);
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Nav lane departure3, change to stop!\r\n");
        }
        //未偏离，则更新记录值及记录时刻
        else if(lfTemp <= gStNavControl.recordDis)
        {
            gStNavControl.recordTime = curTime;
            gStNavControl.recordDis = lfTemp;
        }
        //距上一次距离偏移超过1m，则判为车道偏离
        else if(lfTemp - gStNavControl.recordDis > 1.0f)
        {
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_LANE_DEPARTURE1, ERROR_L_NORMAL);
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Nav lane departure1, change to stop!\r\n");
        }
        //距上一次距离偏移超过3s，则判为车道偏离
        else if(curTime - gStNavControl.recordTime >= 3000)
        {
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_LANE_DEPARTURE2, ERROR_L_NORMAL);
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Nav lane departure2, change to stop!\r\n");
        }
    }
    //进入横向1m范围内，并且向内偏移，则将距离记录为0
    else if(gStNavControl.recordDis > lfTemp + 0.1f)
    {
        gStNavControl.recordTime = curTime;
        gStNavControl.recordDis = 0;
    }
}
/*****************************************************************************
 功能描述  : 导航线程
 输入参数  : TickType_t curTime  当前时间
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年5月27日
*****************************************************************************/
void NavProcess(TickType_t curTime)
{
    static TickType_t lastTime = 0;
    TickType_t timePeriod;

    //非自动导航模式，直接返回
    if((!(sys_para->CAR_RTinf.Link & LINK_AUTO_NAV)) && (2 != gStNavControl.oldTestFlag))
    {
        return;
    }
    //老化测试标志
    else if(1 == gStNavControl.oldTestFlag)
    {
        if((NAV_CMD_STOP == gStNavControl.controlCmd)
            && (NAV_CMD_STOP == gStNavControl.curExeCmd))
        {
            if((NAV_CMD_STOP == gStNavControl.oldExeCmd)
                || (NAV_CMD_START == gStNavControl.oldExeCmd))
            {
                gStNavControl.controlCmd = NAV_CMD_RETURN;
                gStNavControl.oldExeCmd = NAV_CMD_RETURN;
                rt_kprintf("Old test: return.\r\n");
            }
            else
            {
                gStNavControl.controlCmd = NAV_CMD_START;
                gStNavControl.oldExeCmd = NAV_CMD_START;
                rt_kprintf("Old test: start.\r\n");
            }
        }
    }
    //判断惯导状态及命令正确性
    if(NAV_CMD_STOP != gStNavControl.controlCmd)
	{
		if(!gStNavData.validFlag)
		{
            if(NAV_CMD_STOP != gStNavControl.curExeCmd)
            {
                SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_DATA_INVALID, ERROR_L_NORMAL);
            }
            
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("No nav data, change to stop!\r\n");
		}
        else if(!IS_NAV_STATE_OK1(gStNavData.INS_Status, gStNavData.Pos_Type))
        {
            gStNavControl.controlCmd = NAV_CMD_STOP;
            rt_kprintf("Nav state not ok, change to stop!\r\n");
        }
        else if((NAV_CMD_STOP != gStNavControl.curExeCmd) 
            && (gStNavControl.controlCmd != gStNavControl.curExeCmd))
        {
            gStNavControl.controlCmd = gStNavControl.curExeCmd;
            rt_kprintf("Wront cmd, not excute!\r\n");
        }
        else if(GetTargetPosNum() < 2)
        {
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Not enough target pos, change to stop!\r\n");
        }
	}
    //停止
    if(NAV_CMD_STOP == gStNavControl.controlCmd)
    {
        gZXCurVel = 0;
        gStNavControl.YM = 0;
        //gStNavControl.ZX = 0;
        gStNavControl.preIndex1 = gStNavControl.preIndex2;
        if(NAV_CMD_STOP != gStNavControl.curExeCmd)
        {
            gStNavControl.curExeCmd = NAV_CMD_STOP;
        }
    }
    else
    {
        timePeriod = curTime - lastTime;
        if(timePeriod >= NAV_CONTROL_PERIOD_MS)
        {
            lastTime = curTime;
            //目标点判断
            if(IS_AUTO_CALIBRATION_Azimuth)
            {
                NavAutoCalibrationTargetJudge();    //目标距离判断及自校准完成判断
            }
            else
            {
                NavTargetJudge(curTime);    //目标点判断
            }
            //导航控制
            if((NAV_CMD_STOP != gStNavControl.controlCmd) && gStNavControl.lastValidFlag)
            {
                if(gStNavControl.curExeCmd != gStNavControl.controlCmd)
                {
                    gZXCurVel = 0;
                    ClearSCurveMems(&gStNavZXScurveParas);
                    gStNavControl.curExeCmd = gStNavControl.controlCmd;
                }
                NavControl(curTime, timePeriod);    //导航自动控制
                if(!IS_AUTO_CALIBRATION_Azimuth)
                {
                    NavAbnormalJudge(curTime);  //车道偏离判断
                }
            }
        }
    }
}

