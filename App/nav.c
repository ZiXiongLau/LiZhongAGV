/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "usart.h"
#include "math.h"
#include "motor_control.h"
#include <motor_driven.h>
#include <s_curve.h>

#define NAV_CONTROL_PERIOD_MS       50      //������������
#define NAV_BIG_RADIUS_M            1.0f    //��Բ�뾶
#define NAV_SMALL_RADIUS_M          0.1f    //СԲ�뾶
#define NAV_AUTO_CALIBRATION_TOTALDIS_M  4.0f//�Զ�У׼�ܾ���
#define NAV_AUTO_CALIBRATION_DIS_M  1.0f    //�Զ�У׼��ȡ����
#define NAV_AUTO_CALIBRATION_ANGLE  0.2f    //�Զ�У׼�ǶȾ���

#define NAV_LANE_DEPARTURE_DIS      1.0f    //����ƫ�뱨������

#define NAV_HENGXIANG_VEL_MAX       0.3f    //��������ٶ�
#define NAV_HENGXIANG_ACC_MAX       0.1f    //���������ٶ�
#define NAV_HENGXIANG_ACCTIME       8.0f    //����Ӽ���ʱ��s

#define NAV_ZONGXIANG_VEL_MAX       23.0f   //��������ٶ�
#define NAV_ZONGXIANG_ACC_MAX       2.0f    //���������ٶ�
#define NAV_ZONGXIANG_ACCTIME       1.0f    //����Ӽ���ʱ��s

#define NAV_BUFFER_LEN              256
#define NAV_DATA_INVALID_TIMEOUT_MS 200     //�ߵ����δ�յ�������Ϊ������Ч

#define IS_AUTO_CALIBRATION_Azimuth (3 == gStNavControl.oldTestFlag)    //�Զ�У׼�����


static uint8_t gNavBuffer[NAV_BUFFER_LEN];
static uint16_t gNavIndex = 0;

ST_NAV_DATA gStNavData; //��������
ST_NAV_CONTROL gStNavControl;   //������������

PIDParas_incres     gZxPidIncParas;         //ת������ʽpid����
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
 ��������  : �������ݳ�ʼ��
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��25��
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
 ��������  : �������ݶ���ͬ���ֽڵ���1���ֽ�
 �������  : uint16_t findStartIndex  ������ʼ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��24��
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
                        break;  //����ͷУ��ɹ�
                    }
                }
            }
        }
        if((findStartIndex != i) || (0 != findStartIndex))  //ɾ��ǰ����Ч�ֽ�
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
 ��������  : ���ݽ�������������
 �������  : uint16_t datalen  ���ݰ�����
             TickType_t curTime��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��24��
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
        lu16Temp = gNavBuffer[4] + (gNavBuffer[5] << 8);   //��Ϣid
        if(1465 == lu16Temp)   //��Ҫ����Ϣid,INSPVAX*
        {
            if(datalen < gNavBuffer[3] + 126)   //��Ϣ���ȴ���
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
                gStNavControl.AzimuthCal = gStNavData.Azimuth + gFlashData.calibrationAngle;   //����������
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
 ��������  : �������ݶ����ƽ���
 �������  : TickType_t curTime ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��21��
*****************************************************************************/
void NavDataBinRev(TickType_t curTime)
{
    uint16_t lu16Temp, lIndex;
    
    lIndex = UsartDeviceRead(USART6_DEVICE, &gNavBuffer[gNavIndex], NAV_BUFFER_LEN - gNavIndex);

    if(lIndex > 0)  //���յ�����
    {
        if(gNavIndex < 3)  //�ж�����ͷ
        {
            gNavIndex += lIndex;
            NavDataAlignSync(0);
        }
        else
        {
            gNavIndex += lIndex;
            if(gNavIndex > 8)  //���յ�����ͷ���ȼ���Ϣ����
            {
                if(gNavBuffer[3] < 10)   //header���ȴ���
                {
                    NavDataAlignSync(3);
                    if(DEBUG_DATA_TYPE_5)
                    {
                        rt_kprintf("Nav head len err: %d.\r\n", gNavBuffer[3], gStNavData.INS_Status);
                    }
                }
                else
                {
                    lu16Temp = gNavBuffer[8] + (gNavBuffer[9] << 8) + gNavBuffer[3] + 4;    //�ܳ���
                    if((lu16Temp > NAV_BUFFER_LEN) || (lu16Temp < 12))   //�ܳ��ȴ���
                    {
                        NavDataAlignSync(3);
                        if(DEBUG_DATA_TYPE_5)
                        {
                            rt_kprintf("Nav total len err: %d.\r\n", lu16Temp);
                        }
                    }
                    else if(gNavIndex >= lu16Temp)  //����������
                    {
                        NavDataAnalysis(lu16Temp, curTime);
                    }
                }
            }
        }
    }

    //��ʱ��Ϊ�յ����ݣ���������Ч��־
    if(gStNavData.validFlag)
    {
        if(curTime - gStNavData.lastRevTime > NAV_DATA_INVALID_TIMEOUT_MS)
        {
            gStNavData.validFlag = 0;
        }
    }
}
/*****************************************************************************
 ��������  : ��ӡ�ߵ�����
 �������  : TickType_t curTime  ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��24��
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
 ��������  : ��������֮��ľ���
 �������  : ST_NAV_POS* pos1  ��1
             ST_NAV_POS* pos2  ��2
             float* outAngle   �����λ��
 �������  : float  ����m
 ��    ��  : ����
 ��    ��  : 2021��5��26��
*****************************************************************************/
float NavCalDistance(ST_NAV_POS* pos1, ST_NAV_POS* pos2, float* outAngle)
{
    float dy = fabs((pos1->Lat - pos2->Lat) * NAV_PI / 180.0 * NAV_Ea);  
    float dx = fabs((pos1->Lon- pos2->Lon) * NAV_PI / 180.0 * NAV_Ea);
    float xRatio = sqrt(fabs(cos(pos1->Lat * NAV_PI / 180.0) * cos(pos2->Lat * NAV_PI / 180.0)));
    dx *= xRatio;   //��Բ��������
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
 ��������  : ������������
 �������  : uint8_t* cmdData  ��������
             uint8_t size      �����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��3��18��
*****************************************************************************/
void NavControlCmd(uint8_t* cmdData, uint8_t size)
{
    uint32_t i, lu32Temp = 0;
    int16_t i16Temp = 0, i16Temp1 = 0;
    float dis, angle;
    
    switch(cmdData[0])
    {
        case 0x01:  //��ȡĿ�����Ϣ
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
        case 0x02:  //����ָ��
            if(100 == cmdData[1])
            {
                gStNavControl.oldTestFlag = 1;
                gStNavControl.oldExeCmd = NAV_CMD_STOP;
            }  
            else if(101 == cmdData[1])  //��ӡ�������ݲ�����
            {
                gStNavControl.oldTestFlag = 2;
                gStNavControl.controlCmd = NAV_CMD_START;
                gStNavControl.preIndex1 = gStNavControl.preIndex2;
            }
            else if(102 == cmdData[1])  //��ӡ�������ݲ�����
            {
                gStNavControl.oldTestFlag = 2;
                gStNavControl.controlCmd = NAV_CMD_RETURN;
                gStNavControl.preIndex1 = gStNavControl.preIndex2;
            }
            else if(103 == cmdData[1])  //�Զ�У׼�ߵ���ǰ���복����ǰ��
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
        case 0x03:  //pid��������
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
            if(0 == cmdData[1]) //�����������
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
            else if(1 == cmdData[1]) //�����������
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
 ��������  : ��ȡĿ������
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��27��
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
 ��������  : �жϵ�ǰĿ����Ƿ������һ��Ŀ���
 �������  : void  
 �������  : uint32_t   1��ʾ��
 ��    ��  : ����
 ��    ��  : 2021��5��28��
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
 ��������  : �л�Ŀ���
 �������  : uint8_t  �л��ɹ�����0�����ɹ�����1��ʾ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��27��
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
                //Ԥ�����̫�̣������ҵ�
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
                    //Ԥ�����̫�̣������ҵ�
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
    if(2 != preIndexFlag)   //δ�ҵ�Ԥ���߶�
    {
        gStNavControl.controlCmd = NAV_CMD_STOP;
        rt_kprintf("Nav not find pre_line, change to stop!\r\n");
    }
    else if(NAV_CMD_RETURN == gStNavControl.controlCmd)    //�ж��������ȥ���ǵ��˻�ȥ
    {
        NavCalDistance(&(gStNavData.pos), &(gFlashData.targetPos[gStNavControl.targetIndex]), &targetAngle);
        targetAngle = fabs(targetAngle - gStNavControl.preAngle);
        if((targetAngle >= 90.0f) && (targetAngle <= 270.0f))
        {
            gStNavControl.backFlag = 1; //�˻�ȥ
        }
        else
        {
            gStNavControl.backFlag = 0;
        }
    }
    else
    {
        gStNavControl.backFlag = 0; //ǰ��ʱ���ܵ���
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
 ��������  : ��ȡ����ƫ�ƾ��룬˳ʱ��ƫ��Ϊ������ʱ��ƫ��Ϊ��������ʱ�෴
 �������  : float  ����ƫ�ƾ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��28��
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
 ��������  : ����Ŀ����ж�
 �������  : TickType_t curTime  ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��27��
*****************************************************************************/
void NavTargetJudge(TickType_t curTime)
{    
    float lfTemp;
    uint8_t lu8Temp = 0;
    
    //��һ���е����Ԥ��㼰�ٶ�
    if(gStNavControl.preIndex1 == gStNavControl.preIndex2)
    {
        ClearPidMem_incres(&gZxPidIncParas);
        NavChangeTargetPos();
    }
    //Ŀ����뼰����Ǽ���
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
    //��Ŀ����ж�
    if(gStNavControl.targetDistance < NAV_SMALL_RADIUS_M)   //����СԲ�뾶
    {
        lu8Temp = 1;
        rt_kprintf("Nav reach point.\r\n");
    }
    else if(gStNavControl.targetDistance < NAV_BIG_RADIUS_M)  //�����Բ�뾶���е�
    {
        lu8Temp = 2;
    }
    else    //Խ��Ŀ�괹ֱ���ж�
    {
        lfTemp = fabs(gStNavControl.targetAngle - gStNavControl.preAngle);
        if((lfTemp >= 90.0f) && (lfTemp <= 270.0f))
        {
            lu8Temp = 1;
            rt_kprintf("Nav over point.\r\n");
        }
    }
    //�е��ж�
    if(lu8Temp && (NAV_CMD_RETURN != gStNavControl.controlCmd))
    {
        if(!NavIsLastTarget())
        {
            lu8Temp = 0;
            NavChangeTargetPos();   //�л�����һ��Ŀ���
        }
    }
    //���ﵱǰĿ��㣬ѡ���е��ֹͣ
    if(1 == lu8Temp)
    {
        gStNavControl.controlCmd = NAV_CMD_STOP;
    }
    //����ǰֵ����һ��
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
 ��������  : �����Զ�У׼Ŀ����ж�
 �������  : void  
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��6��19��
*****************************************************************************/
void NavAutoCalibrationTargetJudge(void)
{
    float lfTemp, lfDis;
    
    //�״ν���
    if(!gStNavControl.lastValidFlag)
    {
        gStNavControl.lastValidFlag = 1;
        gStNavControl.backFlag = 0;
        ClearPidMem_incres(&gZxPidIncParas);
        gStNavControl.recordPos.Lat = gStNavData.pos.Lat;
        gStNavControl.recordPos.Lon = gStNavData.pos.Lon;
        gStNavControl.lastPos.Lat = gStNavData.pos.Lat;
        gStNavControl.lastPos.Lon = gStNavData.pos.Lon;
        gStNavControl.targetAngle = gStNavControl.AzimuthCal; //Ŀ�귽λΪ��ǰ��λ
        gStNavControl.recordDis = 0;
        gStNavControl.recordCnt = 0;
        gStNavControl.recordAngle = 0;
        gStNavControl.preAngle = 0;
        gFlashData.calibrationAngle = 0;
    }
    //Ŀ��������
    gStNavControl.targetDistance = NavCalDistance(&(gStNavData.pos), 
        &(gStNavControl.recordPos), &lfTemp);
    //У׼����ж�
    if(gStNavControl.targetDistance > NAV_AUTO_CALIBRATION_TOTALDIS_M) //����У׼����
    {
        gStNavControl.controlCmd = NAV_CMD_STOP;
        rt_kprintf("Nav over dis.\r\n");
    }
    else
    {
        if(fabs(gStNavControl.targetAngle - gStNavControl.AzimuthCal) > NAV_AUTO_CALIBRATION_ANGLE)   //��������������ȡ����
        {
            gStNavControl.lastPos.Lat = gStNavData.pos.Lat;
            gStNavControl.lastPos.Lon = gStNavData.pos.Lon;
            gStNavControl.recordCnt = 0;
            gStNavControl.recordAngle = 0;
            gStNavControl.preAngle = 0;
        }
        else    //�����ȡ�ξ���
        {
            lfDis = NavCalDistance(&(gStNavData.pos), 
                &(gStNavControl.lastPos), &lfTemp);
            if(lfDis > NAV_AUTO_CALIBRATION_DIS_M)   //�Ѿ���ȡ����Ч��У׼����
            {
                if(gStNavControl.recordCnt > 0)
                {
                    gStNavControl.controlCmd = NAV_CMD_STOP;
                    gStNavControl.preAngle /= gStNavControl.recordCnt;
                    gStNavControl.recordAngle /= gStNavControl.recordCnt;
                    gFlashData.calibrationAngle = gStNavControl.recordAngle - gStNavControl.preAngle;
                    rt_kprintf("Nav auto cal angle: %.2f, times: %d.\r\n", gFlashData.calibrationAngle, gStNavControl.recordCnt);
                    FlashWriteConfigurePara();
                    FlashReadConfigurePara();//д����ȡһ��
                }
            }
            else if(lfDis > 0.1f)   //��¼����
            {
                gStNavControl.preAngle += gStNavData.Azimuth;
                gStNavControl.recordAngle += lfTemp;
                gStNavControl.recordCnt++;
            }
        }
    }
}
/*****************************************************************************
 ��������  : ��������
 �������  : TickType_t curTime        ��ǰʱ��
             TickType_t controlPeriod  ��������
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��27��
*****************************************************************************/
void NavControl(TickType_t curTime, TickType_t controlPeriod)
{
    float lfTemp, lHxTarvel, lfTarVel, lfHengxiangDis, lTargetAngle;
    int32_t liTemp = 0;

    //��ǰ�����ٶȼ���
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
        //����pi����
        lfTarVel = gFlashData.targetVel[gStNavControl.targetIndex];
        //���һ��Ŀ��㣬���ҵ�����پ��룬���վ������
        if(((NAV_CMD_RETURN == gStNavControl.controlCmd) || NavIsLastTarget())
            && (gStNavControl.targetDistance < gStNavZXScurveParas.aaccDis + gStNavZXScurveParas.accDis))
        {
            gZXCurVel = GetSCurveSpeedFromDis(&gStNavZXScurveParas, gStNavControl.targetDistance);
            gStNavZXScurveParas.timeRecord = curTime;
        }
        //����s���߽����ٶȹ滮
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
        //����ƫ�ƾ������
        lfHengxiangDis = NavGetHengxiangDis();
        //����滮�ٶȼ���
        lHxTarvel = GetSCurveSpeedFromDis(&gStNavScurveParas, fabs(lfHengxiangDis));
        /*lfTemp = gHXVel * gHXVel / gHXAcc / 2;    //����پ��룬Vt^2-V0^2 = 2as
        if(lfTemp >= fabs(lfHengxiangDis))   //ֻ�����ٵľ���
        {
            lHxTarvel = sqrt(2 * gHXAcc * fabs(lfHengxiangDis));
        }
        else
        {
            lHxTarvel = gHXVel;
        }*/
        //Ŀ�귽λ�Ǽ���
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
    
    //ת��pi����
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
 ��������  : �����쳣�ж�
 �������  : TickType_t curTime  ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��6��19��
*****************************************************************************/
void NavAbnormalJudge(TickType_t curTime)
{
    float lfTemp;
    
    //����ƫ���ж�
    lfTemp = fabs(gStNavControl.lastHengxiangDis);
    if(lfTemp > NAV_LANE_DEPARTURE_DIS)
    {
        //��δƫ��״̬����ƫ��״̬������Ϊ����ƫ��
        if(gStNavControl.recordDis < 0.001f)
        {
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_LANE_DEPARTURE3, ERROR_L_NORMAL);
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Nav lane departure3, change to stop!\r\n");
        }
        //δƫ�룬����¼�¼ֵ����¼ʱ��
        else if(lfTemp <= gStNavControl.recordDis)
        {
            gStNavControl.recordTime = curTime;
            gStNavControl.recordDis = lfTemp;
        }
        //����һ�ξ���ƫ�Ƴ���1m������Ϊ����ƫ��
        else if(lfTemp - gStNavControl.recordDis > 1.0f)
        {
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_LANE_DEPARTURE1, ERROR_L_NORMAL);
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Nav lane departure1, change to stop!\r\n");
        }
        //����һ�ξ���ƫ�Ƴ���3s������Ϊ����ƫ��
        else if(curTime - gStNavControl.recordTime >= 3000)
        {
            SetErrorCode(M_TOTAL_NUM, ERROR_CODE_NAV_LANE_DEPARTURE2, ERROR_L_NORMAL);
            gStNavControl.controlCmd = NAV_CMD_STOP;
		    rt_kprintf("Nav lane departure2, change to stop!\r\n");
        }
    }
    //�������1m��Χ�ڣ���������ƫ�ƣ��򽫾����¼Ϊ0
    else if(gStNavControl.recordDis > lfTemp + 0.1f)
    {
        gStNavControl.recordTime = curTime;
        gStNavControl.recordDis = 0;
    }
}
/*****************************************************************************
 ��������  : �����߳�
 �������  : TickType_t curTime  ��ǰʱ��
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2021��5��27��
*****************************************************************************/
void NavProcess(TickType_t curTime)
{
    static TickType_t lastTime = 0;
    TickType_t timePeriod;

    //���Զ�����ģʽ��ֱ�ӷ���
    if((!(sys_para->CAR_RTinf.Link & LINK_AUTO_NAV)) && (2 != gStNavControl.oldTestFlag))
    {
        return;
    }
    //�ϻ����Ա�־
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
    //�жϹߵ�״̬��������ȷ��
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
    //ֹͣ
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
            //Ŀ����ж�
            if(IS_AUTO_CALIBRATION_Azimuth)
            {
                NavAutoCalibrationTargetJudge();    //Ŀ������жϼ���У׼����ж�
            }
            else
            {
                NavTargetJudge(curTime);    //Ŀ����ж�
            }
            //��������
            if((NAV_CMD_STOP != gStNavControl.controlCmd) && gStNavControl.lastValidFlag)
            {
                if(gStNavControl.curExeCmd != gStNavControl.controlCmd)
                {
                    gZXCurVel = 0;
                    ClearSCurveMems(&gStNavZXScurveParas);
                    gStNavControl.curExeCmd = gStNavControl.controlCmd;
                }
                NavControl(curTime, timePeriod);    //�����Զ�����
                if(!IS_AUTO_CALIBRATION_Azimuth)
                {
                    NavAbnormalJudge(curTime);  //����ƫ���ж�
                }
            }
        }
    }
}

