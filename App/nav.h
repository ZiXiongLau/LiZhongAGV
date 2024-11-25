#ifndef _NAV_H
#define _NAV_H

#include <preDef.h>

//������������
enum
{
    NAV_CMD_STOP,       //������
    NAV_CMD_START,      //����
    NAV_CMD_RETURN,     //����
    NAV_CMD_NUM         //�������
};

#define IS_NAV_STATE_OK(status, posType)    ((3 == status) && (56 == posType))
#define IS_NAV_STATE_OK1(status, posType) (((3 == status) || (7 == status)) && ((53 == posType) || (54 == posType) || (55 == posType) || (56 == posType)))

#define NAV_Ea      6378137     //   ����뾶m
#define NAV_Eb      6356725     //   ���뾶m

#define NAV_PI      3.14159265358979

/**
 * Nav Data
 */
typedef struct
{
    double              Lat;                        //γ�ȣ��㣩
    double              Lon;                        //���ȣ��㣩
}ST_NAV_POS;

/**
 * Nav Data
 */
typedef struct
{
    uint8_t             validFlag;                  //������Ч��־
    uint32_t            lastRevTime;                //�ϴν��յ����ݵ�ʱ��
    uint32_t            INS_Status;                 //INS ����״̬���� 4-8 ���Ե���״̬˵��
    uint32_t            Pos_Type;                   //λ����Ϣ���ͣ����� 4-2 ��λ״̬����˵��
    ST_NAV_POS          pos;                        //λ��
    double              North_Veloci;               //�����ٶȣ�m/s��
    double              East_Velocit;               //�����ٶȣ�m/s��
    double              Up_Velocity;                //�����ٶȣ�m/s��
    double              Azimuth;                    //����ǣ�ȡֵ��Χ 0��~360�㣩
}ST_NAV_DATA;

/**
 * Nav Control
 */
typedef struct
{
    uint8_t             controlCmd;                 //��������
    uint8_t             oldTestFlag;                //�ϻ�����
    uint8_t             oldExeCmd;
    uint8_t             curExeCmd;                  //��ǰִ�е�����
    uint16_t            targetIndex;                //Ŀ�������
    uint16_t            preIndex1;                  //Ԥ���1
    uint16_t            preIndex2;                  //Ԥ���2
    uint8_t             backFlag;                   //���˱�־
    uint8_t             lastValidFlag;              //�ϴ�ֵ��Ч��־
    float               targetDistance;             //Ŀ�����
    float               targetAngle;                //Ŀ��Ƕ�
    float               preAngle;                   //Ԥ��Ƕ�
    float               lastHengxiangDis;           //��һ�κ������
    float               curVel;                     //��ǰ�ٶ�
    float               curHengxiangVel;            //��ǰ�����ٶ�
    double              AzimuthCal;                 //���������ǣ�ȡֵ��Χ 0��~360�㣩
    ST_NAV_POS          lastPos;                    //��һ��λ��
    ST_NAV_POS          recordPos;                  //λ�ü�¼
    int16_t             YM;                         //����ֵ
    int16_t             ZX;                         //ת��ֵ
    uint32_t            recordTime;                 //�����¼ʱ���
    float               recordDis;                  //�����¼
    float               recordAngle;                //��λ�Ǽ�¼
    uint32_t            recordCnt;                  //��¼����
}ST_NAV_CONTROL;


void NavDataInit(void);
void NavDataBinRev(TickType_t curTime);
void NavPrintData(TickType_t curTime);
void NavControlCmd(uint8_t* cmdData, uint8_t size);
void NavProcess(TickType_t curTime);

extern ST_NAV_DATA gStNavData; //��������
extern ST_NAV_CONTROL gStNavControl;   //������������

#endif /* _NAV_H */

