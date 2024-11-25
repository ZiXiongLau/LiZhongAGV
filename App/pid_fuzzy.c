#include <string.h>     
#include <stdio.h> 
#include <math.h>
#include "motor_control.h"

//ע1������Ӧģ��pid����Ҫ�ľ��������ѡ��Ҫ����Ӧ�ÿ��ƵĶ������к�
//ע2�����¸���ֵ���޷�ֵ�����ֵ����Ҫ���ݾ����ʹ��������и���
//ע3����Ϊ�ҵĿ��ƶ�����ԱȽϴ��������¸�����ȡֵ��С
//����e:[-5,5]  ec:[-0.5,0.5]

//���ķ�ֵ��С�������ֵ��ʱ�򣬲���PID��������������СʱƵ������������
#define Emin 0
#define Emid 3
#define Emax 15
//����ֵ�޷�����ֹ���ֱ���
#define Umax 5
#define Umin -5 

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

int kp[7][7]={	{PB,PB,PM,PM,PS,ZO,ZO},
				{PB,PB,PM,PS,PS,ZO,ZO},
				{PM,PM,PM,PS,ZO,NS,NS},
				{PM,PM,PS,ZO,NS,NM,NM},
				{PS,PS,ZO,NS,NS,NM,NM},
				{PS,ZO,NS,NM,NM,NM,NB},
				{ZO,ZO,NM,NM,NM,NB,NB}    };

int kd[7][7]={	{PS,NS,NB,NB,NB,NM,PS},
				{PS,NS,NB,NM,NM,NS,ZO},
				{ZO,NS,NM,NM,NS,NS,ZO},
				{ZO,NS,NS,NS,NS,NS,ZO},
				{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
				{PB,NS,PS,PS,PS,PS,PB},
				{PB,PM,PM,PM,PS,PS,PB}    };

int ki[7][7]={	{NB,NB,NM,NM,NS,ZO,ZO},
				{NB,NB,NM,NS,NS,ZO,ZO},
				{NB,NM,NS,NS,ZO,PS,PS},
				{NM,NM,NS,ZO,PS,PM,PM},
				{NM,NS,ZO,PS,PS,PM,PB},
				{ZO,ZO,PS,PS,PM,PB,PB},
				{ZO,ZO,PS,PM,PM,PB,PB}    };

/**************�������ȣ������Σ�***************/
float FTri(float x,float a,float b,float c)//FuzzyTriangle
{
	if(x<=a)
		return 0;
	else if((a<x)&&(x<=b))
		return (x-a)/(b-a);
	else if((b<x)&&(x<=c))
		return (c-x)/(c-b);
	else if(x>c)
		return 0;
	else
		return 0;
}
/*****************�������ȣ�������*******************/
float FTraL(float x,float a,float b)//FuzzyTrapezoidLeft
{
	if(x<=a)  
		return 1;
	else if((a<x)&&(x<=b))
		return (b-x)/(b-a);
	else if(x>b)
		return 0;
	else
		return 0;
}
/*****************�������ȣ������ң�*******************/
float FTraR(float x,float a,float b)//FuzzyTrapezoidRight
{
	if(x<=a)
		return 0;
	if((a<x)&&(x<b))
		return (x-a)/(b-a);
	if(x>=b)
		return 1;
	else
		return 1;
}
/****************�����η�ģ��������**********************/
float uFTri(float x,float a,float b,float c)
{ 
	float y,z;
	z=(b-a)*x+a;
	y=c-(c-b)*x;
	return (y+z)/2;
}
/*******************���Σ��󣩷�ģ����***********************/
float uFTraL(float x,float a,float b)
{
	return b-(b-a)*x;
}
/*******************���Σ��ң���ģ����***********************/
float uFTraR(float x,float a,float b)
{
	return (b-a)*x +a;
}
/**************************�󽻼�****************************/
float fand(float a,float b)
{
	return (a<b)?a:b;
}
/**************************�󲢼�****************************/
float forr(float a,float b)
{
	return (a<b)?b:a;
}
/*==========   PID���㲿��   ======================*/   
int PID_realize(ST_MOTOR_RUN_STATE_DATA *structpid, int32_t s, int32_t in, uint32_t processTimeMs)
{
	float pid_var;//pid������
	float iError;//��ǰ���
	
	//���������ȱ�
	float es[7],ecs[7],e,ec;
	float form[7][7];
	int i=0,j=0;
	int MaxX=0,MaxY=0;
	
	//��¼������������Ӧ������p��i��dֵ
	float lsd;
	int32_t liTemp;
	float detkp = 0,detki = 0;//�����Ľ��
	
	//�����ʽ��ת����ƫ�����
	iError = s - in; // ƫ��
	
	e=iError * 3 / Emax;
	ec=(iError-structpid->pidFuzzyParas.LastError) * 3 / 30;
	
	//���¶Ȳ�ľ���ֵС��Emaxʱ����pid�Ĳ������е���
	if(fabs(iError)<=Emax)
	{
    	//����iError��es��ecs�и����������
    	es[NB]=FTraL(e,-3,-1);  //e 
    	es[NM]=FTri(e,-3,-2,0);
    	es[NS]=FTri(e,-3,-1,1);
    	es[ZO]=FTri(e,-2,0,2);
    	es[PS]=FTri(e,-1,1,3);
    	es[PM]=FTri(e,0,2,3);
    	es[PB]=FTraR(e,1,3);

    	ecs[NB]=FTraL(ec,-3,-1);//ec
    	ecs[NM]=FTri(ec,-3,-2,0);
    	ecs[NS]=FTri(ec,-3,-1,1);
    	ecs[ZO]=FTri(ec,-2,0,2);
    	ecs[PS]=FTri(ec,-1,1,3);
    	ecs[PM]=FTri(ec,0,2,3);
    	ecs[PB]=FTraR(ec,1,3);
    	
    	//���������ȱ�ȷ��e��ec�����������������ȵ�ֵ
    	for(i=0;i<7;i++)
    	{
    		for(j=0;j<7;j++)
    		{
    			form[i][j]=fand(es[i],ecs[j]);
    		}
    	}
    	
    	//ȡ��������������ȵ���һ��
    	for(i=0;i<7;i++)
    	{
    		for(j=0;j<7;j++)
    		{
    			if(form[MaxX][MaxY]<form[i][j]) 
    			{
    				MaxX=i;
    				MaxY=j;
    			}
    		}
    	}
    	//����ģ��������ȥģ��
    	lsd=form[MaxX][MaxY];
        
    	liTemp=kp[MaxX][MaxY];    	
    	if(liTemp==NB)
    		detkp=uFTraL(lsd,-0.3,-0.1);
    	else if(liTemp==NM)
    		detkp=uFTri(lsd,-0.3,-0.2,0);
    	else if(liTemp==NS)
    		detkp=uFTri(lsd,-0.3,-0.1,0.1);
    	else if(liTemp==ZO)
    		detkp=uFTri(lsd,-0.2,0,0.2);
    	else if(liTemp==PS)
    		detkp=uFTri(lsd,-0.1,0.1,0.3);
    	else if(liTemp==PM)
    		detkp=uFTri(lsd,0,0.2,0.3);
    	else if(liTemp==PB)
    		detkp=uFTraR(lsd,0.1,0.3);

        //liTemp=kd[MaxX][MaxY];   
    	/*if(temp_d==NB)
    		detkd=uFTraL(lsd,-3,-1);
    	else if(temp_d==NM)
    		detkd=uFTri(lsd,-3,-2,0);
    	else if(temp_d==NS)
    		detkd=uFTri(lsd,-3,1,1);
    	else if(temp_d==ZO)
    		detkd=uFTri(lsd,-2,0,2);
    	else if(temp_d==PS)
    		detkd=uFTri(lsd,-1,1,3);
    	else if(temp_d==PM)
    		detkd=uFTri(lsd,0,2,3);
    	else if(temp_d==PB)
    		detkd=uFTraR(lsd,1,3);*/

        liTemp=ki[MaxX][MaxY];
    	if(liTemp==NB)
    		detki=uFTraL(lsd,-0.06,-0.02);
    	else if(liTemp==NM)
    		detki=uFTri(lsd,-0.06,-0.04,0);
    	else if(liTemp==NS)
    		detki=uFTri(lsd,-0.06,-0.02,0.02);
    	else if(liTemp==ZO)
    		detki=uFTri(lsd,-0.04,0,0.04);
    	else if(liTemp==PS)
    		detki=uFTri(lsd,-0.02,0.02,0.06);
    	else if(liTemp==PM)
    		detki=uFTri(lsd,0,0.04,0.06);
    	else if (liTemp==PB)
    		detki=uFTraR(lsd,0.02,0.06);

    	//pid����ϵ�����޸�
    	structpid->pidFuzzyParas.Kp+=detkp;
    	structpid->pidFuzzyParas.Ki+=detki;
    	//structpid->Kd+=detkd;
    	structpid->pidFuzzyParas.Kd=0;//ȡ��΢������

    	//��Kp,Ki�����޷�
    	if(structpid->pidFuzzyParas.Kp<30)
    	{
            structpid->pidFuzzyParas.Kp=30;
        }
        else if(structpid->pidFuzzyParas.Kp>200)
    	{
            structpid->pidFuzzyParas.Kp=200;
        }
    	if(structpid->pidFuzzyParas.Ki<1)
    	{
            structpid->pidFuzzyParas.Ki=1;
        }
        else if(structpid->pidFuzzyParas.Ki>20)
    	{
            structpid->pidFuzzyParas.Ki=20;
        }
    	
    	//�����µ�K1,K2,K3
    	structpid->pidFuzzyParas.K1=structpid->pidFuzzyParas.Kp+structpid->pidFuzzyParas.Ki+structpid->pidFuzzyParas.Kd;
    	structpid->pidFuzzyParas.K2=-(structpid->pidFuzzyParas.Kp+2*structpid->pidFuzzyParas.Kd);
    	structpid->pidFuzzyParas.K3=structpid->pidFuzzyParas.Kd;

        if(DEBUG_DATA_TYPE_4)
        {
            rt_kprintf("kp:%.2f,ki:%.2f.\r\n", structpid->pidFuzzyParas.Kp, 
                structpid->pidFuzzyParas.Ki);
        }
	}

	if( fabs(iError) < Emin ) //���ķ�ֵ(��������??)
	{
		pid_var = 0;
	}
	else
	{
        liTemp = 0;
        //���������0������ָ�֮ǰ����
		if((s > 0) || ((s == 0) && (in >= 0)))  //����Ŀ���ٶ�
		{
            if(s + Emid < in)   //�����趨
            {
                if(0 == structpid->pidFuzzyParas.recordTime)
                {
                    structpid->pidFuzzyParas.recordTime = 1;
                    structpid->pidFuzzyParas.offsetRecord = structpid->setCurrent;
                    structpid->setCurrent = structpid->pidFuzzyParas.delt;
                }
            }
            else if(in + Emid < s)  //�ٶȻ���
            {
                if(structpid->pidFuzzyParas.recordTime)
                {
                    liTemp = 1;
                    //structpid->setCurrent = structpid->pidFuzzyParas.offsetRecord;
                }
            }
		}
        if((s < 0) || ((s == 0) && (in < 0)))  //����Ŀ���ٶ�
		{
            if(s - Emid > in)   //�����趨
            {
                if(0 == structpid->pidFuzzyParas.recordTime)
                {
                    structpid->pidFuzzyParas.recordTime = 1;
                    structpid->pidFuzzyParas.offsetRecord = structpid->setCurrent;
                    structpid->setCurrent = -structpid->pidFuzzyParas.delt;
                }
            }
            else if(in - Emid > s)  //�ٶȻ���
            {
                if(structpid->pidFuzzyParas.recordTime)
                {
                    liTemp = 1;
                    //structpid->setCurrent = structpid->pidFuzzyParas.offsetRecord;
                }
            }
		}
        if(structpid->pidFuzzyParas.recordTime)
        {
            structpid->pidFuzzyParas.recordTime += processTimeMs;
        }
        if(liTemp)
        {
            liTemp = (100 - structpid->pidFuzzyParas.recordTime) / 50;
            liTemp = Limit(liTemp, -10, 1);
            structpid->pidFuzzyParas.delt += liTemp * (structpid->pidFuzzyParas.startValue / 10);
            structpid->pidFuzzyParas.recordTime = 0;

            if(DEBUG_DATA_TYPE_4)
            {
                rt_kprintf("dlt:%d.\r\n", structpid->pidFuzzyParas.delt);
            }
        }

		//��������
		pid_var = (structpid->pidFuzzyParas.K1 * iError  //e[k]
			+ structpid->pidFuzzyParas.K2 * structpid->pidFuzzyParas.LastError	//e[k-1]
			+ structpid->pidFuzzyParas.K3 * structpid->pidFuzzyParas.PrevError);	//e[k-2]
		if(pid_var >= Umax)pid_var = Umax;      //����ֵ�޷�����ֹ���ֱ���
		if(pid_var <= Umin)pid_var = Umin;    	//����ֵ�޷�����ֹ���ֱ���
	}
	structpid->pidFuzzyParas.PrevError=structpid->pidFuzzyParas.LastError;
	structpid->pidFuzzyParas.LastError=iError;
	
	structpid->setCurrent += 20 * pid_var;        //����PWM��� 
	
	return structpid->setCurrent; // ΢����
}

void PID_Set(PID *structpid,float Kp,float Ki,float Kd,float T)
{
	(*structpid).Kp=Kp;//Kp*(1+(Td/T));
	(*structpid).Ki=Ki;
	(*structpid).Kd=Kd;
	(*structpid).T=T;
	
	structpid->K1=structpid->Kp*(1+structpid->Ki+structpid->Kd);
	structpid->K2=-(structpid->Kp+2*structpid->Kp*structpid->Kd);
	structpid->K3=structpid->Kp*structpid->Kd;

    structpid->recordTime = 0;
    structpid->offsetRecord = 0;
    structpid->delt = 0;
    structpid->startValue = 1000;
}
