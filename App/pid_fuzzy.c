#include <string.h>     
#include <stdio.h> 
#include <math.h>
#include "motor_control.h"

//注1：自适应模糊pid最重要的就是论域的选择，要和你应该控制的对象相切合
//注2：以下各阀值、限幅值、输出值均需要根据具体的使用情况进行更改
//注3：因为我的控制对象惯性比较大，所以以下各部分取值较小
//论域e:[-5,5]  ec:[-0.5,0.5]

//误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
#define Emin 0
#define Emid 3
#define Emax 15
//调整值限幅，防止积分饱和
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

/**************求隶属度（三角形）***************/
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
/*****************求隶属度（梯形左）*******************/
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
/*****************求隶属度（梯形右）*******************/
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
/****************三角形反模糊化处理**********************/
float uFTri(float x,float a,float b,float c)
{ 
	float y,z;
	z=(b-a)*x+a;
	y=c-(c-b)*x;
	return (y+z)/2;
}
/*******************梯形（左）反模糊化***********************/
float uFTraL(float x,float a,float b)
{
	return b-(b-a)*x;
}
/*******************梯形（右）反模糊化***********************/
float uFTraR(float x,float a,float b)
{
	return (b-a)*x +a;
}
/**************************求交集****************************/
float fand(float a,float b)
{
	return (a<b)?a:b;
}
/**************************求并集****************************/
float forr(float a,float b)
{
	return (a<b)?b:a;
}
/*==========   PID计算部分   ======================*/   
int PID_realize(ST_MOTOR_RUN_STATE_DATA *structpid, int32_t s, int32_t in, uint32_t processTimeMs)
{
	float pid_var;//pid调整量
	float iError;//当前误差
	
	//计算隶属度表
	float es[7],ecs[7],e,ec;
	float form[7][7];
	int i=0,j=0;
	int MaxX=0,MaxY=0;
	
	//记录隶属度最大项及相应推理表的p、i、d值
	float lsd;
	int32_t liTemp;
	float detkp = 0,detki = 0;//推理后的结果
	
	//输入格式的转化及偏差计算
	iError = s - in; // 偏差
	
	e=iError * 3 / Emax;
	ec=(iError-structpid->pidFuzzyParas.LastError) * 3 / 30;
	
	//当温度差的绝对值小于Emax时，对pid的参数进行调整
	if(fabs(iError)<=Emax)
	{
    	//计算iError在es与ecs中各项的隶属度
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
    	
    	//计算隶属度表，确定e和ec相关联后表格各项隶属度的值
    	for(i=0;i<7;i++)
    	{
    		for(j=0;j<7;j++)
    		{
    			form[i][j]=fand(es[i],ecs[j]);
    		}
    	}
    	
    	//取出具有最大隶属度的那一项
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
    	//进行模糊推理，并去模糊
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

    	//pid三项系数的修改
    	structpid->pidFuzzyParas.Kp+=detkp;
    	structpid->pidFuzzyParas.Ki+=detki;
    	//structpid->Kd+=detkd;
    	structpid->pidFuzzyParas.Kd=0;//取消微分作用

    	//对Kp,Ki进行限幅
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
    	
    	//计算新的K1,K2,K3
    	structpid->pidFuzzyParas.K1=structpid->pidFuzzyParas.Kp+structpid->pidFuzzyParas.Ki+structpid->pidFuzzyParas.Kd;
    	structpid->pidFuzzyParas.K2=-(structpid->pidFuzzyParas.Kp+2*structpid->pidFuzzyParas.Kd);
    	structpid->pidFuzzyParas.K3=structpid->pidFuzzyParas.Kd;

        if(DEBUG_DATA_TYPE_4)
        {
            rt_kprintf("kp:%.2f,ki:%.2f.\r\n", structpid->pidFuzzyParas.Kp, 
                structpid->pidFuzzyParas.Ki);
        }
	}

	if( fabs(iError) < Emin ) //误差的阀值(死区控制??)
	{
		pid_var = 0;
	}
	else
	{
        liTemp = 0;
        //超速设电流0，回落恢复之前电流
		if((s > 0) || ((s == 0) && (in >= 0)))  //正向目标速度
		{
            if(s + Emid < in)   //超过设定
            {
                if(0 == structpid->pidFuzzyParas.recordTime)
                {
                    structpid->pidFuzzyParas.recordTime = 1;
                    structpid->pidFuzzyParas.offsetRecord = structpid->setCurrent;
                    structpid->setCurrent = structpid->pidFuzzyParas.delt;
                }
            }
            else if(in + Emid < s)  //速度回落
            {
                if(structpid->pidFuzzyParas.recordTime)
                {
                    liTemp = 1;
                    //structpid->setCurrent = structpid->pidFuzzyParas.offsetRecord;
                }
            }
		}
        if((s < 0) || ((s == 0) && (in < 0)))  //反向目标速度
		{
            if(s - Emid > in)   //超过设定
            {
                if(0 == structpid->pidFuzzyParas.recordTime)
                {
                    structpid->pidFuzzyParas.recordTime = 1;
                    structpid->pidFuzzyParas.offsetRecord = structpid->setCurrent;
                    structpid->setCurrent = -structpid->pidFuzzyParas.delt;
                }
            }
            else if(in - Emid > s)  //速度回落
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

		//增量计算
		pid_var = (structpid->pidFuzzyParas.K1 * iError  //e[k]
			+ structpid->pidFuzzyParas.K2 * structpid->pidFuzzyParas.LastError	//e[k-1]
			+ structpid->pidFuzzyParas.K3 * structpid->pidFuzzyParas.PrevError);	//e[k-2]
		if(pid_var >= Umax)pid_var = Umax;      //调整值限幅，防止积分饱和
		if(pid_var <= Umin)pid_var = Umin;    	//调整值限幅，防止积分饱和
	}
	structpid->pidFuzzyParas.PrevError=structpid->pidFuzzyParas.LastError;
	structpid->pidFuzzyParas.LastError=iError;
	
	structpid->setCurrent += 20 * pid_var;        //调整PWM输出 
	
	return structpid->setCurrent; // 微分项
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
