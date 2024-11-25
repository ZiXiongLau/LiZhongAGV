#ifndef PID_H_
#define PID_H_
#include "predef.h"

typedef struct PID 
{
	float Kp; // 增量式积分系数
	float Ki; 
	float Kd;
	float T;
	
	float K1; // 增量式积分系数
	float K2; 
	float K3; 
	float LastError; //Error[-1]
	float PrevError; // Error[-2]

    float startValue; //启动值学习
    int32_t delt;     //自学习偏移量
    int32_t offsetRecord;//偏移值记录

    int32_t recordTime;//记录时间
}PID;

//void PID_init(PID *structpid);
void PID_Set(PID *structpid,float Kp,float Ki,float Kd,float T);
#endif /* PID_H_ */
