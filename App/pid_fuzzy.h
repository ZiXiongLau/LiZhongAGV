#ifndef PID_H_
#define PID_H_
#include "predef.h"

typedef struct PID 
{
	float Kp; // ����ʽ����ϵ��
	float Ki; 
	float Kd;
	float T;
	
	float K1; // ����ʽ����ϵ��
	float K2; 
	float K3; 
	float LastError; //Error[-1]
	float PrevError; // Error[-2]

    float startValue; //����ֵѧϰ
    int32_t delt;     //��ѧϰƫ����
    int32_t offsetRecord;//ƫ��ֵ��¼

    int32_t recordTime;//��¼ʱ��
}PID;

//void PID_init(PID *structpid);
void PID_Set(PID *structpid,float Kp,float Ki,float Kd,float T);
#endif /* PID_H_ */
