#ifndef _MOTOR_PID_CTRL_
#define _MOTOR_PID_CTRL_

#include "predef.h"
#include <stdio.h>

#define PID_KP_IDX		 	(1)
#define PID_KI_IDX		 	(2)
#define PID_KD_IDX		 	(3)
#define PID_FD_IDX			(4)

#define OVER_VEL_MAX        5
#define ACC_STAND_ONE       100

typedef struct
{
    float kp;
    float ki;
    float kd;
    float fd; // for vel feedforward 
    float sumErr;
    float lastErr;  //err[t-1]
    float preErr;  //err[t-2]
    float lastTgt; // for vel feedforward 
} PIDParas;

void InitPidParas(PIDParas* pPidParas, float kp, float ki, float kd, float fd);

void SetPidParas(PIDParas* pPidParas, uint8_t para_idx, float value);

float CalcPid(PIDParas* pPidParas, float curErr, float curTgt,float uMax1);

void ClearPidMem(PIDParas* pPidParas);


typedef struct
{
    float kp;
    float ki;
    float kd;
    float Err;//current err
    float Err1;  //err[t-1]
    float Err2;  //err[t-2]
} PIDParas_incres;

void InitPidParas_incres(PIDParas_incres* pPidParas_incres, float kp, float ki, float kd, float fd);
void SwitchPidParas_incres(PIDParas_incres* pPidParas_incres, float kp, float ki, float kd, float fd);
float CalcPid_incres(PIDParas_incres* pPidParas_incres, float curErr, uint8_t flag_Ifull);

void ClearPidMem_incres(PIDParas_incres* pPidParas_incres);
 
#endif  //_MOTOR_PID_CTRL_

