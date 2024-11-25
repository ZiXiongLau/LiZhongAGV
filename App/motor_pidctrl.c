#include "motor_pidctrl.h"
#include <math.h>
#include "motor_control.h"
#include "adc.h"
 
void InitPidParas(PIDParas* pPidParas, float kp, float ki, float kd, float fd)
{
		if(pPidParas == NULL) 
			return;
		 
		pPidParas->kp = kp;
		pPidParas->ki = ki;
		pPidParas->kd = kd;		
		pPidParas->fd = fd;
		pPidParas->sumErr = 0;
		pPidParas->lastErr = 0;
		pPidParas->preErr = 0;
		pPidParas->lastTgt = 0;
}

void SetPidParas(PIDParas* pPidParas, uint8_t para_idx, float value)
{
		switch(para_idx)
		{
			case PID_KP_IDX:
				pPidParas->kp = value;
				break;
			
			case PID_KI_IDX:
				pPidParas->ki = value;
				break;
			
			case PID_KD_IDX:
				pPidParas->kd = value;
				break;
			
			case PID_FD_IDX:
				pPidParas->fd = value;
				break;
		}
}
 
float CalcPid(PIDParas* pPidParas, float curErr, float curTgt,float uMax1)
{
		float ctrlValue;
		float alpha =0;
		float uMax;
		uMax = fabs(uMax1);
		ctrlValue = pPidParas->kp * curErr 
								+ pPidParas->ki * pPidParas->sumErr 
								+ pPidParas->kd * (pPidParas->lastErr - pPidParas->preErr)
								+ pPidParas->fd * (curTgt - pPidParas->lastTgt);
		if(ctrlValue > uMax)//抗积分饱和
		{
			if(curErr > 0)
				alpha = 0;
			else
				alpha = 1;
		}
		else if(ctrlValue < -uMax)
		{
			if(curErr > 0)
				alpha = 1;
			else
				alpha = 0;
		}
		else
			alpha = 1;
			
		pPidParas->sumErr += (alpha*curErr);
		pPidParas->preErr = pPidParas->lastErr;
		pPidParas->lastErr = curErr;
		pPidParas->lastTgt = curTgt;
		
		return ctrlValue;
}
 
void ClearPidMem(PIDParas* pPidParas)
{
		pPidParas->sumErr = 0;
		pPidParas->lastErr = 0;
		pPidParas->preErr = 0;
		pPidParas->lastTgt = 0;
}




////////////////////
void InitPidParas_incres(PIDParas_incres* pPidParas_incres, float kp, float ki, float kd, float fd)
{
	if(pPidParas_incres == NULL) 
		return;
	 
	pPidParas_incres->kp = kp;
	pPidParas_incres->ki = ki;
	pPidParas_incres->kd = kd;
	
	pPidParas_incres->Err = 0;
	pPidParas_incres->Err1 = 0;
	pPidParas_incres->Err2 = 0;
}

void SwitchPidParas_incres(PIDParas_incres* pPidParas_incres, float kp, float ki, float kd, float fd)
{
	if(pPidParas_incres == NULL) 
		return;
	 
	pPidParas_incres->kp = kp;
	pPidParas_incres->ki = ki;
	pPidParas_incres->kd = kd;
}

float CalcPid_incres(PIDParas_incres* pPidParas_incres, float curErr, uint8_t flag_Ifull)
{	
		float ctrlValue;
	
		pPidParas_incres->Err = curErr;
		if(flag_Ifull==1)//积分饱和
		{
			ctrlValue = pPidParas_incres->kp * (pPidParas_incres->Err - pPidParas_incres->Err1)
									+ pPidParas_incres->kd * (pPidParas_incres->Err - 2*pPidParas_incres->Err1 + pPidParas_incres->Err2);
		}
		else
		{
			ctrlValue = pPidParas_incres->kp * (pPidParas_incres->Err - pPidParas_incres->Err1) 
									+ pPidParas_incres->ki * pPidParas_incres->Err 
									+ pPidParas_incres->kd * (pPidParas_incres->Err - 2*pPidParas_incres->Err1 + pPidParas_incres->Err2);
			
		}
		
		pPidParas_incres->Err2 = pPidParas_incres->Err1;
		pPidParas_incres->Err1 = pPidParas_incres->Err;
		
		return ctrlValue;
}
/*****************************************************************************
 功能描述  : 驱动电机pid运算，增加一些条件判断
 输入参数  : ST_MOTOR_RUN_STATE_DATA* driverData  
             int32_t tar                            
             int32_t cur                            
             uint8_t skidFlag                      打滑标志
             uint32_t processTimeMs               
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年8月19日
*****************************************************************************/
void CalcPid_incres_driver(uint32_t motorNum, int32_t tar, int32_t cur, int32_t velErrAcc, uint32_t processTimeMs)
{
    int32_t liAccCurrent, liAccOffset, liSetAccTemp, liAccSet = 0, liSlopeCurrent, liSlopeCurrentLimit = 2000, liAccSlopeCurrentLimit = 2000, lSetAccOffsetKp = 10, lSetAccOffset = 0;
    int32_t liBrake1MpaToAcc = 100;
    //int32_t lOverAccRatio = 20, lOverCurrentLimit = 2000, lInitCurrent = 0, lMinCurrent, lMaxVel = 0;

    ST_MOTOR_RUN_STATE_DATA* driverData = &gStMotorRunState[motorNum];
    ST_MOTOR_DATA* motorData = &gStMotorData[motorNum];

    if((motorData->profileVelocity > 0) && (motorData->profileVelocity < 50001))
    {
        liSlopeCurrentLimit = motorData->profileVelocity;
    }
    if(IS_UFOONE_FLAG_SET(UFOONE_SET_ACC_OFFSET_VALID)) //设定目标加速度补偿有效，用于消除速度误差
    {
        if((gStMotorData[M_LEFT].speedGrad > 0) && (gStMotorData[M_LEFT].speedGrad <= 25))
        {
            lSetAccOffsetKp = gStMotorData[M_LEFT].speedGrad;
        }
        if((motorData->pos_limit2 > 0) && (motorData->pos_limit2 < 50001))
        {
            liAccSlopeCurrentLimit = motorData->pos_limit2;
        }
    }
    else
    {
        liAccSlopeCurrentLimit = liSlopeCurrentLimit >> 1;
        liSlopeCurrentLimit = liSlopeCurrentLimit << 2;
    }

    //刹车控制(PC控制下才生效)
    if((M_LEFT == motorNum) && IsBrakeValid)
    {
        PIDBrakeAdjust(motorNum, tar, driverData->setAcc);
    }

    //加速度调节
    liAccCurrent = 0;
    liAccOffset = 0;
	lSetAccOffset = 0;
    if((motorData->flag & MOTOR_TURN_PID_TO_ACC) && (RT_FALSE == gNoLoadFlag))  //转向pid当作加速度补偿用
    {
        lSetAccOffset = 0;
        //目标加速度补偿，当前速度离目标速度偏差较大时需做目标加速度补偿，以便减小速度误差
        if(IS_UFOONE_FLAG_SET(UFOONE_SET_ACC_OFFSET_VALID)) //设定目标加速度补偿有效，用于消除速度误差
        {
            lSetAccOffset = velErrAcc * lSetAccOffsetKp / OVER_VEL_MAX; //速度偏差
            lSetAccOffset = Limit(lSetAccOffset, -800, 800);    //限定补偿范围
        }
        //加速度pid运算
        gStMotorRunState[motorNum].pidIncresParasOne.Err = driverData->setAcc + lSetAccOffset - driverData->curAcc;

        liAccCurrent = gStMotorRunState[motorNum].pidIncresParasOne.kp * (gStMotorRunState[motorNum].pidIncresParasOne.Err - gStMotorRunState[motorNum].pidIncresParasOne.Err1)
    		+ gStMotorRunState[motorNum].pidIncresParasOne.ki * gStMotorRunState[motorNum].pidIncresParasOne.Err 
    		+ gStMotorRunState[motorNum].pidIncresParasOne.kd * (gStMotorRunState[motorNum].pidIncresParasOne.Err - 2*gStMotorRunState[motorNum].pidIncresParasOne.Err1 + gStMotorRunState[motorNum].pidIncresParasOne.Err2);
        liAccCurrent = Limit(liAccCurrent, -liAccSlopeCurrentLimit, liAccSlopeCurrentLimit);
        
        //目标加速度突变补偿
        if((sys_para->CAR_RTinf.Link & LINK_PC) && (!IS_EMERGENCY_STOP))   //PC控制并且非紧急停止，防止紧急停止时大的电流来回切换
        {
            if(motorData->startCurrent > 0)
            {
                if(IsBrakeValid && (700 == gBrakeValue) && (tar >= 0) && (driverData->setAcc < 0))  //除去刹车占比后的设定加速度
                {
                    if((gStMotorData[M_BRAKE].pos_limit2 > 0) && (gStMotorData[M_BRAKE].pos_limit2 <= 200))
                    {
                        liBrake1MpaToAcc = gStMotorData[M_BRAKE].pos_limit2;
                    }
                    liSetAccTemp = AdcGetSensorValue(ADC1_FRONT_BRAKE_CHANNEL, ADC1_OIL_OFFSET, ADC1_OIL_RATIO) * liBrake1MpaToAcc / 10 - 50;
                    liSetAccTemp = Limit(liSetAccTemp, 0, 350);
                    liSetAccTemp = driverData->setAcc + liSetAccTemp;
                    if(liSetAccTemp > 0) liSetAccTemp = 0;
                }
                else
                {
                    liSetAccTemp = driverData->setAcc;
                }
                //增加设定加速度突变补偿限制，防止过大加速度补偿超过最大电流值达到饱和
                liAccSet = driverData->limitCurrent * 100 / motorData->startCurrent;  //最大加速度
                liAccSet -= 50;  //预留0.5m/s2饱和范围
                liAccSet = Limit(liAccSet, 0, 1000);  //限定范围0到10m/s2
                driverData->lastSetAcc = Limit(driverData->lastSetAcc, -liAccSet, liAccSet);    //得到限制饱和后的上次设定加速度
                liAccSet = Limit(liSetAccTemp, -liAccSet, liAccSet); //得到限制饱和后的设定加速度
                
                if(liAccSet != driverData->lastSetAcc)
                {
                    liAccOffset = (liAccSet - driverData->lastSetAcc) * motorData->startCurrent / 100;
                }
                driverData->lastSetAcc = liAccSet;
            }
        }
        if(M_LEFT == motorNum)
        {
            gStMotorRunState[M_LEFT_ONE].accCurrent = liAccCurrent + liAccOffset;
            gStMotorRunState[M_RIGHT].accCurrent = gStMotorRunState[M_LEFT_ONE].accCurrent;
            gStMotorRunState[M_RIGHT_ONE].accCurrent = gStMotorRunState[M_RIGHT].accCurrent;
        }
        //误差赋值
    	gStMotorRunState[motorNum].pidIncresParasOne.Err2 = gStMotorRunState[motorNum].pidIncresParasOne.Err1;
    	gStMotorRunState[motorNum].pidIncresParasOne.Err1 = gStMotorRunState[motorNum].pidIncresParasOne.Err; 
    }
    //右侧电机独立控制时，加速度调节值和左侧一致(非转向pid当作加速度补偿用时的加速度补偿)
    if((!(motorData->flag & MOTOR_TURN_PID_TO_ACC)) &&
        (gStUfoData.flag & UFO_LEFT_RIGHT_ALONE_FLAG))  //左右侧电机独立控制标志
    {
        liAccCurrent = gStMotorRunState[motorNum].accCurrent;
        gStMotorRunState[motorNum].accCurrent = 0;
    }

    //速度调节
	driverData->pidIncresParas.Err = tar - cur; //误差计算
    //pid运算
    liSlopeCurrent = driverData->pidIncresParas.kp * (driverData->pidIncresParas.Err - driverData->pidIncresParas.Err1)
		+ driverData->pidIncresParas.ki * driverData->pidIncresParas.Err 
		+ driverData->pidIncresParas.kd * (driverData->pidIncresParas.Err - 2*driverData->pidIncresParas.Err1 + driverData->pidIncresParas.Err2);
    liSlopeCurrent = Limit(liSlopeCurrent, -liSlopeCurrentLimit, liSlopeCurrentLimit);

    driverData->setCurrent += liSlopeCurrent + liAccCurrent + liAccOffset;
    if(DEBUG_DATA_TYPE_4 || DEBUG_DATA_TYPE_8C || DEBUG_DATA_TYPE_8F || DEBUG_DATA_TYPE_8A)
    {
        rt_kprintf("PID%d,Acc%d:%d,%d,Vel:%d,Ao:%d.\r\n", motorData->idx, liAccSet, liAccOffset, liAccCurrent, liSlopeCurrent, lSetAccOffset);
    }
    //误差赋值	
	driverData->pidIncresParas.Err2 = driverData->pidIncresParas.Err1;
	driverData->pidIncresParas.Err1 = driverData->pidIncresParas.Err; 
}

void ClearPidMem_incres(PIDParas_incres* pPidParas_incres)
{
	pPidParas_incres->Err = 0;
	pPidParas_incres->Err1 = 0;
	pPidParas_incres->Err2 = 0;

}


