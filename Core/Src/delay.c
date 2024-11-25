#include "delay.h"

//��ʱnus
//nusΪҪ��ʱ��us��.�����ʱ19s
void delay_us(uint32_t nus)
{
    uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload = SysTick->LOAD; /* LOAD��ֵ */
	
	//ticks = n * 168; 			     /* ��Ҫ�Ľ����� */		
	ticks = nus * 216; 			     /* ��Ҫ�Ľ����� */	  		 
	told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */

	while(1)
	{
		tnow = SysTick->VAL;	
		if(tnow != told)
		{	
		    /* ����ע��һ��SYSTICK��һ���ݼ��ļ����� */    
			if(tnow < told)
			{
				tcnt += told - tnow;	
			}
			/* ����װ�صݼ� */
			else 
			{
				tcnt += reload - tnow + told;	
			}	    
			told = tnow;

			/*ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
			if(tcnt >= ticks)break;
		}  
	}
}
			 



































