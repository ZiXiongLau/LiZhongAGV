#include "delay.h"

//延时nus
//nus为要延时的us数.最多延时19s
void delay_us(uint32_t nus)
{
    uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload = SysTick->LOAD; /* LOAD的值 */
	
	//ticks = n * 168; 			     /* 需要的节拍数 */		
	ticks = nus * 216; 			     /* 需要的节拍数 */	  		 
	told = SysTick->VAL;             /* 刚进入时的计数器值 */

	while(1)
	{
		tnow = SysTick->VAL;	
		if(tnow != told)
		{	
		    /* 这里注意一下SYSTICK是一个递减的计数器 */    
			if(tnow < told)
			{
				tcnt += told - tnow;	
			}
			/* 重新装载递减 */
			else 
			{
				tcnt += reload - tnow + told;	
			}	    
			told = tnow;

			/*时间超过/等于要延迟的时间,则退出 */
			if(tcnt >= ticks)break;
		}  
	}
}
			 



































