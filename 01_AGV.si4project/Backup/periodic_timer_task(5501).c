#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


extern osTimerId myMotorTestTimerHandle;

float test_vel = 0;


void MotorTestTimerCallback(void const * argument)
{
	static uint8_t cnt = 0;

	if((test_vel >= 1.5) && (cnt <= 20))
	{
		test_vel = 1.5;
		cnt += 1;
	}
	else
	{
		test_vel += 0.1;
	}


}
