#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define STEP_SIZE 0.006f  // 步长
#define MAX_VALUE 0.5f    // 最大值 (1.5 m/s)
#define MIN_VALUE -0.5f   // 最小值 (-1.5 m/s)
#define HOLD_COUNT 323  // 2秒，每20ms调用一次，100次

extern osTimerId myMotorTestTimerHandle;


// 定义状态
typedef enum {
    ACCELERATING_FORWARD,  // 正向加速
    HOLD_FORWARD,          // 正向维持
    DECELERATING_FORWARD,  // 正向减速
    HOLD_ZERO,             // 维持0速
    DECELERATING_BACKWARD, // 反向减速
    HOLD_BACKWARD,         // 反向维持
    ACCELERATING_BACKWARD  // 反向加速
} MotorTestState;

// 定义变量
static float motor_test_value = 0.0f;  // 当前值
static MotorTestState motor_test_state = ACCELERATING_FORWARD;  // 当前状态
static uint32_t hold_count = 0;  // 计数器，用于维持2秒

int flgstart =0;


float GetMotorTestValue(void)
{
	return motor_test_value;
}


// 定时器回调函数，每20ms调用一次
void MotorTestTimerCallback(void)
{
	if(flgstart == 1)
	{
    // 根据当前状态执行不同操作
	    switch (motor_test_state) 
		{
	        case ACCELERATING_FORWARD:
	            // 正向加速
	            if (motor_test_value < MAX_VALUE) 
				{
	                motor_test_value += STEP_SIZE;
	                if (motor_test_value >= MAX_VALUE) 
					{
	                    motor_test_value = MAX_VALUE;  // 防止超出最大值
	                    motor_test_state = HOLD_FORWARD;  // 转到正向维持阶段
	                    hold_count = 0;  // 重置计数器
	                }
	            }
	            break;

	        case HOLD_FORWARD:
	            // 正向维持阶段
	            hold_count++;  // 每次调用计数器+1
	            if (hold_count >= HOLD_COUNT) 
				{
	                motor_test_state = DECELERATING_FORWARD;  // 维持后转到正向减速
	                hold_count = 0;  // 重置计数器
	            }
	            break;

	        case DECELERATING_FORWARD:
	            // 正向减速
	            if (motor_test_value > 0) 
				{
	                motor_test_value -= STEP_SIZE;
	                if (motor_test_value <= 0) 
					{
	                    motor_test_value = 0;  // 防止小于最小值
	                    motor_test_state = ACCELERATING_FORWARD;  // 转到维持0速阶段
	                    hold_count = 0;  // 重置计数器
	                    osTimerStop(myMotorTestTimerHandle);//停止定时器
	                    flgstart = 0;
	                }
	            }
	            break;
	    }
	}
	else if(flgstart == 2)
	{
		switch (motor_test_state) 
		{
	        case ACCELERATING_FORWARD:
	            // 正向加速
	            if (motor_test_value > MIN_VALUE) 
				{
	                motor_test_value -= STEP_SIZE;
	                if (motor_test_value <= MIN_VALUE) 
					{
	                    motor_test_value = MIN_VALUE;  // 防止超出最大值
	                    motor_test_state = HOLD_FORWARD;  // 转到正向维持阶段
	                    hold_count = 0;  // 重置计数器
	                }
	            }
	            break;

	        case HOLD_FORWARD:
	            // 正向维持阶段
	            hold_count++;  // 每次调用计数器+1
	            if (hold_count >= HOLD_COUNT) 
				{
	                motor_test_state = DECELERATING_FORWARD;  // 维持后转到正向减速
	                hold_count = 0;  // 重置计数器
	            }
	            break;

	        case DECELERATING_FORWARD:
	            // 正向减速
	            if (motor_test_value < 0) 
				{
	                motor_test_value += STEP_SIZE;
	                if (motor_test_value >= 0) 
					{
	                    motor_test_value = 0;  // 防止小于最小值
	                    motor_test_state = ACCELERATING_FORWARD;  // 转到维持0速阶段
	                    hold_count = 0;  // 重置计数器
	                    osTimerStop(myMotorTestTimerHandle);//停止定时器
	                    flgstart = 0;
	                }
	            }
	            break;
	    }
	}

//        case HOLD_ZERO:
//            // 维持0速阶段
//            hold_count++;  // 每次调用计数器+1
//            if (hold_count >= HOLD_COUNT) 
//			{
//                motor_test_state = DECELERATING_BACKWARD;  // 维持后转到反向减速
//                hold_count = 0;  // 重置计数器
//            }
//            break;
//
//        case DECELERATING_BACKWARD:
//            // 反向减速
//            if (motor_test_value > MIN_VALUE) 
//			{
//                motor_test_value -= STEP_SIZE;
//                if (motor_test_value <= MIN_VALUE) 
//				{
//                    motor_test_value = MIN_VALUE;  // 防止超出最小值
//                    motor_test_state = HOLD_BACKWARD;  // 转到反向维持阶段
//                    hold_count = 0;  // 重置计数器
//                }
//            }
//            break;
//
//        case HOLD_BACKWARD:
//            // 反向维持阶段
//            hold_count++;  // 每次调用计数器+1
//            if (hold_count >= HOLD_COUNT) 
//			{
//                motor_test_state = ACCELERATING_BACKWARD;  // 维持后转到反向加速
//                hold_count = 0;  // 重置计数器
//            }
//            break;
//
//        case ACCELERATING_BACKWARD:
//            // 反向加速
//            if (motor_test_value < 0) 
//			{
//                motor_test_value += STEP_SIZE;
//                if (motor_test_value >= 0) 
//				{
//                    motor_test_value = 0;  // 防止超过0
//                    motor_test_state = ACCELERATING_FORWARD;  // 转到维持0速阶段
//                    hold_count = 0;  // 重置计数器
//                    osTimerStop(myMotorTestTimerHandle);//停止定时器
//                }
//            }
//            break;
}


