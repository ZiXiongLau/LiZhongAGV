#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define STEP_SIZE 0.02f  // 步长
#define MAX_VALUE 1.5f  // 最大值
#define MIN_VALUE 0.0f  // 最小值
#define HOLD_COUNT 200  // 2秒，每20ms调用一次，100次

extern osTimerId myMotorTestTimerHandle;


// 定义状态
typedef enum {
    INCREASING,  // 增加阶段
    HOLD,        // 维持阶段
    DECREASING   // 减少阶段
} MotorTestState;

// 定义变量
static float motor_test_value = 0.0f;  // 当前值
static MotorTestState motor_test_state = INCREASING;  // 当前状态
static uint32_t hold_count = 0;  // 计数器，用于维持2秒


float GetMotorTestValue(void)
{
	return motor_test_value;
}

// 定时器回调函数，每20ms调用一次
void MotorTestTimerCallback(void const * argument)
{
    // 根据当前状态执行不同操作
    switch (motor_test_state) 
	{
        case INCREASING:
            // 增加阶段
            if (motor_test_value < MAX_VALUE) 
			{
                motor_test_value += STEP_SIZE;
                if (motor_test_value >= MAX_VALUE) 
				{
                    motor_test_value = MAX_VALUE;  // 防止超出最大值
                    motor_test_state = HOLD;  // 切换到维持阶段
                    hold_count = 0;  // 重新计数
                }
            }
            break;

        case HOLD:
            // 维持阶段
            hold_count++;  // 每次调用计数器+1
            if (hold_count >= HOLD_COUNT) 
			{
                motor_test_state = DECREASING;  // 维持2秒后切换到减少阶段
                hold_count = 0;  // 重置计数器
            }
            break;

        case DECREASING:
            // 减少阶段
            if (motor_test_value > MIN_VALUE) 
			{
                motor_test_value -= STEP_SIZE;
                if (motor_test_value <= MIN_VALUE) 
				{
                    motor_test_value = MIN_VALUE;  // 防止小于最小值
                    motor_test_state = INCREASING;  // 切换到增加阶段
                    osTimerStop(myMotorTestTimerHandle);//停止定时器
                }
            }
            break;
    }

}

