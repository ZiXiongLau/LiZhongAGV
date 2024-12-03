#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define STEP_SIZE 0.02f  // ����
#define MAX_VALUE 1.5f  // ���ֵ
#define MIN_VALUE 0.0f  // ��Сֵ
#define HOLD_COUNT 200  // 2�룬ÿ20ms����һ�Σ�100��

extern osTimerId myMotorTestTimerHandle;


// ����״̬
typedef enum {
    INCREASING,  // ���ӽ׶�
    HOLD,        // ά�ֽ׶�
    DECREASING   // ���ٽ׶�
} MotorTestState;

// �������
static float motor_test_value = 0.0f;  // ��ǰֵ
static MotorTestState motor_test_state = INCREASING;  // ��ǰ״̬
static uint32_t hold_count = 0;  // ������������ά��2��


float GetMotorTestValue(void)
{
	return motor_test_value;
}

// ��ʱ���ص�������ÿ20ms����һ��
void MotorTestTimerCallback(void const * argument)
{
    // ���ݵ�ǰ״ִ̬�в�ͬ����
    switch (motor_test_state) 
	{
        case INCREASING:
            // ���ӽ׶�
            if (motor_test_value < MAX_VALUE) 
			{
                motor_test_value += STEP_SIZE;
                if (motor_test_value >= MAX_VALUE) 
				{
                    motor_test_value = MAX_VALUE;  // ��ֹ�������ֵ
                    motor_test_state = HOLD;  // �л���ά�ֽ׶�
                    hold_count = 0;  // ���¼���
                }
            }
            break;

        case HOLD:
            // ά�ֽ׶�
            hold_count++;  // ÿ�ε��ü�����+1
            if (hold_count >= HOLD_COUNT) 
			{
                motor_test_state = DECREASING;  // ά��2����л������ٽ׶�
                hold_count = 0;  // ���ü�����
            }
            break;

        case DECREASING:
            // ���ٽ׶�
            if (motor_test_value > MIN_VALUE) 
			{
                motor_test_value -= STEP_SIZE;
                if (motor_test_value <= MIN_VALUE) 
				{
                    motor_test_value = MIN_VALUE;  // ��ֹС����Сֵ
                    motor_test_state = INCREASING;  // �л������ӽ׶�
                    osTimerStop(myMotorTestTimerHandle);//ֹͣ��ʱ��
                }
            }
            break;
    }

}

