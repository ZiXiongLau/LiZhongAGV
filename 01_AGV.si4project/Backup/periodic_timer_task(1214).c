#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define STEP_SIZE 0.006f  // ����
#define MAX_VALUE 0.5f    // ���ֵ (1.5 m/s)
#define MIN_VALUE -0.5f   // ��Сֵ (-1.5 m/s)
#define HOLD_COUNT 323  // 2�룬ÿ20ms����һ�Σ�100��

extern osTimerId myMotorTestTimerHandle;


// ����״̬
typedef enum {
    ACCELERATING_FORWARD,  // �������
    HOLD_FORWARD,          // ����ά��
    DECELERATING_FORWARD,  // �������
    HOLD_ZERO,             // ά��0��
    DECELERATING_BACKWARD, // �������
    HOLD_BACKWARD,         // ����ά��
    ACCELERATING_BACKWARD  // �������
} MotorTestState;

// �������
static float motor_test_value = 0.0f;  // ��ǰֵ
static MotorTestState motor_test_state = ACCELERATING_FORWARD;  // ��ǰ״̬
static uint32_t hold_count = 0;  // ������������ά��2��

int flgstart =0;


float GetMotorTestValue(void)
{
	return motor_test_value;
}


// ��ʱ���ص�������ÿ20ms����һ��
void MotorTestTimerCallback(void)
{
	if(flgstart == 1)
	{
    // ���ݵ�ǰ״ִ̬�в�ͬ����
	    switch (motor_test_state) 
		{
	        case ACCELERATING_FORWARD:
	            // �������
	            if (motor_test_value < MAX_VALUE) 
				{
	                motor_test_value += STEP_SIZE;
	                if (motor_test_value >= MAX_VALUE) 
					{
	                    motor_test_value = MAX_VALUE;  // ��ֹ�������ֵ
	                    motor_test_state = HOLD_FORWARD;  // ת������ά�ֽ׶�
	                    hold_count = 0;  // ���ü�����
	                }
	            }
	            break;

	        case HOLD_FORWARD:
	            // ����ά�ֽ׶�
	            hold_count++;  // ÿ�ε��ü�����+1
	            if (hold_count >= HOLD_COUNT) 
				{
	                motor_test_state = DECELERATING_FORWARD;  // ά�ֺ�ת���������
	                hold_count = 0;  // ���ü�����
	            }
	            break;

	        case DECELERATING_FORWARD:
	            // �������
	            if (motor_test_value > 0) 
				{
	                motor_test_value -= STEP_SIZE;
	                if (motor_test_value <= 0) 
					{
	                    motor_test_value = 0;  // ��ֹС����Сֵ
	                    motor_test_state = ACCELERATING_FORWARD;  // ת��ά��0�ٽ׶�
	                    hold_count = 0;  // ���ü�����
	                    osTimerStop(myMotorTestTimerHandle);//ֹͣ��ʱ��
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
	            // �������
	            if (motor_test_value > MIN_VALUE) 
				{
	                motor_test_value -= STEP_SIZE;
	                if (motor_test_value <= MIN_VALUE) 
					{
	                    motor_test_value = MIN_VALUE;  // ��ֹ�������ֵ
	                    motor_test_state = HOLD_FORWARD;  // ת������ά�ֽ׶�
	                    hold_count = 0;  // ���ü�����
	                }
	            }
	            break;

	        case HOLD_FORWARD:
	            // ����ά�ֽ׶�
	            hold_count++;  // ÿ�ε��ü�����+1
	            if (hold_count >= HOLD_COUNT) 
				{
	                motor_test_state = DECELERATING_FORWARD;  // ά�ֺ�ת���������
	                hold_count = 0;  // ���ü�����
	            }
	            break;

	        case DECELERATING_FORWARD:
	            // �������
	            if (motor_test_value < 0) 
				{
	                motor_test_value += STEP_SIZE;
	                if (motor_test_value >= 0) 
					{
	                    motor_test_value = 0;  // ��ֹС����Сֵ
	                    motor_test_state = ACCELERATING_FORWARD;  // ת��ά��0�ٽ׶�
	                    hold_count = 0;  // ���ü�����
	                    osTimerStop(myMotorTestTimerHandle);//ֹͣ��ʱ��
	                    flgstart = 0;
	                }
	            }
	            break;
	    }
	}

//        case HOLD_ZERO:
//            // ά��0�ٽ׶�
//            hold_count++;  // ÿ�ε��ü�����+1
//            if (hold_count >= HOLD_COUNT) 
//			{
//                motor_test_state = DECELERATING_BACKWARD;  // ά�ֺ�ת���������
//                hold_count = 0;  // ���ü�����
//            }
//            break;
//
//        case DECELERATING_BACKWARD:
//            // �������
//            if (motor_test_value > MIN_VALUE) 
//			{
//                motor_test_value -= STEP_SIZE;
//                if (motor_test_value <= MIN_VALUE) 
//				{
//                    motor_test_value = MIN_VALUE;  // ��ֹ������Сֵ
//                    motor_test_state = HOLD_BACKWARD;  // ת������ά�ֽ׶�
//                    hold_count = 0;  // ���ü�����
//                }
//            }
//            break;
//
//        case HOLD_BACKWARD:
//            // ����ά�ֽ׶�
//            hold_count++;  // ÿ�ε��ü�����+1
//            if (hold_count >= HOLD_COUNT) 
//			{
//                motor_test_state = ACCELERATING_BACKWARD;  // ά�ֺ�ת���������
//                hold_count = 0;  // ���ü�����
//            }
//            break;
//
//        case ACCELERATING_BACKWARD:
//            // �������
//            if (motor_test_value < 0) 
//			{
//                motor_test_value += STEP_SIZE;
//                if (motor_test_value >= 0) 
//				{
//                    motor_test_value = 0;  // ��ֹ����0
//                    motor_test_state = ACCELERATING_FORWARD;  // ת��ά��0�ٽ׶�
//                    hold_count = 0;  // ���ü�����
//                    osTimerStop(myMotorTestTimerHandle);//ֹͣ��ʱ��
//                }
//            }
//            break;
}


