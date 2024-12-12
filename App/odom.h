#ifndef ODOM_
#define ODOM_

#include <stdint.h>
#include "predef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "motor_control.h"

#define PLUS_TO_DIST		1.0 * 2 * PI * WHEEL_RADIUS / (GEAR_RATIO * ENCODER_RESOLUTION)


typedef struct odom
{
	float odometry_left_wheel;
	float odometry_right_wheel;
}odom_t;

odom_t wheel_odom_get(void);



#endif /* ODOM_ */
