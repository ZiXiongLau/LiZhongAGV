#include "odom.h"

odom_t _odom = {0};


odom_t wheel_odom_get()
{
	ST_MOTOR_REV_DATA* _MotorRevData = motor_revd_data_get();
	static float delta_s[2] = {0};
	static int32_t current_pulse[2] = {0};
    static int32_t last_pulse[2] = {0};



	for(int i = 0;i < 2;i ++)
	{
		current_pulse[i] = _MotorRevData[i].pos;
		delta_s[i] = ((float)(current_pulse[i] - last_pulse[i]));
		delta_s[i] = 1000.0f * delta_s[i];
		delta_s[i] = delta_s[i] * PLUS_TO_DIST;
		last_pulse[i] = current_pulse[i];
	}

	_odom.odometry_left_wheel = _odom.odometry_left_wheel + (delta_s[0] / 1000.0f);
	_odom.odometry_right_wheel = _odom.odometry_right_wheel + (delta_s[1] / 1000.0f);

	return _odom;
}









