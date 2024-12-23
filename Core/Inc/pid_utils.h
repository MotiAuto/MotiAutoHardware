/*
 * pid_utils.h
 *
 *  Created on: Dec 21, 2024
 *      Author: motii
 */

#ifndef INC_PID_UTILS_H_
#define INC_PID_UTILS_H_

#include <stdint.h>

#define MAX_RPM 9000
#define MAX_CURRENT 10000


typedef struct PID
{
	float p_gain;
	float i_gain;
	float d_gain;
	float prev_prop;
	float integral;
	float lpf; // LowPathFilter
}PID;

PID pidInitialize(float p_g, float i_g, float d_g)
{
	PID pid;
	pid.p_gain = p_g;
	pid.i_gain = i_g;
	pid.d_gain = d_g;
	pid.prev_prop = 0.0;
	pid.integral = 0.0;

	return pid;
}

int16_t pidCompute(PID *pid, int16_t target, int16_t actual, float delta_time)
{
	float prop = target - actual;
	if(pid->integral >= 1023)
	{
		pid->integral = 0.0;
	}
	pid->integral += prop * delta_time;
	float derivative = (prop - pid->prev_prop) / delta_time;
	pid->prev_prop = prop;

	pid->lpf = (derivative - pid->lpf) / 8.0;
	float pid_out = pid->p_gain * prop + pid->i_gain * pid->integral + pid->d_gain * pid->lpf;

	int16_t out_current = (int16_t)pid_out * MAX_CURRENT / MAX_RPM;


	return out_current;
}


#endif /* INC_PID_UTILS_H_ */
