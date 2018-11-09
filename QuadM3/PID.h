#ifndef PID_H_
#define PID_H_

#include "config.h"

class PID
{
public:
	PID();
	void Compute(float error);
	void Compute(float error,int16_t output_lim);
	void Compute(float error,int16_t max_p,int16_t max_i);
	void Compute(float error,int16_t max_p,int16_t max_i,int16_t output_lim);
	void Set_gains(float k_p,float k_i,float k_d);
	void Reset();

	float p_Term;
	float i_Term;
	float d_Term;
	float output = 0;
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;
	float prev_error;
};
#endif
