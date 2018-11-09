 //TODO: add "Safe I" if pid isnt used for more than a seconds reset I

#include "PID.h"

PID::PID()
{

}

void PID::Compute(float error)
{
	p_Term = error * Kp;

	i_Term += error;
	i_Term *= Ki;

	d_Term = error - prev_error;
	d_Term *= Kd;
	prev_error = error;

	output = p_Term + i_Term + d_Term;
}

void PID::Compute(float error,int16_t max_p,int16_t max_i)
{
	p_Term = error * Kp;
	p_Term = constrain(p_Term, -max_p,max_p);

	i_Term += error;
	i_Term = constrain(i_Term, -max_i, max_i);
	i_Term *= Ki;

	d_Term = error - prev_error;
	d_Term *= Kd;
	prev_error = error;

	output = p_Term + i_Term + d_Term;
}


void PID::Compute(float error,int16_t max_p,int16_t max_i,int16_t output_lim)
{
	p_Term = error * Kp;
	p_Term = constrain(p_Term, -max_p,max_p);

	i_Term += error;
	i_Term = constrain(i_Term, -max_i, max_i);
	i_Term *= Ki;

	d_Term = error - prev_error;
	d_Term *= Kd;
	prev_error = error;

	output = p_Term + i_Term + d_Term;

	output = constrain(output,-output_lim,output_lim);
}

void PID::Compute(float error,int16_t output_lim)
{
	p_Term = error * Kp;

	i_Term += error;
	i_Term *= Ki;

	d_Term = error - prev_error;
	d_Term *= Kd;
	prev_error = error;

	output = p_Term + i_Term + d_Term;

	output = constrain(output,-output_lim,output_lim);
}

void PID::Set_gains(float k_p,float k_i,float k_d)
{
	Kp = k_p;
	Ki = k_i;
	Kd = k_d;
}

void PID::Reset()
{
	i_Term = 0;
	prev_error = 0;
}
