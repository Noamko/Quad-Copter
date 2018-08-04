 //TODO: add "Safe I" if pid isnt used for more than a seconds reset I

#include "PID.h"

PID::PID()
{

}

//Basic Gyro based PID
void PID::Compute(float error)
{
	p_Term = error * Kp;
	p_Term = constrain(p_Term, -MAX_P,MAX_P);

	i_Term += error;
	i_Term = constrain(i_Term, -MAX_I, MAX_I);
	i_Term *= Ki;

	d_Term = error - prev_error;
	d_Term *= Kd;
	prev_error = error;

	output = p_Term + i_Term + d_Term;
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
