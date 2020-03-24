#ifndef PID_H__
#define PID_H__

#include <limits.h>

class pid_controller_t
{
public:
	float kp;
	float ki;
	float kd;
	float kilim; //!< integral upper/-lower limit
	float kirc; //!< integral remembrance coeffielent
	float kdc; //!< derivertive lpf coeffielent
	float setpoint; //!< target setpoint
	float effective_range; //!< pid effective range
	float low_limit; //!< output range low
	float high_limit; //!< output range high

private:
	float integ; //!< integrated error value
	float perror; //!< previous error value
	float derinteg; //!< integrated error differencial
	float last_p;
	float last_i;
	float last_d;

public:
	pid_controller_t() : kp(0), ki(0), kd(0), kilim(0), kirc(0), kdc(0), setpoint(0), effective_range(0), low_limit(0), high_limit(0),
		integ(0),
		perror(0),
		derinteg(0),
		last_p(0), last_i(0), last_d(0)
		 {}
	pid_controller_t(float kp_, float ki_, float kd_, float kilim_, float kirc_, float kdc_, float eff_, float low_, float high_):
		pid_controller_t()
	{
		kp = kp_,
		ki = ki_,
		kd = kd_,
		kilim = kilim_;
		kirc = kirc_;
		kdc = kdc_;
		effective_range = eff_;
		low_limit = low_;
		high_limit = high_;
	}

	/**
	 * reset internal integral and derivative state
	 * */
	void reset()
	{
		integ = 0;
		perror = 0;
		derinteg = 0;
		last_p = last_i = last_d = 0;
	}

	/**
	 * Update internal state using provided process variable
	 * */
	float update(float pv);

	/**
	 * update set point
	 * */
	void set_set_point(float v) { setpoint = v;}

	/**
	 * dump internal variables
	 * */
	void dump();

};


#endif
