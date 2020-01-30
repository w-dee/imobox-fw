#ifndef PID_H__
#define PID_H__

#include <limits.h>

class pid_controller_t
{
private:
	float kp;
	float ki;
	float kd;
	float kic; //!< integral lpf coeffielent
	float setpoint; //!< target setpoint
	float effective_range; //!< pid effective range
	float low_limit; //!< output range low
	float high_limit; //!< output range high

private:
	float integ; //!< integrated error value
	float perror; //!< previous error value

public:
	pid_controller_t() : kp(0), ki(0), kd(0), kic(0), setpoint(0), effective_range(0), low_limit(0), high_limit(0),
		integ(0),
		perror(0) {}
	pid_controller_t(float kp_, float ki_, float kd_, float kic_, float eff_, float low_, float high_):
		pid_controller_t()
	{
		kp = kp_,
		ki = ki_,
		kd = kd_,
		kic = kic_;
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
	}

	/**
	 * Update internal state using provided process variable
	 * */
	float update(float pv);

	/**
	 * update set point
	 * */
	void set_set_point(float v) { setpoint = v;}

};


#endif
