#include "pid.h"
#include <math.h>

float pid_controller_t::update(float pv)
{
    // compute error value
    float error = setpoint - pv;
    if(error < -effective_range || error > effective_range)
    {
        if(setpoint <= pv)
            return low_limit;
        else
            return high_limit;       
    }

    // compute derivative value
    float der = error - perror;

    // update integral value
    integ += (error - integ) * kic;

    // compute controller output
    float output = kp * error + ki * integ + kd * der;

    // update previous process value
    perror = error;

    // return the result
    if(output < low_limit) output = low_limit;
    else if(output > high_limit) output = high_limit;
    return output;
}