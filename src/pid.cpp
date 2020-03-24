#include <Arduino.h>
#include "pid.h"
#include <math.h>

float pid_controller_t::update(float pv)
{
    // compute error value
    float error = setpoint - pv;

    // compute derivative value
    float der = error - perror;

    // update integral value
    integ *= kirc;
    integ += error;
    if(integ < -kilim) integ = -kilim;
    else if(integ > kilim) integ = kilim;
    derinteg += (der - derinteg) * kdc;

    // compute controller output
    last_p = kp * error;
    last_i = ki * integ;
    last_d = kd * derinteg;
    float output = last_p + last_i + last_d;

    // update previous process value
    perror = error;

    // return the result
    if(output < low_limit) output = low_limit;
    else if(output > high_limit) output = high_limit;

    if(error < -effective_range || error > effective_range)
    {
        if(setpoint <= pv)
            output = low_limit;
        else
            output =  high_limit;       
    }

    return output;
}

void pid_controller_t::dump()
{
    Serial.print(F(" kp:"));
    Serial.print(kp);

    Serial.print(F(" ki:"));
    Serial.print(ki);

    Serial.print(F(" kd:"));
    Serial.print(kd);

    Serial.print(F("\r\n integ:"));
    Serial.print(integ);

    Serial.print(F(" perror:"));
    Serial.print(perror);

    Serial.print(F(" derinteg:"));
    Serial.print(derinteg);

    Serial.print(F("\r\n last_p:"));
    Serial.print(last_p);

    Serial.print(F(" last_i:"));
    Serial.print(last_i);

    Serial.print(F(" last_d:"));
    Serial.print(last_d);

    Serial.println(F(""));


}