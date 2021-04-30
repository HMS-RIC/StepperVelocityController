
// PID code, from "PID Without a PhD", c2016, Tim Wescott
// https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf

// NOTE: Derivative and Integral calculations assume consistent sampling rate

#include "PID.h"

float UpdatePID(SPid * pid, float error, float position) {
	float pTerm, dTerm, iTerm;
	pTerm = pid->propGain * error; // calculate the proportional term

	// calculate the integral state with appropriate limiting
	pid->integratState += error;
	// Limit the integrator state if necessary
	if (pid->integratState > pid->integratMax) {
		pid->integratState = pid->integratMax;
	} else if (pid->integratState < pid->integratMin) {
		pid->integratState = pid->integratMin;
	}

	// calculate the integral term
	iTerm = pid->integratGain * pid->integratState;

	// calculate the derivative term
	dTerm = pid->derGain * (pid->derState - position);

	pid->derState = position;
	return pTerm + dTerm + iTerm;

}
