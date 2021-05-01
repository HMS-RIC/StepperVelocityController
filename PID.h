
// PID code, from "PID Without a PhD", Tim Wescott, 2016
// https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf

// NOTE: Derivative and Integral calculations assume consistent sampling rate.
// (The rule of thumb for digital control systems is that the sample time
//  should be between 1/10th and 1/100th of the desired system settling time.
//  System settling time is the amount of time from the moment that the drive
//  comes out of saturation until the control system has effectively settled
//  out.)

typedef struct {
	float	derState; 		// Last position input
	float	integratState; 	// Integrator state
	float	integratMax, 	// Maximum and minimum
			integratMin; 	// allowable integrator state
	float	integratGain, 	// integral gain
			propGain, 		// proportional gain
			derGain; 		// derivative gain
} SPid;

float UpdatePID(SPid * pid, float error, float position);
