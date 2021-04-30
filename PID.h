
// PID code, from "PID Without a PhD", c2016, Tim Wescott
// https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf

// NOTE: Derivative and Integral calculations assume consistent sampling rate

typedef struct {
	float 	derState; 		// Last position input
	float  integratState; 	// Integrator state
	float  integratMax, 	// Maximum and minimum
			integratMin; 	// allowable integrator state
	float 	integratGain, 	// integral gain
			propGain, 		// proportional gain
			derGain; 		// derivative gain
} SPid;

float UpdatePID(SPid * pid, float error, float position);
