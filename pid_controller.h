#include "Arduino.h"

class PID
{
public:
	// PID coeficients
	float kp;
	float ki;
	float kd;

	float error_integral = 0;   // Sum of errors with respect to time
	float prescalar = 1;
	bool add_i = true;
	float max_output = 255;
	float anti_windup_val = max_output;

	PID(float _kp = 10, float _ki = 0.1, float _kd = 5, float _target = 0, float _prescalar = 1, float _max_output = 255);
	float control(float _value);
	void set_antiwindup(float _max_output);

private:
	float value = 0;            // Value of real object
	float prev_value = 0;       // Previuos value for derivativez
	float target = 0;           // Target value
	float error;                // Error between targtet and value

	unsigned long long prev_time = 0; // Last time it was called
	float delta_t = 0;				  // Delta time in ms

	float p_value();
	float i_value();
	float d_value();
};
