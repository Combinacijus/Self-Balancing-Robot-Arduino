#include "Arduino.h"
#include "pid_controller.h"

PID::PID(float _kp, float _ki, float _kd, float _target, float _prescalar, float _max_output)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;
	target = _target;
	prescalar = _prescalar;

	set_antiwindup(_max_output);
}

float PID::control(float _value)
{
	delta_t = (micros() - prev_time) / 1000.0;
	prev_time = micros();

	value = _value;
	error = _value - target;

	if (add_i)
		error_integral += error * delta_t / 7;
	error_integral = constrain(error_integral, -anti_windup_val, anti_windup_val);
	//Serial.println("E integral: " + String(error_integral) + " " + String(error) + " " + String(delta_t));
	//  Serial.println("Error         : " + String(error));
	//  Serial.println("Error integral: " + String(error_integral));
	// Serial.println("P : " + String(p_value()) + " I : " + String(i_value()) + " D : " + String(d_value()));

	float return_value = prescalar * (p_value() + i_value() + d_value());  // Calculate PID values
	prev_value = value;

	return return_value;
}

float PID::p_value()
{
	return kp * error;
}

float PID::i_value()
{
	return ki * error_integral;
}

float PID::d_value()
{
	return -kd * (prev_value - value) * delta_t;
}

void PID::set_antiwindup(float _max_output)
{
	max_output = _max_output;
	anti_windup_val = max_output / ki;
	//anti_windup_val = max_output; // Might work
}