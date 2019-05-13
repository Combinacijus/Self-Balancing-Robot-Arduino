#include "Arduino.h"
#include "Robot.h"

Robot::Robot(int ena, int enb, int ina, int inb, int inc, int ind)
{
	PIN_ENA = ena;
	PIN_ENB = enb;
	PIN_INA = ina;
	PIN_INB = inb;
	PIN_INC = inc;
	PIN_IND = ind;

	pinMode(PIN_ENA, OUTPUT);
	pinMode(PIN_ENB, OUTPUT);
	pinMode(PIN_INA, OUTPUT);
	pinMode(PIN_INB, OUTPUT);
	pinMode(PIN_INC, OUTPUT);
	pinMode(PIN_IND, OUTPUT);
}

void Robot::motors_turnoff()
{
	digitalWrite(PIN_ENA, LOW);
	digitalWrite(PIN_ENB, LOW);
	digitalWrite(PIN_INA, LOW);
	digitalWrite(PIN_INB, LOW);
	digitalWrite(PIN_INC, LOW);
	digitalWrite(PIN_IND, LOW);
}

void Robot::motor(int motor, int pwm, int base_pwm)
{
	bool is_positive = true;
	if (pwm < 0) is_positive = false;

	if (pwm < 0)
	{
		pwm *= -1;
		motor *= -1;
	}

	if (pwm != 0)
		pwm += base_pwm;
	pwm = constrain(pwm, 0, 255);
	pwm = pwm + pow(1.05, 0.5 * pwm); // Arbritrary function TODO delete?
	pwm = constrain(pwm, 0, 255);

	pwm_integral *= 0.985;
	pwm_integral += (is_positive ? pwm : -pwm) / 350.0; // * delta time
  
	switch (motor)
	{
	case -1:
		digitalWrite(PIN_INB, LOW); // Turn off opossing signal
		digitalWrite(PIN_INA, HIGH); // Turn on signal
		analogWrite(PIN_ENA, pwm);  // Control speed
		break;
	case 1:
		digitalWrite(PIN_INA, LOW);
		digitalWrite(PIN_INB, HIGH);
		analogWrite(PIN_ENA, pwm);
		break;
	case -2:
		digitalWrite(PIN_IND, LOW);
		digitalWrite(PIN_INC, HIGH);
		analogWrite(PIN_ENB, pwm);
		break;
	case 2:
		digitalWrite(PIN_INC, LOW);
		digitalWrite(PIN_IND, HIGH);
		analogWrite(PIN_ENB, pwm);
		break;
	}
}

void Robot::update_angle(float gyro, float _angle_acc)
{
	delta_time = micros() - last_time;
	last_time = micros();

	angle += gyro * delta_time / pow(10.0, 6); // Adding gyro angle

	//	float error = angle_acc - target_angle - target_angle_fix - angle;  // Fixing angle with accelerometer
	//Serial.println("error " + String(error*RAD_TO_DEG) + " " + String(angle_acc*RAD_TO_DEG) + " " + String(angle*RAD_TO_DEG)
	// + " " + String((angle_acc - target_angle - target_angle_fix)*RAD_TO_DEG));

	static float alpha1 = 0.03; // angle_acc correction
	static float alpha2 = 0.002; // Accelerometer and gyro angle fusion correction
	angle_acc_true = _angle_acc;
	//  static float angle_acc_new = angle_acc_true - target_angle_fix;
	angle_acc = (1 - alpha1) * angle_acc + alpha1 * (angle_acc_true - target_angle_fix);
	angle = (1 - alpha2) * angle + alpha2 * angle_acc;
}

float Robot::get_angle(bool gyro, bool in_deg)
{
	float return_angle = 0;

	if (gyro)
		return_angle = angle - target_angle;
	else
		return_angle = angle_acc - target_angle;

	if (in_deg)
		return_angle *= RAD_TO_DEG;

	return return_angle;
}

void Robot::resetTargetAngle()
{
	target_angle_fix = angle_acc_true;
	angle = 0;
	angle_acc = 0;
	target_angle = 0;
}

bool Robot::is_balanced(int time)
{
	const float balanced_angle = 5 * DEG_TO_RAD;

	//Serial.println("Time: " + String(millis() - last_balance_time));
	//Serial.println("Angle: " + String(angle) + " " + String(balanced_angle));

	if (abs(angle) > balanced_angle)
	{
		last_balance_time = millis();
	}
	else if (millis() - last_balance_time > time)
	{
		//is_balancing = true;  // is set manually
		return true;
	}
	return false;
}

void Robot::set_balancing(bool to_balance)
{
	if (to_balance)
	{
		if (!is_balancing)
		{
			is_balancing = true;
			balance_start_time = millis();
		}
	}
	else
	{
		if (is_balancing)
		{
			is_balancing = false;
			balance_end_time = millis();
		}
	}
}

unsigned long Robot::get_balance_time(bool in_sec)
{
	if (is_balancing)
		balance_end_time = millis();

	if (in_sec)
		return (balance_end_time - balance_start_time) / 1000;
	else
		return balance_end_time - balance_start_time; // In milliseconds
}
