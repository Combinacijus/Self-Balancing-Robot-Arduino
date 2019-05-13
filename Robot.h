#include "Arduino.h"

class Robot
{
public:
	float angle = 0;  // Angle from gyro and accelerometer in radians
	float angle_acc_true = 0;  // Angle from accelerometer in radians from sensor
	float angle_acc = 0;  // Angle from accelerometer in radians adjusted for target angle
	const float target_fix_const = 1.5 * DEG_TO_RAD;
	float target_angle = 0;
	float target_angle_fix = target_fix_const;
	float pwm_integral = 0;

	bool is_balancing = false;

	Robot(int ena, int enb, int ina, int inb, int inc, int ind);
	void motors_turnoff();
	void motor(int motor, int pwm = 255, int base_pwm = 0);
	void update_angle(float gyro, float angle_acc);
	float get_angle(bool gyro = true, bool in_deg = true);
	void resetTargetAngle();
	bool is_balanced(int time = 0);
	void set_balancing(bool to_balance = true);
	unsigned long get_balance_time(bool in_sec = true);

private:
	int PIN_ENA, PIN_ENB, PIN_INA, PIN_INB, PIN_INC, PIN_IND;

	unsigned long last_time = 0;
	unsigned long delta_time;
	unsigned long last_balance_time = 0;
	unsigned long balance_start_time = 0;
	unsigned long balance_end_time = 0;
	//unsigned long balance_time = 0
};
