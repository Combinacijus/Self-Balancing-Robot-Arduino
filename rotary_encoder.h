#pragma once

class Encoder
{
public:
	int pin_a, pin_b;
	long position = 0;
	float speed = 0;

	Encoder(int pin_a, int pin_b);
	void update();

private:
	int state_a;
	int state_a_old;
	unsigned long last_update_time = 0;
};