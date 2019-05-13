#include "Arduino.h"

class Button
{
public:
	Button(int pin, int _states_num=1);
	int getState();
	int getOnTime(int t = -1);
	int state_count = 1;
	int num_of_states = 1;

	void nextState();

private:
	const int DEADZONE = 100;
	int PIN;
	int on_time = 0;
	int on_start = 0;
	bool still_on = false;
};
