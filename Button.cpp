#include "Arduino.h"
#include "Button.h"

Button::Button(int pin, int _states_num)
{
	pinMode(pin, INPUT_PULLUP);
	PIN = pin;
	num_of_states = _states_num;
}

int Button::getState()
{
	int analog_val = DEADZONE + 1;

	if (PIN > 13)
		analog_val = analogRead(PIN);
	else
	{
		analog_val = digitalRead(PIN) == 0 ? 0 : 1024;
	}

	if (analog_val <= DEADZONE)
	{
		if (on_start == 0)
			on_start = millis();
		//    else
		//      on_time = millis() - on_start

		return HIGH;
	}
	else if (analog_val >= 1024 - DEADZONE)
	{
		on_start = 0;
		on_time = 0;
		return LOW;
	}
}

int Button::getOnTime(int t)
{
	// (t = -1) Returns how long button was pressed is t = -1
	// (t > -1) Returns true if it's been on for time t

	int state = getState();

	if (state == LOW)
		still_on = false;

	if (on_start != 0)
		on_time = millis() - on_start;

	if (t == -1)
	{
		return on_time;
	}
	else
	{
		if (on_time >= t && still_on == false)
		{
			still_on = true;
			nextState();
			return true;
		}
		else
		{
			return false;
		}
	}
}

void Button::nextState()
{
	++state_count;
	if (state_count > num_of_states)
	{
		state_count = 1;
	}
}
