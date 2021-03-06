#include "Arduino.h"
#include "rotary_encoder.h"

Encoder::Encoder(int _pin_a, int _pin_b)
{
	pin_a = _pin_a;
	pin_b = _pin_b;

	pinMode(pin_a, INPUT);
	pinMode(pin_b, INPUT);
}

void Encoder::update()
{
	state_a = digitalRead(pin_a); // Reads the "current" state of the outputA
	// If the previous and the current state of the outputA are different, that means a Pulse has occured
	if (state_a != state_a_old)
	{
		speed = 1000000.0 / (micros() - last_update_time);
		// If the A state is different to the B state, that means the encoder is rotating clockwise
		if (digitalRead(pin_b) != state_a)
		{
			++position;
		}
		else
		{
			--position;
			speed = -speed;
		}
		//Serial.print("Enc ");
		//Serial.println(position);

		//Serial.println(micros() - last_update_time);
		last_update_time = micros();
		//Serial.println("speed " + String(speed));
	}
	else
	{
		speed *= 0.95;
	}

	state_a_old = state_a; // Updates the previous state of the outputA with the current state
}