#include "Timer.h"
#include <Arduino.h>
bool Timer::ready(void)
{
	const uint32_t t = millis();
	bool rval = false;
	if (t - value >= interval) {
		value = t;
		rval = true;
	}
	return rval;
}
bool SecondsTimer::ready(void)
{
	bool rval = false;;
	if (Timer::ready()) {
		if (++value >= seconds) {
			value = 0;
			rval = true;
		}
	}
	return rval;
}

