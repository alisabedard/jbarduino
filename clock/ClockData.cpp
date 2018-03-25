#include "ClockData.h"
#include <Arduino.h>
#include "serial_printf.h"
void ClockData::print(void)
{
	int32_t values[] = {this->h, this->m, this->s,
		this->month, this->day, this->year};
	serial_printf("%d:%d:%d, %d/%d/%d\n", values);
}
