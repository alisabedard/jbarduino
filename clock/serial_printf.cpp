#include "serial_printf.h"
#include <Arduino.h>
void serial_printf(const char * format, int32_t * numbers)
{
	uint8_t len = 0;
	while (format[++len])
		;
	for (uint8_t i = 0, n = 0; i < len; ++i) {
		switch (format[i]) {
		case '%':
			++i;
			switch (format[i]) {
			case 'b':
				Serial.print(numbers[n++], BIN);
				break;
			case 'd':
				Serial.print(numbers[n++], DEC);
				break;
			case 'h':
				Serial.print(numbers[n++], HEX);
				break;
			case 'o':
				Serial.print(numbers[n++], HEX);
				break;
			}
			break;
		default:
			Serial.print(format[i]);
		}
	}
}
