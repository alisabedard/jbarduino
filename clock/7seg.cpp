#include "7seg.h"
#include <Arduino.h>
/*  7 segments:
    -a-
    b-c
    -d-
    e-f
    -g-
 */
/* Idea for mapping technique originally from
 * https://github.com/qub1750ul/Arduino_sevenSegmentDisplay/blob/master/src/sevenSegmentDisplay.h
 * */
//this array maps the state of each segment for each symbol
static uint8_t symbolMap[] = {
	B01110111, // 0, o
	B00010010, // 1, i
	B01011101, // 2, z
	B01011011, // 3
	B00111010, // 4
	B01101011, // 5
	B01101111, // 6
	B01010010, // 7
	B01111111, // 8
	B01111011, // 9
	B00001000, // -
	B00011111, // d
	B01111110, // a
	B00101100, // t, +
	B01101101, // e
	B00101110, // h
	B00001100, // r
	B00001110, // m, n
	B00111011, // y
	B00001101, // c
	B00001111, // o
	B01111100, // p
	B00000100, // i
	B00100101, // o
	B00110110, // ll
	0, // nothing
};
/*  7 segments:
    -a-
    b-c
    -d-
    e-f
    -g-
 */
static uint8_t pinMap[11];
void Seven_map(const uint8_t * pins)
{
	memcpy(pinMap, pins, 11);
}
void Seven_set(const uint8_t id)
{
	for(uint8_t i = 0; i < 7; ++i)
		digitalWrite(pinMap[i], symbolMap[id] & (1 << (i)));
}
void Seven_flash(const uint8_t digit, const uint8_t id, const uint16_t delay)
{
	Seven_set(id);
	pinMode(Seven_get_digit(digit), OUTPUT); // on
	delayMicroseconds(delay);
	pinMode(Seven_get_digit(digit), INPUT); // off
}
void Seven_flash_array(const uint8_t * a, const uint16_t delay)
{
	Seven_flash(1, a[0], delay);
	Seven_flash(2, a[1], delay);
	Seven_flash(3, a[2], delay);
	Seven_flash(4, a[3], delay);
}
uint8_t Seven_get_digit(const uint8_t id)
{
	switch (id) {
	case 1:
		return pinMap[FIRST];
	case 2:
		return pinMap[SECOND];
	case 3:
		return pinMap[THIRD];
	case 4:
		return pinMap[FOURTH];
	default:
		return 0;
	}
}
