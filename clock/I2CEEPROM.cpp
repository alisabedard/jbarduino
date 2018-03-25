#include "I2CEEPROM.h"
#include <Arduino.h>
#include <Wire.h>
void I2CEEPROM::open_device(const uint32_t addr)
{
	Wire.beginTransmission(AT24C32);
	Wire.write(addr>>8);
	Wire.write(addr&0xff);
}
void I2CEEPROM::request(const uint32_t addr,
	const uint8_t size)
{
	open_device(addr);
	Wire.endTransmission();
	Wire.requestFrom(AT24C32, size);
}
uint8_t I2CEEPROM::read(const uint32_t addr)
{
	I2CEEPROM::request(addr, 1);
	return Wire.read();
}
void I2CEEPROM::write(const uint32_t addr, const byte * value,
	const uint8_t size)
{
	open_device(addr);
	for (uint8_t i = 0; i < size; ++i)
		Wire.write(value[i]);
	Wire.endTransmission();
	delay(10);
}
void I2CEEPROM::write(const uint32_t addr, uint8_t value)
{
	I2CEEPROM::write(addr, &value, 1);
}
