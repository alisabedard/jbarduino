// Copyright 2018, Jeffrey E. Bedard
#include <Arduino.h>
#include <stdint.h>
class I2CEEPROM {
	public:
		static void request(const uint32_t addr,
			const uint8_t size);
		static uint8_t read(const uint32_t addr);
		static void write(const uint32_t addr, const byte * value,
			const uint8_t size);
		static void write(const uint32_t addr, uint8_t value);
	private:
		enum { AT24C32 = 0x50 };
		static void open_device(const uint32_t addr);
};
