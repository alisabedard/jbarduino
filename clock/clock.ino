// jbclock - Copyright 2018, Jeffrey E. Bedard
#include "ClockData.h"
#include "7seg.h"
#include "I2CEEPROM.h"
#include "Timer.h"
#include "serial_printf.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <extEEPROM.h>
#include <EEPROM.h>
#include <MD_DS1307.h>
#include <Wire.h>
#define DEBUG_EEPROM
#define SLEEP_ENABLE
#define FLASH_SECONDS
#define FLASH_DATE
/* Uncomment the following to enable the serial console.  */
#define USE_SERIAL
/* Uncomment the following to display the current time to the serial console.
 * */
//#define USE_SERIAL_DISPLAY
//#define USE_LDR
//#define RTC_POWER_SWITCH
//#define SLEEPTIME
#define FLAGS GPIOR2
enum Constants { RUN_MODE, HR_MODE, MIN_MODE, DAY_MODE, MONTH_MODE,
	YEAR_PLUS_MODE, YEAR_MINUS_MODE, DOW_MODE, MODES, DEBOUNCE = 200,
	FLASH_SECONDS_MOD = 10, FLASH_DATE_MOD = 14, FLASHLEN = 1333,
	MODE_PIN=A0, ADJ_PIN=2, FLAG_SLEEP = 1, FLAG_MESSAGE = 2,
	LDR_PIN = A1, FLAG_CLEAR_INT = 4, CLK_VCC = A3, EEPROM_ADR = 0x50,
	CLK_SDA = A4, CLK_SCL = A5, CLK_GND = A2, SLEEP_TIMEOUT = 3600,
	EEINDEX_CLOCKDATA = 0xff};
static uint8_t current_mode;
static uint8_t message_digits[4];
#ifdef USE_LDR
class Light {
	public:
		uint16_t get(void) {return value;};
		void update(void) {value = analogRead(LDR_PIN);}
	private:
		uint16_t value;
} Light;
#endif//USE_LDR
#ifdef RTC_POWER_SWITCH
static inline void rtc_on(void)
{
	digitalWrite(CLK_VCC, HIGH);
}
static inline void rtc_off(void)
{
	digitalWrite(CLK_VCC, LOW);
}
#else//!RTC_POWER_SWITCH
#define rtc_on()
#define rtc_off()
#endif//RTC_POWER_SWITCH
static void rtc_get()
{
	rtc_on();
	RTC.readTime();
	rtc_off();
#ifdef USE_SERIAL
	Serial.println(F("rtc_get()"));
#endif//USE_SERIAL
}
static void rtc_set()
{
	rtc_on();
	RTC.writeTime();
	rtc_off();
#ifdef USE_SERIAL
	Serial.println(F("rtc_set()"));
#endif//USE_SERIAL
	backup_to_eeprom();
}
static void get_ClockData(ClockData &d)
{
	d.year = RTC.yyyy;
	d.month = RTC.mm;
	d.day = RTC.dd;
	d.h = RTC.h;
	d.m = RTC.m;
	d.s = RTC.s;
}
static void set_RTC_from_ClockData(ClockData &d)
{
	RTC.yyyy = d.year;
	RTC.mm = d.month;
	RTC.dd = d.day;
	RTC.h = d.h;
	RTC.m = d.m;
	RTC.s = d.s;
}
#ifdef USE_SERIAL_DISPLAY
static void log_to_serial(void)
{
	Serial.print(RTC.h, DEC);
	Serial.print(':');
	Serial.print(RTC.m, DEC);
	Serial.print(':');
	Serial.print(RTC.s, DEC);
#ifdef USE_LDR
	Serial.print(F(" light: "));
	Serial.print(Light.get(), DEC);


#endif//USE_LDR
	Serial.print('\n');
}
#else//!USE_SERIAL_DISPLAY
#define log_to_serial()
#endif//USE_SERIAL_DISPLAY

static void read_clock_from_i2c_eeprom(uint32_t addr, ClockData * d)
{
	ClockData * e;
	uint8_t v[sizeof(ClockData)];
	for (uint8_t i = 0; i < sizeof (ClockData); ++i)
		v[i] = I2CEEPROM::read(addr+i);
	e = (ClockData *)v;
	memcpy(d, e, sizeof(ClockData));
}
static void read_rtc_from_i2c_eeprom(uint32_t addr)
{
	ClockData d;
	read_clock_from_i2c_eeprom(addr, &d);
	set_RTC_from_ClockData(d);
}
static void write_rtc_to_i2c_eeprom(uint32_t addr)
{
	ClockData d = {RTC.yyyy, RTC.mm, RTC.dd, RTC.h, RTC.m, RTC.s};
	I2CEEPROM::write(addr, (byte *)&d, sizeof(ClockData));
}
static void restore_from_eeprom(void)
{
	uint32_t addr;
	EEPROM.get(EEINDEX_CLOCKDATA, addr);
	read_rtc_from_i2c_eeprom(addr);
#ifdef USE_SERIAL
	Serial.print(F("i2c eeprom restored from "));
	Serial.println(addr);
#endif//USE_SERIAL
}

static void backup_to_eeprom(void)
{
	uint32_t addr;
	EEPROM.get(EEINDEX_CLOCKDATA, addr);
	write_rtc_to_i2c_eeprom(addr);
#ifdef USE_SERIAL
	Serial.print(F("i2c eeprom backed up at "));
	Serial.println(addr);
#endif//USE_SERIAL
}
#ifdef DEBUG_EEPROM
static void debug_eeprom(void)
{
	uint32_t addr;
	EEPROM.get(EEINDEX_CLOCKDATA, addr);
	Serial.print(F("I2C Clock Data at address "));
	Serial.println(addr, DEC);
	ClockData d;
	read_clock_from_i2c_eeprom(addr, &d);
	Serial.print("I2C EEPROM time is ");
	d.print();
	//backup_to_eeprom();
	//restore_from_eeprom();
}
#else//!DEBUG_EEPROM
#define debug_eeprom()
#endif//DEBUG_EEPROM


static void setup_rtc(void)
{
#ifdef RTC_POWER_SWITCH
	pinMode(CLK_GND, INPUT);
	pinMode(CLK_VCC, OUTPUT);
#endif//RTC_POWER_SWITCH
	rtc_get();
	RTC.control(DS1307_CLOCK_HALT, DS1307_ON);
	RTC.control(DS1307_12H, DS1307_OFF);
	RTC.control(DS1307_SQW_RUN, DS1307_OFF);
	/* Uncomment the following two lines to program the index into the i2c
	 * eeprom */
	//uint32_t addr = 128;
	//EEPROM.put(EEINDEX_CLOCKDATA, addr);
#ifdef RTC_POWER_SWITCH
	pinMode(CLK_VCC, OUTPUT);
#endif//RTC_POWER_SWITCH
	if (RTC.yyyy == 0) {
		restore_from_eeprom();
		rtc_set();
	}
}
void setup() {
	Wire.begin();
	{
		const uint8_t m[] = {3, 8, 4, 6, 13, 5, 7, 10, 9, 12, 11};
		Seven_map(m);
	}
	// set the digital pin as output:
	for (GPIOR0 = 3; GPIOR0 < 9; GPIOR0++)
		pinMode(GPIOR0, OUTPUT);
	pinMode(13, OUTPUT);
	for (; GPIOR0 < 13; GPIOR0++)
		pinMode(GPIOR0, INPUT);
	pinMode(MODE_PIN, INPUT_PULLUP);
	pinMode(ADJ_PIN, INPUT_PULLUP);
	/* with as-needed RTC:
		28.4mA sleep
		42.4mA running
	*/
	/* with constant RTC:
		Runs at 59mA
		Sleep -20mA
		RTC uses 20mA
	*/
#ifndef USE_LDR
	ADCSRA = 0; // Disable ADC to save power
//	PRR |= PRADC | PRSPI | PRTIM1 | PRTIM2;
	MCUCR |= BODS; // Disable brown out detector
#endif//!USE_LDR
	// PRR = 0xFF;
#ifdef USE_SERIAL
	Serial.begin(9600);
#else//!USE_SERIAL
	PRR |= PRUSART0;
#endif//USE_SERIAL
	setup_rtc();
	//CLKPR=0x80; // enable
	//CLKPR=0x02; // run at 2 MHz
	debug_eeprom();
}
#ifdef SLEEP_ENABLE
static void toggle_run_mode(void)
{
	Serial.println("RUN_MODE");
	FLAGS ^= FLAG_SLEEP;
	if (FLAGS & FLAG_SLEEP) {
		message_digits[0] = SEV_N;
		message_digits[1] = SEV_I;
		message_digits[2] = SEV_T;
		message_digits[3] = SEV_E;
	} else {
		message_digits[0] = SEV_H;
		message_digits[1] = SEV_E;
		message_digits[2] = SEV_LL;
		message_digits[3] = SEV_O;
	}
	FLAGS |= FLAG_MESSAGE;
}
static void pin2_isr(void)
{
	FLAGS |= FLAG_CLEAR_INT;

	Serial.println(F("woke up"));
}
static void sleepNow(void)
{
	sleep_enable();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	attachInterrupt(digitalPinToInterrupt(2), pin2_isr, LOW);
	PRR |= PRTIM0; // stop timer;
	sleep_mode(); // do it
	sleep_disable(); // resume point
	detachInterrupt(digitalPinToInterrupt(2));
	PRR &= ~PRTIM0; // start timer;
	rtc_get();
}
#endif//SLEEP_ENABLE
static bool adjust_time_field_by_amount(uint8_t * field,
	const uint8_t limit, const uint8_t minimum,
	const uint8_t adjustment)
{
	bool rval = false;
	if ((*field += adjustment) > limit) {
		*field = minimum;
		rval = true;
	}
	return rval;
}
static inline bool adjust_time_field(uint8_t * field,
	const uint8_t limit, const uint8_t minimum)
{
	return adjust_time_field_by_amount(field, limit, minimum, 1);
}
static void update_clock(uint8_t * v)
{
#ifdef DEBUG_UPDATE_TIMING
	Serial.print("last update: ");
	Serial.println(millis(), DEC);
#endif//DEBUG_UPDATE_TIMING
	if (adjust_time_field(&RTC.s, 59, 0)) {
		if (adjust_time_field(&RTC.m, 59, 0)) {
			backup_to_eeprom(); // backup every hour
			rtc_get(); // sync to account for drift
		}
	}
	switch (current_mode) {
	case HR_MODE:
		v[0] = SEV_H;
		v[1] = SEV_R;
		v[2] = RTC.h / 10;
		v[3] = RTC.h % 10;
		break;
	case MIN_MODE:
		v[0] = SEV_M;
		v[1] = SEV_N;
		v[2] = RTC.m / 10;
		v[3] = RTC.m % 10;
		break;
	case DAY_MODE:
		v[0] = SEV_D;
		v[1] = SEV_Y;
		v[2] = RTC.dd / 10;
		v[3] = RTC.dd % 10;
		break;
	case MONTH_MODE:
		v[0] = SEV_M;
		v[1] = SEV_O;
		v[2] = RTC.mm / 10;
		v[3] = RTC.mm % 10;
		break;
	case YEAR_PLUS_MODE:
		v[0] = SEV_Y;
		v[1] = SEV_T;
		goto year_common;
		// fall-through
	case YEAR_MINUS_MODE:
		v[0] = SEV_Y;
		v[1] = SEV_DASH;
year_common:
		/* Use the GPIOR0 to store the two digit year.  */
		GPIOR0 = RTC.yyyy - 2000;
		v[2] = GPIOR0 / 10;
		v[3] = GPIOR0 % 10;
		break;
	case DOW_MODE:
		RTC.dow = RTC.calcDoW(RTC.yyyy, RTC.mm, RTC.dd);
		v[0] = SEV_D;
		v[1] = SEV_A;
		v[2] = SEV_Y;
		v[3] = RTC.dow;
		break;
	default:
#ifdef FLASH_DATE
		if (RTC.s % FLASH_DATE_MOD == 0) {
			v[0] = RTC.mm / 10;
			v[1] = RTC.mm % 10;
			v[2] = RTC.dd / 10;
			v[3] = RTC.dd % 10;
		} else
#endif//FLASH_DATE
#ifdef FLASH_SECONDS
			if (RTC.s % FLASH_SECONDS_MOD == 0) {
				v[2] = v[3] = 0xa;
				v[0] = RTC.s / 10;
				v[1] = RTC.s % 10;
			} else {
#endif//FLASH_SECONDS
				v[0] = RTC.h / 10;
				v[1] = RTC.h % 10;
				v[2] = RTC.m / 10;
				v[3] = RTC.m % 10;
#ifdef FLASH_SECONDS
			}
#endif//FLASH_SECONDS
	}
}
static void adjust_pin_action(const unsigned long now, bool * rval)
{
	Serial.println(F("ADJ_PIN"));
	static uint8_t favor = 1; // whether to click quickly
	static unsigned long last_ADJ_PIN;
	if (now - last_ADJ_PIN > DEBOUNCE) {
		last_ADJ_PIN = now;
		switch (current_mode) {
		case RUN_MODE:
#ifdef SLEEP_ENABLE
			toggle_run_mode();
#else//!SLEEP_ENABLE
			rtc_get();
#endif//SLEEP_ENABLE
			*rval = true;
			break;
		case HR_MODE:
			adjust_time_field(&RTC.h, 23, 0);
			rtc_set();
			break;
		case MIN_MODE:
			adjust_time_field_by_amount(&RTC.m, 59, 0, favor);
			rtc_set();
			break;
		case DAY_MODE:
			adjust_time_field_by_amount(&RTC.dd, 31, 1, favor);
			rtc_set();
			break;
		case MONTH_MODE:
			adjust_time_field(&RTC.mm, 12, 1);
			rtc_set();
			break;
		case YEAR_PLUS_MODE:
			++RTC.yyyy;
			rtc_set();
			break;
		case YEAR_MINUS_MODE:
			--RTC.yyyy;
			rtc_set();
			break;
		case DOW_MODE:
			rtc_set();
			break;
		default: // fix
			current_mode = 0;
		}
	} else if (now - last_ADJ_PIN < 1000) {
		favor++;
	} else
		favor = 1;
}
static bool adjust_clock(void)
{
	const uint32_t now = millis();
	bool rval = false;
	if (digitalRead(MODE_PIN) == LOW) {
		Serial.println("MODE_PIN");
		static unsigned long last_MODE_PIN;
		if (now - last_MODE_PIN > DEBOUNCE) {
			last_MODE_PIN = now;
			FLAGS |= FLAG_CLEAR_INT;
			if (++current_mode >= MODES)
				current_mode = RUN_MODE; // cycle
		}
	}
	if (digitalRead(ADJ_PIN) == LOW)
		adjust_pin_action(now, &rval);
	return rval;
}
void loop(void)
{
	static Timer adjustment(333), flash(10);
	static SecondsTimer clock(1), sleep(SLEEP_TIMEOUT), message(2);
	static uint8_t v[4]; // displayed values;
	if (FLAGS & FLAG_CLEAR_INT) {
		FLAGS &= ~FLAG_CLEAR_INT;
		adjustment.reset();
		clock.reset();
		sleep.reset();
	}
#ifdef USE_LDR
	static uint16_t flash_length;
#else//!USE_LDR
#define flash_length FLASHLEN
#endif//USE_LDR
	if (clock.ready()) {
#ifdef USE_LDR
		Light.update();
#endif//USE_LDR
		update_clock(v);
		log_to_serial();
#ifdef USE_LDR
		flash_length = (Light.get() ^ 2) >> 2;
#endif//USE_LDR
#if defined(USE_SERIAL) && defined(DEBUG_FLASH_LENGTH)
		Serial.println(flash_length);
#endif//USE_SERIAL&&DEBUG_FLASH_LENGTH
#if defined(SLEEP_ENABLE) && SLEEPTIME
		if (sleep.ready())
			toggle_run_mode();
#endif//SLEEP_ENABLE&&SLEEPTIME
	}
	if (adjustment.ready() && adjust_clock())
			FLAGS |= FLAG_MESSAGE;
	if (flash.ready()) {
#ifdef SLEEP_ENABLE
		if (!(FLAGS & FLAG_MESSAGE)) {
			if (!(FLAGS & FLAG_SLEEP))
				Seven_flash_array(v, flash_length);
			else { // sleep
				if (current_mode != RUN_MODE)
					FLAGS |= FLAG_CLEAR_INT;
				else
					sleepNow();
			}
		} else
			if (message.ready())
				FLAGS &= ~FLAG_MESSAGE;
			else
#endif//SLEEP_ENABLE
				Seven_flash_array(message_digits,
					flash_length);
	}
}
