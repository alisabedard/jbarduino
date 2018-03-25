# jbarduino
My Arduino code, made available with hopes that it may be useful
## clock
Clock is the primary project here, containing several routines that may be
helpful in other projects.  This is the code for my hand-soldered
LED 7-segment display clock.  The clock uses the "Tiny RTC I2C Module" based
on the DS1307 RTC and AT24C32 EEPROM.  Included is code to access the
module's I2C EEPROM, on which time is backed up hourly in case of battery
failure.  If the RTC time is zeroed out upon reading the first time,
it is set to the stored time and date in the I2C EEPROM (at an address indexed
by the onboard EEPROM).  Planned features may include use of AVR assembly,
EEPROM wear leveling, and an alarm clock function.
This clock supports two buttons, one which selects the current mode,
and one which actuates the mode's function.  This allows setting the time
and activating functions, such as sleep mode.
