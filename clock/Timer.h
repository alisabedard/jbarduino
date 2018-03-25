#include <stdint.h>
class Timer
{
	public:
		Timer(const uint32_t i)
			: interval(i) {}
		bool ready(void);
		void reset(void) {value = 0;}
	private:
		const uint16_t interval;
		uint32_t value = 0;
};
class SecondsTimer : public Timer
{
	public:
		SecondsTimer(const uint16_t i)
			: Timer(1000), seconds(i) {}
		bool ready(void);
		void reset(void) {value = 0;}
	private:
		const uint16_t seconds;
		uint16_t value = 0;
};
