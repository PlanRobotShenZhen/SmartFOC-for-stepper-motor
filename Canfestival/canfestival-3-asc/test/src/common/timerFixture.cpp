#include "timerFixture.h"

extern "C" {
	#include "timer.h"
	#include "applicfg.h"

	extern s_timer_entry timers[MAX_NB_TIMER];
	extern TIMEVAL total_sleep_time;
	extern TIMER_HANDLE last_timer_raw;
}

void TimerFixture::SetUp()
{
	memset(timers, 0, sizeof(timers));
	total_sleep_time = TIMEVAL_MAX;
	last_timer_raw = -1;
}
