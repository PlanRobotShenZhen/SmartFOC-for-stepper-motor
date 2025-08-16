#include "../common/timerFixture.h"
#include <hippomocks.h>


void TimerCallback(CO_Data* d, UNS32 id)
{
	//Done this way to stop the compiler to remove the variables away and make hippomocks fail
	static CO_Data* d2;
	static UNS32 id2;

	d2 = d;
	id2 = id;
}
TEST_F(TimerFixture, setAlarm)
{
	MockRepository mocks;
	mocks.ExpectCallFunc(getElapsedTime).Return(10);
	mocks.ExpectCallFunc(setTimer).With(10);
	SetAlarm((CO_Data*)1, 1, &TimerCallback, 10, 0);
}
TEST_F(TimerFixture, alarmOverflow)
{
	MockRepository mocks;
	TIMER_HANDLE h;
	mocks.OnCallFunc(setTimer);
	for(int i = 0 ; i<MAX_NB_TIMER;i++)
	{
		mocks.ExpectCallFunc(getElapsedTime).Return(0);
		h = SetAlarm((CO_Data*)0, 0, &TimerCallback, 10+i, 0);  
	}
	h = SetAlarm((CO_Data*)0, 0, &TimerCallback, 10+MAX_NB_TIMER, 0);
  
	ASSERT_EQ(h, TIMER_NONE);
}
TEST_F(TimerFixture, singelDispatch)
{
	MockRepository mocks;
	mocks.OnCallFunc(setTimer);

	mocks.ExpectCallFunc(TimerCallback).With((CO_Data*)0xAA, 0xFF);
	SetAlarm((CO_Data*)0xAA, 0xFF, &TimerCallback, 10, 0);

	TimeDispatch();
}
TEST_F(TimerFixture, periodicDispatch)
{
	MockRepository mocks;
	mocks.OnCallFunc(setTimer);

	SetAlarm((CO_Data*)0xAA, 0xFF, &TimerCallback, 0, 10);

	mocks.ExpectCallFunc(TimerCallback).With((CO_Data*)0xAA, 0xFF);
	TimeDispatch();
	mocks.ExpectCallFunc(TimerCallback).With((CO_Data*)0xAA, 0xFF);
	TimeDispatch();
}