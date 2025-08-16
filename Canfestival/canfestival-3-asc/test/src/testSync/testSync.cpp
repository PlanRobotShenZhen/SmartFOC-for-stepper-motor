#include "../common/timerFixture.h"
#include <hippomocks.h>

extern "C"
{
	#include "testNodeMaster.h"
	extern UNS32 testMasterNode_obj1005;
	extern UNS32 testMasterNode_obj1006;
}
//See can301 on index 1005 and 1006 for the way it should work

//extern Message curTestMessage;

TEST_F(TimerFixture, Sync_Start)
{
	MockRepository mocks;

	//1005 says no and timer says no
	testMasterNode_obj1005 = 0x0;
	testMasterNode_obj1006 = 0;
	startSYNC(&testMasterNode_Data);
	EXPECT_EQ(testMasterNode_Data.syncTimer,TIMER_NONE) << "sync wrongly started while 1005=0 1006=0";
	testMasterNode_Data.syncTimer = DelAlarm(testMasterNode_Data.syncTimer);//in case we got a timer

	//1005 says yes and timer says no
	testMasterNode_obj1005 = 0x7FFFFFFF;
	testMasterNode_obj1006 = 0;
	startSYNC(&testMasterNode_Data);
	EXPECT_EQ(testMasterNode_Data.syncTimer,TIMER_NONE) << "sync wrongly started while 1005=1 1006=0";
	testMasterNode_Data.syncTimer = DelAlarm(testMasterNode_Data.syncTimer);//in case we got a timer

	//1005 say no and timer yes
	testMasterNode_obj1005 = 0x0;
	testMasterNode_obj1006 = 1;
	startSYNC(&testMasterNode_Data);
	EXPECT_EQ(testMasterNode_Data.syncTimer,TIMER_NONE) << "sync wrongly started while 1005=0 1006=1";
	testMasterNode_Data.syncTimer = DelAlarm(testMasterNode_Data.syncTimer);//in case we got a timer
    
	//1005 say yes and timer yes. Should now start
	testMasterNode_obj1005 = 0x7FFFFFFF; //enable, with 29-bit can ID
	testMasterNode_obj1006 = 10;
	mocks.ExpectCallFunc(getElapsedTime).Return(0);
	mocks.ExpectCallFunc(setTimer).With(US_TO_TIMEVAL(testMasterNode_obj1006));
	startSYNC(&testMasterNode_Data);
	EXPECT_NE(testMasterNode_Data.syncTimer,TIMER_NONE) << "sync did not start while 1005=1 1006=1 (29bit)";
	EXPECT_EQ(*testMasterNode_Data.Sync_Cycle_Period, testMasterNode_obj1006) << "wrong sync time periode (29bit)";
	testMasterNode_Data.syncTimer = DelAlarm(testMasterNode_Data.syncTimer);//in case we got a timer

	//1005 say yes and timer yes. Should now start
	testMasterNode_obj1005 = 0x5FFFFFFF; //enable, with 11-bit can ID
	mocks.ExpectCallFunc(getElapsedTime).Return(0);
	startSYNC(&testMasterNode_Data);
	EXPECT_NE(testMasterNode_Data.syncTimer,TIMER_NONE) << "sync did not start while 1005=1 1006=1 (11bit)";
	EXPECT_EQ(*testMasterNode_Data.Sync_Cycle_Period, testMasterNode_obj1006) << "wrong sync time periode (11bit)";
	testMasterNode_Data.syncTimer = DelAlarm(testMasterNode_Data.syncTimer);//in case we got a timer
}

TEST_F(TimerFixture, Sync_Stop)
{
	MockRepository mocks;
	//pre start check
	testMasterNode_Data.syncTimer = DelAlarm(testMasterNode_Data.syncTimer);//in case we got a timer

	//1005 say yes and timer yes. should Start
	testMasterNode_obj1005 = 0x5FFFFFFF; //enable, with 11-bit can ID
	testMasterNode_obj1006 = 10;
	mocks.ExpectCallFunc(getElapsedTime).Return(0);
//	mocks.ExpectCallFunc(setTimer).With(10);
	mocks.OnCallFunc(setTimer);

	startSYNC(&testMasterNode_Data);

	EXPECT_NE(testMasterNode_Data.syncTimer,TIMER_NONE); //checking its running 

	//setting timer to 0
	testMasterNode_obj1006 = 0;
	startSYNC(&testMasterNode_Data);
	EXPECT_EQ(testMasterNode_Data.syncTimer,TIMER_NONE); //checking its not running

	//1005 say yes and timer yes. should Start
	testMasterNode_obj1005 = 0x5FFFFFFF; //enable, with 11-bit can ID
	testMasterNode_obj1006 = 10;
	mocks.ExpectCallFunc(getElapsedTime).Return(0);
	startSYNC(&testMasterNode_Data);
	EXPECT_NE(testMasterNode_Data.syncTimer,TIMER_NONE); //checking its running 

	//setting register to to 0
	testMasterNode_obj1005 = 0;
	startSYNC(&testMasterNode_Data);
	EXPECT_EQ(testMasterNode_Data.syncTimer,TIMER_NONE); //checking its not running
}
TEST_F(TimerFixture, Sync_Config11bit)
{
	MockRepository mocks;
	mocks.OnCallFunc(setTimer);
	TimeDispatch();
	//1005 say yes and timer yes. should Start
	testMasterNode_obj1005 = 0x5FFFFFFF; //enable, with 11-bit can ID
	testMasterNode_obj1006 = 10;
	//	mocks.ExpectCallFunc(getElapsedTime).Return(0);
	startSYNC(&testMasterNode_Data);
	EXPECT_NE(testMasterNode_Data.syncTimer,TIMER_NONE); //checking its running
 
	mocks.ExpectCallFunc(getElapsedTime).Return(100);
	mocks.ExpectCallFunc(setTimer).With(10);
	TimeDispatch();
  
//	EXPECT_EQ(curTestMessage.cob_id, 0x7FF); //checking its not running
}
TEST_F(TimerFixture, Sync_Config29bit)
{
	MockRepository mocks;
	mocks.OnCallFunc(setTimer);
	//1005 say yes and timer yes. should Start
	testMasterNode_obj1005 = 0x7FFFFFFF; //enable, with 29-bit can ID
	testMasterNode_obj1006 = 10;
	mocks.ExpectCallFunc(getElapsedTime).Return(0);
	startSYNC(&testMasterNode_Data);
	EXPECT_NE(testMasterNode_Data.syncTimer,TIMER_NONE); //checking its running
 
	mocks.ExpectCallFunc(getElapsedTime).Return(100);
	mocks.ExpectCallFunc(setTimer).With(10);
	TimeDispatch();
  
//	EXPECT_EQ(curTestMessage.cob_id, 0x1FFFFFFF); //checking its not running
}
