#include "gtest/gtest.h"
#include "canfestival.h"

#include "functionFixture.h"

class TCPDriverTest : public DriverFunctionTest {;
public:
	virtual void SetUp()
	{
#if !defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__)
		system("can_tcp_win32_server.exe &");
#else
		WinExec("can_tcp_win32_server.exe", 1);//windows 
#endif
		DriverFunctionTest::SetUp();
	}
	virtual void TearDown()
	{
#if !defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__)
//		system("can_tcp_win32_server.exe &");
#else
		system("taskkill /F /IM can_tcp_win32_server.exe");
#endif
		DriverFunctionTest::TearDown();
	}
};

TEST_P(TCPDriverTest, TryInit) {
	EXPECT_NE(drv_Handle[0], (void*)0) << "failed to init driver0 with busname:baudrate: " << testParam.boardData[0].busname << ":" << testParam.boardData[0].baudrate;
	EXPECT_NE(drv_Handle[1], (void*)0) << "failed to init driver1 with busname:baudrate: " << testParam.boardData[0].busname << ":" << testParam.boardData[0].baudrate;
}

TEST_P(TCPDriverTest, TrySendReceive) {
	ASSERT_NE(drv_Handle[0], (void*)0) << "driver not inited";
	ASSERT_NE(drv_Handle[1], (void*)0) << "driver not inited";
	Message msg;
	UNS8 res;
	res = canSend(drv_Handle[0], &testMsg);
	EXPECT_EQ(res, 0) << "canSend returned error code" << res;
	res = canReceive(drv_Handle[1], &msg);
	EXPECT_EQ(res, 0) << "canReceive returned error code" << res;

	EXPECT_EQ(memcmp(&testMsg, &msg,sizeof(msg)) , 0);
}

//start test
 
using ::testing::Values;
const char* busName = "127.0.0.1";
INSTANTIATE_TEST_CASE_P(driverFunctionTest_TCPWin32, TCPDriverTest, Values(InitDriverTestParam("can_tcp_win32.dll", busName, busName, "")));