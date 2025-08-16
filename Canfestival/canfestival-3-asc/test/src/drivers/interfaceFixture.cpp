#include "interfaceFixture.h"

driverTestParameters InitDriverTestParam(const char* driverFilename, const char* busname1, const char* busname2, const char* baudrate)
{
#define MAX_STR_LEN 255
	static char tmpBusname[2][MAX_STR_LEN] = { 0 };
	static char tmpBaudrate[MAX_STR_LEN] = {0};

	struct driverTestParameters param;
	param.driverFilename = driverFilename;
	
	memcpy(tmpBusname[0], busname1, sizeof(tmpBusname[0]));
	memcpy(tmpBusname[1], busname2, sizeof(tmpBusname[0]));
	memcpy(tmpBaudrate, baudrate, sizeof(tmpBaudrate));
	
	param.boardData[0].busname = tmpBusname[0];
	param.boardData[0].baudrate = tmpBaudrate;
	param.boardData[1].busname = tmpBusname[1];
	param.boardData[1].baudrate = tmpBaudrate;

	return param;
}

void DriverFixture::SetUp()
{
	testParam = GetParam();
	dllName = testParam.driverFilename;

#if !defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__)
	handle = dlopen(dllName, RTLD_LAZY);
#else
	handle = LoadLibrary(dllName);
#endif

	if (handle)
	{
#if !defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__)
		canReceive = (CANRECEIVE_DRIVER_PROC)dlsym(handle, "canReceive_driver");
		canSend = (CANSEND_DRIVER_PROC)dlsym(handle, "canSend_driver");
		canOpen = (CANOPEN_DRIVER_PROC)dlsym(handle, "canOpen_driver");
		canClose = (CANCLOSE_DRIVER_PROC)dlsym(handle, "canClose_driver");
		canChangeBaudRate = (CANCHANGEBAUDRATE_DRIVER_PROC)dlsym(handle, "canChangeBaudRate_driver");
#else
		canReceive = (CANRECEIVE_DRIVER_PROC)GetProcAddress(handle, "canReceive_driver");
		canSend = (CANSEND_DRIVER_PROC)GetProcAddress(handle, "canSend_driver");
		canOpen = (CANOPEN_DRIVER_PROC)GetProcAddress(handle, "canOpen_driver");
		canClose = (CANCLOSE_DRIVER_PROC)GetProcAddress(handle, "canClose_driver");
		canChangeBaudRate = (CANCHANGEBAUDRATE_DRIVER_PROC)GetProcAddress(handle, "canChangeBaudRate_driver");
#endif
	}
	else
	{
		canReceive        = 0;
		canSend           = 0;
		canOpen           = 0;
		canClose          = 0;
		canChangeBaudRate = 0;
#if !defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__)

#else
		DWORD errorCode = GetLastError();
		FAIL() << "failed to load so/dll file: " << errorCode << dllName;
#endif
	}
}
void DriverFixture::TearDown()
{
	if (handle)
	{
#if !defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__)
		dlclose(handle);
#else
		FreeLibrary(handle);
#endif
	}
}

TEST_P(DriverFixture, TryLoadLibrary) {
	ASSERT_NE(handle, (void*)NULL) << "failed to load so/dll file: " << dllName;
}
TEST_P(DriverFixture, TryLoadFunctions) {
	EXPECT_EQ(canReceive        > 0, true) << "failed to load function canReceive";
	EXPECT_EQ(canSend           > 0, true) << "failed to load function canSend";
	EXPECT_EQ(canOpen           > 0, true) << "failed to load function canOpen";
	EXPECT_EQ(canClose	        > 0, true) << "failed to load function canClose";
	EXPECT_EQ(canChangeBaudRate > 0, true) << "failed to load function canChangeBaudRate";
}
using ::testing::Values;

#if (CF_TARGET==unix) && defined(CF_TARGET)
INSTANTIATE_TEST_CASE_P(driverBasicSOTest_can4linux, DriverFixture, Values(InitDriverTestParam("./libcanfestival_can_can4linux.so", "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicSOTest_lincan,    DriverFixture, Values(InitDriverTestParam("./libcanfestival_can_lincan.so",    "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicSOTest_socketCAN, DriverFixture, Values(InitDriverTestParam("./libcanfestival_can_socket.so",    "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicSOTest_virtual,   DriverFixture, Values(InitDriverTestParam("./libcanfestival_can_virtual.so",   "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicSOTest_vscom,     DriverFixture, Values(InitDriverTestParam("./libcanfestival_can_vscom.so",     "", "", "")));
#else
INSTANTIATE_TEST_CASE_P(driverBasicDLLTest_Anagate, DriverFixture, Values(InitDriverTestParam("can_anagate_win32.dll", "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicDLLTest_UVCCM,   DriverFixture, Values(InitDriverTestParam("can_uvccm_win32.dll",   "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicDLLTest_Peak,    DriverFixture, Values(InitDriverTestParam("can_peak_win32.dll",    "", "", "")));
INSTANTIATE_TEST_CASE_P(driverBasicDLLTest_TCP,     DriverFixture, Values(InitDriverTestParam("can_tcp_win32.dll",     "", "", "")));
#endif