#include "gtest/gtest.h"
#include "canfestival.h"

#include "functionFixture.h"

class HW_PeakDriverTest : public DriverFunctionTest {};

using ::testing::Values;

TEST_P(HW_PeakDriverTest, TryInit) {
	EXPECT_NE(drv_Handle[0], (void*)0) << "failed to init driver0 with busname:baudrate: " << testParam.boardData[0].busname << ":" << testParam.boardData[0].baudrate;
	EXPECT_NE(drv_Handle[1], (void*)0) << "failed to init driver1 with busname:baudrate: " << testParam.boardData[0].busname << ":" << testParam.boardData[0].baudrate;
}

TEST_P(HW_PeakDriverTest, TrySendReceive) {
	ASSERT_NE(drv_Handle[0], (void*)0) << "driver not inited";
	ASSERT_NE(drv_Handle[1], (void*)0) << "driver not inited";
	Message msg;
	UNS8 res;
	res = canSend(drv_Handle[0], &testMsg);
	EXPECT_EQ(res, 0) << "canSend returned error code" << res;
	res = canReceive(drv_Handle[1], &msg);
	EXPECT_EQ(res, 0) << "canReceive returned error code" << res;

	EXPECT_EQ(memcmp(&testMsg, &msg, sizeof(msg)), 0);
}

TEST_P(HW_PeakDriverTest, TryChangeSingleBaudrate) {
	ASSERT_NE(drv_Handle[0], (void*)0) << "driver not inited";
	ASSERT_NE(drv_Handle[1], (void*)0) << "driver not inited";
	Message msg;
	UNS8 res;

	char baudrate[] = "250k";
	res = canChangeBaudRate(drv_Handle[0], baudrate);
	EXPECT_EQ(res, 0) << "canChangeBaudRate returned error code" << res;

	res = canSend(drv_Handle[0], &testMsg);
	EXPECT_EQ(res, 0) << "canSend returned error code" << res;

	res = canReceive(drv_Handle[1], &msg);
	EXPECT_EQ(res, 0) << "canReceive returned unexpected ok code" << res;

	EXPECT_EQ(memcmp(&testMsg, &msg, sizeof(msg)), 0);
}

//start tests

char PCAN_USBBUS1[] = "0x51U";
char PCAN_USBBUS2[] = "0x52U";
INSTANTIATE_TEST_CASE_P(driverFunctionTest_PeakWin32, HW_PeakDriverTest, Values(InitDriverTestParam("can_peak_win32.dll", PCAN_USBBUS1, PCAN_USBBUS2, "125K")));
