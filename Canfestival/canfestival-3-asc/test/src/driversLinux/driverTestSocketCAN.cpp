#include "gtest/gtest.h"
#include "canfestival.h"

#include "functionFixture.h"

class SocketCANDriverTest : public DriverFunctionTest {};

using ::testing::Values;

TEST_P(SocketCANDriverTest, TryInit) {
	EXPECT_NE(drv_Handle[0], (void*)0) << "failed to init driver0 with busname:baudrate: " << testParam.boardData[0].busname << ":" << testParam.boardData[0].baudrate;
	EXPECT_NE(drv_Handle[1], (void*)0) << "failed to init driver1 with busname:baudrate: " << testParam.boardData[0].busname << ":" << testParam.boardData[0].baudrate;
}

TEST_P(SocketCANDriverTest, TrySendReceive) {
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
/*
TEST_P(SocketCANDriverTest, TryChangeSingleBaudrate) {
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
*/
//start tests

char socket_bus[] = "vcan0";
INSTANTIATE_TEST_CASE_P(SocketCANDriverTest, SocketCANDriverTest, Values(InitDriverTestParam("../drivers/can_socket/libcanfestival_can_socket.so", socket_bus, socket_bus, "125K")));
