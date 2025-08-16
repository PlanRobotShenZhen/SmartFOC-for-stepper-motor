#include "functionFixture.h"

void DriverFunctionTest::SetUp()
{
	testParam = GetParam();
	DriverFixture::SetUp();

	if (canOpen)
	{
		drv_Handle[0] = canOpen(&testParam.boardData[0]);
		drv_Handle[1] = canOpen(&testParam.boardData[1]);
	}
	else
	{
		drv_Handle[0] = 0;
		drv_Handle[1] = 0;
	}

	testMsg.cob_id = 0xAAA;
	testMsg.rtr = 0;
	testMsg.len = 8;
	unsigned char testData[8] = { 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
	memcpy(testMsg.data, testData, sizeof(testMsg.data));
}
void DriverFunctionTest::TearDown()
{
	DriverFixture::TearDown();
}

TEST_P(DriverFunctionTest, TryLoadLibrary) {
	ASSERT_NE(handle, (void*)NULL) << "failed to load so/dll file";
}
using ::testing::Values;
