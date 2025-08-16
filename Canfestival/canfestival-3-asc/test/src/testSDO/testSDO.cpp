#include "gtest/gtest.h"
#include "gtest/gtest.h"
#include <hippomocks.h>

extern "C" {
#include "canfestival.h"
#include "applicfg.h"
#include "timer.h"
#include "sdo.h"
#include "testNodeSlave.h"
}
#include "testCommon.h"
#include "testCanDriver.h"

class SDO : public ::testing::Test {
public:
	virtual void SetUp()
	{
		static const char* testString = "!TESTDATA!";
		intptr_t dummy;
		CO = &testSlaveNode_Data;

		setState(CO, Operational);
		CO->canHandle = &dummy;
		data = 0x12345678;
		memset(strData, 0, sizeof(strData));
		memcpy(strData, testString,strlen(testString));
	}
	virtual void TearDown()
	{
		resetSDO(CO);
	}
protected:
	int data;
	char strData[20];
	Message txMsg;
	Message rxMsg;
	CO_Data* CO;
	UNS32 abortCode;
};

TEST_F(SDO, ClientExpediteRead)
{
	MockRepository mocks;
	mocks.OnCallFunc(canSend).Do(TestcanSend);
	mocks.OnCallFunc(setTimer);

	UNS8 res = readNetworkDict(CO, 1, 0xAABB, 0xCC, 0,0);
	EXPECT_EQ(res, 0) << "readNetworkDict call failed";
	Test_GetDriverTxCanMessage(&txMsg);

	Message expectedTx = { 0x601,0,8,{ 0x40,0xBB,0xAA,0xCC,0x00,0x00,0x00,0x00 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx), 0) << "Can Message sent by canFestival does not look OK";


	Message sendData = { 0x581,0,8,{ 0x43,0xBB,0xAA,0xCC,0x78,0x56,0x34,0x12 } };
	canDispatch(CO, &sendData);

	UNS32 tmpData;
	UNS32 dataSize = sizeof(tmpData);
	UNS8 result = getReadResultNetworkDict(CO, 1, &tmpData, &dataSize,&abortCode);
	EXPECT_EQ(result, SDO_FINISHED) << "CanFestival did not accept the SDO reply message";
	EXPECT_EQ(tmpData, data) << "Data read has wrong value";
}
TEST_F(SDO, ServerExpediteRead)
{
	//from testNodeSlave.h
	varUINT32 = 0xAABBCCDD;

	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	Message requestRead = { 0x601,0,8,{ 0x40,0x07,0x20,0x00,0x00,0x00,0x00,0x00 } };
	canDispatch(CO, &requestRead);

	Test_GetDriverTxCanMessage(&txMsg);

	Message expectedTx = { 0x581,0,8,{ 0x43,0x07,0x20,0x00,0xDD,0xCC,0xBB,0xAA } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx), 0) << "Can Message sent by canFestival does not look OK";
}
TEST_F(SDO, ClientExpediteWrite)
{
	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	UNS8 res = writeNetworkDict(CO, 1, 0xAABB, 0xCC, sizeof(data), 0, &data, 0);
	EXPECT_EQ(res, 0) << "writeNetworkDict call failed";
	Test_GetDriverTxCanMessage(&txMsg);

	Message expectedTx = { 0x601,0,8,{ 0x23,0xbb,0xaa,0xcc,0x78,0x56,0x34,0x12 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx),0) << "Can Message sent by canFestival does not look OK";

	Message writeOk = { 0x581,0,8,{ 0x60,0xbb,0xaa,0xcc,0x00,0x00,0x00,0x00 } };
	canDispatch(CO, &writeOk);

	UNS8 result = getWriteResultNetworkDict(CO, 1, &abortCode);
	EXPECT_EQ(result, SDO_FINISHED) << "CanFestival did not accept the SDO reply message";
}
TEST_F(SDO, ServerExpediteWrite)
{
	//from testNodeSlave.h
	varUINT32 = 0;

	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	Message requestDataWrite = { 0x601,0,8,{ 0x23,0x07,0x20,0x00,0xDD,0xCC,0xBB,0xAA } };
	canDispatch(CO, &requestDataWrite);

	Test_GetDriverTxCanMessage(&txMsg);

	Message expectedTx = { 0x581,0,8,{ 0x60,0x07,0x20,0x00,0,0,0,0 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx), 0) << "Can Message sent by canFestival does not look OK";
	EXPECT_EQ(varUINT32, 0xaabbccdd) << "Data did not arrived ok";
}
TEST_F(SDO, ClientSegmentedRead)
{
	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	UNS8 res = readNetworkDict(CO, 1, 0xAABB, 0xCC, 0, 0);
	EXPECT_EQ(res, 0) << "readNetworkDict call failed";

	Test_GetDriverTxCanMessage(&txMsg);

	Message expectedTx1 = { 0x601,0,8,{ 0x40,0xBB,0xAA,0xCC,0x00,0x00,0x00,0x00 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx1), 0) << "Can Message sent by canFestival does not look OK, transfer init";

	Message acceptRequest = { 0x581,0,8,{ 0x41,0xBB,0xAA,0xCC,10,0x00,0x00,0x00 } };
	canDispatch(CO, &acceptRequest);

	//Init done, checking data transfer
	Test_GetDriverTxCanMessage(&txMsg);
	Message expectedTx2 = { 0x601,0,8,{ 0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx2), 0) << "Can Message sent by canFestival does not look OK, transfer request 1";

	Message dataSend1 = { 0x581,0,8,{ 0x00,'!','T','E','S','T','D','A'} };
	canDispatch(CO, &dataSend1);

	Test_GetDriverTxCanMessage(&txMsg);
	Message expectedTx3 = { 0x601,0,8,{ 0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedTx3), 0) << "Can Message sent by canFestival does not look OK, transfer request 2";

	Message dataSend2 = { 0x581,0,8,{ 0x19,'T','A','!',0x00,0x00,0x00,0x00 } };
	canDispatch(CO, &dataSend2);

	char tmpData[20];
	UNS32 dataSize = sizeof(tmpData);
	UNS8 result = getReadResultNetworkDict(CO, 1, &tmpData, &dataSize, &abortCode);
	EXPECT_EQ(result, SDO_FINISHED) << "CanFestival did not accept the SDO reply message";
	EXPECT_EQ(memcmp(tmpData, "!TESTDATA!", result), 0) << "Data read has wrong value";
}
TEST_F(SDO, ServerSegmentedRead)
{
	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	//from testNodeSlave.h
	memcpy(varString, "!TESTDATA!",sizeof(varString));

	Message requestData = { 0x601,0,8,{ 0x40,0x09,0x20,0,0,0,0,0} };
	canDispatch(CO, &requestData);

	Test_GetDriverTxCanMessage(&txMsg);
	Message expectedAccept = { 0x581,0,8,{ 0x41,0x09,0x20,0,11,0,0,0} };
	EXPECT_EQ(CmpMessage(txMsg, expectedAccept), 0) << "Can Message sent by canFestival does not look OK, transfer reply";
	//Init done, checking data transfer
	Message requestData1 = { 0x601,0,8,{ 0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } };
	canDispatch(CO, &requestData1);

	Test_GetDriverTxCanMessage(&txMsg);
	Message data1 = { 0x581,0,8,{ 0x00,'!','T','E','S','T','D','A' } };
	EXPECT_EQ(CmpMessage(txMsg, data1), 0) << "Can Message sent by canFestival does not look OK, data transfer 1";

	Message requestData2 = { 0x601,0,8,{ 0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } };
	canDispatch(CO, &requestData2);

	Test_GetDriverTxCanMessage(&txMsg);
	Message data2 = { 0x581,0,8,{ 0x17,'T','A','!',0,0,0,0 } };
	EXPECT_EQ(CmpMessage(txMsg, data2), 0) << "Can Message sent by canFestival does not look OK, data transfer 2";
}
TEST_F(SDO, ClientSegmentedWrite)
{
	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	UNS8 res = writeNetworkDict(CO, 1, 0xAABB, 0xCC, (UNS32)strlen(strData), visible_string, strData, 0);
	EXPECT_EQ(res, 0) << "writeNetworkDict call failed";
	Test_GetDriverTxCanMessage(&txMsg);

	Message requestWrite = { 0x601,0,8,{ 0x21,0xBB,0xAA,0xCC,(UNS8)strlen(strData),0,0,0 } };
	EXPECT_EQ(CmpMessage(txMsg, requestWrite), 0) << "Can Message sent by canFestival does not look OK, transfer init";

	Message acceptWrite = { 0x581,0,8,{ 0x60,0xBB,0xAA,0xCC,0,0,0,0 } };
	canDispatch(CO, &acceptWrite);

	//Init done, checking data transfer
	Test_GetDriverTxCanMessage(&txMsg);
	Message sendData1 = { 0x601,0,8,{ 0,'!','T','E','S','T','D','A' } };
	EXPECT_EQ(CmpMessage(txMsg, sendData1), 0) << "Can Message sent by canFestival does not look OK, transfer1 1";

	Message dataTransferOk1 = { 0x581,0,8,{ 0x60,0,00,0,0,0,0,0 } };
	canDispatch(CO, &dataTransferOk1);

	Test_GetDriverTxCanMessage(&txMsg);
	Message sendData2 = { 0x601,0,8,{ 0x09,'T','A','!',0,0,0,0 } };
	EXPECT_EQ(CmpMessage(txMsg, sendData2), 0) << "Can Message sent by canFestival does not look OK, transfer 2";

	Message dataTransferOk2 = { 0x581,0,8,{ 0x70,0,00,0,0,0,0,0 } };
	canDispatch(CO, &dataTransferOk2);

	UNS8 result = getWriteResultNetworkDict(CO, 1, &abortCode);
	EXPECT_EQ(result, SDO_FINISHED) << "CanFestival did not accept the SDO reply message";
}
TEST_F(SDO, ServerSegmentedWrite)
{
	MockRepository mocks;
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);
	mocks.ExpectCallFunc(canSend).Do(TestcanSend);

	Message requestWrite = { 0x601,0,8,{ 0x21,0x09,0x20,0x00,(UNS8)strlen(strData),0,0,0 } };
	canDispatch(CO, &requestWrite);
	
	Test_GetDriverTxCanMessage(&txMsg);

	Message expectedWriteAccept = { 0x581,0,8,{ 0x60,0x09,0x20,0x0,0,0,0,0 } };
	EXPECT_EQ(CmpMessage(txMsg, expectedWriteAccept), 0) << "Can Message sent by canFestival does not look OK, transfer request ack";

	//Init done, checking data transfer
	Message sendData1 = { 0x601,0,8,{ 0,'!','T','E','S','T','D','A' } };
	canDispatch(CO, &sendData1);

	Message expectedDataOk1 = { 0x581,0,8,{ 0x20,0,0,0,0,0,0,0 } };
	Test_GetDriverTxCanMessage(&txMsg);
	EXPECT_EQ(CmpMessage(txMsg, expectedDataOk1), 0) << "Can Message sent by canFestival does not look OK, transfer request ack";

	Message sendData2 = { 0x601,0,8,{ 0x19,'T','A','!',0,0,0,0 } };
	canDispatch(CO, &sendData2);

	Message expectedDataOk2 = { 0x581,0,8,{ 0x30,0,00,0,0,0,0,0 } };
	Test_GetDriverTxCanMessage(&txMsg);
	EXPECT_EQ(CmpMessage(txMsg, expectedDataOk2), 0) << "Can Message sent by canFestival does not look OK, transfer request ack";
}