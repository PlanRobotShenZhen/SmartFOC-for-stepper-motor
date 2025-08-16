#pragma once
#include "gtest/gtest.h"
#include "canfestival.h"

struct driverTestParameters
{
	const char* driverFilename;
	struct struct_s_BOARD boardData[2];
};

driverTestParameters InitDriverTestParam(const char* driverFilename, const char* busname1, const char* busname2, const char* baudrate);

typedef UNS8  (LIBAPI* CANRECEIVE_DRIVER_PROC)(void* inst, Message *m);
typedef UNS8  (LIBAPI* CANSEND_DRIVER_PROC)(void* inst, const Message *m);
typedef void* (LIBAPI* CANOPEN_DRIVER_PROC)(s_BOARD *board);
typedef int   (LIBAPI* CANCLOSE_DRIVER_PROC)(void* inst);
typedef UNS8  (LIBAPI* CANCHANGEBAUDRATE_DRIVER_PROC)(void* fd, char* baud);

class DriverFixture : public ::testing::TestWithParam<driverTestParameters> {
public:
	virtual void SetUp();
	virtual void TearDown();
protected:
	struct driverTestParameters testParam;
	const char* dllName;
	LIB_HANDLE	handle;

	CANRECEIVE_DRIVER_PROC canReceive;
	CANSEND_DRIVER_PROC canSend;
	CANOPEN_DRIVER_PROC canOpen;
	CANCLOSE_DRIVER_PROC canClose;
	CANCHANGEBAUDRATE_DRIVER_PROC canChangeBaudRate;
};