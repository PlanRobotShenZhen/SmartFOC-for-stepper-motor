#pragma once
#include "gtest/gtest.h"
#include "interfaceFixture.h"

class DriverFunctionTest : public DriverFixture {
public:
	virtual void SetUp();
	virtual void TearDown();
protected:
	CAN_HANDLE drv_Handle[2];

	Message testMsg;
};