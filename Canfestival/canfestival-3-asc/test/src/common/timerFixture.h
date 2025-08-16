#include "gtest/gtest.h"
extern "C" {
#include "applicfg.h"
#include "timer.h"
}

class TimerFixture : public ::testing::Test {
protected:
	virtual void SetUp();
};