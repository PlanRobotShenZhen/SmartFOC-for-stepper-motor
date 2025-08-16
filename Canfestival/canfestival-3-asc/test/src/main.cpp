#include "gtest/gtest.h"
#include <hippomocks.h>

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);

	int returnValue;

	returnValue = RUN_ALL_TESTS();

	return returnValue;
}
