#include "testCommon.h"

#include "testNodeMaster.h"
#include "testNodeSlave.h"


void ResetTestNode()
{
}

int CmpMessage(const Message a, const Message b)
{
	return memcmp(&a, &b, sizeof(a));
}