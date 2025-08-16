#include "testCanDriver.h"

#include <list>

std::list<Message> txQueue;

UNS8 TestcanSend(CAN_HANDLE fd0, const Message *m)
{
	(void)fd0;
	txQueue.push_back(*m);
	return 0;
}

bool Test_GetDriverTxCanMessage(Message *m)
{
	if (txQueue.empty())
	{
		return false;
	}
	else
	{
		memcpy(m, &txQueue.front(), sizeof(*m));
		txQueue.pop_front();
		return true;
	}
}