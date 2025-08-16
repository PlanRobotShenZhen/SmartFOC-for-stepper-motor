#include "gtest/gtest.h"
#include <hippomocks.h>

#include "testCommon.h"

extern "C" {
#include "applicfg.h"
#include "timer.h"
#include "testNodeMaster.h"
#include "testNodeSlave.h"
}

//See can301 on index 1005 and 1006 for the way it should work

extern Message curTestMessage;

//See 301 7.3.2.2.5 for PDO,SDO,Sync,Energency,lifeguard(Node control and error control)
//See 305 for LSS

//Test the state 'Initialisation'
TEST(states,StateInit)
{
	setState(&testMasterNode_Data,Initialisation);
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csPDO,       0) << "wrong PDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSDO,       1) << "wrong SDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSYNC,      1) << "wrong sync active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csEmergency, 1) << "wrong emergency active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLifeGuard, 1) << "wrong lifeguard active state";
#ifdef CO_ENABLE_LSS
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLSS,		 1) << "wrong LSS active state";
#endif
}
TEST(states, StatePreOperational)
{
	setState(&testMasterNode_Data,Pre_operational);
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csPDO,       0) << "wrong PDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSDO,       1) << "wrong SDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSYNC,      1) << "wrong sync active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csEmergency, 1) << "wrong emergency active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLifeGuard, 1) << "wrong lifeguard active state";
#ifdef CO_ENABLE_LSS
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLSS,		 1) << "wrong LSS active state";
#endif
}
TEST(states, stateOperational)
{
	setState(&testMasterNode_Data,Operational);
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csPDO,       1) << "wrong PDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSDO,       1) << "wrong SDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSYNC,      1) << "wrong sync active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csEmergency, 1) << "wrong emergency active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLifeGuard, 1) << "wrong lifeguard active state";
#ifdef CO_ENABLE_LSS
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLSS,		 0) << "wrong LSS active state";
#endif
}
TEST(states, stateStopped)
{
	setState(&testMasterNode_Data,Stopped);
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csPDO,       0) << "wrong PDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSDO,       0) << "wrong SDO active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csSYNC,      0) << "wrong sync active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csEmergency, 0) << "wrong emergency active state";
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLifeGuard, 1) << "wrong lifeguard active state";
#ifdef CO_ENABLE_LSS
	EXPECT_EQ(testMasterNode_Data.CurrentCommunicationState.csLSS,       1) << "wrong LSS active state";
#endif
}
TEST(states, dispatcher_reserved)
{
	//testing that reserved range messages does not get dispatched
	Message msg;
	memset(&msg, 0, sizeof(msg));

	MockRepository mocks;
	mocks.NeverCallFunc(proceedSYNC);
	mocks.NeverCallFunc(proceedEMCY);
	mocks.NeverCallFunc(proceedPDO);
	mocks.NeverCallFunc(proceedSDO);
	mocks.NeverCallFunc(proceedNODE_GUARD);
	mocks.NeverCallFunc(proceedNMTstateChange);
#ifdef CO_ENABLE_LSS
//	mocks.NeverCallFunc(proceedLSS_Slave);
//	mocks.NeverCallFunc(proceedLSS_Master);
#endif
	for (int i = 0x001; i <= 0x07F;i++)
	{
		msg.cob_id = (unsigned short)i;
		canDispatch(&testMasterNode_Data, &msg);
	}
	for (int i = 0x101; i <= 0x180; i++)
	{
		msg.cob_id = (unsigned short)i;
		canDispatch(&testMasterNode_Data, &msg);
	}
	for (int i = 0x6E0; i <= 0x6FF; i++)
	{
		msg.cob_id = (unsigned short)i;
		canDispatch(&testMasterNode_Data, &msg);
	}
	for (int i = 0x780; i <= 0x7FF; i++)
	{
		msg.cob_id = (unsigned short)i;
		canDispatch(&testMasterNode_Data, &msg);
	}
}

//NMT					0x000
//reserved				0x001 – 0x07F
//reserved				0x101 – 0x180
//default SDO(tx)		0x581 – 0x5FF
//default SDO(rx)		0x601 – 0x67F
//reserved				0x6E0 – 0x6FF
//NMT Error Control		0x701 – 0x77F
//reserved				0x780 – 0x7FF
//EMCY					0x081 - 0x0FF
//PDO1(tx)				0x181 – 0x1FF
//PDO1(rx)				0x201 – 0x27F
//PDO2(tx)				0x281 – 0x2FF
//PDO2(rx)				0x301 – 0x37F
//PDO3(tx)				0x381 – 0x3FF
//PDO3(rx)				0x401 – 0x47F
//PDO4(tx)				0x481 – 0x4FF
//PDO4(rx)				0x501 – 0x57F
//SDO(tx)				0x581 – 0x5FF
//SDO(rx)				0x601 – 0x67F
//NMT error control		0x701 – 0x77F
