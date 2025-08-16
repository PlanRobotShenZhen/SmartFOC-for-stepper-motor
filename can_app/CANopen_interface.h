#ifndef __CANOPEN_INTERFACE_H
#define __CANOPEN_INTERFACE_H


#define MASTER_NODE_ID  0x01 //主设备NODE ID
#define SLAVE_NODE_ID   0x02 //从设备NODE ID


void CANopen_Init(void);
void CANopen_Task(void);



#endif

