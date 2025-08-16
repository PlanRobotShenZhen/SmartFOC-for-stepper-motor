#include "applicfg.h"
#include "data.h"
#include "can_driver.h"
void initTimer(void);
void clearTimer(void);

unsigned char canSend(CAN_PORT notused, Message *m);
unsigned char canInit(CO_Data * d, uint32_t bitrate);
void canClose(void);
CAN_PORT canOpen(s_BOARD *board, CO_Data * d);

void disable_it(void);
void enable_it(void);
