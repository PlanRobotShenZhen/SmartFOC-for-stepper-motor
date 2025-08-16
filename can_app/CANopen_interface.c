#include "CANopen_interface.h"
#include "can_driver.h"
#include "canfestival.h"
#include "common.h"
#include "n32g45x.h"
#include "../can_app/Ds402_SlaveCb.h"
#include "../can_app/Ds402_Slave.h"
#include "timer.h"
#include "data.h"
#include "cm4.h"
#include "common.h"
#include "ExternGlobals.h"

#define CANx                       CAN1
#define CAN_CLK                    RCC_APB1_PERIPH_CAN1
#define CAN_RX_PIN                 GPIO_PIN_8
#define CAN_TX_PIN                 GPIO_PIN_9
#define CAN_GPIO_PORT              GPIOB
#define CAN_GPIO_CLK               RCC_APB2_PERIPH_GPIOB


/*******************************修改为适用于n32g45x的底层配置*****************************************/

TIMEVAL last_counter_val = 0;
TIMEVAL elapsed_time = 0;

static CO_Data *co_data = NULL;

s_BOARD SlaveBoard = {"0", "1000K"};

/*
 * @note:设置设备是从设备还是主设备，由于在NMT通信时，系统中只允许有一个
 *      NMT网络节点，所以我们在进行NMT实验时，需要将当前设备设置为slave设备
 * @param:
 *      - d:数据字典对象
 *      - nodeRole:设备类型，1:从设备，0:主设备
 * */

CanTxMessage CAN_TxMessages;


extern UNS32  Ds402_Slave_obj1200_COB_ID_Client_to_Server_Receive_SDO ;
extern UNS32 Ds402_Slave_obj1200_COB_ID_Server_to_Client_Transmit_SDO;

extern UNS32 Ds402_Slave_obj1400_COB_ID_used_by_PDO;
extern UNS32 Ds402_Slave_obj1401_COB_ID_used_by_PDO;
extern UNS32 Ds402_Slave_obj1402_COB_ID_used_by_PDO;
extern UNS32 Ds402_Slave_obj1403_COB_ID_used_by_PDO;

extern UNS32 Ds402_Slave_obj1800_COB_ID_used_by_PDO;
extern UNS32 Ds402_Slave_obj1801_COB_ID_used_by_PDO;
extern UNS32 Ds402_Slave_obj1802_COB_ID_used_by_PDO;
extern UNS32 Ds402_Slave_obj1803_COB_ID_used_by_PDO;


void CANopen_Init(){
	
    SystemVar.CanopenID = 4;
	  SystemVar.CanopenBaudrate=1000;

    Ds402_Slave_obj1200_COB_ID_Client_to_Server_Receive_SDO = 0x600 + SystemVar.CanopenID;
    Ds402_Slave_obj1200_COB_ID_Server_to_Client_Transmit_SDO = 0x580 + SystemVar.CanopenID;

    Ds402_Slave_obj1400_COB_ID_used_by_PDO = 0x200 + SystemVar.CanopenID;
    Ds402_Slave_obj1401_COB_ID_used_by_PDO = 0x300 + SystemVar.CanopenID;
    Ds402_Slave_obj1402_COB_ID_used_by_PDO = 0x400 + SystemVar.CanopenID;
    Ds402_Slave_obj1403_COB_ID_used_by_PDO = 0x500 + SystemVar.CanopenID;

    Ds402_Slave_obj1800_COB_ID_used_by_PDO = 0x180 + SystemVar.CanopenID;
    Ds402_Slave_obj1801_COB_ID_used_by_PDO = 0x280 + SystemVar.CanopenID;
    Ds402_Slave_obj1802_COB_ID_used_by_PDO = 0x380 + SystemVar.CanopenID;
    Ds402_Slave_obj1803_COB_ID_used_by_PDO = 0x480 + SystemVar.CanopenID;	
	

	if(strcmp(SlaveBoard.baudrate, "none")){
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6000,1, TestSlave_ODCallback);
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6100,1, TestSlave_ODCallback);
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6200,1, TestSlave_ODCallback);
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6206,1, TestSlave_ODCallback);
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6040,0, TestSlave_ODCallback); //继电器回调函数，
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x60ff,0, TestSlave_ODCallback);
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6320,1, TestSlave_ODCallback);
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6401,1, TestSlave_ODCallback); //温度回调函数
        //RegisterSetODentryCallBack(&Ds402_Slave_Data,0x6411,1, TestSlave_ODCallback);
    
    if(!canOpen(&SlaveBoard,&Ds402_Slave_Data)){
        //SYS_DEBUG(("Cannot open Slave Board (%s,%s)\n",SlaveBoard.busname, SlaveBoard.baudrate));
        }
    }
    if(strcmp(SlaveBoard.baudrate, "none")) {
        //setNodeId(&Ds402_Slave_Data, SLAVE_NODE_ID);
			 
			  setNodeId(&Ds402_Slave_Data, SystemVar.CanopenID);
        //set_sync(0);
        //set_slave(&Ds402_Slave_Data, ROLE_SLAVE);    //设置当前节点为从设备
        /* init */
		    //lifeGuardInit(&Ds402_Slave_Data);			
        setState(&Ds402_Slave_Data, Initialisation);
        setState(&Ds402_Slave_Data, Pre_operational);
        setState(&Ds402_Slave_Data, Operational);
     }
}

// Canopen控制循环任务
void CANopen_Task(void){

				sendOnePDOevent(&Ds402_Slave_Data, 0);
				sendOnePDOevent(&Ds402_Slave_Data, 1);
				sendOnePDOevent(&Ds402_Slave_Data, 2);		
        //sendOnePDOevent(&Ds402_Slave_Data, 3);				 
				//proceedSYNC(&Ds402_Slave_Data);
}


// Initializes the timer, turn on the interrupt and put the interrupt time to zero
void initTimer(void)
{
    NVIC_InitType NVIC_InitStructure;
    TIM_TimeBaseInitType  TIM_TimeBaseStructure;

    /* TIM3 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Compute the prescaler value */
    /* SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f37x.c file */
    uint16_t PrescalerValue = (uint16_t) (SystemCoreClock  / 100000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.Period = 65535;
    TIM_TimeBaseStructure.Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.ClkDiv = 0;
    TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    TIM_ClrIntPendingBit(TIM3, TIM_INT_UPDATE);

    /* TIM3 disable counter */
    TIM_Enable(TIM3, DISABLE);

    /* Preset counter for a safe start */
    TIM_SetCnt(TIM3, 1);

    /* TIM Interrupts enable */
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);
}

//Set the timer for the next alarm.
void setTimer(TIMEVAL value)
{
    uint32_t timer = TIM_GetCnt(TIM3);        // Copy the value of the running timer
    elapsed_time += timer - last_counter_val;
    last_counter_val = 65535 - value;
    TIM_SetCnt(TIM3, 65535 - value);
    TIM_Enable(TIM3, ENABLE);
    //printf("setTimer %lu, elapsed %lu\r\n", value, elapsed_time);
}

//Return the elapsed time to tell the Stack how much time is spent since last call.
TIMEVAL getElapsedTime(void)
{
    uint32_t timer = TIM_GetCnt(TIM3);        // Copy the value of the running timer
    if(timer < last_counter_val)
        timer += 65535;
    TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
    //printf("elapsed %lu - %lu %lu %lu\r\n", elapsed, timer, last_counter_val, elapsed_time);
    return elapsed;
}

// This function handles Timer 3 interrupt request.
void TIM3_IRQHandler1(void) 
{
    //printf("--\r\n");
    if(TIM_GetFlagStatus(TIM3, TIM_FLAG_UPDATE) == RESET)
        return;
    last_counter_val = 0;
    elapsed_time = 0;
    TIM_ClrIntPendingBit(TIM3, TIM_FLAG_UPDATE);
    TimeDispatch();
}



//Initialize the CAN hardware
unsigned char canInit(CO_Data * d, uint32_t bitrate)
{
    GPIO_InitType  GPIO_InitStructure;
    NVIC_InitType  NVIC_InitStructure;
    CAN_InitType   CAN_InitStructure;
    CAN_FilterInitType  CAN_FilterInitStructure;

    /* save the canfestival handle */
    co_data = d;

    /* CAN GPIOs configuration **************************************************/

    /* Enable GPIO clock */
    GPIO_InitStruct(&GPIO_InitStructure);
    RCC_EnableAPB2PeriphClk(CAN_GPIO_CLK, ENABLE);

    /* Configure CAN RX and TX pins */
    GPIO_InitStructure.Pin = CAN_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(CAN_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(CAN_GPIO_PORT, &GPIO_InitStructure);

    /* CAN configuration ********************************************************/
    /* Enable CAN clock */
    RCC_EnableAPB1PeriphClk(CAN_CLK, ENABLE);

    /* CAN register init */
    CAN_DeInit(CANx);
    CAN_InitStruct(&CAN_InitStructure);

    /* CAN cell init */
    CAN_InitStructure.TTCM = DISABLE;      //是否使用时间触发功能
    CAN_InitStructure.ABOM = DISABLE;      //是否使用自动离线管理，使用的话可以在结点离线后，适时的自动恢复，不需要软件干预
    CAN_InitStructure.AWKUM = DISABLE;     //是否使用自动唤醒
    CAN_InitStructure.NART = ENABLE;//DISABLE;      //是否使用自动重传
    CAN_InitStructure.RFLM = ENABLE;//DISABLE;      //是否使用锁存接收，接收FIFO溢出时，不使能该功能，则新的会覆盖旧的。使能该功能，会丢弃新的数据。
    CAN_InitStructure.TXFP = ENABLE;//DISABLE;      //设置发送报文优先级判定方法，使能时以报文发送邮箱的先后顺序发送，不使能，按照ID优先级发送。
    CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
    CAN_InitStructure.RSJW = CAN_RSJW_1tq;

    /* CAN Baudrate (CAN clocked at 36 MHz)  36e6 / ( prescaler * (1+BS1+BS2))  */

//  CAN_InitStructure.RSJW = CAN_RSJW_1tq;
//  if(bitrate == 1000000){
//  	CAN_InitStructure.TBS1 = CAN_TBS1_7tq;
//  	CAN_InitStructure.TBS2 = CAN_TBS2_1tq;
//  }
//  else if(bitrate == 500000){
//  	CAN_InitStructure.TBS1 = CAN_TBS1_6tq;
//  	CAN_InitStructure.TBS2 = CAN_TBS2_1tq;
//  }
//  else{
// 	  CAN_InitStructure.TBS1 = CAN_TBS1_13tq;
//  	CAN_InitStructure.TBS2 = CAN_TBS2_2tq;
//  }
//  CAN_InitStructure.BaudRatePrescaler = 18;// 2*brp_from_birate(bitrate);

    CAN_InitStructure.RSJW = CAN_RSJW_1tq;

    if(bitrate == 1000000)
    {
        CAN_InitStructure.TBS1 = CAN_TBS1_7tq;
        CAN_InitStructure.TBS2 = CAN_TBS2_1tq;
        CAN_InitStructure.BaudRatePrescaler = 8;
    }
    else if(bitrate == 500000)
    {
        CAN_InitStructure.TBS1 = CAN_TBS1_6tq;
        CAN_InitStructure.TBS2 = CAN_TBS2_1tq;
        CAN_InitStructure.BaudRatePrescaler = 18;
    }
    else
    {
        CAN_InitStructure.TBS1 = CAN_TBS1_13tq; //100k
        CAN_InitStructure.TBS2 = CAN_TBS2_2tq;
        CAN_InitStructure.BaudRatePrescaler = 45;
    }

    CAN_Init(CANx, &CAN_InitStructure);

    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num = 0;
    CAN_FilterInitStructure.Filter_Mode = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId = 0x0000;
    CAN_FilterInitStructure.Filter_LowId = 0x0000;
    CAN_FilterInitStructure.FilterMask_HighId = 0x0000;
    CAN_FilterInitStructure.FilterMask_LowId = 0x0000;
    CAN_FilterInitStructure.Filter_FIFOAssignment = 0;
    CAN_FilterInitStructure.Filter_Act = ENABLE;
    CAN1_InitFilter(&CAN_FilterInitStructure);

    /* Enable FIFO 0 message pending Interrupt */
    CAN_INTConfig(CANx, CAN_INT_FMP0, ENABLE);

    /* NVIC configuration *******************************************************/
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; //原为CAN1_RX1_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return 1;
}

// The driver send a CAN message passed from the CANopen stack
unsigned char canSend(CAN_PORT notused, Message *m)
{
    int i, res;
    CanTxMessage TxMessage = {0};
    TxMessage.StdId = m->cob_id;
    TxMessage.IDE = CAN_ID_STD;

    if(m->rtr)
        TxMessage.RTR = CAN_RTRQ_REMOTE;
    else
        TxMessage.RTR = CAN_RTRQ_DATA;

    TxMessage.DLC = m->len;

    for(i = 0 ; i < m->len ; i++)
        TxMessage.Data[i] = m->data[i];

    res = CAN_TransmitMessage(CANx, &TxMessage);

//    if(res == CAN_STS_NO_MB)
//        return 0; 	// error
//    return 1;		// succesful
		
    i = 0;
    while((CAN_TransmitSTS(CANx, res) == CAN_TxSTS_Failed) && (i < 0X2FFF))i++;	//等待发送结束
    if(i >= 0X2FFF)return 0;
    return 1;				
}

//The driver pass a received CAN message to the stack
/*
unsigned char canReceive(Message *m)
{
}
*/
unsigned char canChangeBaudRate_driver( CAN_HANDLE fd, char* baud)
{
    return 0;
}

/**
  * @brief  This function handles CAN1 RX0 interrupt request.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    int i;
    CanRxMessage RxMessage = {0};
    Message rxm = {0};
    CAN_ReceiveMessage(CAN1, CAN_FIFO0, &RxMessage);

    // Drop extended frames
    if(RxMessage.IDE == CAN_ID_EXT)
        return;

    rxm.cob_id = RxMessage.StdId;

    if(RxMessage.RTR == CAN_RTRQ_REMOTE)
        rxm.rtr = 1;

    rxm.len = RxMessage.DLC;

    for(i = 0 ; i < rxm.len ; i++)
        rxm.data[i] = RxMessage.Data[i];

    canDispatch(co_data, &rxm);
}

void disable_it(void)
{
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, DISABLE);
    CAN_INTConfig(CANx, CAN_INT_FMP0, DISABLE);
}

void enable_it(void)
{
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);
    CAN_INTConfig(CANx, CAN_INT_FMP0, ENABLE);
}

CAN_PORT canOpen(s_BOARD *board, CO_Data * d)
{
    uint32_t baudrate = 0;

    baudrate = ((uint32_t)(SystemVar.CanopenBaudrate)) * 1000;
    canInit(d, baudrate);
    //initTimer();
    return (CAN_PORT)1;
}
