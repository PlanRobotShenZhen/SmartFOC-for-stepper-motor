#include "drv8323.h"
#include "ExternGlobals.h"

uint16_t read_value[5];
uint16_t w_read_value[5];
u16 SPI1_ReadWriteByte(u16 TxData) ;
extern void delay_Ms(uint16_t nms);
//extern volatile uint16_t flag;
void DRV8323_GPIO_Init(void)
{
    // SPI相关引脚
    RCC_EnableAPB2PeriphClk(DRC8323_SPI_CLK | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    // DRV相关引脚
    // GATE
    GPIO_InitStructure.Pin = DRV8323_GATE_EN_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(DRV8323_GATE_EN_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(DRV8323_GATE_EN_PORT, DRV8323_GATE_EN_PIN);
    // nFAULT
    GPIO_InitStructure.Pin = DRV8323_nFAULT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(DRV8323_nFAULT_PORT, &GPIO_InitStructure);
    GPIO_ConfigEXTILine(nFAULT_EXTI_PORT_SOURCE, nFAULT_EXTI_PIN_SOURCE);
    EXTI_InitStructure.EXTI_Line    = nFAULT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = nFAULT_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // CAL_EN
    GPIO_InitStructure.Pin = DRV8323_CAL_EN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(DRV8323_CAL_EN_PORT, &GPIO_InitStructure);
    GPIO_SetBits(DRV8323_CAL_EN_PORT, DRV8323_CAL_EN_PIN);
    delay_Ms(1000);
    delay_Ms(10);
    GPIO_ResetBits(DRV8323_CAL_EN_PORT, DRV8323_CAL_EN_PIN);
    delay_Ms(10);

}
// DRV故障外部中断
void nFAULT_EXTI_IRQHandler(void)
{
    if(RESET != EXTI_GetITStatus(nFAULT_EXTI_LINE))
    {
        EXTI_ClrITPendBit(nFAULT_EXTI_LINE);
//wjj        SystemError.SysErr = M_SYSERR_IPM;
    }
}

void DRV8323_SPI_Init(void)
{
    SPI_InitType SPI_InitStructure;
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_16BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_128;  //或16
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(DRC8323_SPI, &SPI_InitStructure);
    SPI_Enable(DRC8323_SPI, ENABLE);
}


// SPI读&写16bit函数 - 寄存器版本
// uint16_t SPI_ReadWrite16bit(uint16_t ReadAddr)
// {
// 	int SPITimeout = SPIT_FLAG_TIMEOUT;     // 等待超时时间
//     // 等待发送缓冲区为空，TXE 事件
//     while (SPI_I2S_GetStatus(DRC8323_SPI,  SPI_I2S_TE_FLAG ) == SET)
//     {
//       	if((SPITimeout--) == 0){
// 			return 0;
// 		}
//     }
//     // 写入数据寄存器，把要写入的数据写入发送缓冲区
//     WRITE_REG(DRC8323_SPI->DAT, ReadAddr);

//     SPITimeout = SPIT_FLAG_TIMEOUT;
//     // 等待接收缓冲区非空，RNE 事件
//     while (SPI_I2S_GetStatus(DRC8323_SPI, SPI_I2S_RNE_FLAG) == RESET)
//     {
// 		if((SPITimeout--) == 0){
// 			return 0;
// 		}
//     }
//     // 读取数据寄存器，获取接收缓冲区数据
//     return READ_REG(DRC8323_SPI->DAT);
// }

// DRV8323写入函数 16位
uint16_t SPI_WRITE_DRV8323(uint16_t RegAddr, uint16_t Data)
{
    uint16_t RxData;
    uint16_t ctrlWord = DRV8323_buildCtrlWord(CtrlMode_Write, (DRV8323_Address_e)RegAddr, Data);
    DRV8323_SPI_NSS_LOW();   // SPI1-nSCS 拉低使能
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");
    RxData = SPI1_ReadWriteByte(ctrlWord);
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");
    DRV8323_SPI_NSS_HIGH();  // SPI1-nSCS 拉高关断
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");

    return (RxData & DRV8323_DATA_MASK);
}

// DRV8323读取函数 16位
uint16_t SPI_READ_DRV8323(uint16_t RegAddr, uint16_t Data)
{
    uint16_t RxData;
    uint16_t ctrlWord = DRV8323_buildCtrlWord((DRV8323_CtrlMode_e)CtrlMode_Read, (DRV8323_Address_e)RegAddr, Data);
    DRV8323_SPI_NSS_LOW();    // SPI1-nSCS 拉低使能
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");
    RxData = SPI1_ReadWriteByte(ctrlWord);
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");
    DRV8323_SPI_NSS_HIGH();
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");

    return (RxData & DRV8323_DATA_MASK);
}


// 用SPI设置DRV8323的函数
void Set_DRV8323(void)
{
    DRV8323_GATE_EN_HIGH(); //使能Drv-Enable
    //DRV8323_GATE_EN_LOW();
    delay_Ms(50);
//	uint16_t state_0 = SPI_READ_DRV8323(Address_Status_0,0x000);

    //while(((state_0 & DRV8323_STATUS00_FAULT_BITS) != 0) || (DRV8323_do_checks() != 1)){};
    /*通信协议：
    	bit 15   : R/W    R=1，W=0
    	bit 14-11: 地址, 可写的地址为0x02到0x06
    	bit 10-0 : 具体的指令
    */
    // 只需要看低11位的数据
//	w_read_value[0] = SPI_WRITE_DRV8323(Address_Control_2,0x000); 	//address 02，写入000 0000 0000，默认值
//	w_read_value[1] = SPI_WRITE_DRV8323(Address_Control_3,0x3ff); 	//address 03，写入011 1111 1111，默认值
//	w_read_value[2] = SPI_WRITE_DRV8323(Address_Control_4,0x7ff); 	//address 04，写入111 1111 1111，默认值
//	w_read_value[3] = SPI_WRITE_DRV8323(Address_Control_5,0x169); 	//address 05，写入001 0110 1001，修改死区时间为200ns，默认100ns
//	w_read_value[4] = SPI_WRITE_DRV8323(Address_Control_6,0x023); 	//address 06，写入000 0010 0011，修改放大倍数为10V/V，默认为20V/V
//  delay_Ms(50);
//	read_value[0] = SPI_READ_DRV8323(Address_Status_0,0x000); //取地址0x02
//	read_value[1] = SPI_READ_DRV8323(Address_Status_1,0x000); //取地址0x03
//	read_value[0] = SPI_READ_DRV8323(Address_Control_2,0x000); //取地址0x02
//	read_value[1] = SPI_READ_DRV8323(Address_Control_3,0x000); //取地址0x03
//	read_value[2] = SPI_READ_DRV8323(Address_Control_4,0x000); //取地址0x04
//	read_value[3] = SPI_READ_DRV8323(Address_Control_5,0x000); //取地址0x05
//	read_value[4] = SPI_READ_DRV8323(Address_Control_6,0x000); //取地址0x06
//	delay_Ms(50);

//  delay_Ms(50);

    //DRV8323_GATE_EN_LOW();
}

// 检查DRV驱动器故障
uint16_t DRV8323_do_checks()
{
    uint8_t nFAULT_state = GPIO_ReadInputDataBit(DRV8323_nFAULT_PORT, DRV8323_nFAULT_PIN);
    if(nFAULT_state == (uint8_t)Bit_RESET)    //如果nFAULT引脚处于低电平，表示驱动器出现故障
    {
        return 0;
    };
    return 1;
}


u16 SPI1_ReadWriteByte(u16 TxData)                                        //SPI读写数据函数
{
    uint16_t retry = 0;
    /* Loop while DR register in not emplty */
    while(SPI_I2S_GetStatus(SPI1, SPI_I2S_TE_FLAG) == RESET)       //发送缓存标志位为空
    {
        retry++;
        if(retry > 0xfff0)return 0;
    }
    /* Send byte through the SPI1 peripheral */
    SPI_I2S_TransmitData(SPI1, TxData);                                    //通过外设SPI1发送一个数据
//	DRV8323_SPI_NSS_HIGH();   //可能需要在传输时将NSS置位
//	delay_Ms(3);
//	DRV8323_SPI_NSS_LOW();
    delay_Ms(3);
    retry = 0;
    /* Wait to receive a byte */
    while(SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET)    //接收缓存标志位不为空
    {
        retry++;
        if(retry > 0xfff0)return 0;
    }
    uint16_t recv = 1;
    recv = SPI_I2S_ReceiveData(SPI1);
    return recv;                                 //通过SPI2返回接收数据
}
