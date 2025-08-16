#include "drv8323.h"
#include "ExternGlobals.h"

uint16_t read_value[5];
uint16_t w_read_value[5];
u16 SPI1_ReadWriteByte(u16 TxData) ;
extern void delay_Ms(uint16_t nms);
//extern volatile uint16_t flag;
void DRV8323_GPIO_Init(void)
{
    // SPI�������
    RCC_EnableAPB2PeriphClk(DRC8323_SPI_CLK | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    // DRV�������
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
// DRV�����ⲿ�ж�
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
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_128;  //��16
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(DRC8323_SPI, &SPI_InitStructure);
    SPI_Enable(DRC8323_SPI, ENABLE);
}


// SPI��&д16bit���� - �Ĵ����汾
// uint16_t SPI_ReadWrite16bit(uint16_t ReadAddr)
// {
// 	int SPITimeout = SPIT_FLAG_TIMEOUT;     // �ȴ���ʱʱ��
//     // �ȴ����ͻ�����Ϊ�գ�TXE �¼�
//     while (SPI_I2S_GetStatus(DRC8323_SPI,  SPI_I2S_TE_FLAG ) == SET)
//     {
//       	if((SPITimeout--) == 0){
// 			return 0;
// 		}
//     }
//     // д�����ݼĴ�������Ҫд�������д�뷢�ͻ�����
//     WRITE_REG(DRC8323_SPI->DAT, ReadAddr);

//     SPITimeout = SPIT_FLAG_TIMEOUT;
//     // �ȴ����ջ������ǿգ�RNE �¼�
//     while (SPI_I2S_GetStatus(DRC8323_SPI, SPI_I2S_RNE_FLAG) == RESET)
//     {
// 		if((SPITimeout--) == 0){
// 			return 0;
// 		}
//     }
//     // ��ȡ���ݼĴ�������ȡ���ջ���������
//     return READ_REG(DRC8323_SPI->DAT);
// }

// DRV8323д�뺯�� 16λ
uint16_t SPI_WRITE_DRV8323(uint16_t RegAddr, uint16_t Data)
{
    uint16_t RxData;
    uint16_t ctrlWord = DRV8323_buildCtrlWord(CtrlMode_Write, (DRV8323_Address_e)RegAddr, Data);
    DRV8323_SPI_NSS_LOW();   // SPI1-nSCS ����ʹ��
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");
    RxData = SPI1_ReadWriteByte(ctrlWord);
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");
    DRV8323_SPI_NSS_HIGH();  // SPI1-nSCS ���߹ض�
    for(uint16_t n = 0; n < 0x10; n++)
        __asm(" NOP");

    return (RxData & DRV8323_DATA_MASK);
}

// DRV8323��ȡ���� 16λ
uint16_t SPI_READ_DRV8323(uint16_t RegAddr, uint16_t Data)
{
    uint16_t RxData;
    uint16_t ctrlWord = DRV8323_buildCtrlWord((DRV8323_CtrlMode_e)CtrlMode_Read, (DRV8323_Address_e)RegAddr, Data);
    DRV8323_SPI_NSS_LOW();    // SPI1-nSCS ����ʹ��
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


// ��SPI����DRV8323�ĺ���
void Set_DRV8323(void)
{
    DRV8323_GATE_EN_HIGH(); //ʹ��Drv-Enable
    //DRV8323_GATE_EN_LOW();
    delay_Ms(50);
//	uint16_t state_0 = SPI_READ_DRV8323(Address_Status_0,0x000);

    //while(((state_0 & DRV8323_STATUS00_FAULT_BITS) != 0) || (DRV8323_do_checks() != 1)){};
    /*ͨ��Э�飺
    	bit 15   : R/W    R=1��W=0
    	bit 14-11: ��ַ, ��д�ĵ�ַΪ0x02��0x06
    	bit 10-0 : �����ָ��
    */
    // ֻ��Ҫ����11λ������
//	w_read_value[0] = SPI_WRITE_DRV8323(Address_Control_2,0x000); 	//address 02��д��000 0000 0000��Ĭ��ֵ
//	w_read_value[1] = SPI_WRITE_DRV8323(Address_Control_3,0x3ff); 	//address 03��д��011 1111 1111��Ĭ��ֵ
//	w_read_value[2] = SPI_WRITE_DRV8323(Address_Control_4,0x7ff); 	//address 04��д��111 1111 1111��Ĭ��ֵ
//	w_read_value[3] = SPI_WRITE_DRV8323(Address_Control_5,0x169); 	//address 05��д��001 0110 1001���޸�����ʱ��Ϊ200ns��Ĭ��100ns
//	w_read_value[4] = SPI_WRITE_DRV8323(Address_Control_6,0x023); 	//address 06��д��000 0010 0011���޸ķŴ���Ϊ10V/V��Ĭ��Ϊ20V/V
//  delay_Ms(50);
//	read_value[0] = SPI_READ_DRV8323(Address_Status_0,0x000); //ȡ��ַ0x02
//	read_value[1] = SPI_READ_DRV8323(Address_Status_1,0x000); //ȡ��ַ0x03
//	read_value[0] = SPI_READ_DRV8323(Address_Control_2,0x000); //ȡ��ַ0x02
//	read_value[1] = SPI_READ_DRV8323(Address_Control_3,0x000); //ȡ��ַ0x03
//	read_value[2] = SPI_READ_DRV8323(Address_Control_4,0x000); //ȡ��ַ0x04
//	read_value[3] = SPI_READ_DRV8323(Address_Control_5,0x000); //ȡ��ַ0x05
//	read_value[4] = SPI_READ_DRV8323(Address_Control_6,0x000); //ȡ��ַ0x06
//	delay_Ms(50);

//  delay_Ms(50);

    //DRV8323_GATE_EN_LOW();
}

// ���DRV����������
uint16_t DRV8323_do_checks()
{
    uint8_t nFAULT_state = GPIO_ReadInputDataBit(DRV8323_nFAULT_PORT, DRV8323_nFAULT_PIN);
    if(nFAULT_state == (uint8_t)Bit_RESET)    //���nFAULT���Ŵ��ڵ͵�ƽ����ʾ���������ֹ���
    {
        return 0;
    };
    return 1;
}


u16 SPI1_ReadWriteByte(u16 TxData)                                        //SPI��д���ݺ���
{
    uint16_t retry = 0;
    /* Loop while DR register in not emplty */
    while(SPI_I2S_GetStatus(SPI1, SPI_I2S_TE_FLAG) == RESET)       //���ͻ����־λΪ��
    {
        retry++;
        if(retry > 0xfff0)return 0;
    }
    /* Send byte through the SPI1 peripheral */
    SPI_I2S_TransmitData(SPI1, TxData);                                    //ͨ������SPI1����һ������
//	DRV8323_SPI_NSS_HIGH();   //������Ҫ�ڴ���ʱ��NSS��λ
//	delay_Ms(3);
//	DRV8323_SPI_NSS_LOW();
    delay_Ms(3);
    retry = 0;
    /* Wait to receive a byte */
    while(SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET)    //���ջ����־λ��Ϊ��
    {
        retry++;
        if(retry > 0xfff0)return 0;
    }
    uint16_t recv = 1;
    recv = SPI_I2S_ReceiveData(SPI1);
    return recv;                                 //ͨ��SPI2���ؽ�������
}
