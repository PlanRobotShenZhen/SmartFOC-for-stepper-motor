//#include "SystemDefine.h"
#include "Spi.h"
#include "ExternGlobals.h"


extern short SpiReadState; 																	//SPI��ȡ״̬
extern short SpiReadState2; 																//SPI��ȡ״̬
u16 SpiBuffer[2] = {0}; 																		//�洢��ȡ����
short dattt = 0;
short dattt2 = 0;


void SPI3_IRQHandler(void)																	 //SPI3  ͨ�Ź���
{
    uint32_t tempdat = 0, tmpv = 0;
//	NoMg �ⲿ�ų�̫������		LPV ����Ƿѹ����		Over_Speed ת�ٹ��챨��			CRCV
	
    short 	NoMg = 0, LPV = 0, Over_Speed = 0, CRCV = 0;
    if(SPI_I2S_GetIntStatus(SPI3, SPI_I2S_INT_RNE) != RESET) 	//���SPI���ж��Ƿ���
    {
        switch(SpiReadState)																	//SPI��ȡ״̬
        {
            case 0:
            {
                SPI_I2S_ReceiveData(SPI3);										//����SPI3���ص���������
                SpiReadState = 1;															//
                MT6835_Read_Reg(0xFFFF);											//
            }
            break;

            case 1:
            {
                SpiBuffer[0] = SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 2;
                MT6835_Read_Reg(0xFFFF);
            }
            break;

            case 2:
            {
                SpiBuffer[1] = SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 0;
                CSn1_H;                //��ֹSPI����

                tmpv = SpiBuffer[0];
                tempdat = (tmpv << 16);
                tmpv = SpiBuffer[1];
                tempdat += tmpv;

                //��������
                if(tempdat & 0x00000400)
                    LPV = 1; //Ƿѹ������1����0����
                else
                    LPV = 0;

                if(tempdat & 0x00000200)
                    NoMg = 1; //��Ӵų�̫��1����0����
                else
                    NoMg = 0;

                if(tempdat & 0x00000100)
                    Over_Speed = 1; //������ʾ1����0����
                else
                    Over_Speed = 0;

                CRCV = (u8)tempdat;
                tempdat = (tempdat >> 11); //21λ
                NoMg = NoMg;
                LPV = LPV;
                Over_Speed = Over_Speed;
                CRCV = CRCV;

                if((LPV == 0) && (NoMg == 0) && (Over_Speed == 0))
                {
                    MotorControler.AngleFromMT6835 = (0x7fff - (tempdat >> 6));
                    //  MotorControler.AngleFromMT6835 = (tempdat >> 6);
                }

                SpiReadState 	= 0;
                SpiReadState2 = 0;
            }
            break;

            case 0x600E :
            {
                SpiBuffer[1] = SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 0x600F;
                MT6835_Read_Reg(0x7070);
            }
            break;

            case 0x600F :
            {
                SpiBuffer[1] = SPI_I2S_ReceiveData(SPI3);
                CSn1_H;
            }
            break;

            case  0x3113 :
            {
                SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 0x3114;
                MT6835_Read_Reg(0xFFFF);
            }
            break;

            case 0x3114 :
            {
                SpiBuffer[1] = SPI_I2S_ReceiveData(SPI3);
                MotorControler.MT6835_Cla_State = (SpiBuffer[1] >> 14);
                //dattt = SpiBuffer[1];
                SpiReadState = 0;
                CSn1_H;                //��ֹSPI����
            }
            break;

        }
    }
}

//΢��
void Delay_Us(unsigned int i)  //j=5,i=1ʱ��Լ600-700ns����
{
    unsigned int j;

    for(j = 5; j > 0; j--)
        for(; i > 0; i--);
}




void MT6835_Init(void)																	//�Ÿ�Ӧ������MT6835�ĳ�ʼ��
{
    NVIC_InitType NVIC_InitStructure;
    /* Configure and enable SPI_MASTER interrupt -------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel                   = SPI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	
	
    SPI_InitType  SPI_InitStructure;

    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen = SPI_DATA_SIZE_16BITS;//gj��
    SPI_InitStructure.CLKPOL = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CLKPHA = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_16;//SPI_BR_PRESCALER_256;
    SPI_InitStructure.FirstBit = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly = 7;
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_I2S_EnableInt(SPI3, SPI_I2S_INT_RNE, ENABLE); //ʹ�ܽ����ж�

    SPI_Enable(SPI3, ENABLE);
}






u16 SPI_ReadWriteByte(u16 TxData)                                        //SPI��д���ݺ���
{
    uint16_t retry = 0;

    /* Loop while DR register in not emplty */
    while(SPI_I2S_GetStatus(SPI3, SPI_I2S_TE_FLAG) == RESET)       				//���ͻ����־λΪ��
																																					//���ָ����SPI/I2S��־�Ƿ����á�
    {
        retry++;

        if(retry > 200)
            return 0;
    }

    /* Send byte through the SPI1 peripheral */
    SPI_I2S_TransmitData(SPI3, TxData);     				//ͨ������SPI3����һ������ TxData
    retry = 0;
    return 0;   																		//��ͨ��������ȡ�������ݣ�����ͨ���ж�
}







u16 MT6835_Read_Reg(u16 regaddr)
{
    u16 reg_val = 0;
    reg_val = SPI_ReadWriteByte(regaddr); 															//��ȡ�Ĵ�������
    return(reg_val);       																							//����״ֵ̬
}




/************************************������ʱ����***********************************************************/
void delay_Ms(uint16_t nms)   	  
{
    u32 i = 0, j = 0;

    for(i = 0; i < nms; i++)
    {
        for(j = 0; j < 7200; j++);
    }
}
