
#include "uart_interface.h"
#include "ExternGlobals.h"

extern int Istargetposition;  //�������Ŀ��λ�õı�־λ

uint32_t PROTOCOL_SERVER_TIMEOUT_MS = 10;
uint8_t tx_buf_[UART_TX_BUFFER_SIZE];

uint8_t RxBuffer[20];
uint8_t uart_cmd = 0;

uint8_t modbus_recv_data[MODBUS_RX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t modbus_send_data[MODBUS_TX_MAXBUFF] = { 0 };  // �������ݻ�����

uint32_t modbus_recv_flag = 0;        								// ������ɱ�־λ
uint32_t modbus_recv_len = 0;         								// ���յ����ݳ���
uint32_t modbus_send_len = 0;         								// ���͵����ݳ���
uint32_t modbus_send_flag = 0;        								// ������ɱ�־λ

void Modbus_USART_Init(unsigned int baud)			//
{

    GPIO_InitType GPIO_InitStucture;
    USART_InitType USART_InitStucture;
    NVIC_InitType NVIC_InitStucture;

    RCC_EnableAPB2PeriphClk(MODBUS_USART_GPIO_CLK, ENABLE);
    MODBUS_USART_APBxClkCmd(MODBUS_USART_CLK, ENABLE);
    GPIO_InitStruct(&GPIO_InitStucture);
    //GPIO���Ͷ˲��ø������������
    GPIO_InitStucture.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStucture.Pin = MODBUS_USART_TxPin;
    GPIO_InitPeripheral(MODBUS_USART_GPIO, &GPIO_InitStucture);
    //GPIO���ն˲��ø������룻
    GPIO_InitStucture.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStucture.Pin = MODBUS_USART_RxPin;
    GPIO_InitPeripheral(MODBUS_USART_GPIO, &GPIO_InitStucture);
    //485���Ϳ�������
    GPIO_InitStucture.Pin = RS485_DE_Pin;
    GPIO_InitStucture.GPIO_Mode = GPIO_Mode_Out_PP;   //�������
    GPIO_InitStucture.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(RS485_DE_GPIO, &GPIO_InitStucture);
    GPIO_ResetBits(RS485_DE_GPIO, RS485_DE_Pin);      //����Ϊ����ģʽ��Ĭ�Ͻ���

    //USART�ṹ������
    USART_DeInit(MODBUS_USART);
    USART_InitStucture.BaudRate = baud;
    USART_InitStucture.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStucture.Mode = USART_MODE_RX | USART_MODE_TX;
    USART_InitStucture.Parity = USART_PE_NO;
    USART_InitStucture.StopBits = USART_STPB_1;
    USART_InitStucture.WordLength = USART_WL_8B;

    USART_Init(MODBUS_USART, &USART_InitStucture);
    USART_EnableDMA(MODBUS_USART, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

    USART_Enable(MODBUS_USART, ENABLE);
    USART_ConfigInt(MODBUS_USART, USART_INT_IDLEF, ENABLE); //���������ж�

    //NVIC�Ĵ������ã��ж�����ͨ������ΪUSART3,����ʹ�ܣ���ռ���ȼ�Ϊ1���ӣ���Ӧ�����ȼ�Ϊ1
    NVIC_InitStucture.NVIC_IRQChannel = MODBUS_USART_IRQn;
    NVIC_InitStucture.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStucture.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStucture.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStucture);
}


void Modbus_DMA_Init(void)								
{
    DMA_InitType DMA_InitStruct;
    NVIC_InitType NVIC_InitStruct;

    RCC_EnableAHBPeriphClk(MODBUS_DMA_CLK, ENABLE);

    DMA_DeInit(MODBUS_DMA_TX_Channel);
    DMA_DeInit(MODBUS_DMA_RX_Channel);
    DMA_StructInit(&DMA_InitStruct);  //��Ĭ�ϳ�ʼ���ṹ��

    // ���� DMA1 ͨ��2, USART3_TX
    DMA_InitStruct.PeriphAddr = (uint32_t) & (MODBUS_USART->DAT); // USART_DR ��ַƫ�ƣ�0x04
    DMA_InitStruct.MemAddr = (uint32_t)modbus_send_data;  // �ڴ��ַ
    DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStruct.BufSize = 0; 			//�Ĵ���������Ϊ0ʱ��ͨ���Ƿ��������ᷢ�����ݴ���
    DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStruct.Priority = DMA_PRIORITY_LOW;
    DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(MODBUS_DMA_TX_Channel, &DMA_InitStruct);
    // ��ӳ��
    DMA_RequestRemap(MODBUS_DMA_TX_REMAP, MODBUS_DMA, MODBUS_DMA_TX_Channel, ENABLE);

    // ���� DMA1 ͨ��3, USART3_RX
    DMA_InitStruct.PeriphAddr = (uint32_t) & (MODBUS_USART->DAT); // (USART_DR) ��ַƫ�ƣ�0x04
    DMA_InitStruct.MemAddr = (uint32_t)modbus_recv_data;  // �ڴ��ַ
    DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;       // ���赽�ڴ�
    DMA_InitStruct.BufSize = MODBUS_RX_MAXBUFF;
    DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStruct.Priority = DMA_PRIORITY_LOW;
    DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(MODBUS_DMA_RX_Channel, &DMA_InitStruct);
    // ��ӳ��
    DMA_RequestRemap(MODBUS_DMA_RX_REMAP, MODBUS_DMA, MODBUS_DMA_RX_Channel, ENABLE);

    // ����DMA��������ж�
    NVIC_InitStruct.NVIC_IRQChannel = MODBUS_DMA_TX_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
    NVIC_Init(&NVIC_InitStruct);
    // ����MODBUS_DMA_TX_Channel��������ж�
    DMA_ConfigInt(MODBUS_DMA_TX_Channel, DMA_INT_TXC, ENABLE);

    //��ֹ���ͣ�ʹ�ܽ���
    DMA_EnableChannel(MODBUS_DMA_TX_Channel, DISABLE);
    DMA_EnableChannel(MODBUS_DMA_RX_Channel, ENABLE);
}

void Modbus_DMA_ReEnable(DMA_ChannelType* DMA_Channel, uint16_t len)
{
    DMA_EnableChannel(DMA_Channel, DISABLE);
    DMA_SetCurrDataCounter(DMA_Channel, len);
    DMA_EnableChannel(DMA_Channel, ENABLE);
}

// DMA������ɻص�
void MODBUS_DMA_TX_IRQHandler(void)
{
    if (DMA_GetIntStatus(MODBUS_DMA_TX_INT_TXC, MODBUS_DMA) != RESET) // �������
    {
        DMA_ClrIntPendingBit(MODBUS_DMA_TX_INT_TXC, MODBUS_DMA);     // ����ж�
        SET_RS485_RX_ENABLE;                         // ���ñ�־
        DMA_EnableChannel(MODBUS_DMA_TX_Channel, DISABLE); //ʧ��DMAͨ��
        Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel, MODBUS_RX_MAXBUFF);
    }
}

//�����жϻص�
void MODBUS_USART_IRQHandler(void)
{
    if(USART_GetIntStatus(MODBUS_USART, USART_INT_IDLEF) != RESET)
    {
        MODBUS_USART->STS;
        MODBUS_USART->DAT;
        modbus_recv_flag = 1;                // ���ձ�־��1
        modbus_recv_len = MODBUS_RX_MAXBUFF - DMA_GetCurrDataCounter(MODBUS_DMA_RX_Channel);// ͳ���յ������ݵĳ���
        //ʧ��ͨ�����������ý��ճ���Ϊ�����ʹ��ͨ��
        Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel, MODBUS_RX_MAXBUFF);
    }
}

// ����ȫ��ֻ������
//  �����¶�   ����ת��

void Update_Modbus_RO_Data(void)
{
    uint16_t* pdu = getPDUData();

    pdu[0] = SystemError.SysErr;                               //�������
    pdu[1] = SystemError.PwmOn;                                //�������״̬ ��ʹ�� ��ʹ��
    pdu[2] = SystemError.RuningModeFdb;                        //��ǰ����ģʽ
    pdu[3] = 0;                                             	 //�������
    pdu[4] = MotorControler.SpeedFdbpFilter3;                  //����ٶ�
    pdu[5] = (MotorControler.MotorActivePostion) >> 16;        //�����ǰλ��
    pdu[6] = (MotorControler.MotorActivePostion) & 0xffff;     //�����ǰλ��
    pdu[7] = MotorControler.MT6835_Cla_State;                  //
    pdu[8] = SystemError.DcVolt;                               //ĸ�ߵ�ѹ
    pdu[9] = 0;


    pdu[10] = SystemVar.SoftwareVersion;                  			//����汾
    pdu[11] = SystemVar.HardwareVersion;                  			//Ӳ���汾
    pdu[12] = SystemVar.MotorVersion;                     			//����ͺ�
	
    //�������
	  pdu[13] = svpwm.As;
    pdu[14] = svpwm.Bs;
    pdu[15] = 103;
		
//	  pdu[13] = 101;
//    pdu[14] = 102;
//    pdu[15] = 103;
	
    pdu[16] = SystemVar.ModbusID;																//MODBUS ID			
    pdu[17] = SystemVar.ModbusBaudrate;													//MOUBUS ������
		
    pdu[18] = SystemVar.CanopenID;															//CANOPEN ID
    pdu[19] = SystemVar.CanopenBaudrate;												//CANOPEN ������

    //��������
    pdu[20] = SystemError.DcRefMax;                 //���ĸ�ߵ�ѹ
    pdu[21] = SystemError.DcRefMin;                 //��Сĸ�ߵ�ѹ
    pdu[22] = SystemError.ImeMax;                   //��������
    pdu[23] = SystemError.ImeMax;;                  //������������� ------------
    pdu[24] = SystemError.OverSpeed;                //λ�û�����޷� ----------
    pdu[25] = SystemError.OverSpeed;                //�ٶȻ�����޷�
    pdu[26] = SystemError.OverLoadTimer;            //d ��������޷� ------
    pdu[27] = 105;                                  //��ѹʸ���޷�   --------------

    pdu[28] = pidc_position_Kp;                      //λ�û���һ����
    pdu[29] = pidc_position_Ki;                      //λ�û��ڶ�����
    pdu[30] = 10;                                    //λ�û��ٶ�ǰ��ϵ��---------------
    pdu[31] = 100;                                   //�ٶ�ǰ����ͨ�˲�����-----------
    pdu[32] = pidpv_Kp;                              //�ٶȻ���һ����
    pdu[33] = 101;                                   //�ٶȻ��ڶ�����       ------------
    pdu[34] = pidpv_Ki;                              //�ٶȻ���һ����
    pdu[35] = 103;                                   //�ٶȻ��ڶ�����            --------
    pdu[36] = 104;                                   //���ŵֿ�����       --------
    pdu[37] = SystemError.ThresholdCoeff ;           //���ٶ�ǰ��ϵ�� ------------
    pdu[38] = SystemError.TorquePIDLimitCoeff;       //���ٶ�ǰ����ͨ�˲����� ----------

    pdu[39] = pidpt_Kp;                              //����������
    pdu[40] = pidpt_Ki;                              //����������
    pdu[41] = 100;                                   //�������� -----------
    pdu[42] = 101;                                   //����ת��������----------
    pdu[43] = 101;                                   //�ն�ϵ��-----------
    pdu[44] = 102;                                   //�˲�����������-----------
    pdu[45] = 103;                                   //�ٶȸ����˲�����----------
    pdu[46] = 104;                                   //�ٶȷ����˲�����----------
    pdu[47] = 105;                                   //���������˲�����---------
    pdu[48] = 106;                                   //���������˲�����--------
    pdu[49] = 107;                                   //ת���ݲ���Ƶ��------------
    pdu[40] = 10;                                    //ת���ݲ������---------
    pdu[41] = 100;                                   //ת���ݲ������------------
    pdu[42] = 101;                                   //�˲�����������---------
    pdu[43] = 101;                                   //Jerk ƽ��ϵ��-------
    pdu[44] = 102;                                   //���ӳ��ַ���-------
    pdu[45] = 103;                                   //���ӳ��ַ�ĸ---------
    pdu[46] = 104;                                   //Ĭ�Ϸ���-------
    pdu[47] = 105;                                   //ɲ������ʱ-------
    pdu[48] = 106;                                   //ɲ���ر���ʱ---------
    pdu[49] = 107;                                   //й�Ŵ���ֵ-----
    pdu[50] = 10;                                    //й�Źر���ֵ--------
    pdu[51] = 100;                                   //��λ���-----------
    pdu[52] = 101;                                   //��λʱ��----------
    pdu[53] = 101;                                   //������ֵ-------
    pdu[54] = 102;                                   //��������˲�------------
    pdu[55] = 103;                                   //����λ����-------
    pdu[56] = 104;                                   //ת��ģʽ���ٶ��޷�----------
    pdu[57] = 105;                                   //������Ӽ���------------
		
    // �˶�����
    pdu[58] = 106;                                   //����ָ��
    pdu[59] = 107;                                   //����ģʽ
    pdu[60] = 10;                                    //λ��ģʽĿ��λ�ø�λ
    pdu[61] = 100;                                   //λ��ģʽĿ���λ
    pdu[62] = 101;                                   //λ��ģʽĿ���ٶȸ�λ
    pdu[63] = 101;                                   //λ��ģʽĿ���ٶȵ�λ
    pdu[64] = 102;                                   //λ��ģʽ����ʱ���λ
    pdu[65] = 103;                                   //λ��ģʽ����ʱ���λ
    pdu[66] = 104;                                   //λ��ģʽ����ʱ���λ
    pdu[67] = 105;                                   //λ��ģʽ����ʱ���λ
    pdu[68] = 106;                                   //�ٶ�ģʽĿ���ٶȸ�λ
    pdu[69] = 107;                                   //�ٶ�ģʽĿ���ٶȵ�λ
    pdu[70] = 10;                                    //����ģʽ
    pdu[71] = 100;                                   //�����ٶȸ�λ
    pdu[72] = 101;                                   //�����ٶȵ�λ
    pdu[73] = 101;                                   //�����ѯ�ٶȸ�λ
    pdu[74] = 102;                                   //�����ѯ�ٶȵ�λ
    pdu[75] = 103;                                   //����Ӽ��ٸ�λ
    pdu[76] = 104;                                   //����Ӽ��ٵ�λ
    pdu[77] = 105;                                   //����ƫ�Ƹ�λ
    pdu[78] = 106;                                   //����ƫ�Ƶ�λ
    pdu[79] = 107;                                   //��������λ��λ
    pdu[80] = 10;                                    //��������λ��λ
    pdu[81] = 100;                                   //��������λ��λ
    pdu[82] = 101;                                   //��������λ��λ
    pdu[83] = 101;                                   //Ŀ��ת��
    pdu[84] = 102;                                   //Ŀ��ת���޷�
    pdu[85] = 103;                                   //����ת��
    pdu[86] = 104;                                   //ת��б�¸�λ
    pdu[87] = 105;                                   //ת��б�µ�λ
    pdu[88] = 106;
    pdu[89] = 107;
    pdu[90] = Istargetposition;
    pdu[91] = 100;
    pdu[92] = 101;
    pdu[93] = 101;
    pdu[94] = 102;
    pdu[95] = 103;
    pdu[96] = 104;
    pdu[97] = 105;
    pdu[98] = 106;
    pdu[99] = 106;
    pdu[100] = 0;//MotorControler.spiread_anlge;
    pdu[101] = 0;//MotorControler.AngleFromMT6835;
    pdu[102] = 0;//MotorControler.AngleFromMT6835Offset;
    pdu[103] = 0;//MotorControler.AngleFromMT6835Offset1;
    pdu[104] = SystemVar.VF_Voltage;
    pdu[105] = SystemVar.VF_ElectricalAngle;
    pdu[106] = SystemVar.VF_Coefficient;
    pdu[107] = SystemVar.VF_Coefficient_B;
    pdu[108] = SystemVar.VF_ElectricalAngleStepMax;
    pdu[109] = SystemVar.VF_ElectricalAngleStepMin;
    pdu[110] = SystemVar.VF_ElectricalAngleStep;
    pdu[111] = SystemVar.VF_ElectricalAngleStepCount;
    pdu[112] = SystemVar.test_uqs;
    pdu[113] = SystemVar.test_angle;
    pdu[114] = 0;

    pdu[115] = 0;
    pdu[116] = SystemVar.ModbusID;
    pdu[117] = SystemVar.ModbusBaudrate;
    pdu[118] = SystemVar.CanopenID;
    pdu[119] = SystemVar.CanopenBaudrate;
    pdu[120] = 10;
    pdu[121] = 100;
    pdu[122] = 101;
    pdu[123] = 101;
    pdu[124] = 102;
    pdu[125] = 103;
    pdu[126] = 104;
    pdu[127] = 105;
    pdu[128] = 106;
    pdu[129] = 107;
    pdu[130] = 10;
    pdu[131] = 100;
    pdu[132] = 101;
    pdu[133] = 101;
    pdu[134] = 102;
    pdu[135] = 103;
    pdu[136] = 104;
    pdu[137] = 105;
    pdu[138] = 106;
    pdu[139] = 107;
    pdu[140] = 10;
    pdu[141] = 100;
    pdu[142] = 101;
    pdu[143] = 101;
    pdu[144] = 102;
    pdu[145] = 103;
    pdu[146] = 104;
    pdu[147] = 105;
    pdu[148] = 106;
    pdu[149] = 107;
    pdu[140] = 10;
    pdu[141] = 100;
    pdu[142] = 101;
    pdu[143] = 101;
    pdu[144] = 102;
    pdu[145] = 103;
    pdu[146] = 104;
    pdu[147] = 105;
    pdu[148] = 106;
    pdu[149] = 107;
    pdu[150] = SystemError.SpeedFdbpPost;
    pdu[151] = SystemError.MotorRunFlag;
    pdu[152] = SystemError.OverLoadTimer;
    pdu[153] = SystemError.CoderTimer;
    pdu[154] = SystemError.DcCoeff;
    pdu[155] = SystemError.PositiveDcCoeff;
    pdu[156] = SystemError.IqsMax;
    pdu[157] = SystemError.IdsMax;
    pdu[158] = SystemError.IqsMaxOverTime;
    pdu[159] = SystemError.DcRef;
    pdu[160] = SystemError.DcRefK;
    pdu[161] = SystemError.DcRefB;
    pdu[162] = SystemError.AdDcVolt;
    pdu[163] = SystemError.SysErr;
    pdu[164] = SystemError.LowSpeedTimer;
    pdu[165] = SystemError.DcVolt;
    pdu[166] = SystemError.DcRefMax;
    pdu[167] = SystemError.DcMaxErrRef;
    pdu[168] = SystemError.DcRefMin;
    pdu[169] = SystemError.DcMinErrRef;
    pdu[170] = SystemError.DcArrestMax;
    pdu[171] = SystemError.DcArrestDelayMax;
    pdu[172] = SystemError.DcArrest2DelayMax;
    pdu[173] = SystemError.ImeMax;
    pdu[174] = SystemError.LossACTimeMax;
    pdu[175] = SystemError.NoSysErrPowerOff;
    pdu[176] = SystemError.ImeasCheckDelay;
    pdu[177] = SystemError.SysErrRuningAsk;
    pdu[178] = SystemError.SysErrRuning;
    pdu[179] = SystemError.SysResetVar;
    pdu[180] = SystemError.SysErrSaveLock;
    pdu[181] = SystemError.SaveAllRsetFlag;
    pdu[182] = SystemError.RuningMode;
    pdu[183] = SystemError.SaveAllParaFlag;
    pdu[184] = SystemError.OverSpeed;
    pdu[185] = 103;
    pdu[186] = 104;
    pdu[187] = 105;
    pdu[188] = 106;
    pdu[189] = 107;
    pdu[190] = 10;
    pdu[191] = 100;
    pdu[192] = 101;
    pdu[193] = 101;
    pdu[194] = 102;
    pdu[195] = 103;
    pdu[196] = 104;
    pdu[197] = 105;
    pdu[198] = 107;
    pdu[199] = 106;
    pdu[200] = MotorControler.State;
    pdu[201] = MotorControler.SpeedRef;
    pdu[202] = MotorControler.SpeedFdb;
    pdu[203] = MotorControler.TorqueRef;
    pdu[204] = MotorControler.TorqueFdb;
    pdu[205] = MotorControler.PositionRef;
    pdu[206] = MotorControler.PositionFdb;
    pdu[207] = MotorControler.MotorPoles;
    pdu[208] = MotorControler.MotorActivePostion;
    pdu[209] = MotorControler.RotorCount;
    pdu[210] = MotorControler.RatedTorque;
    pdu[211] = MotorControler.MaxTorque;
    pdu[212] = MotorControler.MaxSpeed;
    pdu[213] = MotorControler.SpeedAcc;
    pdu[214] = MotorControler.SpeedDcc;

    pdu[215] = 103;
    pdu[216] = 104;
    pdu[217] = 105;
    pdu[218] = 106;
    pdu[219] = 107;
    pdu[220] = 10;
    pdu[221] = 100;
    pdu[222] = 101;
    pdu[223] = 101;
    pdu[224] = 102;
    pdu[225] = 103;
    pdu[226] = 104;
    pdu[227] = 105;
    pdu[228] = 106;
    pdu[229] = 107;
    pdu[230] = 10;
    pdu[231] = 100;
    pdu[232] = 101;
    pdu[233] = 101;
    pdu[234] = 102;
    pdu[235] = 103;
    pdu[236] = 104;
    pdu[237] = 105;
    pdu[238] = 106;
    pdu[239] = 107;
    pdu[240] = 10;
    pdu[241] = 100;
    pdu[242] = 101;
    pdu[243] = 101;
    pdu[244] = 102;
    pdu[245] = 103;
    pdu[246] = 104;
    pdu[247] = 105;
    pdu[248] = 106;
    pdu[249] = 107;
    pdu[240] = 10;
    pdu[241] = 100;
    pdu[242] = 101;
    pdu[243] = 101;
    pdu[244] = 102;
    pdu[245] = 103;
    pdu[246] = 104;
    pdu[247] = 105;
    pdu[248] = 106;
    pdu[249] = 107;
    pdu[250] = 10;
    pdu[251] = 100;
    pdu[252] = 101;
    pdu[253] = 101;
    pdu[254] = 102;
    pdu[255] = 103;
    pdu[256] = 104;
    pdu[257] = 105;
    pdu[258] = 106;
    pdu[259] = 107;
    pdu[260] = 10;
    pdu[261] = 100;
    pdu[262] = 101;
    pdu[263] = 101;
    pdu[264] = 102;
    pdu[265] = 103;
    pdu[266] = 104;
    pdu[267] = 105;
    pdu[268] = 106;
    pdu[269] = 107;
    pdu[270] = 10;
    pdu[271] = 100;
    pdu[272] = 101;
    pdu[273] = 101;
    pdu[274] = 102;
    pdu[275] = 103;
    pdu[276] = 104;
    pdu[277] = 105;
    pdu[278] = 106;
    pdu[279] = 107;
    pdu[280] = 10;
    pdu[281] = 100;
    pdu[282] = 101;
    pdu[283] = 101;
    pdu[284] = 102;
    pdu[285] = 103;
    pdu[286] = 104;
    pdu[287] = 105;
    pdu[288] = 106;
    pdu[289] = 107;
    pdu[290] = 10;
    pdu[291] = 100;
    pdu[292] = 101;
    pdu[293] = 101;
    pdu[294] = 102;
    pdu[295] = 103;
    pdu[296] = 104;
    pdu[297] = 105;
    pdu[298] = UartMode.TargetPosition >> 16;  // ��ȡ�� 16 λ���洢�� pdu[298]
    pdu[299] = UartMode.TargetPosition & 0xFFFF;         // ��ȡ�� 16 λ���洢�� pdu[299]
    pdu[306] = (UartMode.Speed * 1000) >> 16; // ��ȡ�� 16 λ���洢�� pdu[306]
    pdu[307] = (UartMode.Speed * 1000) & 0xFFFF;       // ��ȡ�� 16 λ���洢�� pdu[307]
    pdu[310] = (UartMode.Torque * 10000) >> 16;
    pdu[311] = (UartMode.Torque * 10000) & 0xFFFF;
    pdu[312] = UartMode.DisEnable + 1;
    pdu[314] = SystemError.SaveAllRsetFlag;
    pdu[315] = 1;//Ŀ�ⲻ���޸�
    pdu[316] = 1;//Ŀ�ⲻ���޸�
    pdu[317] = SystemError.RuningMode - 1;
    pdu[318] = 1;//Ŀ�ⲻ���޸�

    pdu[351] = Uds_OutMax;
    pdu[352] = Uds_OutMin;
    pdu[353] = Uds_UiMax >> 16;
    pdu[354] = Uds_UiMax & 0xFFFF;
    pdu[355] = Uds_UiMin >> 16;
    pdu[356] = Uds_UiMin & 0xFFFF;
    pdu[357] = Uds_Kp;
    pdu[358] = Uds_Ki;
    pdu[371] = Uqs_OutMax;
    pdu[372] = Uqs_OutMin;
    pdu[373] = Uqs_UiMax >> 16;
    pdu[374] = Uqs_UiMax & 0xFFFF;
    pdu[375] = Uqs_UiMin >> 16;
    pdu[376] = Uqs_UiMin & 0xFFFF;
    pdu[377] = Uqs_Kp;
    pdu[378] = Uqs_Ki;
    pdu[391] = pidpt_OutMax;
    pdu[392] = pidpt_OutMin;
    pdu[393] = pidpt_UiMax >> 16;
    pdu[394] = pidpt_UiMax & 0xFFFF;
    pdu[395] = pidpt_UiMin >> 16;
    pdu[396] = pidpt_UiMin & 0xFFFF;
    pdu[397] = pidpt_Kp;
    pdu[398] = pidpt_Ki;
    pdu[411] = pidpv_OutMax;
    pdu[412] = pidpv_OutMin;
    pdu[413] = pidpv_UiMax >> 16;
    pdu[414] = pidpv_UiMax & 0xFFFF;
    pdu[415] = pidpv_UiMin >> 16;
    pdu[416] = pidpv_UiMin & 0xFFFF;
    pdu[417] = pidpv_Kp;
    pdu[418] = pidpv_Ki;
    pdu[431] = pidholding_OutMax;
    pdu[432] = pidholding_OutMin;
    pdu[433] = pidholding_UiMax >> 16;
    pdu[434] = pidpv_UiMax & 0xFFFF;
    pdu[435] = pidholding_UiMin >> 16;
    pdu[436] = pidholding_UiMin & 0xFFFF;
    pdu[437] = pidholding_Kp;
    pdu[438] = pidholding_Ki;
    pdu[451] = pidc_position_OutMax;
    pdu[452] = pidc_position_OutMin;
    pdu[453] = pidc_position_UiMax >> 16;
    pdu[454] = pidc_position_UiMax & 0xFFFF;
    pdu[455] = pidc_position_UiMin >> 16;
    pdu[456] = pidc_position_UiMin & 0xFFFF;
    pdu[457] = pidc_position_Kp;
    pdu[458] = pidc_position_Ki;
    pdu[471] = PostionPlan_accel_max;
    pdu[472] = PostionPlan_a_accel;
    pdu[473] = PostionPlan_a_decel;
    pdu[474] = PostionPlan_decel_max;
    pdu[475] = PostionPlan_vel_init;
    pdu[476] = PostionPlan_vel_tar;
    pdu[488] = 175;
}

uint16_t test = 0;
int test222 = 0;
// �����յ���������¶�Ӧ��ַ�Ĳ���
extern short CiA402_CMD;
extern int CiA402_SpeedRef;
short CONTROL_MODE_t = 0;
extern short PostionPlan_vel_tar;

void Modbus_Respond(MBModify* modify)
{
    if (modify->is_modify == 0)return;

    uint16_t* pdu = getPDUData();

    switch (modify->modify_addr)
    {
 
        case 16:
            SystemVar.ModbusID = pdu[16];
            break;

			case 28 :
            pidc_position_Kp = pdu[28];
            break;

        case 29 :
            pidc_position_Ki = pdu[29];
            break;

        case 30 :
            pidc_position_Kp = pdu[30];
            break;

        case 31 :
            pidc_position_Kp = pdu[31];
            break;

        case 32 :
            pidpv_Kp = pdu[32];
            break;

        case 33 :
            pidc_position_Kp = pdu[33];
            break;

        case 34 :
            pidpv_Ki = pdu[34];
            break;

        case 35 :
            pidc_position_Kp = pdu[35];
            break;

        case 36 :
            pidc_position_Kp = pdu[36];
            break;

        case 37 :
            SystemError.ThresholdCoeff = pdu[37];
            break;

        case 38 :
            SystemError.TorquePIDLimitCoeff = pdu[38];
            break;

        case 100:

        case 101:

        case 102:

        case 103:

        case 104:
            SystemVar.VF_Voltage = pdu[104];

        case 105:
            SystemVar.VF_ElectricalAngle = pdu[105];

        case 106:
            SystemVar.VF_Coefficient = pdu[106];

        case 107:
            SystemVar.VF_Coefficient_B = pdu[107];

        case 108:
            SystemVar.VF_ElectricalAngleStepMax = pdu[108];

        case 109:
            SystemVar.VF_ElectricalAngleStepMin = pdu[109];

        case 110:
            SystemVar.VF_ElectricalAngleStep = pdu[110];

        case 111:
            SystemVar.VF_ElectricalAngleStepCount = pdu[111];

        case 112:
            SystemVar.test_uqs = pdu[112];

        case 113:
            SystemVar.test_angle = pdu[113];

        case 114:


        case 115:


        case 116:
            SystemVar.ModbusID = pdu[116];

        case 117:
            SystemVar.ModbusBaudrate = pdu[117];

        case 118:
            SystemVar.CanopenID = pdu[118];

        case 119:
            SystemVar.CanopenBaudrate = pdu[119];

        case 150:
            SystemError.SpeedFdbpPost = pdu[150];

        case 151:
            SystemError.MotorRunFlag = pdu[151];

        case 152:
            SystemError.OverLoadTimer = pdu[152];

        case 153:
            SystemError.CoderTimer = pdu[153];

        case 154:
            SystemError.DcCoeff = pdu[154];

        case 155:
            SystemError.PositiveDcCoeff = pdu[155];

        case 156:
            SystemError.IqsMax = pdu[156];

        case 157:
            SystemError.IdsMax = pdu[157];

        case 158:
            SystemError.IqsMaxOverTime = pdu[158];

        case 159:
            SystemError.DcRef = pdu[159];

        case 160:
            SystemError.DcRefK = pdu[160];

        case 161:
            SystemError.DcRefB = pdu[161];

        case 162:
            SystemError.AdDcVolt = pdu[162];

        case 163:
            SystemError.SysErr = pdu[163];

        case 164:
            SystemError.LowSpeedTimer = pdu[164];

        case 165:
            SystemError.DcVolt = pdu[165];

        case 166:
            SystemError.DcRefMax = pdu[166];

        case 167:
            SystemError.DcMaxErrRef = pdu[167];

        case 168:
            SystemError.DcRefMin = pdu[168];

        case 169:
            SystemError.DcMinErrRef = pdu[169];

        case 170:
            SystemError.DcArrestMax = pdu[170];

        case 171:
            SystemError.DcArrestDelayMax = pdu[171];

        case 172:
            SystemError.DcArrest2DelayMax = pdu[172];

        case 173:
            SystemError.ImeMax = pdu[173];

        case 174:
            SystemError.LossACTimeMax = pdu[174];

        case 175:
            SystemError.NoSysErrPowerOff = pdu[175];

        case 176:
            SystemError.ImeasCheckDelay = pdu[176];

        case 177:
            SystemError.SysErrRuningAsk = pdu[177];

        case 178:
            SystemError.SysErrRuning = pdu[178];

        case 179:
            SystemError.SysResetVar = pdu[179];

        case 180:
            SystemError.SysErrSaveLock = pdu[180];

//			case 181:
//				SystemError.SaveAllRsetFlag=pdu[181];
        case 182:
            SystemError.RuningMode = pdu[182];

//			case 183:
//				SystemError.SaveAllParaFlag=pdu[183];
        case 184:
            SystemError.OverSpeed = pdu[184];

        case 200:
            MotorControler.State = pdu[200];

        case 201:
            MotorControler.SpeedRef = pdu[201];

        case 202:
            MotorControler.SpeedFdb = pdu[202];

        case 203:
            MotorControler.TorqueRef = pdu[203];

        case 204:
            MotorControler.TorqueFdb = pdu[204];

        case 205:
            MotorControler.PositionRef = pdu[205];

        case 206:
            MotorControler.PositionFdb = pdu[206];

        case 207:
            MotorControler.MotorPoles = pdu[207];

        case 208:
            MotorControler.MotorActivePostion = pdu[208];

        case 209:
            MotorControler.RotorCount = pdu[209];

        case 210:
            MotorControler.RatedTorque = pdu[210];

        case 211:
            MotorControler.MaxTorque = pdu[211];

        case 212:
            MotorControler.MaxSpeed = pdu[212];

        case 213:
            MotorControler.SpeedAcc = pdu[213];

        case 214:
            MotorControler.SpeedDcc = pdu[214];

        case 215:
            MotorControler.spiread_anlge = pdu[215];

        case 216:
            MotorControler.AngleFromMT6835 = pdu[216];

        case 217:
            MotorControler.AngleFromMT6835Offset = pdu[217];

        case 218:
            MotorControler.AngleFromMT6835Offset1 = pdu[218];

        case 263:
            CONTROL_MODE_t = pdu[263];
            UartMode.Mode = CONTROL_MODE_t + 1;
            break;

        case 298:
        case 299:
            UartMode.TargetPosition =   (((int)(pdu[298])) << 16) + (int)(pdu[299]);
            //PostionPlan_vel_tar = pdu[301]; //ע���ٶȷ���
            break;

        case 300 :       //����ٶ�
            break;

        case 301 :       //���ٶ�
            break;

        case 302 :       //�Ӽ��ٶ�
            break;

        case 303 :       //���ٶ�
            break;

        case 304 :       //���ٶ�
            break;

        case 305 :       //��ʱδ��
            break;



        case 306 :
        case 307 :
            test222 = ((int)(pdu[306]) * 65536) + (int)(pdu[307]);
            UartMode.Speed = test222 / 1000;
            break;

        case 308 : //����ʱ��
            break;

        case 309 : //����ʱ��
            break;


        case 310:
        case 311:
            test222 = ((int)(pdu[310]) * 65536) + (int)(pdu[311]);
            UartMode.Torque = test222 / 10000;
            break;

        case 312:
            test = pdu[312];

            if(test == 1)
            {
                UartMode.Enable = 1;
                UartMode.DisEnable = 0;
            }

            if(test == 2)
            {
                UartMode.Enable = 0;
                UartMode.DisEnable = 1;
            }

            break;

        case 314:
            SystemError.SaveAllRsetFlag = pdu[314];
            break;

        case 315:
            UartMode.Start = 1;//?
            break;

        case 316:
            UartMode.Stop = 1;//?
            break;

        case 317:
            SystemError.RuningMode = pdu[317] + 1;
            break;

        case 318:
            SystemError.SaveAllParaFlag = 1;
            break;

        case 319:
            SystemError.SysClearFlag = 1;
            break;


        /***************************** PID����д�룬��351��ʼ******************************/
        // Uds PID
        case UdsPID_STARTINDEX:
            Uds_OutMax = pdu[UdsPID_STARTINDEX];
            break;

        case UdsPID_STARTINDEX+1:
            Uds_OutMin = pdu[UdsPID_STARTINDEX + 1];
            break;

        case UdsPID_STARTINDEX+2:
        case UdsPID_STARTINDEX+3:
            Uds_UiMax = (((int)pdu[UdsPID_STARTINDEX + 2]) << 16) + pdu[UdsPID_STARTINDEX + 3];
            break;

        case UdsPID_STARTINDEX+4:
        case UdsPID_STARTINDEX+5:
            Uds_UiMin = (((int)pdu[UdsPID_STARTINDEX + 4]) << 16) + pdu[UdsPID_STARTINDEX + 5];
            break;

        case UdsPID_STARTINDEX+6:
            Uds_Kp = pdu[UdsPID_STARTINDEX + 6];
            break;

        case UdsPID_STARTINDEX+7:
            Uds_Ki = pdu[UdsPID_STARTINDEX + 7];
            break;

        // Uqs PID
        case UqsPID_STARTINDEX:
            Uqs_OutMax = pdu[UqsPID_STARTINDEX];
            break;

        case UqsPID_STARTINDEX+1:
            Uqs_OutMin = pdu[UqsPID_STARTINDEX + 1];
            break;

        case UqsPID_STARTINDEX+2:
        case UqsPID_STARTINDEX+3:
            Uqs_UiMax = (((int)pdu[UqsPID_STARTINDEX + 2]) << 16) + pdu[UqsPID_STARTINDEX + 3];
            break;

        case UqsPID_STARTINDEX+4:
        case UqsPID_STARTINDEX+5:
            Uqs_UiMin = (((int)pdu[UqsPID_STARTINDEX + 4]) << 16) + pdu[UqsPID_STARTINDEX + 5];
            break;

        case UqsPID_STARTINDEX+6:
            Uqs_Kp = pdu[UqsPID_STARTINDEX + 6];
            break;

        case UqsPID_STARTINDEX+7:
            Uqs_Ki = pdu[UqsPID_STARTINDEX + 7];
            break;

        // Torque PID
        case TorPID_STARTINDEX:
            pidpt_OutMax = pdu[TorPID_STARTINDEX];
            break;

        case TorPID_STARTINDEX+1:
            pidpt_OutMin = pdu[TorPID_STARTINDEX + 1];
            break;

        case TorPID_STARTINDEX+2:
        case TorPID_STARTINDEX+3:
            pidpt_UiMax = (((int)pdu[TorPID_STARTINDEX + 2]) << 16) + pdu[TorPID_STARTINDEX + 3];
            break;

        case TorPID_STARTINDEX+4:
        case TorPID_STARTINDEX+5:
            pidpt_UiMin = (((int)pdu[TorPID_STARTINDEX + 4]) << 16) + pdu[TorPID_STARTINDEX + 5];
            break;

        case TorPID_STARTINDEX+6:
            pidpt_Kp = pdu[TorPID_STARTINDEX + 6];
            break;

        case TorPID_STARTINDEX+7:
            pidpt_Ki = pdu[TorPID_STARTINDEX + 7];
            break;

        // Velocity PID
        case VelPID_STARTINDEX:
            pidpv_OutMax = pdu[VelPID_STARTINDEX];
            break;

        case VelPID_STARTINDEX+1:
            pidpv_OutMin = pdu[VelPID_STARTINDEX + 1];
            break;

        case VelPID_STARTINDEX+2:
        case VelPID_STARTINDEX+3:
            pidpv_UiMax = (((int)pdu[VelPID_STARTINDEX + 2]) << 16) + pdu[VelPID_STARTINDEX + 3];
            break;

        case VelPID_STARTINDEX+4:
        case VelPID_STARTINDEX+5:
            pidpv_UiMin = (((int)pdu[VelPID_STARTINDEX + 4]) << 16) + pdu[VelPID_STARTINDEX + 5];
            break;

        case VelPID_STARTINDEX+6:
            pidpv_Kp = pdu[VelPID_STARTINDEX + 6];
            break;

        case VelPID_STARTINDEX+7:
            pidpv_Ki = pdu[VelPID_STARTINDEX + 7];
            break;

        // Holding PID
        case HodPID_STARTINDEX:
            pidholding_OutMax = pdu[HodPID_STARTINDEX];
            break;

        case HodPID_STARTINDEX+1:
            pidholding_OutMin = pdu[HodPID_STARTINDEX + 1];
            break;

        case HodPID_STARTINDEX+2:
        case HodPID_STARTINDEX+3:
            pidholding_UiMax = (((int)pdu[HodPID_STARTINDEX + 2]) << 16) + pdu[HodPID_STARTINDEX + 3];
            break;

        case HodPID_STARTINDEX+4:
        case HodPID_STARTINDEX+5:
            pidholding_UiMin = (((int)pdu[HodPID_STARTINDEX + 4]) << 16) + pdu[HodPID_STARTINDEX + 5];
            break;

        case HodPID_STARTINDEX+6:
            pidholding_Kp = pdu[HodPID_STARTINDEX + 6];
            break;

        case HodPID_STARTINDEX+7:
            pidholding_Ki = pdu[HodPID_STARTINDEX + 7];
            break;

        // Position PID
        case PosPID_STARTINDEX:
            pidc_position_OutMax = pdu[PosPID_STARTINDEX];
            break;

        case PosPID_STARTINDEX+1:
            pidc_position_OutMin = pdu[PosPID_STARTINDEX + 1];
            break;

        case PosPID_STARTINDEX+2:
        case PosPID_STARTINDEX+3:
            pidc_position_UiMax = (((int)pdu[PosPID_STARTINDEX + 2]) << 16) + pdu[PosPID_STARTINDEX + 3];
            break;

        case PosPID_STARTINDEX+4:
        case PosPID_STARTINDEX+5:
            pidc_position_UiMin = (((int)pdu[PosPID_STARTINDEX + 4]) << 16) + pdu[PosPID_STARTINDEX + 5];
            break;

        case PosPID_STARTINDEX+6:
            pidc_position_Kp = pdu[PosPID_STARTINDEX + 6];
				    SystemVar.ModbusID = pdu[PosPID_STARTINDEX + 6];
				     
            break;

        case PosPID_STARTINDEX+7:
            pidc_position_Ki = pdu[PosPID_STARTINDEX + 7];
            break;

        case 471:
            PostionPlan_accel_max = pdu[471];

        case 472:
            PostionPlan_a_accel = pdu[472];

        case 473:
            PostionPlan_a_decel = pdu[473];

        case 474:
            PostionPlan_decel_max = pdu[474];

        case 475:
            PostionPlan_vel_init = pdu[475];

        case 476:
            PostionPlan_vel_tar = pdu[476];


    }
}

//modbus����ͨ���߳�
void Modbus_Task(MBModify* modify_)   
{

    if (modbus_recv_flag == 1)  		//���յ�һ֡����
    {
        modbus_recv_flag = 0;
        pxMBFrameCBByteReceived(); 	//����xMBRTUReceiveFSM�����յ������ݸ���ucRTUBuf
        Update_Modbus_RO_Data();   	//����ȫ��ֻ������
    }

    eMBPoll(modify_);           		//�����յ�������֡�������д�룬����modify_

    if (modbus_send_flag == 2)  		//������ɣ�ʹ��DMA���ظ�����
    {
        //< �ظ�֡
        modbus_send_flag = 0;
        SET_RS485_TX_ENABLE;
        Modbus_DMA_ReEnable(MODBUS_DMA_TX_Channel, modbus_send_len);
        Modbus_Respond(modify_); 		//��������д���ֵ���¶�Ӧ����
    }

}




