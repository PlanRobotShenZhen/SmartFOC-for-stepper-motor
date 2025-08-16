
#include "uart_interface.h"
#include "ExternGlobals.h"

extern int Istargetposition;  //电机到达目标位置的标志位

uint32_t PROTOCOL_SERVER_TIMEOUT_MS = 10;
uint8_t tx_buf_[UART_TX_BUFFER_SIZE];

uint8_t RxBuffer[20];
uint8_t uart_cmd = 0;

uint8_t modbus_recv_data[MODBUS_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t modbus_send_data[MODBUS_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区

uint32_t modbus_recv_flag = 0;        								// 接收完成标志位
uint32_t modbus_recv_len = 0;         								// 接收的数据长度
uint32_t modbus_send_len = 0;         								// 发送的数据长度
uint32_t modbus_send_flag = 0;        								// 发送完成标志位

void Modbus_USART_Init(unsigned int baud)			//
{

    GPIO_InitType GPIO_InitStucture;
    USART_InitType USART_InitStucture;
    NVIC_InitType NVIC_InitStucture;

    RCC_EnableAPB2PeriphClk(MODBUS_USART_GPIO_CLK, ENABLE);
    MODBUS_USART_APBxClkCmd(MODBUS_USART_CLK, ENABLE);
    GPIO_InitStruct(&GPIO_InitStucture);
    //GPIO发送端采用复用推挽输出；
    GPIO_InitStucture.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStucture.Pin = MODBUS_USART_TxPin;
    GPIO_InitPeripheral(MODBUS_USART_GPIO, &GPIO_InitStucture);
    //GPIO接收端采用浮空输入；
    GPIO_InitStucture.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStucture.Pin = MODBUS_USART_RxPin;
    GPIO_InitPeripheral(MODBUS_USART_GPIO, &GPIO_InitStucture);
    //485发送控制引脚
    GPIO_InitStucture.Pin = RS485_DE_Pin;
    GPIO_InitStucture.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出
    GPIO_InitStucture.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(RS485_DE_GPIO, &GPIO_InitStucture);
    GPIO_ResetBits(RS485_DE_GPIO, RS485_DE_Pin);      //设置为接收模式，默认接收

    //USART结构体配置
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
    USART_ConfigInt(MODBUS_USART, USART_INT_IDLEF, ENABLE); //开启空闲中断

    //NVIC寄存器配置：中断请求通道设置为USART3,启动使能，抢占优先级为1，子（响应）优先级为1
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
    DMA_StructInit(&DMA_InitStruct);  //先默认初始化结构体

    // 配置 DMA1 通道2, USART3_TX
    DMA_InitStruct.PeriphAddr = (uint32_t) & (MODBUS_USART->DAT); // USART_DR 地址偏移：0x04
    DMA_InitStruct.MemAddr = (uint32_t)modbus_send_data;  // 内存地址
    DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStruct.BufSize = 0; 			//寄存器的内容为0时，通道是否开启都不会发生数据传输
    DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStruct.Priority = DMA_PRIORITY_LOW;
    DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(MODBUS_DMA_TX_Channel, &DMA_InitStruct);
    // 重映射
    DMA_RequestRemap(MODBUS_DMA_TX_REMAP, MODBUS_DMA, MODBUS_DMA_TX_Channel, ENABLE);

    // 配置 DMA1 通道3, USART3_RX
    DMA_InitStruct.PeriphAddr = (uint32_t) & (MODBUS_USART->DAT); // (USART_DR) 地址偏移：0x04
    DMA_InitStruct.MemAddr = (uint32_t)modbus_recv_data;  // 内存地址
    DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;       // 外设到内存
    DMA_InitStruct.BufSize = MODBUS_RX_MAXBUFF;
    DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStruct.Priority = DMA_PRIORITY_LOW;
    DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(MODBUS_DMA_RX_Channel, &DMA_InitStruct);
    // 重映射
    DMA_RequestRemap(MODBUS_DMA_RX_REMAP, MODBUS_DMA, MODBUS_DMA_RX_Channel, ENABLE);

    // 配置DMA发送完成中断
    NVIC_InitStruct.NVIC_IRQChannel = MODBUS_DMA_TX_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
    NVIC_Init(&NVIC_InitStruct);
    // 配置MODBUS_DMA_TX_Channel传输完成中断
    DMA_ConfigInt(MODBUS_DMA_TX_Channel, DMA_INT_TXC, ENABLE);

    //禁止发送，使能接收
    DMA_EnableChannel(MODBUS_DMA_TX_Channel, DISABLE);
    DMA_EnableChannel(MODBUS_DMA_RX_Channel, ENABLE);
}

void Modbus_DMA_ReEnable(DMA_ChannelType* DMA_Channel, uint16_t len)
{
    DMA_EnableChannel(DMA_Channel, DISABLE);
    DMA_SetCurrDataCounter(DMA_Channel, len);
    DMA_EnableChannel(DMA_Channel, ENABLE);
}

// DMA发送完成回调
void MODBUS_DMA_TX_IRQHandler(void)
{
    if (DMA_GetIntStatus(MODBUS_DMA_TX_INT_TXC, MODBUS_DMA) != RESET) // 传输完成
    {
        DMA_ClrIntPendingBit(MODBUS_DMA_TX_INT_TXC, MODBUS_DMA);     // 清除中断
        SET_RS485_RX_ENABLE;                         // 设置标志
        DMA_EnableChannel(MODBUS_DMA_TX_Channel, DISABLE); //失能DMA通道
        Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel, MODBUS_RX_MAXBUFF);
    }
}

//空闲中断回调
void MODBUS_USART_IRQHandler(void)
{
    if(USART_GetIntStatus(MODBUS_USART, USART_INT_IDLEF) != RESET)
    {
        MODBUS_USART->STS;
        MODBUS_USART->DAT;
        modbus_recv_flag = 1;                // 接收标志置1
        modbus_recv_len = MODBUS_RX_MAXBUFF - DMA_GetCurrDataCounter(MODBUS_DMA_RX_Channel);// 统计收到的数据的长度
        //失能通道，重新设置接收长度为最大，再使能通道
        Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel, MODBUS_RX_MAXBUFF);
    }
}

// 更新全部只读参数
//  增加温度   增加转矩

void Update_Modbus_RO_Data(void)
{
    uint16_t* pdu = getPDUData();

    pdu[0] = SystemError.SysErr;                               //电机故障
    pdu[1] = SystemError.PwmOn;                                //电机运行状态 上使能 下使能
    pdu[2] = SystemError.RuningModeFdb;                        //当前运行模式
    pdu[3] = 0;                                             	 //电机电流
    pdu[4] = MotorControler.SpeedFdbpFilter3;                  //电机速度
    pdu[5] = (MotorControler.MotorActivePostion) >> 16;        //电机当前位置
    pdu[6] = (MotorControler.MotorActivePostion) & 0xffff;     //电机当前位置
    pdu[7] = MotorControler.MT6835_Cla_State;                  //
    pdu[8] = SystemError.DcVolt;                               //母线电压
    pdu[9] = 0;


    pdu[10] = SystemVar.SoftwareVersion;                  			//软件版本
    pdu[11] = SystemVar.HardwareVersion;                  			//硬件版本
    pdu[12] = SystemVar.MotorVersion;                     			//电机型号
	
    //电机电流
	  pdu[13] = svpwm.As;
    pdu[14] = svpwm.Bs;
    pdu[15] = 103;
		
//	  pdu[13] = 101;
//    pdu[14] = 102;
//    pdu[15] = 103;
	
    pdu[16] = SystemVar.ModbusID;																//MODBUS ID			
    pdu[17] = SystemVar.ModbusBaudrate;													//MOUBUS 波特率
		
    pdu[18] = SystemVar.CanopenID;															//CANOPEN ID
    pdu[19] = SystemVar.CanopenBaudrate;												//CANOPEN 波特率

    //工作参数
    pdu[20] = SystemError.DcRefMax;                 //最大母线电压
    pdu[21] = SystemError.DcRefMin;                 //最小母线电压
    pdu[22] = SystemError.ImeMax;                   //最大相电流
    pdu[23] = SystemError.ImeMax;;                  //最大线连续电流 ------------
    pdu[24] = SystemError.OverSpeed;                //位置环输出限幅 ----------
    pdu[25] = SystemError.OverSpeed;                //速度环输出限幅
    pdu[26] = SystemError.OverLoadTimer;            //d 轴电流环限幅 ------
    pdu[27] = 105;                                  //电压矢量限幅   --------------

    pdu[28] = pidc_position_Kp;                      //位置环第一增益
    pdu[29] = pidc_position_Ki;                      //位置换第二增益
    pdu[30] = 10;                                    //位置环速度前馈系数---------------
    pdu[31] = 100;                                   //速度前馈低通滤波带宽-----------
    pdu[32] = pidpv_Kp;                              //速度环第一增益
    pdu[33] = 101;                                   //速度环第二增益       ------------
    pdu[34] = pidpv_Ki;                              //速度环第一积分
    pdu[35] = 103;                                   //速度环第二积分            --------
    pdu[36] = 104;                                   //干扰抵抗增益       --------
    pdu[37] = SystemError.ThresholdCoeff ;           //加速度前馈系数 ------------
    pdu[38] = SystemError.TorquePIDLimitCoeff;       //加速度前馈低通滤波带宽 ----------

    pdu[39] = pidpt_Kp;                              //电流环增益
    pdu[40] = pidpt_Ki;                              //电流环积分
    pdu[41] = 100;                                   //增益设置 -----------
    pdu[42] = 101;                                   //负载转动惯量比----------
    pdu[43] = 101;                                   //刚度系数-----------
    pdu[44] = 102;                                   //滤波器开启配置-----------
    pdu[45] = 103;                                   //速度给定滤波带宽----------
    pdu[46] = 104;                                   //速度反馈滤波带宽----------
    pdu[47] = 105;                                   //电流给定滤波带宽---------
    pdu[48] = 106;                                   //电流反馈滤波带宽--------
    pdu[49] = 107;                                   //转矩陷波器频率------------
    pdu[40] = 10;                                    //转矩陷波器宽度---------
    pdu[41] = 100;                                   //转矩陷波器深度------------
    pdu[42] = 101;                                   //滤波器参数计算---------
    pdu[43] = 101;                                   //Jerk 平滑系数-------
    pdu[44] = 102;                                   //电子齿轮分子-------
    pdu[45] = 103;                                   //电子齿轮分母---------
    pdu[46] = 104;                                   //默认方向-------
    pdu[47] = 105;                                   //刹车打开延时-------
    pdu[48] = 106;                                   //刹车关闭延时---------
    pdu[49] = 107;                                   //泄放打开阈值-----
    pdu[50] = 10;                                    //泄放关闭阈值--------
    pdu[51] = 100;                                   //到位误差-----------
    pdu[52] = 101;                                   //到位时间----------
    pdu[53] = 101;                                   //超差阈值-------
    pdu[54] = 102;                                   //输入端子滤波------------
    pdu[55] = 103;                                   //软限位开关-------
    pdu[56] = 104;                                   //转矩模式下速度限幅----------
    pdu[57] = 105;                                   //输入端子极性------------
		
    // 运动参数
    pdu[58] = 106;                                   //控制指令
    pdu[59] = 107;                                   //工作模式
    pdu[60] = 10;                                    //位置模式目标位置高位
    pdu[61] = 100;                                   //位置模式目标低位
    pdu[62] = 101;                                   //位置模式目标速度高位
    pdu[63] = 101;                                   //位置模式目标速度低位
    pdu[64] = 102;                                   //位置模式加速时间高位
    pdu[65] = 103;                                   //位置模式加速时间低位
    pdu[66] = 104;                                   //位置模式减速时间高位
    pdu[67] = 105;                                   //位置模式减速时间低位
    pdu[68] = 106;                                   //速度模式目标速度高位
    pdu[69] = 107;                                   //速度模式目标速度低位
    pdu[70] = 10;                                    //回零模式
    pdu[71] = 100;                                   //回零速度高位
    pdu[72] = 101;                                   //回零速度低位
    pdu[73] = 101;                                   //回零查询速度高位
    pdu[74] = 102;                                   //回零查询速度低位
    pdu[75] = 103;                                   //回零加减速高位
    pdu[76] = 104;                                   //回零加减速低位
    pdu[77] = 105;                                   //回零偏移高位
    pdu[78] = 106;                                   //回零偏移低位
    pdu[79] = 107;                                   //正向软限位高位
    pdu[80] = 10;                                    //正向软限位低位
    pdu[81] = 100;                                   //负向软限位高位
    pdu[82] = 101;                                   //负向软限位低位
    pdu[83] = 101;                                   //目标转矩
    pdu[84] = 102;                                   //目标转矩限幅
    pdu[85] = 103;                                   //反馈转矩
    pdu[86] = 104;                                   //转矩斜坡高位
    pdu[87] = 105;                                   //转矩斜坡低位
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
    pdu[298] = UartMode.TargetPosition >> 16;  // 提取高 16 位并存储到 pdu[298]
    pdu[299] = UartMode.TargetPosition & 0xFFFF;         // 提取低 16 位并存储到 pdu[299]
    pdu[306] = (UartMode.Speed * 1000) >> 16; // 提取高 16 位并存储到 pdu[306]
    pdu[307] = (UartMode.Speed * 1000) & 0xFFFF;       // 提取低 16 位并存储到 pdu[307]
    pdu[310] = (UartMode.Torque * 10000) >> 16;
    pdu[311] = (UartMode.Torque * 10000) & 0xFFFF;
    pdu[312] = UartMode.DisEnable + 1;
    pdu[314] = SystemError.SaveAllRsetFlag;
    pdu[315] = 1;//目测不能修改
    pdu[316] = 1;//目测不能修改
    pdu[317] = SystemError.RuningMode - 1;
    pdu[318] = 1;//目测不能修改

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
// 根据收到的命令更新对应地址的参数
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
            //PostionPlan_vel_tar = pdu[301]; //注意速度方向
            break;

        case 300 :       //最大速度
            break;

        case 301 :       //加速度
            break;

        case 302 :       //加加速度
            break;

        case 303 :       //减速度
            break;

        case 304 :       //减速度
            break;

        case 305 :       //暂时未用
            break;



        case 306 :
        case 307 :
            test222 = ((int)(pdu[306]) * 65536) + (int)(pdu[307]);
            UartMode.Speed = test222 / 1000;
            break;

        case 308 : //加速时间
            break;

        case 309 : //减速时间
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


        /***************************** PID参数写入，从351开始******************************/
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

//modbus周期通信线程
void Modbus_Task(MBModify* modify_)   
{

    if (modbus_recv_flag == 1)  		//接收到一帧数据
    {
        modbus_recv_flag = 0;
        pxMBFrameCBByteReceived(); 	//调用xMBRTUReceiveFSM，将收到的数据赋给ucRTUBuf
        Update_Modbus_RO_Data();   	//更新全部只读参数
    }

    eMBPoll(modify_);           		//处理收到的数据帧，如果是写入，更新modify_

    if (modbus_send_flag == 2)  		//处理完成，使能DMA，回复主机
    {
        //< 回复帧
        modbus_send_flag = 0;
        SET_RS485_TX_ENABLE;
        Modbus_DMA_ReEnable(MODBUS_DMA_TX_Channel, modbus_send_len);
        Modbus_Respond(modify_); 		//根据主机写入的值更新对应参数
    }

}




