/*function.c��Ҫִ�й��ϼ�⡢���������Լ�״̬��ʾ����*/

#include "DeviceConfig.h"     // Headerfile Include File
#include "ExternGlobals.h"
#include "FLASH_WR.h"
#include "CanOpenMode.h"

//��������
void SysErrManage(void);                     //������Ϣ����
short ImeasCheck(short ImeasA, short ImeasB, short ImeasC);     //�������������
void LostPhase(void);                        //ȱ��
void LostCoder(void);                        //���������Ϸ���
void TorsionAnalyse(void);                   //���ط���
void SpeedAnalyse(void);                     //���ٷ���
void SaveVar(void);                          //�������
void SaveAllRsetVar(void);                   //�������в���
void ReadVar(void);                          //��ȡ����
void PowerManage(void);                      //��Դ����
void Led(void);                              //ָʾ��

// ���������쳣��⣨����ȱ�࣬���������������жϣ�
short ImeasCheck(short ImeasA, short ImeasB, short ImeasC)
{
    static short phaseA_noCurrent_count = 0;   //A����Ч��������
    static short phaseB_noCurrent_count = 0;   //B����Ч��������
    static short phaseC_noCurrent_count = 0;   //C����Ч��������
    static short ImeasACheck = 0;
    static short ImeasBCheck = 0;
    static short ImeasCCheck = 0;
    static short sum = 0;
    //���������������ֵ֮�ͣ��ж��Ƿ�Ϊ��Ч����
    sum = Abs(ImeasA) + Abs(ImeasB) + Abs(ImeasC);
    //�����ܺʹ�����ֵ�����и���������
    if(sum > 500)
    {
			  //����ֵ����6λ������64�����ж��Ƿ�ӽ���0
        ImeasACheck = (Abs(ImeasA)) >> 6;
        ImeasBCheck = (Abs(ImeasB)) >> 6;
        ImeasCCheck = (Abs(ImeasC)) >> 6;
        //A��������
        if(ImeasACheck == 0)//û�м�⵽����
        {
            phaseA_noCurrent_count++;
        }
        else
        {
            phaseA_noCurrent_count = 0;
        }
        //B��������
        if(ImeasBCheck == 0)//û�м�⵽����
        {
            phaseB_noCurrent_count++;
        }
        else
        {
            phaseB_noCurrent_count = 0;
        }
        //C��������
        if(ImeasCCheck == 0)//û�м�⵽����
        {
            phaseC_noCurrent_count++;
        }
        else
        {
            phaseC_noCurrent_count = 0;
        }
    }
    else
    {   //�ܵ�����С���������м���
        phaseA_noCurrent_count = 0;
        phaseB_noCurrent_count = 0;
        phaseC_noCurrent_count = 0;
    }

		//ȱ������жϣ��쳣���������趨���ֵ��
    if(phaseA_noCurrent_count > SystemError.ImeMax)
    {
        return M_SYSERR_LACKPHASE ; //A��ȱ��
    }
    if(phaseB_noCurrent_count > SystemError.ImeMax)
    {
        return M_SYSERR_LACKPHASE ; //B��ȱ��

    }
    if(phaseC_noCurrent_count > SystemError.ImeMax)
    {
        return M_SYSERR_LACKPHASE ; //C��ȱ��
    }
    return 0;
}

//ȱ����
void LostPhase(void)
{   //�����������״̬����ȱ����
    if(SystemError.MotorRunFlag)
    {   //��������ֵ����6λΪ0���ж�Ϊ��Ч��������
        if((((Abs(SystemError.ImeasA - SystemError.ImeasAOffset)) >> 6) == 0) && (((Abs(SystemError.ImeasB - SystemError.ImeasBOffset)) >> 6) == 0) && (((Abs(SystemError.ImeasC - SystemError.ImeasCOffset)) >> 6) == 0))
        {
            if((MotorControler.SpeedFdbp == 0) && (UartMode.Mode != 5) && (svpwm.SvpwmControlState != 4)) // û�����ӵ��������
            {   //ȱ�����ӳټ���+1
                SystemError.ImeasCheckDelay++; 
                //�ӳټ��������趨��ֵ���ж�Ϊȱ��
                if(SystemError.ImeasCheckDelay >= SystemError.LowSpeedTimer)
                {
                    SystemError.SysErr = M_SYSERR_LACKPHASE;
                }
            }
            else   //�������
            {
                SystemError.ImeasCheckDelay = 0;
            }
        }

    }
    else //���δ���У��������
    {
        SystemError.ImeasCheckDelay = 0;
    }
 
		//������ƫ��������ֵ�Ƿ�����ж�Ϊ��������������
    if(Abs(SystemError.ImeasAOffset) > 130)
    {
        SystemError.SysErr = M_SYSERR_CURRENTSENSOR;
    }

    if(Abs(SystemError.ImeasBOffset) > 130)
    {
        SystemError.SysErr = M_SYSERR_CURRENTSENSOR;
    }


    if(Abs(SystemError.ImeasCOffset) > 130)
    {
        SystemError.SysErr = M_SYSERR_CURRENTSENSOR;
    }

}
//���������
void LostCoder(void)
{
    static int count = 0; //��������Ч���ݼ���
    //û�б궨���
    if(MotorControler.AngleFromMT6835Offset1 == 0) 
    {
        SystemError.SysErr = M_SYSERR_CODER;
    }
    //  ������û�ж�����Ч����
    if(MotorControler.AngleFromMT6835 == -1999)    
    {
        count++;
			//��Ч����������ֵ
        if(count >= 80)
        {
            SystemError.SysErr = M_SYSERR_CODERNESS;
            count = 0;
        }
    }

}

// //���ط���
void TorsionAnalyse(void)
{   //ת�ص�������ֵ��������ת��ֵ
    if(Abs(svpwm.IQs) > MotorControler.RatedTorque)
    {
        SystemError.OverLoadTimer++; //���ؼ���+1
        //��������ٶ�<3���ж�Ϊ��ת
        if(Abs(MotorControler.SpeedFdbp) < 3)
        {
            if(SystemError.OverLoadTimer >= (SystemError.IqsMaxOverTime * 10))
            {
                SystemError.OverLoadTimer = SystemError.IqsMaxOverTime * 10;
                SystemError.SysErr = M_SYSERR_ROTOR_LOCKED ;
            }
        }
        else //������ٶȣ��ж�Ϊ����
        {    //���ؼ���������ֵ
            if(SystemError.OverLoadTimer >= (SystemError.IqsMaxOverTime * 20))
            {
                SystemError.OverLoadTimer = SystemError.IqsMaxOverTime * 20;
                SystemError.SysErr = M_SYSERR_OVER_LOAD;
            }
        }
    }
    else //ת�ص������������ؼ�������
    {
        SystemError.OverLoadTimer = 0;
    }
}



// //���ٶȷ���
void SpeedAnalyse(void)
{   //�����ٶȳ����������ٶ�
    if(Abs(MotorControler.SpeedFdbp) > MotorControler.MaxSpeed)
    {
        SystemError.OverSpeedTimer++; //���ټ���+1
        //����ֵ�����趨��ֵ
        if(SystemError.OverSpeedTimer >= (SystemError.OverSpeed))
        {
            SystemError.OverSpeedTimer = SystemError.OverSpeed ;
            SystemError.SysErr = M_SYSERR_OVER_SPEED;
        }
    }
    else //�ٶ�����������
    {
        SystemError.OverSpeedTimer = 0;
    }
}
//ADCת��ֵ���飬���ڵ�ѹ�����ȡ��ѹ
extern uint16_t  ADC_ConvertedValue[];
//��flash��ȡ����
void ReadVar(void)
{
    MyFLASH_ReadByte(FLASH_ADDRESS0, (uint16_t*)VarDateFlash0, 512);//��ֵ
    MyFLASH_ReadByte(FLASH_ADDRESS1, (uint16_t*)VarDateFlash1, 512);//��ֵ
    MyFLASH_ReadByte(FLASH_ADDRESS2, (uint16_t*)VarDateFlash2, 512);//��ֵ
    //ϵͳ�汾������ֵ
    SystemVar.SoftwareVersion = VarDateFlash0[0];
    SystemVar.HardwareVersion = VarDateFlash0[1];
    SystemVar.MotorVersion = VarDateFlash0[2];


    /****��������*********/
    //SystemVar.VF_Voltage = VarDateFlash0[104];
    //SystemVar.VF_ElectricalAngle = VarDateFlash0[105];
    SystemVar.VF_Coefficient = VarDateFlash0[106];
    SystemVar.VF_Coefficient_B = VarDateFlash0[107];
    //SystemVar.VF_ElectricalAngleStepMax = VarDateFlash0[108];
    //SystemVar.VF_ElectricalAngleStepMin = VarDateFlash0[109];
    //SystemVar.VF_ElectricalAngleStep = VarDateFlash0[110];
    //SystemVar.VF_ElectricalAngleStepCount = VarDateFlash0[111];
    /****��������*********/

    /****�������*********/
    //SystemVar.test_uqs = VarDateFlash0[112];
    //SystemVar.test_angle = VarDateFlash0[113];
    /****�������*********/


    SystemVar.ModbusID = VarDateFlash0[116];	       //Modbus      ID��
    SystemVar.ModbusBaudrate = VarDateFlash0[117];	 //Modbus      ������
    SystemVar.CanopenID = VarDateFlash0[118];        //Canopen     ID��
    SystemVar.CanopenBaudrate = VarDateFlash0[119];  //Canopen     ������


    //SystemError.SpeedFdbpPost = VarDateFlash0[150];
    //SystemError.MotorRunFlag = VarDateFlash0[151];
    //SystemError.OverLoadTimer = VarDateFlash0[152];
    //SystemError.CoderTimer = VarDateFlash0[153];
    //SystemError.DcCoeff = VarDateFlash0[154];
    //SystemError.PositiveDcCoeff = VarDateFlash0[155];

    SystemError.IqsMax = VarDateFlash0[156];
    SystemError.IdsMax = VarDateFlash0[157];
    SystemError.IqsMaxOverTime = VarDateFlash0[158];
    SystemError.DcRef = VarDateFlash0[159];
    SystemError.DcRefK = VarDateFlash0[160];
    SystemError.DcRefB = VarDateFlash0[161];

    //SystemError.AdDcVolt = VarDateFlash0[162];
    //SystemError.SysErr = VarDateFlash0[163];
    SystemError.LowSpeedTimer = VarDateFlash0[164];
    //SystemError.DcVolt = VarDateFlash0[165];
    SystemError.DcRefMax = VarDateFlash0[166];
    SystemError.DcMaxErrRef = VarDateFlash0[167];
    SystemError.DcRefMin = VarDateFlash0[168];
    SystemError.DcMinErrRef = VarDateFlash0[169];
    SystemError.DcArrestMax = VarDateFlash0[170];
    SystemError.DcArrestDelayMax = VarDateFlash0[171];
    SystemError.DcArrest2DelayMax = VarDateFlash0[172];
    SystemError.ImeMax = VarDateFlash0[173];

    SystemError.LossACTimeMax = VarDateFlash0[174];
    SystemError.NoSysErrPowerOff = VarDateFlash0[175];
    //SystemError.ImeasCheckDelay = VarDateFlash0[176];
    //SystemError.SysErrRuningAsk = VarDateFlash0[177];
    //SystemError.SysErrRuning = VarDateFlash0[178];
    //SystemError.SysResetVar = VarDateFlash0[179];
    //SystemError.SysErrSaveLock = VarDateFlash0[180];
    //SystemError.SaveAllRsetFlag = VarDateFlash0[181];
    //SystemError.RuningMode = VarDateFlash0[182]; //����ģʽ   CANCOPEN==1 MODBUS==2
    //SystemError.SaveAllParaFlag = VarDateFlash0[183];
    SystemError.OverSpeed = VarDateFlash0[184];          //�����ʱ��
    //SystemError.KpGainCoeff = VarDateFlash0[185];
    //SystemError.KiGainCoeff = VarDateFlash0[186];
    SystemError.ThresholdCoeff = VarDateFlash0[187];     //�ٶ��л���ֵϵ��
    //SystemError.TorqueCoeff = VarDateFlash0[188];
    SystemError.TorquePIDLimitCoeff = VarDateFlash0[189];//���ת��PID�������
    //SystemError.TorqueDECLimitCoeff = VarDateFlash0[190];

    //MotorControler.State = VarDateFlash0[200];
    //MotorControler.SpeedRef = VarDateFlash0[201];
    //MotorControler.SpeedFdb = VarDateFlash0[202];
    //MotorControler.TorqueRef = VarDateFlash0[203];
    //MotorControler.TorqueFdb = VarDateFlash0[204];
    //MotorControler.PositionRef = VarDateFlash0[205];
    //MotorControler.PositionFdb = VarDateFlash0[206];
    MotorControler.MotorPoles = VarDateFlash0[207];
    //MotorControler.MotorActivePostion = VarDateFlash0[208];
    //MotorControler.RotorCount = VarDateFlash0[209];

    MotorControler.RatedTorque = VarDateFlash0[210]; //�ת��
    MotorControler.MaxTorque = VarDateFlash0[211];   //���ת��
    MotorControler.MaxSpeed = VarDateFlash0[212];

    MotorControler.SpeedAcc = VarDateFlash0[213];
    MotorControler.SpeedDcc = VarDateFlash0[214];

  //MotorControler.spiread_anlge = VarDateFlash0[215];
  //MotorControler.AngleFromMT6835 = VarDateFlash0[216];
  //MotorControler.AngleFromMT6835Offset = VarDateFlash0[217];
    MotorControler.AngleFromMT6835Offset1 = VarDateFlash0[218];
    
		//PID������ȡ
    Uds_OutMax = VarDateFlash1[0];
    Uds_OutMin = VarDateFlash1[1];
    Uds_UiMax = ((int)(VarDateFlash1[2]) << 16) + VarDateFlash1[3];
    Uds_UiMin = ((int)(VarDateFlash1[4]) << 16) + VarDateFlash1[5];
    Uds_Kp = VarDateFlash1[6];
    Uds_Ki = VarDateFlash1[7];

    Uqs_OutMax = VarDateFlash1[20];
    Uqs_OutMin = VarDateFlash1[21];
    Uqs_UiMax = ((int)(VarDateFlash1[22]) << 16) + VarDateFlash1[23];
    Uqs_UiMin = ((int)(VarDateFlash1[24]) << 16) + VarDateFlash1[25];
    Uqs_Kp = VarDateFlash1[26];
    Uqs_Ki = VarDateFlash1[27];

    pidpt_OutMax = VarDateFlash1[40];
    pidpt_OutMin = VarDateFlash1[41];
    pidpt_UiMax = ((int)(VarDateFlash1[42]) << 16) + VarDateFlash1[43];
    pidpt_UiMin = ((int)(VarDateFlash1[44]) << 16) + VarDateFlash1[45];
    pidpt_Kp = VarDateFlash1[46];
    pidpt_Ki = VarDateFlash1[47];

    pidpv_OutMax = VarDateFlash1[60];
    pidpv_OutMin = VarDateFlash1[61];
    pidpv_UiMax = ((int)(VarDateFlash1[62]) << 16) + VarDateFlash1[63];
    pidpv_UiMin = ((int)(VarDateFlash1[64]) << 16) + VarDateFlash1[65];
    pidpv_Kp = VarDateFlash1[66];
    pidpv_Ki = VarDateFlash1[67];

    pidholding_OutMax = VarDateFlash1[80];
    pidholding_OutMin = VarDateFlash1[81];
    pidholding_UiMax = ((int)(VarDateFlash1[82]) << 16) + VarDateFlash1[83];
    pidholding_UiMin = ((int)(VarDateFlash1[84]) << 16) + VarDateFlash1[85];
    pidholding_Kp = VarDateFlash1[86];
    pidholding_Ki = VarDateFlash1[87];

    pidc_position_OutMax = VarDateFlash1[100];
    pidc_position_OutMin = VarDateFlash1[101];
    pidc_position_UiMax = ((int)(VarDateFlash1[102]) << 16) + VarDateFlash1[103];
    pidc_position_UiMin = ((int)(VarDateFlash1[104]) << 16) + VarDateFlash1[105];
    pidc_position_Kp = VarDateFlash1[106];
    pidc_position_Ki = VarDateFlash1[107];

    //λ�ù滮������ȡ
    PostionPlan_accel_max = VarDateFlash2[0] / 1000.0f;
    PostionPlan_a_accel  = VarDateFlash2[1] / 1000.0f;
    PostionPlan_a_decel  = VarDateFlash2[2] / 1000.0f;
    PostionPlan_decel_max = VarDateFlash2[3] / 1000.0f;

//	PostionPlan_accel_max = VarDateFlash2[0];
//  PostionPlan_a_accel  = VarDateFlash2[1];
//  PostionPlan_a_decel  = VarDateFlash2[2];
//  PostionPlan_decel_max = VarDateFlash2[3];
    PostionPlan_vel_init = VarDateFlash2[4];
    PostionPlan_vel_tar = VarDateFlash2[5];

}

//����Ĭ�ϲ���
void SaveAllRsetVar(void)
{
    SystemVar.SoftwareVersion = SoftwareVersion;
    SystemVar.HardwareVersion = HardwareVersion;
    SystemVar.MotorVersion = MotorVersion;

    /****��������*********/
    SystemVar.VF_Voltage = 0;
    SystemVar.VF_ElectricalAngle = 0;
    SystemVar.VF_Coefficient = 13;
    SystemVar.VF_Coefficient_B = 1350;
    SystemVar.VF_ElectricalAngleStepMax = 500;
    SystemVar.VF_ElectricalAngleStepMin = -500;
    SystemVar.VF_ElectricalAngleStep = 0;
    SystemVar.VF_ElectricalAngleStepCount = 0;

    /****�������*********/
    SystemVar.test_uqs = 0;
    SystemVar.test_angle = 30;
	
    /****ͨ�Ų���*********/
    SystemVar.ModbusID = 1;	              //Modbus      ID��
    SystemVar.ModbusBaudrate = 1152;	    //Modbus      ������
    SystemVar.CanopenID = 1;              //Canopen     ID��
    SystemVar.CanopenBaudrate = 1000;		  //Canopen     ������

    SystemError.SpeedFdbpPost = 0;
    SystemError.MotorRunFlag = 0;
    SystemError.OverLoadTimer = 0;
    SystemError.CoderTimer = 0;
    SystemError.DcCoeff = 0;
    SystemError.PositiveDcCoeff = 0;

    SystemError.IqsMax = 100;
    SystemError.IdsMax = 20;
    SystemError.IqsMaxOverTime = 20;
    SystemError.DcRef = 4800;
    SystemError.DcRefK = 5435;
    SystemError.DcRefB = 8;

    SystemError.AdDcVolt = 0;
    SystemError.SysErr = 0;
    SystemError.LowSpeedTimer = 30;
    SystemError.DcVolt = 0;
    SystemError.DcRefMax = 5600;
    SystemError.DcMaxErrRef = 3000;
    SystemError.DcRefMin = 4200;
    SystemError.DcMinErrRef = 3000;

    SystemError.DcArrestMax = 350;
    SystemError.DcArrestDelayMax = 100;
    SystemError.DcArrest2DelayMax = 100;
    SystemError.ImeMax = 30;

    SystemError.LossACTimeMax = 100;
    SystemError.NoSysErrPowerOff = 0;
    SystemError.ImeasCheckDelay = 0;
    SystemError.SysErrRuningAsk = 0;
    SystemError.SysErrRuning = 0;
    SystemError.SysResetVar = 0;
    SystemError.SysErrSaveLock = 0;
    SystemError.SaveAllRsetFlag = 0;
    SystemError.RuningMode = 1;     //����ģʽ   CANCOPEN==1 MODBUS==2
    SystemError.SaveAllParaFlag = 0;
    SystemError.OverSpeed = 5;
    SystemError.KpGainCoeff = 30;
    SystemError.KiGainCoeff = 30;
    SystemError.ThresholdCoeff = 0;
    SystemError.TorqueCoeff = 6;
    SystemError.TorquePIDLimitCoeff = 5;
    SystemError.TorqueDECLimitCoeff = 0;
    MotorControler.State = 0;
    MotorControler.SpeedRef = 0;
    MotorControler.SpeedFdb = 0;
    MotorControler.TorqueRef = 700;
    MotorControler.TorqueFdb = 0;
    MotorControler.PositionRef = 0;
    MotorControler.PositionFdb = 0;
    MotorControler.MotorPoles = 5;   //���������
    MotorControler.MotorActivePostion = 0;
    MotorControler.RotorCount = 0;
    MotorControler.spiread_anlge = 0;
    MotorControler.AngleFromMT6835 = -19999;
    MotorControler.AngleFromMT6835Offset = 0;
    MotorControler.AngleFromMT6835Offset1 = 0;

    MotorControler.RatedTorque = 1000;    //�ת��
    MotorControler.MaxTorque =   2500;    //���ת��
    MotorControler.MaxSpeed =    3500;

    MotorControler.SpeedAcc = 1;
    MotorControler.SpeedDcc = 1;

    PostionPlan_accel_max = 0.2;          //λ�ù滮���ٶ�
    PostionPlan_a_accel = 0.01;           //λ�ù滮�Ӽ��ٶ�
    PostionPlan_a_decel = 0.01;           //λ�ù滮���ٶ�
    PostionPlan_decel_max = 0.2;          //λ�ù滮�����ٶ�
    PostionPlan_vel_init = 0;             //λ�ù滮���ٶ�
    PostionPlan_vel_tar = 100;            //λ�ù滮����ٶ�


    /****PID����*********/
    Uds_OutMax = _IQ15(0.5);
    Uds_OutMin = _IQ15(-0.5);
    Uds_UiMax = _IQ30(0.5);
    Uds_UiMin = _IQ30(-0.5);
    Uds_Kp = 80;
    Uds_Ki = 80;

    Uqs_OutMax = _IQ15(0.5);
    Uqs_OutMin = _IQ15(-0.5);
    Uqs_UiMax = _IQ30(0.5);
    Uqs_UiMin = _IQ30(-0.5);
    Uqs_Kp = 80;
    Uqs_Ki = 80;

    pidpt_OutMax = _IQ15(0.5);
    pidpt_OutMin = _IQ15(-0.5);
    pidpt_UiMax = _IQ30(0.5);
    pidpt_UiMin = _IQ30(-0.5);
    pidpt_Kp = 200;
    pidpt_Ki = 30;

    pidpv_OutMax = _IQ15(0.9);
    pidpv_OutMin = _IQ15(-0.9);
    pidpv_UiMax = _IQ30(0.9);
    pidpv_UiMin = _IQ30(-0.9);
    pidpv_Kp = 300;
    pidpv_Ki = 30;

    pidc_position_OutMax = _IQ15(0.8);
    pidc_position_OutMin = _IQ15(-0.8);
    pidc_position_UiMax = _IQ30(0.8);
    pidc_position_UiMin = _IQ30(-0.8);
    pidc_position_Kp = 30;
    pidc_position_Ki = 3;

    pidholding_OutMax = _IQ15(0.18);
    pidholding_OutMin = _IQ15(-0.18);
    pidholding_UiMax = _IQ30(0.18);
    pidholding_UiMin = _IQ30(-0.18);
    pidholding_Kp = 30;
    pidholding_Ki = 20;

    PostionPlan_accel_max = 0.2;          //λ�ù滮���ٶ�
    PostionPlan_a_accel = 0.01;           //λ�ù滮�Ӽ��ٶ�
    PostionPlan_a_decel = 0.01;           //λ�ù滮���ٶ�
    PostionPlan_decel_max = 0.2;          //λ�ù滮�����ٶ�

    PostionPlan_vel_init = 0;            //λ�ù滮���ٶ�
    PostionPlan_vel_tar = 100;           //λ�ù滮����ٶ�

    VarDateFlash0[0] = SystemVar.SoftwareVersion;
    VarDateFlash0[1] = SystemVar.HardwareVersion;
    VarDateFlash0[2] = SystemVar.MotorVersion;

    /****��������*********/
    VarDateFlash0[104] = SystemVar.VF_Voltage;
    VarDateFlash0[105] = SystemVar.VF_ElectricalAngle;
    VarDateFlash0[106] = SystemVar.VF_Coefficient;
    VarDateFlash0[107] = SystemVar.VF_Coefficient_B;
    VarDateFlash0[108] = SystemVar.VF_ElectricalAngleStepMax;
    VarDateFlash0[109] = SystemVar.VF_ElectricalAngleStepMin;
    VarDateFlash0[110] = SystemVar.VF_ElectricalAngleStep;
    VarDateFlash0[111] = SystemVar.VF_ElectricalAngleStepCount;
    /****��������*********/

    /****�������*********/
    VarDateFlash0[112] = SystemVar.test_uqs;
    VarDateFlash0[113] = SystemVar.test_angle;
		
    /****ͨ�Ų���*********/
    VarDateFlash0[116] = SystemVar.ModbusID;	                //Modbus      ID��
    VarDateFlash0[117] = SystemVar.ModbusBaudrate;	          //Modbus      ������
    VarDateFlash0[118] = SystemVar.CanopenID;                 //Canopen     ID��
    VarDateFlash0[119] = SystemVar.CanopenBaudrate;		        //Canopen     ������

    VarDateFlash0[150] = SystemError.SpeedFdbpPost;
    VarDateFlash0[151] = SystemError.MotorRunFlag;
    VarDateFlash0[152] = SystemError.OverLoadTimer;
    VarDateFlash0[153] = SystemError.CoderTimer;
    VarDateFlash0[154] = SystemError.DcCoeff;
    VarDateFlash0[155] = SystemError.PositiveDcCoeff;

    VarDateFlash0[156] = SystemError.IqsMax;
    VarDateFlash0[157] = SystemError.IdsMax;
    VarDateFlash0[158] = SystemError.IqsMaxOverTime;
    VarDateFlash0[159] = SystemError.DcRef;
    VarDateFlash0[160] = SystemError.DcRefK;
    VarDateFlash0[161] = SystemError.DcRefB;

    VarDateFlash0[162] = SystemError.AdDcVolt;
    VarDateFlash0[163] = SystemError.SysErr;
    VarDateFlash0[164] = SystemError.LowSpeedTimer;
    VarDateFlash0[165] = SystemError.DcVolt;
    VarDateFlash0[166] = SystemError.DcRefMax;
    VarDateFlash0[167] = SystemError.DcMaxErrRef;
    VarDateFlash0[168] = SystemError.DcRefMin;
    VarDateFlash0[169] = SystemError.DcMinErrRef;
    VarDateFlash0[170] = SystemError.DcArrestMax;
    VarDateFlash0[171] = SystemError.DcArrestDelayMax;
    VarDateFlash0[172] = SystemError.DcArrest2DelayMax;
    VarDateFlash0[173] = SystemError.ImeMax;

    VarDateFlash0[174] = SystemError.LossACTimeMax;
    VarDateFlash0[175] = SystemError.NoSysErrPowerOff;
    VarDateFlash0[176] = SystemError.ImeasCheckDelay;
    VarDateFlash0[177] = SystemError.SysErrRuningAsk;
    VarDateFlash0[178] = SystemError.SysErrRuning;
    VarDateFlash0[179] = SystemError.SysResetVar;
    VarDateFlash0[180] = SystemError.SysErrSaveLock;
    VarDateFlash0[181] = SystemError.SaveAllRsetFlag;
    VarDateFlash0[182] = SystemError.RuningMode; //����ģʽ   CANCOPEN==1 MODBUS==2
    VarDateFlash0[183] = SystemError.SaveAllParaFlag;
    VarDateFlash0[184] = SystemError.OverSpeed;
    VarDateFlash0[185] = SystemError.KpGainCoeff;
    VarDateFlash0[186] = SystemError.KiGainCoeff;
    VarDateFlash0[187] = SystemError.ThresholdCoeff;
    VarDateFlash0[188] = SystemError.TorqueCoeff;
    VarDateFlash0[189] = SystemError.TorquePIDLimitCoeff;
    VarDateFlash0[190] = SystemError.TorqueDECLimitCoeff;

    VarDateFlash0[200] = MotorControler.State = 0;
    VarDateFlash0[201] = MotorControler.SpeedRef = 0;
    VarDateFlash0[202] = MotorControler.SpeedFdb = 0;
    VarDateFlash0[203] = MotorControler.TorqueRef = 700;
    VarDateFlash0[204] = MotorControler.TorqueFdb = 0;
    VarDateFlash0[205] = MotorControler.PositionRef = 0;
    VarDateFlash0[206] = MotorControler.PositionFdb = 0;
    VarDateFlash0[207] = MotorControler.Error = 5;
    VarDateFlash0[208] = MotorControler.MotorActivePostion = 0;
    VarDateFlash0[209] = MotorControler.RotorCount = 0;

    VarDateFlash0[210] = MotorControler.RatedTorque; //�ת��
    VarDateFlash0[211] = MotorControler.MaxTorque;   //���ת��
    VarDateFlash0[212] = MotorControler.MaxSpeed;

    VarDateFlash0[213] = MotorControler.SpeedAcc;
    VarDateFlash0[214] = MotorControler.SpeedDcc;

    VarDateFlash0[215] = MotorControler.spiread_anlge;
    VarDateFlash0[216] = MotorControler.AngleFromMT6835;
    VarDateFlash0[217] = MotorControler.AngleFromMT6835Offset;
    VarDateFlash0[218] = MotorControler.AngleFromMT6835Offset1;

    VarDateFlash1[0] = Uds_OutMax;
    VarDateFlash1[1] = Uds_OutMin;
    VarDateFlash1[2] = Uds_UiMax >> 16;
    VarDateFlash1[3] = Uds_UiMax & 0xffff;
    VarDateFlash1[4] = Uds_UiMin >> 16;
    VarDateFlash1[5] = Uds_UiMin & 0xffff;
    VarDateFlash1[6] = Uds_Kp;
    VarDateFlash1[7] = Uds_Ki;

    VarDateFlash1[20] = Uqs_OutMax;
    VarDateFlash1[21] = Uqs_OutMin;
    VarDateFlash1[22] = Uqs_UiMax >> 16;
    VarDateFlash1[23] = Uqs_UiMax & 0xffff;
    VarDateFlash1[24] = Uqs_UiMin >> 16;
    VarDateFlash1[25] = Uqs_UiMin & 0xffff;
    VarDateFlash1[26] = Uqs_Kp;
    VarDateFlash1[27] = Uqs_Ki;

    VarDateFlash1[40] = pidpt_OutMax;
    VarDateFlash1[41] = pidpt_OutMin;
    VarDateFlash1[42] = pidpt_UiMax >> 16;
    VarDateFlash1[43] = pidpt_UiMax & 0xffff;
    VarDateFlash1[44] = pidpt_UiMin >> 16;
    VarDateFlash1[45] = pidpt_UiMin & 0xffff;
    VarDateFlash1[46] = pidpt_Kp;
    VarDateFlash1[47] = pidpt_Ki;

    VarDateFlash1[60] = pidpv_OutMax;
    VarDateFlash1[61] = pidpv_OutMin;
    VarDateFlash1[62] = pidpv_UiMax >> 16;
    VarDateFlash1[63] = pidpv_UiMax & 0xffff;
    VarDateFlash1[64] = pidpv_UiMin >> 16;
    VarDateFlash1[65] = pidpv_UiMin & 0xffff;
    VarDateFlash1[66] = pidpv_Kp;
    VarDateFlash1[67] = pidpv_Ki;

    VarDateFlash1[80] = pidholding_OutMax;
    VarDateFlash1[81] = pidholding_OutMin;
    VarDateFlash1[82] = pidholding_UiMax >> 16;
    VarDateFlash1[83] = pidholding_UiMax & 0xffff;
    VarDateFlash1[84] = pidholding_UiMin >> 16;
    VarDateFlash1[85] = pidholding_UiMin & 0xffff;
    VarDateFlash1[86] = pidholding_Kp;
    VarDateFlash1[87] = pidholding_Ki;

    VarDateFlash1[100] = pidc_position_OutMax;
    VarDateFlash1[101] = pidc_position_OutMin;
    VarDateFlash1[102] = pidc_position_UiMax >> 16;
    VarDateFlash1[103] = pidc_position_UiMax & 0xffff;
    VarDateFlash1[104] = pidc_position_UiMin >> 16;
    VarDateFlash1[105] = pidc_position_UiMin & 0xffff;
    VarDateFlash1[106] = pidc_position_Kp;
    VarDateFlash1[107] = pidc_position_Ki;


    VarDateFlash2[0] = (short)(PostionPlan_accel_max * 1000);
    VarDateFlash2[1] = (short)(PostionPlan_a_accel * 1000);
    VarDateFlash2[2] = (short)(PostionPlan_a_decel * 1000);
    VarDateFlash2[3] = (short)(PostionPlan_decel_max * 1000);

    VarDateFlash2[4] = PostionPlan_vel_init;
    VarDateFlash2[5] = PostionPlan_vel_tar;

    MyFLASH_WriteWord(FLASH_ADDRESS0, (uint16_t*)VarDateFlash0, 512);//дֵ
    MyFLASH_WriteWord(FLASH_ADDRESS1, (uint16_t*)VarDateFlash1, 512);//дֵ
    MyFLASH_WriteWord(FLASH_ADDRESS2, (uint16_t*)VarDateFlash2, 512);//дֵ
}


//�������
void SaveAllVar(void)
{

    VarDateFlash0[0] = SystemVar.SoftwareVersion;
    VarDateFlash0[1] = SystemVar.HardwareVersion;
    VarDateFlash0[2] = SystemVar.MotorVersion;


    /****��������*********/
    VarDateFlash0[104] = SystemVar.VF_Voltage;
    VarDateFlash0[105] = SystemVar.VF_ElectricalAngle;
    VarDateFlash0[106] = SystemVar.VF_Coefficient;
    VarDateFlash0[107] = SystemVar.VF_Coefficient_B;
    VarDateFlash0[108] = SystemVar.VF_ElectricalAngleStepMax;
    VarDateFlash0[109] = SystemVar.VF_ElectricalAngleStepMin;
    VarDateFlash0[110] = SystemVar.VF_ElectricalAngleStep;
    VarDateFlash0[111] = SystemVar.VF_ElectricalAngleStepCount;
    /****��������*********/

    /****�������*********/
    VarDateFlash0[112] = SystemVar.test_uqs;
    VarDateFlash0[113] = SystemVar.test_angle;

    /****ͨ�Ų���*********/
    VarDateFlash0[116] = SystemVar.ModbusID;	                 //Modbus      ID��
    VarDateFlash0[117] = SystemVar.ModbusBaudrate;	           //Modbus      ������
    VarDateFlash0[118] = SystemVar.CanopenID;                  //Canopen     ID��
    VarDateFlash0[119] = SystemVar.CanopenBaudrate;		         //Canopen     ������

    VarDateFlash0[150] = SystemError.SpeedFdbpPost;
    VarDateFlash0[151] = SystemError.MotorRunFlag;
    VarDateFlash0[152] = SystemError.OverLoadTimer;
    VarDateFlash0[153] = SystemError.CoderTimer;
    VarDateFlash0[154] = SystemError.DcCoeff;
    VarDateFlash0[155] = SystemError.PositiveDcCoeff;

    VarDateFlash0[156] = SystemError.IqsMax;
    VarDateFlash0[157] = SystemError.IdsMax;
    VarDateFlash0[158] = SystemError.IqsMaxOverTime;
    VarDateFlash0[159] = SystemError.DcRef;
    VarDateFlash0[160] = SystemError.DcRefK;
    VarDateFlash0[161] = SystemError.DcRefB;

    VarDateFlash0[162] = SystemError.AdDcVolt;
    VarDateFlash0[163] = SystemError.SysErr;
    VarDateFlash0[164] = SystemError.LowSpeedTimer;
    VarDateFlash0[165] = SystemError.DcVolt;
    VarDateFlash0[166] = SystemError.DcRefMax;
    VarDateFlash0[167] = SystemError.DcMaxErrRef;
    VarDateFlash0[168] = SystemError.DcRefMin;
    VarDateFlash0[169] = SystemError.DcMinErrRef;
    VarDateFlash0[170] = SystemError.DcArrestMax;
    VarDateFlash0[171] = SystemError.DcArrestDelayMax;
    VarDateFlash0[172] = SystemError.DcArrest2DelayMax;
    VarDateFlash0[173] = SystemError.ImeMax;

    VarDateFlash0[174] = SystemError.LossACTimeMax;
    VarDateFlash0[175] = SystemError.NoSysErrPowerOff;
    VarDateFlash0[176] = SystemError.ImeasCheckDelay;
    VarDateFlash0[177] = SystemError.SysErrRuningAsk;
    VarDateFlash0[178] = SystemError.SysErrRuning;
    VarDateFlash0[179] = SystemError.SysResetVar;
    VarDateFlash0[180] = SystemError.SysErrSaveLock;
    VarDateFlash0[181] = SystemError.SaveAllRsetFlag;
    VarDateFlash0[182] = SystemError.RuningMode; //����ģʽ   CANCOPEN==1 MODBUS==2
    VarDateFlash0[183] = SystemError.SaveAllParaFlag;
    VarDateFlash0[184] = SystemError.OverSpeed;
    VarDateFlash0[185] = SystemError.KpGainCoeff;
    VarDateFlash0[186] = SystemError.KiGainCoeff;
    VarDateFlash0[187] = SystemError.ThresholdCoeff;
    VarDateFlash0[188] = SystemError.TorqueCoeff;
    VarDateFlash0[189] = SystemError.TorquePIDLimitCoeff;
    VarDateFlash0[190] = SystemError.TorqueDECLimitCoeff;

    VarDateFlash0[200] = MotorControler.State = 0;
    VarDateFlash0[201] = MotorControler.SpeedRef = 0;
    VarDateFlash0[202] = MotorControler.SpeedFdb = 0;
    VarDateFlash0[203] = MotorControler.TorqueRef;
    VarDateFlash0[204] = MotorControler.TorqueFdb = 0;
    VarDateFlash0[205] = MotorControler.PositionRef = 0;
    VarDateFlash0[206] = MotorControler.PositionFdb = 0;
    VarDateFlash0[207] = MotorControler.Error = 5;
    VarDateFlash0[208] = MotorControler.MotorActivePostion = 0;
    VarDateFlash0[209] = MotorControler.RotorCount = 0;

    VarDateFlash0[210] = MotorControler.RatedTorque; //�ת��
    VarDateFlash0[211] = MotorControler.MaxTorque;   //���ת��
    VarDateFlash0[212] = MotorControler.MaxSpeed;
    VarDateFlash0[213] = MotorControler.SpeedAcc;
    VarDateFlash0[214] = MotorControler.SpeedDcc;

    VarDateFlash0[215] = MotorControler.spiread_anlge;
    VarDateFlash0[216] = MotorControler.AngleFromMT6835;
    VarDateFlash0[217] = MotorControler.AngleFromMT6835Offset;
    VarDateFlash0[218] = MotorControler.AngleFromMT6835Offset1;

    VarDateFlash1[0] = Uds_OutMax;
    VarDateFlash1[1] = Uds_OutMin;
    VarDateFlash1[2] = Uds_UiMax >> 16;
    VarDateFlash1[3] = Uds_UiMax & 0xffff;
    VarDateFlash1[4] = Uds_UiMin >> 16;
    VarDateFlash1[5] = Uds_UiMin & 0xffff;
    VarDateFlash1[6] = Uds_Kp;
    VarDateFlash1[7] = Uds_Ki;

    VarDateFlash1[20] = Uqs_OutMax;
    VarDateFlash1[21] = Uqs_OutMin;
    VarDateFlash1[22] = Uqs_UiMax >> 16;
    VarDateFlash1[23] = Uqs_UiMax & 0xffff;
    VarDateFlash1[24] = Uqs_UiMin >> 16;
    VarDateFlash1[25] = Uqs_UiMin & 0xffff;
    VarDateFlash1[26] = Uqs_Kp;
    VarDateFlash1[27] = Uqs_Ki;

    VarDateFlash1[40] = pidpt_OutMax;
    VarDateFlash1[41] = pidpt_OutMin;
    VarDateFlash1[42] = pidpt_UiMax >> 16;
    VarDateFlash1[43] = pidpt_UiMax & 0xffff;
    VarDateFlash1[44] = pidpt_UiMin >> 16;
    VarDateFlash1[45] = pidpt_UiMin & 0xffff;
    VarDateFlash1[46] = pidpt_Kp;
    VarDateFlash1[47] = pidpt_Ki;

    VarDateFlash1[60] = pidpv_OutMax;
    VarDateFlash1[61] = pidpv_OutMin;
    VarDateFlash1[62] = pidpv_UiMax >> 16;
    VarDateFlash1[63] = pidpv_UiMax & 0xffff;
    VarDateFlash1[64] = pidpv_UiMin >> 16;
    VarDateFlash1[65] = pidpv_UiMin & 0xffff;
    VarDateFlash1[66] = pidpv_Kp;
    VarDateFlash1[67] = pidpv_Ki;

    VarDateFlash1[80] = pidholding_OutMax;
    VarDateFlash1[81] = pidholding_OutMin;
    VarDateFlash1[82] = pidholding_UiMax >> 16;
    VarDateFlash1[83] = pidholding_UiMax & 0xffff;
    VarDateFlash1[84] = pidholding_UiMin >> 16;
    VarDateFlash1[85] = pidholding_UiMin & 0xffff;
    VarDateFlash1[86] = pidholding_Kp;
    VarDateFlash1[87] = pidholding_Ki;

    VarDateFlash1[100] = pidc_position_OutMax;
    VarDateFlash1[101] = pidc_position_OutMin;
    VarDateFlash1[102] = pidc_position_UiMax >> 16;
    VarDateFlash1[103] = pidc_position_UiMax & 0xffff;
    VarDateFlash1[104] = pidc_position_UiMin >> 16;
    VarDateFlash1[105] = pidc_position_UiMin & 0xffff;
    VarDateFlash1[106] = pidc_position_Kp;
    VarDateFlash1[107] = pidc_position_Ki;

    VarDateFlash2[0] = (short)(PostionPlan_accel_max * 1000);
    VarDateFlash2[1] = (short)(PostionPlan_a_accel * 1000);
    VarDateFlash2[2] = (short)(PostionPlan_a_decel * 1000);
    VarDateFlash2[3] = (short)(PostionPlan_decel_max * 1000);

    VarDateFlash2[4] = PostionPlan_vel_init;
    VarDateFlash2[5] = PostionPlan_vel_tar;

    MyFLASH_WriteWord(FLASH_ADDRESS0, (uint16_t*)VarDateFlash0, 512);//дֵ
    MyFLASH_WriteWord(FLASH_ADDRESS1, (uint16_t*)VarDateFlash1, 512);//дֵ
    MyFLASH_WriteWord(FLASH_ADDRESS2, (uint16_t*)VarDateFlash2, 512);//дֵ
}

//��Դ����
void PowerManage(void)
{
    static short DcMinErr = 0;
    static short DcMaxErr = 0;

    SystemError.DcRef = 2400;
    SystemError.DcRefMin = 1600;
    SystemError.DcRefMax = 2800;
    //SystemError.Temp = (short)((ADC_ConvertedValue[4]));
    SystemError.DcVolt = (short)(((float)ADC_ConvertedValue[2]) * 0.8942f) + 15;
    SystemError.DcCoeff = (((int)SystemError.DcRef) << 12) / ((int)SystemError.DcVolt);

    /***********************��ѹ���Կ�ʼ************************/
    DcMinErr = DcMinErr + (SystemError.DcRefMin - SystemError.DcVolt);

    if(DcMinErr <= 0)
    {
        DcMinErr = 0;
        if(SystemError.SysErr == M_SYSERR_DV)
        {
            SystemError.SysErr = 0;
        }
    }
    else if(DcMinErr >= SystemError.DcMinErrRef)
    {
        DcMinErr = SystemError.DcMinErrRef;
        SystemError.SysErr = M_SYSERR_DV;
    }

    /***********************��ѹ���Խ���************************/

    /***********************��ѹ���Կ�ʼ************************/
    DcMaxErr = DcMaxErr + (SystemError.DcVolt - SystemError.DcRefMax);

    if(DcMaxErr <= 0)
    {
        DcMaxErr = 0;
        if(SystemError.SysErr == M_SYSERR_OV)
        {
            SystemError.SysErr = 0;
        }
    }
    else if(DcMaxErr >= SystemError.DcMaxErrRef)
    {
        DcMaxErr = SystemError.DcMaxErrRef;
        SystemError.SysErr = M_SYSERR_OV;
    }

    /***********************��ѹ���Խ���************************/
}


void SysErrManage(void)
{
    if(SystemError.SysErr)
    {
        if(SystemError.SysClearFlag)
        {
            SystemError.SysClearFlag = 0;

            switch(SystemError.SysErr)
            {
                case M_SYSERR_IPM :    //ϵͳ1
                {

                }
                break;

                case M_SYSERR_OV :    //��ѹ
                {

                }
                break;
                case M_SYSERR_DV :   //��ѹ
                {

                }
                break;

                case M_SYSERR_CODER :  //������û�б궨���
                {

                }
                break;

                case M_SYSERR_CODERNESS :   //��������ȡ����
                {
                    SystemError.SysClearFlag = 0;
                }
                break;

                case M_SYSERR_CURRENTSENSOR :  //����������
                {
                    SystemError.SysErr = 0;
                    SystemError.ImeasOffsetFlag = 0; //���²���  offsetֵ
                    SystemError.ImeasCheckDelay = 0;
                }
                break;

                case M_SYSERR_LACKPHASE :    //ȱ��
                {

                }
                break;
                case M_SYSERR_ROTOR_LOCKED :  //��ת
                {

                }
                break;
                case M_SYSERR_OVER_LOAD :    //����
                {

                }
                break;

                case M_SYSERR_OVER_SPEED :  //������ٹ���
                {
                    SystemError.SysErr = 0;
                }
                break;
                case M_SYSERR_ARRESTER :   //�ƶ���·����
                {

                }
                break;
                case M_SYSERR_ARRESTER2 : //�ƶ���·����2
                {

                }
                break;
                case M_SYSERR_LOCATED1 :  //ϵͳ��λ����1
                {
                    SystemError.SysErr = 0;
                }
                break;
                case M_SYSERR_LOCATED2 :  //ϵͳ��λ����2
                {
                    SystemError.SysErr = 0;
                }
                break;
                case M_SYSERR_LOCATED3 :  //ϵͳ��λ����3
                {
                    SystemError.SysErr = 0;
                }
                break;
                case M_SYSERR_COMM :    //��λ��ͨ�Ź���
                {

                }
                break;

                case M_SYSERR_POWEROFF :    //������Դ����
                {

                }
                break;

                case M_SYSERR_SYSVARRESET :    //ϵͳ��λ����
                {

                }
                break;


                case M_SYSERR_OVERTolerance1 :    //����1
                {

                }
                break;


                case M_SYSERR_OVERTolerance2 :    //����2
                {

                }
                break;

                case M_SYSERR_OVERTolerance3 :    //ϵͳ��λ����
                {

                }
                break;

                case M_SYSERR_OVERTolerance4 :    //ϵͳ��λ����
                {

                }
                break;

                default:
                {
                }
                break;
            }
        }
        else   //  ��� û�й������������ǿ����ʹ�ܡ�
        {
            if(SystemError.SysErr != M_SYSERR_CODER)
            {
                MotorControler.State = 0; //��ʹ��
                svpwm.SvpwmControlState = 0;


            }
        }

    }


}

/*led  �̵� ��ʾ  �ϵ硢 ��ʹ�ܡ��˶��� ��� ������ʾ�����5S*/
void Led(void)
{
    static short R_count = 0;      //��ɫLED��˸����
    static short R_count_flag = 0; //��ɫLED�����־
    static short R_count_Max = 0;  //��ɫLED��˸������
    static short R_count_flag2 = 0;//��ɫLED״̬����־

    static short G_count = 0;      //��ɫLED��˸����
    static short G_count_flag = 0; //��ɫLED�����־
    static short G_count_Max = 0;  //��ɫLED��˸������
    LedStateG = 1; //��ʼ״̬����ɫLED����
   
   	//��ɫָʾ��״̬�ж�
    //if((svpwm.UQs < -1000) || (svpwm.UQs > 1000))
    if(SystemError.PwmOn == 1)
    {
        LedStateG = 2;//ʹ��״̬����
    }

    if((MotorControler.SpeedFdbp > 3) || (MotorControler.SpeedFdbp < -3))
    {
        LedStateG = 3;//����״̬����
    }
    //��ɫLED״̬�ж�
		//��������
    LedStateR = SystemError.SysErr;
    if(LedStateR)
    {
        LedStateG = 0;//�ر���ɫLED
    }
    //��ɫLED�����߼�
    if(LedStateG == 0)
    {
        G_count = 0; //�ϵ糣��
        LED1_OFF;
    }

    if(LedStateG == 1)
    {
        G_count = 0; //�ϵ糣��
        LED1_ON;
    }

    else if(LedStateG == 2)
    {
        G_count_Max = 4; //ʹ��ʱ����
    }

    else if(LedStateG == 3)
    {
        G_count_Max = 1; //�˶�ʱ����
    }
    //��ɫLED��˸����
		//�ﵽ��˸����
    if(G_count > G_count_Max)
    {   
        G_count = 0;
			//�л�����״̬
        G_count_flag = !G_count_flag;
        if(G_count_flag)
        {
            LED1_ON;
        }
        else
        {
            LED1_OFF;
        }
    }
    G_count++;//��������
		//��ɫLED����
    if((R_count_flag2 == 0) && (LedStateR)) //��⵽����
    {
        R_count_flag2 = 1;
        R_count_Max = LedStateR;
    }

    if(R_count_flag2 == 1)  //��ʾ���ϴ���
    {   //��ת����״̬
        R_count_flag = !R_count_flag;
        if(R_count_flag)
        {
            LED2_ON;
        }
        else
        {
            LED2_OFF;
            R_count++;

            if(R_count >= R_count_Max)
            {
                R_count_flag2 = 2;
            }
        }
    }
    else  if(R_count_flag2 == 2)//���ϱ���
    {
        R_count++;
        if(R_count > 10)
        {
            R_count = 0;
            R_count_flag2 = 0;
        }
    }
}
