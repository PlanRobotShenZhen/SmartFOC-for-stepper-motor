#include "ExternGlobals.h"
#include "Spi.h"
#include "Ds402_Slave.h"
#include "CanOpenMode.h"
#include "Function.h"
#include "Kalman.h"
extern Set_SP_Para PostionPlanCiA402Mode_1;
extern int PostionPlanStepCount;
extern int PostionPlanStepMax;
extern int PostionPlanStep;
extern bool postion_direction_flag;
int current_ref_test=1;
int UQsRef_test=5000;

short deadtime=15;
short  Moving_Average_Window_Filter_4096(short input, short CHANNEL_ID,short width);
short  Moving_Average_Window_Filter(short input, short CHANNEL_ID,short width);
Current_value value;

void SpeedCalculate(void);   // 1K    HZ
void MotorControl(void);     // 1K    HZ
void SvpwmControl(void) ;    // 16K   HZ
void CurrentLoopISR(void);   // 16K   HZ
void PID_IdIq(void);         // 16K   HZ
void park(SVPVM *v);


short tmp20 = 0;             //��������
short speedref_test=0;
int As_mean;
int Bs_mean;
int TEMP_A;
int TEMP_B;
int TEMP_Uq_filter32;
int TEMP_Uq_filter8;

/************************ SVPWM����״̬�궨�� ************************/
#define SVPWM_STATE_DISABLE           0   // �ر�״̬����ʼ������������
#define SVPWM_STATE_VF_MODE           1   // V/F����ģʽ
#define SVPWM_STATE_FIND_ZERO         2   // Ѱ�����
#define SVPWM_STATE_BASIC_TEST        3   // �������ܲ���ģʽ
#define SVPWM_STATE_POS_LOCK          4   // ʹ�ܺ�λ������ģʽ
#define SVPWM_STATE_CURRENT_MODE      5   // ��������ģʽ
#define SVPWM_STATE_SPEED_MODE        6   // �ٶȿ���ģʽ����Ӧ��λ���ٶȿ��ƣ�
#define SVPWM_STATE_SPEED_MODE2       61  // �ٶȿ���ģʽ2��δʹ�ã�
#define SVPWM_STATE_POS_MODE          7   // λ�ñջ�����ģʽ
#define SVPWM_STATE_POS_LOCK_TEST     10  // λ����������ģʽ���������ã�

/************************ �������״̬�궨�� ************************/
#define MOTOR_STATE_STOP              0   // ���ֹͣ״̬����ʼ��������
#define MOTOR_STATE_VF_RUN            1   // V/Fģʽ����
#define MOTOR_STATE_FIND_ZERO         2   // Ѱ����� 
#define MOTOR_STATE_BASIC_TEST        3   // �������ܲ���
#define MOTOR_STATE_LOCK_AFTER_ENABLE 4   // ʹ�ܺ��λ������
#define MOTOR_STATE_TORQUE_CONTROL    5   // ת�ؿ���ģʽ
#define MOTOR_STATE_SPEED_CONTROL     6   // �ٶȱջ�����
#define MOTOR_STATE_SPEED_CONTROL2    61  // �ٶȱջ�����2�����ã�
#define MOTOR_STATE_POS_HOLD          7   // λ�ñ���ģʽ
#define MOTOR_STATE_POS_MOVE          71  // λ�ù滮�˶�ģʽ��ִ��λ�ù켣��

#define Bias_current_mode 3

#define FLITER_MODE 0            //1Ϊ�������˲���0Ϊ������ֵ

/************************ǰ������ ************************/
 static int prev_speed_ref = 0; // ��һ���ڵ��ٶ�����
 #define FF_GAIN 10 // ǰ������

#if FLITER_MODE == 0
void CurrentLoopISR(void)   //CurrentLoopISR 16K HZ     ��������ISR
{
	
	#if Bias_current_mode==1
    if (SystemError.ImeasOffsetFlag < 10000) {  //�ڳ�ʼʱ�����ɼ�����λ�ĵ���ƫ����Ϊ�����ɼ����������
        SystemError.ImeasAOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);//ADC��ȡ��A������ľ�ֵ �õ�ϵͳ����е�A��λ��������
        SystemError.ImeasBOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);//ADC��ȡ��B������ľ�ֵ �õ�ϵͳ����е�B��λ��������
		    SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
			
    } else 
		{ //�������������ȥ����ֵӦΪ��ʵ����ֵ
        TEMP_A = Moving_Average_Window_Filter(SystemError.ImeasA,0,8) - SystemError.ImeasAOffset;			//As ��������ǲ������� - ���������Ĳ���  
        TEMP_B = Moving_Average_Window_Filter(SystemError.ImeasB,1,8) - SystemError.ImeasBOffset;			//Bs ��������ǲ������� - ���������Ĳ���  
		    As_mean = Moving_Average_Window_Filter_4096(TEMP_A,2,4095);
		    Bs_mean = Moving_Average_Window_Filter_4096(TEMP_B,3,4095);
		    svpwm.As=TEMP_A-As_mean;
		    svpwm.Bs=TEMP_B-Bs_mean;
		}
	#endif
	#if	Bias_current_mode==2
	    if (SystemError.ImeasOffsetFlag < 10000) 
		{  //�ڳ�ʼʱ�����ɼ�����λ�ĵ���ƫ����Ϊ�����ɼ����������
        SystemError.ImeasAOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);//ADC��ȡ��A������ľ�ֵ �õ�ϵͳ����е�A��λ��������
        SystemError.ImeasBOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);//ADC��ȡ��B������ľ�ֵ �õ�ϵͳ����е�B��λ��������
		    SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
		} 
		else 
		{ //�������������ȥ����ֵӦΪ��ʵ����ֵ
		    svpwm.As =SystemError.ImeasA- SystemError.ImeasAOffset-30;			//As ��������ǲ������� - ���������Ĳ���  
		    svpwm.Bs =SystemError.ImeasB - SystemError.ImeasBOffset+40;
		}

		
	#endif
	#if Bias_current_mode==3
		  SystemError.ImeasAOffset = Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);
		  SystemError.ImeasBOffset = Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);
	
	  	svpwm.As = Moving_Average_Window_Filter(SystemError.ImeasA,0,8)- SystemError.ImeasAOffset;			//As ��������ǲ������� - ���������Ĳ���  
    	svpwm.Bs = Moving_Average_Window_Filter(SystemError.ImeasB,1,8) - SystemError.ImeasBOffset;
	#endif
	//�����Ƕ� = ��е�Ƕȡ�������
		
    svpwm.Angle2 = (((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset1) & 0x7fff))) * ((
                   int)MotorControler.MotorPoles)) & 0x7fff;           //����0x7fff����ʹ���ݱ�����15λ��Χ��
		//park
	  park(&svpwm);
    PID_IdIq();     //���������Ǽ����������õĽǶ���Ϣ����
    SvpwmControl(); // 16KHZ  ���ݲ�ͬ�Ĺ���ģʽ�趨��Ӧ��Uq��Uq�仯��������

    //б�±ƽ����ɲ�ͬģʽ��Ӧ�Ĳ����仯����ֹ����
    if (svpwm.UQs < svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs + svpwm.UQsstep;
    }

    if (svpwm.UQs > svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs - svpwm.UQsstep;
    }

    //�޷�
    if (svpwm.UQs > svpwm.UQ_MAX) {
        svpwm.UQs = svpwm.UQ_MAX;
    }

    if (svpwm.UQs < -svpwm.UQ_MAX) {
        svpwm.UQs = -svpwm.UQ_MAX;
    }

    if ((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER)) { //����ϵͳ���ϣ�ǿ�ƹر����
        svpwm.UQs = 0;
        svpwm.UDs = 0;
    }

    ipark(&svpwm);           //��Park�任����d-q����ת������-������
    PWM(&svpwm);             //��Va��Vb����ռ�ձ�


	  TIM1->CCDAT1 = svpwm.Va;  // A��ռ�ձ�
    TIM1->CCDAT2 = svpwm.Va;  // A�໥��ռ�ձ�
    TIM1->CCDAT3 = svpwm.Vb;  // B��ռ�ձ�
    TIM1->CCDAT4 = svpwm.Vb;  // B�໥��ռ�ձ�
}

/*-----------------------------------------------*/
void PID_IdIq(void)					 //D��Q�������PID
{
	  int speed_change = pidpv.Ref - prev_speed_ref; // �ٶȱ仯��
    prev_speed_ref = pidpv.Ref; // ���·���ֵ
	
    //D�������
    UdsPid.Ref = 0;
	  UdsPid.Fdb = svpwm.IDs;
    UdsPid.calc(&UdsPid);
		
    //Q�������
    UqsPid.Fdb =svpwm.IQs;//q15
		
    //�ٶȻ����������ǰ������
	  float pidOutput = pidpv.Out * 500 / 8000;
    int feedforward = speed_change * FF_GAIN; //ǰ������ֵ
	  UqsPid.Ref = pidOutput + feedforward;

    UqsPid.CurrentRef = svpwm.CurrentRef;
		UqsPid.SpeedFdbp  = MotorControler.SpeedFdbp;
    UqsPid.UQs = svpwm.UQs;
    UqsPid.calc(&UqsPid);//���ڵ����޷�
    MotorControler.AngleFromMT6835Offset = MotorControler.AngleFromMT6835Offset1 + tmp20; // + UdsPid.Out;
		
}
/*-----------------------------------------------*/


void SvpwmControl(void)   // 16K HZ					SVPWM����
{
    static short RotorElectricalMacAngle = 0;
    int   RotorMacAngleTemp = 0;

    switch (svpwm.SvpwmControlState) {
    case SVPWM_STATE_DISABLE ://��ʹ��
        PwmShut();  //�������
        svpwm.DcCoeff = SystemError.DcCoeff;     //32400; //_IQ(0.99);
        svpwm.PeriodMax = 4499;
        svpwm.MfuncPeriod = 0x7fff;
        svpwm.UDs = 0;
        svpwm.UQsRef = 0;
        svpwm.UQs = 0;
		    svpwm.UQ_MAX =_IQ15(0.9);//8192//16384//29491
        RotorElectricalMacAngle = 0;
        pidpv.Ui = 0;
        pidholding.Ui = 0;
        UdsPid.Ui = 0;         //���ø���PID����
        UqsPid.Ui = 0;
        UdsPid.OutMax = _IQ15(0.3);
        UdsPid.OutMin = _IQ15(-0.3);
        UdsPid.UiMax = _IQ30(0.2);
        UdsPid.UiMin = _IQ30(-0.2);
		
        UdsPid.Kp = 200;
        UdsPid.Ki = 30;
        UdsPid.Err = 0;
		
        UqsPid.Kp = 2000;
        UqsPid.Ki =5;
        UqsPid.Err = 0;
				
        UqsPid.OutMax = _IQ15(0.9);
        UqsPid.OutMin = _IQ15(-0.9);
        UqsPid.UiMax = pidpv_UiMax;
        UqsPid.UiMin = pidpv_UiMin;
        svpwm.UQsstep = 20;
        //LED_Red_Flash();//���Ե�********************************
        break;

    case SVPWM_STATE_VF_MODE :    //  V/Fģʽ
        svpwm.DcCoeff = SystemError.DcCoeff;//��ѹ����ϵ������
        PwmOpen();
       // svpwm.UQsRef = SystemVar.VF_Voltage;   //ʹ�ù̶��Ŀ����������е��
		    svpwm.UQsRef = 4000;
	
        svpwm.Angle = SystemVar.VF_ElectricalAngle;
        break;

    case SVPWM_STATE_FIND_ZERO :    //  Ѱ�����
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorElectricalMacAngle = RotorElectricalMacAngle + 10;//ÿ��������10��λ
        RotorElectricalMacAngle = RotorElectricalMacAngle & 0x7fff;//��֤�Ƕ������÷�Χ��ѭ��
        svpwm.Angle = RotorElectricalMacAngle;
        break;

    case SVPWM_STATE_BASIC_TEST :    //  ��������
        svpwm.DcCoeff = 4096;//SystemError.DcCoeff;
        PwmOpen();
	    	//����������ȡ�ĽǶ�ת��Ϊ��Ƕ�
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.UQsRef = SystemVar.test_uqs;//���ò��Ե�ѹUqĿ��ֵ
        svpwm.Angle = RotorElectricalMacAngle;//���ñ�������Ƕ�
        break;

    case SVPWM_STATE_POS_LOCK ://  ��ʹ��  λ������
		if( MotorControler.State == 4){
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
       
				svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Fdb = MotorControler.MotorActivePostion;  //��������Ϊ����λ��
        pidholding.Speedref = 0;             //�ٶȲο�ֵ����Ϊ0��������ĳһλ�ò���
        pidholding.Speedfdb = MotorControler.SpeedFdbp;
        pidholding.Iqsfdb = svpwm.IQs;
		
        pidholding.calc(&pidholding);
		
        svpwm.UQsRef = pidholding.Out;
        UqsPid.Ui = 0;
        UqsPid.Out = 0;			//�����󣬻���һ���ֵ���
	   } else
	  	{
			 PwmShut();   //�������
			 pidholding.Ui = 0;   //��������
	    }
        break;

    case SVPWM_STATE_CURRENT_MODE :   // ����ģʽ
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        svpwm.UQs = pidpt.Out;
        break;

    case SVPWM_STATE_SPEED_MODE :   // �ٶ�ģʽ  --��Ӧ��λ�����ٶȿ���ģʽ
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
				//svpwm.UQsRef = (Ids_filter4(UqsPid.Out)*svpwm.UQ_MAX)/UqsPid.OutMax;		//����б�±ƽ���UqĿ��ֵΪ�ٶ�PID�������UqPID�����	
				TEMP_Uq_filter32 = Moving_Average_Window_Filter(UqsPid.Out,2,31);
				TEMP_Uq_filter8 = Moving_Average_Window_Filter(UqsPid.Out,3,7);
				
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref<80)
				{
//					svpwm.UQsRef=UqsPid.Out;
					svpwm.UQsRef=TEMP_Uq_filter32;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
//				svpwm.UQsRef = pidpv.Out;
//				svpwm.UQsRef=UQsRef_test;
        break;

    case SVPWM_STATE_SPEED_MODE2 :   // �ٶ�ģʽ2								--	û�б�ʹ�õ�
        svpwm.DcCoeff = SystemError.DcCoeff;//32400;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidptv.Ref = (pidpv.Out) >> 5; //���ٶȻ�PID���������ӳ��ɵ�����-�ٶ�ģʽPID��Ԥ��ֵ�����޷�
        if (pidptv.Ref > 600) {
            pidptv.Ref = 600;
        }

        if (pidptv.Ref < -600) {
            pidptv.Ref = -600;
        }

        pidptv.Fdb = svpwm.IQs;
        pidptv.calc(&pidptv);
        svpwm.UQs = pidptv.Out;  //����ģʽ�������ڶ�Ӧ��PID�����ֱ��������SVPWM��������ͨ��б�±ƽ�
        break;

    case SVPWM_STATE_POS_MODE:   //λ�û�ģʽ      -----------��λ�� λ�ÿ���ģʽ
		    svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
				TEMP_Uq_filter32 = Moving_Average_Window_Filter(UqsPid.Out,2,31);
				TEMP_Uq_filter8 = Moving_Average_Window_Filter(UqsPid.Out,3,7);
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
        break;

    case SVPWM_STATE_POS_LOCK_TEST ://  ��ʹ��  λ������  ������
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Ref = 0;
        pidholding.Fdb = MotorControler.MotorActivePostion;
        pidholding.calc(&pidholding);
        svpwm.UQs = pidholding.Out;
        break;

    default: {
    }
    break;
    }
}
#endif
#if FLITER_MODE == 1
void CurrentLoopISR(void)   //CurrentLoopISR 16K HZ     ��������ISR
{
	
	#if Bias_current_mode==1
    if (SystemError.ImeasOffsetFlag < 10000) {  //�ڳ�ʼʱ�����ɼ�����λ�ĵ���ƫ����Ϊ�����ɼ����������
        SystemError.ImeasAOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);//ADC��ȡ��A������ľ�ֵ �õ�ϵͳ����е�A��λ��������
        SystemError.ImeasBOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);//ADC��ȡ��B������ľ�ֵ �õ�ϵͳ����е�B��λ��������
		    SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
			
    } else 
		{ //�������������ȥ����ֵӦΪ��ʵ����ֵ
        TEMP_A = Kalman_Filter(SystemError.ImeasA,0) - SystemError.ImeasAOffset;			//As ��������ǲ������� - ���������Ĳ���  
        TEMP_B = Kalman_Filter(SystemError.ImeasB,1) - SystemError.ImeasBOffset;			//Bs ��������ǲ������� - ���������Ĳ���  
		As_mean = Kalman_Filter(TEMP_A,2);
		Bs_mean = Kalman_Filter(TEMP_B,3);
		svpwm.As=TEMP_A-As_mean;
		svpwm.Bs=TEMP_B-Bs_mean;
		}
	#endif
	#if	Bias_current_mode==2
	    if (SystemError.ImeasOffsetFlag < 10000) 
		{  //�ڳ�ʼʱ�����ɼ�����λ�ĵ���ƫ����Ϊ�����ɼ����������
        SystemError.ImeasAOffset =  Kalman_Filter(SystemError.ImeasA,0);//ADC��ȡ��A������ľ�ֵ �õ�ϵͳ����е�A��λ��������
        SystemError.ImeasBOffset =  Kalman_Filter(SystemError.ImeasB,1);//ADC��ȡ��B������ľ�ֵ �õ�ϵͳ����е�B��λ��������
		SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
		} 
		else 
		{ //�������������ȥ����ֵӦΪ��ʵ����ֵ
		svpwm.As =SystemError.ImeasA- SystemError.ImeasAOffset-30;			//As ��������ǲ������� - ���������Ĳ���  
		svpwm.Bs =SystemError.ImeasB - SystemError.ImeasBOffset+40;
		}

		
	#endif
	#if Bias_current_mode==3
		SystemError.ImeasAOffset = Kalman_Filter(SystemError.ImeasA,0);
		SystemError.ImeasBOffset = Kalman_Filter(SystemError.ImeasB,1);
	
	  	svpwm.As = Kalman_Filter(SystemError.ImeasA,0)- SystemError.ImeasAOffset;			//As ��������ǲ������� - ���������Ĳ���  
    	svpwm.Bs = Kalman_Filter(SystemError.ImeasB,1) - SystemError.ImeasBOffset;
	#endif
	//�����Ƕ� = ��е�Ƕȡ�������
		
    svpwm.Angle2 = (((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset1) & 0x7fff))) * ((
                   int)MotorControler.MotorPoles)) & 0x7fff;           //����0x7fff����ʹ���ݱ�����15λ��Χ��
		//park
	park(&svpwm);
    PID_IdIq();     //���������Ǽ����������õĽǶ���Ϣ����
    SvpwmControl(); // 16KHZ  ���ݲ�ͬ�Ĺ���ģʽ�趨��Ӧ��Uq��Uq�仯��������

    //б�±ƽ����ɲ�ͬģʽ��Ӧ�Ĳ����仯����ֹ����
    if (svpwm.UQs < svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs + svpwm.UQsstep;
    }

    if (svpwm.UQs > svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs - svpwm.UQsstep;
    }

    //�޷�
    if (svpwm.UQs > svpwm.UQ_MAX) {
        svpwm.UQs = svpwm.UQ_MAX;
    }

    if (svpwm.UQs < -svpwm.UQ_MAX) {
        svpwm.UQs = -svpwm.UQ_MAX;
    }

    if ((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER)) { //����ϵͳ���ϣ�ǿ�ƹر����
        svpwm.UQs = 0;
        svpwm.UDs = 0;
    }

    ipark(&svpwm);           //��Park�任����d-q����ת������-������
    PWM(&svpwm);             //��Va��Vb����ռ�ձ�


	  TIM1->CCDAT1= svpwm.Va;  // A��ռ�ձ�
    TIM1->CCDAT2= svpwm.Va;  // A�໥��ռ�ձ�
    TIM1->CCDAT3 = svpwm.Vb; // B��ռ�ձ�
    TIM1->CCDAT4 = svpwm.Vb; // B�໥��ռ�ձ�
}

/*-----------------------------------------------*/
void PID_IdIq(void)					 //D��Q�������PID
{
	  int speed_change = pidpv.Ref - prev_speed_ref; // �ٶȱ仯��
    prev_speed_ref = pidpv.Ref; // ����
    //D�������
    UdsPid.Ref = 0;
	  UdsPid.Fdb = svpwm.IDs;
    UdsPid.calc(&UdsPid);
		
    //Q�������
    UqsPid.Fdb =svpwm.IQs;     //q15
		
    //�ٶȻ����������ǰ������
	  float pidOutput = pidpv.Out * 500 / 8000;
    int feedforward = speed_change * FF_GAIN; //ǰ������ֵ
    UqsPid.Ref = pidOutput + feedforward;

    UqsPid.UQs = svpwm.UQs;
    UqsPid.calc(&UqsPid);//���ڵ����޷�
    MotorControler.AngleFromMT6835Offset = MotorControler.AngleFromMT6835Offset1 + tmp20; // + UdsPid.Out;
		
}
/*-----------------------------------------------*/

void SvpwmControl(void)   // 16K HZ					SVPWM����
{
    static short RotorElectricalMacAngle = 0;
    int   RotorMacAngleTemp = 0;

    switch (svpwm.SvpwmControlState) {
    case SVPWM_STATE_DISABLE ://��ʹ��
        PwmShut();  //�������
        svpwm.DcCoeff = SystemError.DcCoeff;//32400; //_IQ(0.99);
        svpwm.PeriodMax = 4499;
        svpwm.MfuncPeriod = 0x7fff;
        svpwm.UDs = 0;
        svpwm.UQsRef = 0;
        svpwm.UQs = 0;
	    	svpwm.UQ_MAX =_IQ15(0.8);//8192//16384//29491
        RotorElectricalMacAngle = 0;
        pidpv.Ui = 0;
        pidholding.Ui = 0;
        UdsPid.Ui = 0;         //���ø���PID����
        UqsPid.Ui = 0;
        UdsPid.OutMax = _IQ15(0.3);
        UdsPid.OutMin = _IQ15(-0.3);
        UdsPid.UiMax = _IQ30(0.2);
        UdsPid.UiMin = _IQ30(-0.2);
		
        UdsPid.Kp = 200;
        UdsPid.Ki = 30;
        UdsPid.Err = 0;
		
        UqsPid.Kp = 2000;
        UqsPid.Ki =5;
        UqsPid.Err = 0;
				
        UqsPid.OutMax = _IQ15(0.8);
        UqsPid.OutMin = _IQ15(-0.8);
        UqsPid.UiMax = pidpv_UiMax;
        UqsPid.UiMin = pidpv_UiMin;
        svpwm.UQsstep = 20;
        //LED_Red_Flash();//���Ե�********************************
        break;

    case SVPWM_STATE_VF_MODE :    //  V/Fģʽ
        svpwm.DcCoeff = SystemError.DcCoeff;//��ѹ����ϵ������
        PwmOpen();
       // svpwm.UQsRef = SystemVar.VF_Voltage;   //ʹ�ù̶��Ŀ����������е��
		svpwm.UQsRef = 4000;
	
        svpwm.Angle = SystemVar.VF_ElectricalAngle;
        break;

    case SVPWM_STATE_FIND_ZERO :    //  Ѱ�����
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorElectricalMacAngle = RotorElectricalMacAngle + 10;//ÿ��������10��λ
        RotorElectricalMacAngle = RotorElectricalMacAngle & 0x7fff;//��֤�Ƕ������÷�Χ��ѭ��
        svpwm.Angle = RotorElectricalMacAngle;
        break;

    case SVPWM_STATE_BASIC_TEST :    //  ��������
        svpwm.DcCoeff = 4096;//SystemError.DcCoeff;
        PwmOpen();
		//����������ȡ�ĽǶ�ת��Ϊ��Ƕ�
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.UQsRef = SystemVar.test_uqs;//���ò��Ե�ѹUqĿ��ֵ
        svpwm.Angle = RotorElectricalMacAngle;//���ñ�������Ƕ�
        break;

    case SVPWM_STATE_POS_LOCK ://  ��ʹ��  λ������
		if( MotorControler.State == 4){
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Fdb = MotorControler.MotorActivePostion;  //��������Ϊ����λ��
        pidholding.Speedref = 0;             //�ٶȲο�ֵ����Ϊ0��������ĳһλ�ò���
        pidholding.Speedfdb = MotorControler.SpeedFdbp;
        pidholding.Iqsfdb = svpwm.IQs;
		
        pidholding.calc(&pidholding);
		
        svpwm.UQsRef = pidholding.Out;
        UqsPid.Ui = 0;
        UqsPid.Out = 0;			//�����󣬻���һ���ֵ���
	   } else
	  	{
			 PwmShut();   //�������
			 pidholding.Ui = 0;   //��������
	    }
        break;

    case SVPWM_STATE_CURRENT_MODE :   // ����ģʽ
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        svpwm.UQs = pidpt.Out;
        break;

    case SVPWM_STATE_SPEED_MODE :   // �ٶ�ģʽ  --��Ӧ��λ�����ٶȿ���ģʽ
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
			//svpwm.UQsRef = (Ids_filter4(UqsPid.Out)*svpwm.UQ_MAX)/UqsPid.OutMax;		//����б�±ƽ���UqĿ��ֵΪ�ٶ�PID�������UqPID�����	
				TEMP_Uq_filter32 = Kalman_Filter(UqsPid.Out,2);
				TEMP_Uq_filter8 = Kalman_Filter(UqsPid.Out,3);
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}

        break;

    case SVPWM_STATE_SPEED_MODE2 :   // �ٶ�ģʽ2								--	û�б�ʹ�õ�
        svpwm.DcCoeff = SystemError.DcCoeff;//32400;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidptv.Ref = (pidpv.Out) >> 5; //���ٶȻ�PID���������ӳ��ɵ�����-�ٶ�ģʽPID��Ԥ��ֵ�����޷�
        if (pidptv.Ref > 600) {
            pidptv.Ref = 600;
        }

        if (pidptv.Ref < -600) {
            pidptv.Ref = -600;
        }

        pidptv.Fdb = svpwm.IQs;
        pidptv.calc(&pidptv);
        svpwm.UQs = pidptv.Out;  //����ģʽ�������ڶ�Ӧ��PID�����ֱ��������SVPWM��������ͨ��б�±ƽ�
        break;

    case SVPWM_STATE_POS_MODE:   //λ�û�ģʽ      -----------��λ�� λ�ÿ���ģʽ
		    svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
				//svpwm.UQsRef = (Ids_filter4(UqsPid.Out)*svpwm.UQ_MAX)/UqsPid.OutMax;		//����б�±ƽ���UqĿ��ֵΪ�ٶ�PID�������UqPID�����	
				TEMP_Uq_filter32 = Kalman_Filter(UqsPid.Out,2);
				TEMP_Uq_filter8 = Kalman_Filter(UqsPid.Out,3);
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
        break;

    case SVPWM_STATE_POS_LOCK_TEST ://  ��ʹ��  λ������  ������
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Ref = 0;
        pidholding.Fdb = MotorControler.MotorActivePostion;
        pidholding.calc(&pidholding);
        svpwm.UQs = pidholding.Out;
        break;

    default: {
    }
    break;
    }
}
#endif

//���ݵ������״ִ̬�ж�Ӧ�Ŀ��Ʋ���
void MotorControl(void)  //1KHZ
{
    static short ExampleCount = 0;
    static short VF_Dir = 0;              //����ģʽ
    static int RotorMacAngleTemp1 = 0;    //����ϱ�
    static int RotorMacAngleTemp2 = 0;    //����ϱ�
    static short RotorMacAngleTemp3 = 0;  //����ϱ�
    ExampleCount++;

    if (ExampleCount >= 10) {
        ExampleCount = 0;

        switch (MotorControler.State) {
        case MOTOR_STATE_STOP : {
            svpwm.SvpwmControlState = SVPWM_STATE_DISABLE;
            MotorControler.Error = 0;
            /*****VF��������**************/
            SystemVar.VF_ElectricalAngleStep = 0;
            SystemVar.VF_ElectricalAngle = 0;
            VF_Dir = 0;
            SystemVar.VF_Voltage = 0;
            RotorMacAngleTemp3 = 0;
            RotorMacAngleTemp2 = 0;
            svpwm.CurrentRef = 0;  //���ת������
        }
        break;

        case MOTOR_STATE_VF_RUN : {       //��������
            svpwm.SvpwmControlState = SVPWM_STATE_VF_MODE;
            SystemVar.VF_ElectricalAngleStepCount++;

            if (SystemVar.VF_ElectricalAngleStepCount >= 100) {
                SystemVar.VF_ElectricalAngleStepCount = 0;
				//�޶���������
				//������Ƕ��޷�&�ж�ת������
                if ((SystemVar.VF_ElectricalAngleStep < SystemVar.VF_ElectricalAngleStepMax)
                        && (SystemVar.VF_ElectricalAngleStepMin == 0)) {
                    SystemVar.VF_ElectricalAngleStep = SystemVar.VF_ElectricalAngleStep + 1;
                    VF_Dir = 1;
                }

                if ((SystemVar.VF_ElectricalAngleStep > SystemVar.VF_ElectricalAngleStepMin)
                        && (SystemVar.VF_ElectricalAngleStepMax == 0)) {
                    SystemVar.VF_ElectricalAngleStep = SystemVar.VF_ElectricalAngleStep - 1;
                    VF_Dir = -1;
                }

                if ((SystemVar.VF_ElectricalAngleStepMax != 0) && (SystemVar.VF_ElectricalAngleStepMin != 0)) {
                    SystemVar.VF_ElectricalAngleStep = 0;
                    SystemVar.VF_ElectricalAngle = 0;
                    VF_Dir = 0;
                    SystemVar.VF_Voltage = 0;
                }

                if ((SystemVar.VF_ElectricalAngleStepMax == 0) && (SystemVar.VF_ElectricalAngleStepMin == 0)) {
                    SystemVar.VF_ElectricalAngleStep = 0;
                    SystemVar.VF_ElectricalAngle = 0;
                    VF_Dir = 0;
                    SystemVar.VF_Voltage = 0;
                }

                //����ʵ��Ч���������޷�������
            }

			      //���µ�Ƕ�
            SystemVar.VF_ElectricalAngle = (SystemVar.VF_ElectricalAngle + ((short)((((float)SystemVar.VF_ElectricalAngleStep)) *
                                            27.31f))) & 0x7FFF;
            //���������ѹ
			      SystemVar.VF_Voltage = (SystemVar.VF_ElectricalAngleStep >> 1) * (SystemVar.VF_Coefficient * 10) + ((
                                       SystemVar.VF_Coefficient_B - 500) * VF_Dir);
        }
        break;

        case MOTOR_STATE_FIND_ZERO : { //�����
			//�����������ǰ��ȡ�ĵ�Ƕ�ֵ
            RotorMacAngleTemp1 = ((int)(((MotorControler.AngleFromMT6835 + RotorMacAngleTemp2) & 0x7fff) *
                                        MotorControler.MotorPoles)) & 0x7fff;
            //�������е�Ƕ����㣬�����ѹ����Ϊ��ֵ�����뿪������ģʽ�������
			      SystemVar.VF_ElectricalAngle = 0;
            SystemVar.VF_Voltage = 1500;
            svpwm.SvpwmControlState = SVPWM_STATE_VF_MODE;
            RotorMacAngleTemp3++;

            if (RotorMacAngleTemp3 >= 1000) {
                RotorMacAngleTemp3 = 1000;
                RotorMacAngleTemp2 = RotorMacAngleTemp2 + 1;
                RotorMacAngleTemp2 = RotorMacAngleTemp2 & 0x7fff;

				//Ѱ���ض�����ֵ�϶�Ϊ���
                if ((RotorMacAngleTemp1 > 8188) && (RotorMacAngleTemp1 < 8196)) {
                    MotorControler.AngleFromMT6835Offset1 = RotorMacAngleTemp2 + 25;
                    MotorControler.State = 0;
                }
            }
        }
        break;

        case MOTOR_STATE_BASIC_TEST : {              //��������
            svpwm.SvpwmControlState = SVPWM_STATE_BASIC_TEST;
        }
        break;

        case MOTOR_STATE_LOCK_AFTER_ENABLE : {             //ʹ�ܺ�����λ��
            svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
            pidholding.Ref = MotorControler.PositionRef;
            pidpv.OutMax = pidpv_OutMax;
            pidpv.OutMin = pidpv_OutMin;
            pidpv.UiMax = pidpv_UiMax;
            pidpv.UiMin = pidpv_UiMin;
            pidpv.Kp = 300;
            pidpv.Ki = 30;

        }
        break;

        case MOTOR_STATE_TORQUE_CONTROL: {  //���ؿ���ģʽ
            svpwm.SvpwmControlState = SVPWM_STATE_CURRENT_MODE;
            svpwm.CurrentRef = MotorControler.TorqueRef;
            pidpt.Fdb = svpwm.IQs;
            pidpt.calc(&pidpt);
        }
        break;

        case MOTOR_STATE_SPEED_CONTROL: {                     //�ٶȻ�
            svpwm.SvpwmControlState = SVPWM_STATE_SPEED_MODE;
            if (pidpv.Ref < MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref + MotorControler.SpeedAcc;

                if (pidpv.Ref >= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            } else if (pidpv.Ref > MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref - MotorControler.SpeedDcc;

                if (pidpv.Ref <= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            }
			
			      pidpv.Fdb=MotorControler.SpeedFdbp;
            pidpv.calc(&pidpv);
        }
        break;

        case MOTOR_STATE_SPEED_CONTROL2: {                     //�ٶȻ��������ŷ�
            pidptv.Kp = 50;
            pidptv.Ki = 50;
            pidptv.UiMax = pidpv_UiMax;
            pidptv.UiMin = pidpv_UiMin;
            pidptv.OutMax = pidpv_OutMax;
            pidptv.OutMin = pidpv_OutMin;
            svpwm.SvpwmControlState = SVPWM_STATE_SPEED_MODE2;
            svpwm.CurrentRef = MotorControler.TorqueRef;      //ת������

            if (pidpv.Ref < MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref + MotorControler.SpeedAcc;

                if (pidpv.Ref >= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            } else if (pidpv.Ref > MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref - MotorControler.SpeedDcc;

                if (pidpv.Ref <= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            }

            pidpv.Fdb = MotorControler.SpeedFdbp;
            pidpv.calc(&pidpv);
        }
        break;

        case MOTOR_STATE_POS_HOLD :
            svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
            pidholding.Ref = (int)(value.position);
            break;

        case MOTOR_STATE_POS_MOVE: {
            //pidc_position
            value = SpeedPlant_positionControl(&PostionPlanCiA402Mode_1, PostionPlanStepCount);
            //�ٶ�ת��ʱ������rpmת��cnt/ms������S�����ڲ���ʱ�䵥λ��ms�����������1�����Ǽ�0.001
            PostionPlanStepCount = PostionPlanStepCount + PostionPlanStep;
            svpwm.SvpwmControlState = SVPWM_STATE_POS_MODE;
            svpwm.CurrentRef = MotorControler.TorqueRef; //ת������
            pidc_position.Ref = value.position;
            pidc_position.Speedref = (short)(value.vel * 1.831f);
            pidc_position.Speedfdb = MotorControler.SpeedFdbp;
            pidc_position.Fdb = MotorControler.MotorActivePostion;
            pidc_position.Iqs = svpwm.IQs;
            pidc_position.calc(&pidc_position);
			
            //λ�û������ֵ�ٶȻ�Ref
					  pidpv.Kp = 300;
				    pidpv.Ki = 30;
            pidpv.Ref = 500*pidc_position.Out/pidc_position.OutMax;
              
			      pidpv.Fdb=MotorControler.SpeedFdbp;
            pidpv.calc(&pidpv);
		
           if ((PostionPlanStepCount > (PostionPlanStepMax))||((MotorControler.MotorActivePostion>UartMode.TargetPosition)&&(postion_direction_flag==1))||((MotorControler.MotorActivePostion<UartMode.TargetPosition)&&(postion_direction_flag==0))) 
				     {
					     MotorControler.State = MOTOR_STATE_POS_HOLD;
					     svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
					     pidholding.Ui =	0;
					     MotorControler.PositionRef = MotorControler.MotorActivePostion;
				     }
        }
        break;

        default: {
        }
        break;
        }
    }
}

//1�������һ��
//1Ȧ32767
void SpeedCalculate(void)
{
    static short t1 = 0;
    static short t2 = 0;
    static int   sum1 = 0;
    static int   sum2 = 0;
    static int   sum3 = 0;
    static short buffer1[16];
    static short buffer2[32];
    static short buffer3[64];
    static short point_a1 = 0;
    static short flag1 = 0;
    static short point_a2 = 0;
    static short flag2 = 0;
    static short point_a3 = 0;
    static short flag3 = 0;
    static short SpeedLoop = 0;
    int speedfdb_tmp = 0;

	//�жϵ������ת��Ȧ
    if ((MotorControler.AngleFromMT6835 >> 12) == 0) {
        if (t2 == 1) {
            MotorControler.RotorCount++;
            t2 = 0;
        }

        t1 = 1;
    } else if ((MotorControler.AngleFromMT6835 >> 12) == 7) {
        if (t1 == 1) {
            MotorControler.RotorCount--;
            t1 = 0;
        }

        t2 = 1;
    } else if ((MotorControler.AngleFromMT6835 >> 12) == 3) {
        t1 = 0;
        t2 = 0;
    }
	//��������ǰλ��
    MotorControler.MotorActivePostion = (MotorControler.RotorCount * 0x7fff) + ((MotorControler.AngleFromMT6835 + 0) &
                                        0x7fff);
	//ÿ10ms����һ��ת��
    SpeedLoop++;
	
    if (SpeedLoop >= 10) {
        SpeedLoop = 0;
        speedfdb_tmp = MotorControler.AngleFromMT6835 - SystemError.RotorMacAnglePastp;
        SystemError.RotorMacAnglePastp = MotorControler.AngleFromMT6835;

		//�����޶���ֵ����
        if (speedfdb_tmp > 5000) {
            speedfdb_tmp = SystemError.SpeedFdbpPost;
        }

        if (speedfdb_tmp < -5000) {
            speedfdb_tmp = SystemError.SpeedFdbpPost;
        }

		//�����ϴβ����ٶ�
        SystemError.SpeedFdbpPost = speedfdb_tmp;
		//�ٶ�ֵ�����Ŀ�굥λ
        MotorControler.SpeedFdbp = (short)((speedfdb_tmp * 18311) / 10000);
        //�����˲�
		/*************************************************************/
        short temp = buffer1[point_a1];
        sum1 = sum1 + MotorControler.SpeedFdbp;
        buffer1[point_a1] = MotorControler.SpeedFdbp;
        point_a1++;

        if (flag1 == 0) {
            MotorControler.SpeedFdbpFilter1 = (short)(sum1 / point_a1);

            if (point_a1 >= 16) {
                flag1 = 1;
                point_a1 = 0;
            }
        } else {
            sum1 = sum1 - temp;
            MotorControler.SpeedFdbpFilter1 = (short)(sum1 / 16);

            if (point_a1 >= 16) {
                point_a1 = 0;
            }
        }

        /*************************************************************/
        temp = buffer2[point_a2];
        sum2 = sum2 + MotorControler.SpeedFdbp;
        buffer2[point_a2] = MotorControler.SpeedFdbp;
        point_a2++;

        if (flag2 == 0) {
            MotorControler.SpeedFdbpFilter2 = (short)(sum2 / point_a2);

            if (point_a2 >= 32) {
                flag2 = 1;
                point_a2 = 0;
            }
        } else {
            sum2 = sum2 - temp;
            MotorControler.SpeedFdbpFilter2 = (short)(sum2 / 32);

            if (point_a2 >= 32) {
                point_a2 = 0;
            }
        }

        /*************************************************************/
        temp = buffer3[point_a3];
        sum3 = sum3 + MotorControler.SpeedFdbp;
        buffer3[point_a3] = MotorControler.SpeedFdbp;
        point_a3++;

        if (flag3 == 0) {
            MotorControler.SpeedFdbpFilter3 = (short)(sum3 / point_a3);

            if (point_a3 >= 64) {
                flag3 = 1;
                point_a3 = 0;
            }
        } else {
            sum3 = sum3 - temp;
            MotorControler.SpeedFdbpFilter3 = (short)(sum3 / 64);

            if (point_a3 >= 64) {
                point_a3 = 0;
            }
        }
    }
}