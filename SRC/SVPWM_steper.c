#include "svpwm_dsp.h"
#include "IQmathLib.h"  // Include header for IQmath library 


// ���峣����ʹ��Q15��ʽ��ʾ��
#define _IQ15_SQRT3_DIV_2   _IQ15(0.8660254)  // ��3/2 �� 0.8660254
#define _IQ15_HALF          _IQ15(0.5)        // 0.5
#define _IQ15_ONE           _IQ15(1.0)        // 1.0
#define _IQ15_TWO           _IQ15(2.0)        // 2.0
#define _IQ15_ONE_OVER_SQRT3 _IQ15(0.57735026918963)  // 1/��3

// �����޷���ֵ��Q15��ʽ��
#define VOLTAGE_Q_LIMIT    30500  // Լ0.935*32767������6.5%ԣ��
#define CURRENT_Q_LIMIT     6500  // Լ0.2*32767�������޷�������ʵ��Ӳ��������

// ���岽�����PWM���ڣ���λ���������ڣ�����Ӳ��ƥ�䣩
#define STEPPER_PWM_PERIOD  4500  // ʾ��ֵ��4500��������

/**
 * @brief Clark�任�������ྲֹ����ϵ(ABC)ת��Ϊ���ྲֹ����ϵ(����)
 * @param v �ṹ��ָ�룬���������������
 * @note ��ʽ��
 *  i�� = iA
 *  i�� = (iA + 2*iB) / ��3
 */
void clarke(SVPVM *v)			
{
		//ԭ�������������Ǻ�������
    short Cosine, Sine;
    short tmpAngle;
    tmpAngle = v->Angle;
    Sine 		= _IQ15sinPU(tmpAngle); //0--32767
    Cosine 	= _IQ15cosPU(tmpAngle);	//_IQ15 ������ת��Ϊ��������������ݾ���

    v->IDs = _IQ15mpy(v->As, Cosine) + _IQ15mpy(v->Bs, Sine);
    v->IQs = _IQ15mpy(v->Bs, Cosine) - _IQ15mpy(v->As, Sine);
	
}

/**
 * @brief Park�任�������ྲֹ����ϵ(����)ת��Ϊ��ת����ϵ(dq)
 * @param v �ṹ��ָ�룬���������������
 * @note ��ʽ��
 *  id = i��*cos�� + i��*sin��
 *  iq = -i��*sin�� + i��*cos��
 */
void park(SVPVM *v) {
    short cos_theta = _IQ15cosPU(v->Angle2);  // ��ȡת�ӽǶȵ�����ֵ��Q15��ʽ��
    short sin_theta = _IQ15sinPU(v->Angle2);  // ��ȡת�ӽǶȵ�����ֵ��Q15��ʽ��
    
    // ����dq�������ʹ��IQ�˷����ۼӣ�
    v->IDs = _IQ15mpy(v->As, cos_theta) + _IQ15mpy(v->Bs, sin_theta); // id
    v->IQs = _IQ15mpy(v->Bs, cos_theta) - _IQ15mpy(v->As, sin_theta); // iq
}

short tmpDs, tmpQs;


/**01
3.2
 * @brief ��Park�任������ת����ϵ(dq)ת��Ϊ���ྲֹ����ϵ(����)
 * @param v �ṹ��ָ�룬���������������
 * @note ��ʽ��
 *  u�� = ud*cos�� - uq*sin��
 *  u�� = ud*sin�� + uq*cos��
 */

void ipark(SVPVM *v)			//��park�任
{
		//tmpǰ׺����ֻ���ڴ˺����� ���ã�һ�θ�ֵ��������
    short Cosine, Sine;
    short tmpAngle;
	
    tmpAngle = v->Angle; //0--32767   --- >> 65536 ��������Ȧ   65536/2  =32768 

		// ��ȡת�ӽǶȵ����ҡ�����ֵ
    Sine 		= _IQ15sinPU(tmpAngle);							
    Cosine 	= _IQ15cosPU(tmpAngle);							
		
		// ��ѹ���ţ�����ֱ��ĸ��ϵ���������ڵ�ѹ��һ����
    tmpQs = _IQ12mpy(v->UQs, v->DcCoeff);
    tmpDs = _IQ12mpy(v->UDs, v->DcCoeff);
	
		// ��ѹ�޷���������ֹ������
    if(tmpQs > 30500)// _IQ15(0.9))
    {
        tmpQs = 30500;//935
    }
    else if(tmpQs < -30500)// _IQ15(0.9))
    {
        tmpQs = -30500;//935
    }

    if(tmpDs > 6500)
    {
        tmpDs = 6500;
    }
    else if(tmpDs < -6500)
    {
        tmpDs = -6500;
    }

		//ִ�з�park�任
    v->Ualpha = _IQ15mpy(tmpDs, Cosine) - _IQ15mpy(tmpQs, Sine);		
    v->Ubeta  = _IQ15mpy(tmpQs, Cosine) + _IQ15mpy(tmpDs, Sine);			
}


/**
 * @brief ������SVPWM���ɺ�������������������
 * @param v �ṹ��ָ�룬���������������
 * @note ʵ���߶�ʽSVPWM�����������ű�ռ�ձ�
 */
void svgendq(SVPVM *v)			
{
    short va, vb, vc;		// ��ʱ�洢�������ѹת��ֵ
    short sector = 0;  				 // ������ţ�1-6��
		short t1 = 0, t2 = 0;   // ����ʸ������ʱ�䣨Q15��ʽ��
	
    // 1. �������������ѹ�����������жϣ�
    va = v->Ubeta;                          // Va = u��
    vb = _IQ15mpy(_IQ15_HALF, v->Ubeta) + _IQ15_SQRT3_DIV_2 * v->Ualpha; // Vb = (u��/2) + (��3/2)u��
    vc = _IQ15mpy(_IQ15_HALF, v->Ubeta) - _IQ15_SQRT3_DIV_2 * v->Ualpha; // Vc = (u��/2) - (��3/2)u��

    // 2. ȷ����ѹʸ������������60���������֣�
    if (va > _IQ15(0))   sector |= 1;  // λ1��Va>0����1
    if (vb > _IQ15(0))   sector |= 2;  // λ2��Vb>0����1
    if (vc > _IQ15(0))   sector |= 4;  // λ3��Vc>0����1

    // 3. �������ʸ������ʱ�䣨X=Va, Y=Vb, Z=Vc��
    short x = va;          // X = Va = u��
    short y = vb;          // Y = Vb = (u��/2)+(��3/2)u��
    short z = vc;          // Z = Vc = (u��/2)-(��3/2)u��

    // ��������ѡ��t1��t2����ΪQ15��ʽ��
    switch (sector) {
        case 1: t1 = z; t2 = y; break;  // ����1��t1=Z, t2=Y
        case 2: t1 = x; t2 = z; break;  // ����2��t1=X, t2=Z
        case 3: t1 = -y; t2 = x; break; // ����3��t1=-Y, t2=X
        case 4: t1 = -z; t2 = -x; break;// ����4��t1=-Z, t2=-X
        case 5: t1 = -x; t2 = -y; break;// ����5��t1=-X, t2=-Y
        case 6: t1 = y; t2 = -z; break; // ����6��t1=Y, t2=-Z
        default: sector = 0; break;      // ��ʸ�����
    }

    // 4. ���������ű�ռ�ձȣ��߶�ʽ�е�ԳƷֲ���
    if (sector == 0) {
        // ��ʸ�����������ռ�ձȾ�Ϊ0.5�����Ķ���PWM��
        v->Ta = v->Tb = v->Tc = _IQ15_HALF;
    } else {
        // ��������Чʸ��ʱ�䣨�۳���ʸ����
        short t_total = _IQ15mpy(_IQ15_ONE, _IQ15_HALF) - _IQ15mpy(_IQ15mpy(t1, _IQ15_HALF), _IQ15_HALF);
        
        // ������ർͨʱ�䣨Q15��ʽ����Χ0~1��
        v->Ta = _IQ15_HALF + _IQ15mpy(t1, _IQ15_HALF) + _IQ15mpy(t2, _IQ15_HALF);
        v->Tb = _IQ15_HALF + _IQ15mpy(t1, _IQ15_HALF) - _IQ15mpy(t2, _IQ15_HALF);
        v->Tc = _IQ15_HALF - _IQ15mpy(t1, _IQ15_HALF) - _IQ15mpy(t2, _IQ15_HALF);
    }

    // 5. ת��Ϊ�Գ�ռ�ձȣ�-1~1��Χ������Ӳ������
    v->Ta = _IQ15mpy(_IQ15_TWO, v->Ta) - _IQ15_ONE;
    v->Tb = _IQ15mpy(_IQ15_TWO, v->Tb) - _IQ15_ONE;
    v->Tc = _IQ15mpy(_IQ15_TWO, v->Tc) - _IQ15_ONE;

}

/**
 * @brief ������SVPWM���ɺ��������������ಽ�������
 * @param v �ṹ��ָ�룬���������������
 * @note 90���������֣�ֱ�ӿ�����������
 */
void svgendq2(SVPVM *v)  	 			 			//42 �������2�� 4����
{
    int Tmp;
    short MPeriod = 4500;
	//short MPeriod = 10000;
    short sector = 1;  // ��ʼ������Ϊ1����������

    // 1. ȷ������������Ualpha��Ubeta���ţ�90�㻮�֣�
    if (v->Ualpha >= _IQ15(0)) sector += 1;  // �Ұ�ƽ�棨U����0��
    if (v->Ubeta >= _IQ15(0)) sector += 2;  // �ϰ�ƽ�棨U�¡�0��
    
    // ������ʸ�����
    if ((v->Ualpha == _IQ15(0)) && (v->Ubeta == _IQ15(0))) {
        sector = 0;
    }
		
		// 2. ����������������ռ�ձ�
    if(sector == 0)  
    {
        v->Ta = _IQ15(0.5);
        v->Tb = _IQ15(0.5);
        v->Va = (MPeriod >> 1);
        v->Vb = (MPeriod >> 1);
    }
    else if(sector == 1)  // sector 1: ��Ӧ��������
    {
        v->Ta = -v->Ualpha;
        v->Tb = -v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) + (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) + (short)(Tmp >> 15);
    }
    else if(sector == 2)  // sector 2: ��Ӧ��������
    {
        v->Ta = v->Ualpha;
        v->Tb = -v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) - (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) + (short)(Tmp >> 15);
    }
    else if(sector == 3)  // sector 3: ��Ӧ�ڶ�����
    {
        v->Ta = -v->Ualpha;
        v->Tb = v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) + (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) - (short)(Tmp >> 15);
    }
    else if(sector == 4)  // sector 4: ��Ӧ��һ����
    {
        v->Ta = v->Ualpha;
        v->Tb = v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) - (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) - (short)(Tmp >> 15);
    }
}


/**
 * @brief PWM������������ݿ�������������PWMռ�ձ�
 * @param v �ṹ��ָ�룬���������������
 * @note ֧�ֶ�̬����PWM���ں�ռ�ձ�
 */
void PWM(SVPVM *v)		 	 							//Va  Vb  ռ�ձȼ���
{
    short MPeriod;
    int Tmp;

    v->MfuncC1 = v->Ubeta;
    v->MfuncC2 = v->Ualpha;

    // 1. ����PWM���ڣ���̬������Q15ת������
    Tmp = (int)v->PeriodMax * (int)v->MfuncPeriod;         // Q15 = Q0*Q15
    MPeriod = (short)(Tmp >> 16) + (short)(v->PeriodMax >> 1); // ���Ķ���ƫ��
	

    // 2. ����ͨ��Aռ�ձȣ�Q15ת������
    Tmp = (int)MPeriod * (int)v->MfuncC1;                  // Q15 = Q0*Q15
    v->Va = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // ���Ķ���ģʽ

    // 3. ����ͨ��Bռ�ձȣ�Q15ת������
    Tmp = (int)MPeriod * (int)v->MfuncC2;                 // Q15 = Q0*Q15
    v->Vb = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // ���Ķ���ģʽ
}









