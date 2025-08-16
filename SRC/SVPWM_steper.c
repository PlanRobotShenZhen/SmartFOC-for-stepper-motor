#include "svpwm_dsp.h"
#include "IQmathLib.h"  // Include header for IQmath library 


// 定义常量（使用Q15格式表示）
#define _IQ15_SQRT3_DIV_2   _IQ15(0.8660254)  // √3/2 ≈ 0.8660254
#define _IQ15_HALF          _IQ15(0.5)        // 0.5
#define _IQ15_ONE           _IQ15(1.0)        // 1.0
#define _IQ15_TWO           _IQ15(2.0)        // 2.0
#define _IQ15_ONE_OVER_SQRT3 _IQ15(0.57735026918963)  // 1/√3

// 定义限幅阈值（Q15格式）
#define VOLTAGE_Q_LIMIT    30500  // 约0.935*32767，保留6.5%裕量
#define CURRENT_Q_LIMIT     6500  // 约0.2*32767，电流限幅（根据实际硬件调整）

// 定义步进电机PWM周期（单位：计数周期，需与硬件匹配）
#define STEPPER_PWM_PERIOD  4500  // 示例值：4500计数周期

/**
 * @brief Clark变换：将三相静止坐标系(ABC)转换为两相静止坐标系(αβ)
 * @param v 结构体指针，包含输入输出数据
 * @note 公式：
 *  iα = iA
 *  iβ = (iA + 2*iB) / √3
 */
void clarke(SVPVM *v)			
{
		//原代码中利用三角函数计算
    short Cosine, Sine;
    short tmpAngle;
    tmpAngle = v->Angle;
    Sine 		= _IQ15sinPU(tmpAngle); //0--32767
    Cosine 	= _IQ15cosPU(tmpAngle);	//_IQ15 浮点数转化为定子数，提高数据精度

    v->IDs = _IQ15mpy(v->As, Cosine) + _IQ15mpy(v->Bs, Sine);
    v->IQs = _IQ15mpy(v->Bs, Cosine) - _IQ15mpy(v->As, Sine);
	
}

/**
 * @brief Park变换：将两相静止坐标系(αβ)转换为旋转坐标系(dq)
 * @param v 结构体指针，包含输入输出数据
 * @note 公式：
 *  id = iα*cosθ + iβ*sinθ
 *  iq = -iα*sinθ + iβ*cosθ
 */
void park(SVPVM *v) {
    short cos_theta = _IQ15cosPU(v->Angle2);  // 获取转子角度的余弦值（Q15格式）
    short sin_theta = _IQ15sinPU(v->Angle2);  // 获取转子角度的正弦值（Q15格式）
    
    // 计算dq轴电流（使用IQ乘法并累加）
    v->IDs = _IQ15mpy(v->As, cos_theta) + _IQ15mpy(v->Bs, sin_theta); // id
    v->IQs = _IQ15mpy(v->Bs, cos_theta) - _IQ15mpy(v->As, sin_theta); // iq
}

short tmpDs, tmpQs;


/**01
3.2
 * @brief 反Park变换：将旋转坐标系(dq)转换为两相静止坐标系(αβ)
 * @param v 结构体指针，包含输入输出数据
 * @note 公式：
 *  uα = ud*cosθ - uq*sinθ
 *  uβ = ud*sinθ + uq*cosθ
 */

void ipark(SVPVM *v)			//反park变换
{
		//tmp前缀变量只用于此函数中 作用：一次赋值减少引用
    short Cosine, Sine;
    short tmpAngle;
	
    tmpAngle = v->Angle; //0--32767   --- >> 65536 代表着两圈   65536/2  =32768 

		// 获取转子角度的正弦、余弦值
    Sine 		= _IQ15sinPU(tmpAngle);							
    Cosine 	= _IQ15cosPU(tmpAngle);							
		
		// 电压缩放：乘以直流母线系数（可用于电压归一化）
    tmpQs = _IQ12mpy(v->UQs, v->DcCoeff);
    tmpDs = _IQ12mpy(v->UDs, v->DcCoeff);
	
		// 电压限幅保护，防止过调制
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

		//执行反park变换
    v->Ualpha = _IQ15mpy(tmpDs, Cosine) - _IQ15mpy(tmpQs, Sine);		
    v->Ubeta  = _IQ15mpy(tmpQs, Cosine) + _IQ15mpy(tmpDs, Sine);			
}


/**
 * @brief 六扇区SVPWM生成函数（适用于三相电机）
 * @param v 结构体指针，包含输入输出数据
 * @note 实现七段式SVPWM，计算三相桥臂占空比
 */
void svgendq(SVPVM *v)			
{
    short va, vb, vc;		// 临时存储αβ轴电压转换值
    short sector = 0;  				 // 扇区编号（1-6）
		short t1 = 0, t2 = 0;   // 基本矢量作用时间（Q15格式）
	
    // 1. 计算三相虚拟电压（用于扇区判断）
    va = v->Ubeta;                          // Va = uβ
    vb = _IQ15mpy(_IQ15_HALF, v->Ubeta) + _IQ15_SQRT3_DIV_2 * v->Ualpha; // Vb = (uβ/2) + (√3/2)uα
    vc = _IQ15mpy(_IQ15_HALF, v->Ubeta) - _IQ15_SQRT3_DIV_2 * v->Ualpha; // Vc = (uβ/2) - (√3/2)uα

    // 2. 确定电压矢量所在扇区（60°扇区划分）
    if (va > _IQ15(0))   sector |= 1;  // 位1：Va>0则置1
    if (vb > _IQ15(0))   sector |= 2;  // 位2：Vb>0则置1
    if (vc > _IQ15(0))   sector |= 4;  // 位3：Vc>0则置1

    // 3. 计算基本矢量作用时间（X=Va, Y=Vb, Z=Vc）
    short x = va;          // X = Va = uβ
    short y = vb;          // Y = Vb = (uβ/2)+(√3/2)uα
    short z = vc;          // Z = Vc = (uβ/2)-(√3/2)uα

    // 根据扇区选择t1和t2（均为Q15格式）
    switch (sector) {
        case 1: t1 = z; t2 = y; break;  // 扇区1：t1=Z, t2=Y
        case 2: t1 = x; t2 = z; break;  // 扇区2：t1=X, t2=Z
        case 3: t1 = -y; t2 = x; break; // 扇区3：t1=-Y, t2=X
        case 4: t1 = -z; t2 = -x; break;// 扇区4：t1=-Z, t2=-X
        case 5: t1 = -x; t2 = -y; break;// 扇区5：t1=-X, t2=-Y
        case 6: t1 = y; t2 = -z; break; // 扇区6：t1=Y, t2=-Z
        default: sector = 0; break;      // 零矢量情况
    }

    // 4. 计算三相桥臂占空比（七段式中点对称分布）
    if (sector == 0) {
        // 零矢量情况：三相占空比均为0.5（中心对齐PWM）
        v->Ta = v->Tb = v->Tc = _IQ15_HALF;
    } else {
        // 计算总有效矢量时间（扣除零矢量）
        short t_total = _IQ15mpy(_IQ15_ONE, _IQ15_HALF) - _IQ15mpy(_IQ15mpy(t1, _IQ15_HALF), _IQ15_HALF);
        
        // 分配各相导通时间（Q15格式，范围0~1）
        v->Ta = _IQ15_HALF + _IQ15mpy(t1, _IQ15_HALF) + _IQ15mpy(t2, _IQ15_HALF);
        v->Tb = _IQ15_HALF + _IQ15mpy(t1, _IQ15_HALF) - _IQ15mpy(t2, _IQ15_HALF);
        v->Tc = _IQ15_HALF - _IQ15mpy(t1, _IQ15_HALF) - _IQ15mpy(t2, _IQ15_HALF);
    }

    // 5. 转换为对称占空比（-1~1范围，适配硬件需求）
    v->Ta = _IQ15mpy(_IQ15_TWO, v->Ta) - _IQ15_ONE;
    v->Tb = _IQ15mpy(_IQ15_TWO, v->Tb) - _IQ15_ONE;
    v->Tc = _IQ15mpy(_IQ15_TWO, v->Tc) - _IQ15_ONE;

}

/**
 * @brief 四扇区SVPWM生成函数（适用于两相步进电机）
 * @param v 结构体指针，包含输入输出数据
 * @note 90°扇区划分，直接控制两相绕组
 */
void svgendq2(SVPVM *v)  	 			 			//42 步进电机2相 4扇区
{
    int Tmp;
    short MPeriod = 4500;
	//short MPeriod = 10000;
    short sector = 1;  // 初始扇区设为1（第三象限

    // 1. 确定扇区（基于Ualpha和Ubeta符号，90°划分）
    if (v->Ualpha >= _IQ15(0)) sector += 1;  // 右半平面（Uα≥0）
    if (v->Ubeta >= _IQ15(0)) sector += 2;  // 上半平面（Uβ≥0）
    
    // 处理零矢量情况
    if ((v->Ualpha == _IQ15(0)) && (v->Ubeta == _IQ15(0))) {
        sector = 0;
    }
		
		// 2. 根据扇区计算两相占空比
    if(sector == 0)  
    {
        v->Ta = _IQ15(0.5);
        v->Tb = _IQ15(0.5);
        v->Va = (MPeriod >> 1);
        v->Vb = (MPeriod >> 1);
    }
    else if(sector == 1)  // sector 1: 对应第三扇区
    {
        v->Ta = -v->Ualpha;
        v->Tb = -v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) + (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) + (short)(Tmp >> 15);
    }
    else if(sector == 2)  // sector 2: 对应第四扇区
    {
        v->Ta = v->Ualpha;
        v->Tb = -v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) - (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) + (short)(Tmp >> 15);
    }
    else if(sector == 3)  // sector 3: 对应第二扇区
    {
        v->Ta = -v->Ualpha;
        v->Tb = v->Ubeta;
        Tmp = (int)(MPeriod >> 1) * (int)v->Ta;
        v->Va = (MPeriod >> 1) + (short)(Tmp >> 15);
        Tmp = (int)(MPeriod >> 1) * (int)v->Tb;
        v->Vb = (MPeriod >> 1) - (short)(Tmp >> 15);
    }
    else if(sector == 4)  // sector 4: 对应第一扇区
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
 * @brief PWM输出函数：根据控制量生成最终PWM占空比
 * @param v 结构体指针，包含输入输出数据
 * @note 支持动态调整PWM周期和占空比
 */
void PWM(SVPVM *v)		 	 							//Va  Vb  占空比计算
{
    short MPeriod;
    int Tmp;

    v->MfuncC1 = v->Ubeta;
    v->MfuncC2 = v->Ualpha;

    // 1. 计算PWM周期（动态调整，Q15转整数）
    Tmp = (int)v->PeriodMax * (int)v->MfuncPeriod;         // Q15 = Q0*Q15
    MPeriod = (short)(Tmp >> 16) + (short)(v->PeriodMax >> 1); // 中心对齐偏移
	

    // 2. 计算通道A占空比（Q15转整数）
    Tmp = (int)MPeriod * (int)v->MfuncC1;                  // Q15 = Q0*Q15
    v->Va = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // 中心对齐模式

    // 3. 计算通道B占空比（Q15转整数）
    Tmp = (int)MPeriod * (int)v->MfuncC2;                 // Q15 = Q0*Q15
    v->Vb = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // 中心对齐模式
}









