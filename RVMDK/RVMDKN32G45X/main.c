#include "Globals.h"

#include "SystemDefine.h"
#include "Function.h"

#include "LED_TEST.h"

int main()
{
    System_Init();					
    ReadVar();       															 	//读取配置参数
    if(SystemVar.SoftwareVersion != SoftwareVersion)//对比软件版本
    {
        SaveAllRsetVar();														//参数复位
        ReadVar();       														//读取配置参数
    }
    delay_Ms(500);
		
    //modbus通信
    eMBInit(MB_RTU, SystemVar.ModbusID, 3, (SystemVar.ModbusBaudrate) * 100, MB_PAR_NONE); //0x01;//< 从机地址

    CANopen_Init();  															//CanOpen外设初始化

    eMBEnable();																	//MODBUS 通讯使能
    SystemError.SysErr = 0;												//系统误差
    SystemError.RuningMode = 2;										//RuningMode==1 CANopen  ;RuningMode==2 MODBUS
    MotorControler.MotorPoles = 50;								//电机极对数
    while(1)
    {
	
        /**************************************1ms***************************************/
        if(timer.Flag1ms)
        {
            timer.Flag1ms = M_DISABLE0;
        }//end timer.Flag1ms

        /**************************************10ms**************************************/
        if(timer.Flag10ms)
        {
            timer.Flag10ms = M_DISABLE0;
        }//end timer.Flag10ms

        if(timer.Flag25ms)
        {
            timer.Flag25ms = M_DISABLE0;
        }

        if(timer.Flag50ms)
        {
            timer.Flag50ms = M_DISABLE0;

            Modbus_Task(&modify);  		//modbus通信
            CANopen_Task();        		//CANopen通信
            LostCoder();           		//编码器检查
            //LostPhase();            //缺相检查
        }

        /**************************************100ms**************************************/
        if(timer.Flag100ms)
        {
            timer.Flag100ms = M_DISABLE0;
            //TorsionAnalyse();      	//过载分析
            SpeedAnalyse();        		//超速分析
            PowerManage();						//电源管理

            if(SystemError.RuningMode == 2)
            {
                UartMode_Runing();  	//modbus逻辑处理
            }

            if(SystemError.RuningMode == 1)
            {
                CiA402Mode_Runing(); 	//CANopen逻辑处理
            }
            SysErrManage();   				//系统异常处理     	

        }//end timer.Flag100ms

        /**************************************500ms**************************************/
        if(timer.Flag500ms)
        {
            timer.Flag500ms = M_DISABLE0;
            Led();											//LED功能灯

            if(SystemError.SaveAllRsetFlag)
            {
                SystemError.SaveAllRsetFlag = 0;
                SaveAllRsetVar();   	 //参数复位
            }

            if(SystemError.SaveAllParaFlag)
            {
                SystemError.SaveAllParaFlag = 0;
                SaveAllVar();    				//参数保存
            }

        }//end timer.Flag500ms

        /**************************************1s****************************************/
        if(timer.Flag1s)
        {
            timer.Flag1s = M_DISABLE0;

        }//end timer.Flag1s

        if(timer.Flag1m)
        {
            timer.Flag1m = M_DISABLE0;
        } //end timer.Flag1m

        if(timer.Flag1h)
        {
            timer.Flag1h = M_DISABLE0;
        }
    }
}
