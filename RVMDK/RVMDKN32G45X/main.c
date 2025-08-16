#include "Globals.h"

#include "SystemDefine.h"
#include "Function.h"

#include "LED_TEST.h"

int main()
{
    System_Init();					
    ReadVar();       															 	//��ȡ���ò���
    if(SystemVar.SoftwareVersion != SoftwareVersion)//�Ա�����汾
    {
        SaveAllRsetVar();														//������λ
        ReadVar();       														//��ȡ���ò���
    }
    delay_Ms(500);
		
    //modbusͨ��
    eMBInit(MB_RTU, SystemVar.ModbusID, 3, (SystemVar.ModbusBaudrate) * 100, MB_PAR_NONE); //0x01;//< �ӻ���ַ

    CANopen_Init();  															//CanOpen�����ʼ��

    eMBEnable();																	//MODBUS ͨѶʹ��
    SystemError.SysErr = 0;												//ϵͳ���
    SystemError.RuningMode = 2;										//RuningMode==1 CANopen  ;RuningMode==2 MODBUS
    MotorControler.MotorPoles = 50;								//���������
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

            Modbus_Task(&modify);  		//modbusͨ��
            CANopen_Task();        		//CANopenͨ��
            LostCoder();           		//���������
            //LostPhase();            //ȱ����
        }

        /**************************************100ms**************************************/
        if(timer.Flag100ms)
        {
            timer.Flag100ms = M_DISABLE0;
            //TorsionAnalyse();      	//���ط���
            SpeedAnalyse();        		//���ٷ���
            PowerManage();						//��Դ����

            if(SystemError.RuningMode == 2)
            {
                UartMode_Runing();  	//modbus�߼�����
            }

            if(SystemError.RuningMode == 1)
            {
                CiA402Mode_Runing(); 	//CANopen�߼�����
            }
            SysErrManage();   				//ϵͳ�쳣����     	

        }//end timer.Flag100ms

        /**************************************500ms**************************************/
        if(timer.Flag500ms)
        {
            timer.Flag500ms = M_DISABLE0;
            Led();											//LED���ܵ�

            if(SystemError.SaveAllRsetFlag)
            {
                SystemError.SaveAllRsetFlag = 0;
                SaveAllRsetVar();   	 //������λ
            }

            if(SystemError.SaveAllParaFlag)
            {
                SystemError.SaveAllParaFlag = 0;
                SaveAllVar();    				//��������
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
