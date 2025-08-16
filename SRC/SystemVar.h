#ifndef __SYSTEMVAR_H__
#define __SYSTEMVAR_H__

typedef struct
{
    short SoftwareVersion;
    short HardwareVersion;
    short MotorVersion;
    short Version[8];
    short Administrators[10];
    short MotorVar[50];


    /****��������*********/
    short VF_Voltage;
    short VF_ElectricalAngle;
    short VF_Coefficient;
    short VF_Coefficient_B;
    short VF_ElectricalAngleStepMax;
    short VF_ElectricalAngleStepMin;
    short VF_ElectricalAngleStep;
    short VF_ElectricalAngleStepCount;

    /****��������*********/

    /****�������*********/
    short test_uqs;
    short test_angle;
    /****�������*********/

    short ModbusID;
    short ModbusBaudrate;
    short CanopenID;
    short CanopenBaudrate;

} SYSTEMVAR;

typedef SYSTEMVAR* SYSTEMVAR_handle;

#endif
