#include "IQmathLib.h"
#include "pida.h"
int test5 = 25;
void pida_calc(PIDA *v)
{
    static int Timer1ms = 0;
    int SpeedUds = 0;

    v->Err = v->Ref - v->Fdb;

    if(v->Err < -5)
    {
        Timer1ms = 0;
    }

    if(v->Ref <= 0)
    {
        Timer1ms = 0;
    }

    if(Timer1ms < 0)
    {
        Timer1ms = 0;
    }

    if(Timer1ms > 1000)
    {
        Timer1ms = 0;
    }

    v->Up = v->Kp * v->Err;

    /*******************强制加速********************************/
    if((v->Err > v->EzhAdd) && (Timer1ms < v->InvarianceAddTime))
    {
        Timer1ms++;
        /*
           v->Fdb 本身是一个不太确认的数，容易形成自激
        */
        v->OutAdd = ((v->Fdb) * (v->KfdbAdd)) + v->AddTor;
        v->Ui = 0;
        v->OutQs = v->OutAdd;
        /****************************************/
        v->OutDs = ((v->Fdb * v->KfdbAddDs) + v->AddTorDs) ;
    }


    /*******************强制减速****************************/
    else if(v->Err < (0 - v->EzhSub))
    {
        v->OutAdd = ((v->Fdb) * (v->KfdbSub))  - (v->SubTor);
        v->Ui = 0;
        v->OutQs = v->OutAdd;
        /****************************************/
        v->OutDs = ((v->Fdb * v->KfdbSubDs) + v->SubTorDs) ;
    }
    /********************匀速运行**************************/
    else
    {
        v->Ui += v->Ki * v->Err;

        if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }

        if(v->Ui > v->UiMax)
        {
            v->Ui = v->UiMax;
        }

        v->OutPreSat = (v->Up + v->Ui) >> 15;
        if(v->Ui < 0)
        {
            v->OutAdd = v->OutAdd - test5;
        }

        if(v->OutAdd < v->AddTor - 1500)
        {
            v->OutAdd  =  v->AddTor - 1500;
        }

        v->OutQs = v->OutAdd + v->OutPreSat;


        if(v->OutQs < (((v->Fdb) * (v->KfdbAdd - 40)) + (v->AddTor - 3000)))
        {
            if(v->Ui < 0) v->Ui = 0;
        }

        /****************************************/
        SpeedUds = v->Fdb + 60;

        if(SpeedUds > (v->Ref))
        {
            SpeedUds = v->Ref;
        }

        v->OutDs = ((SpeedUds * v->KfdbSatDs) + v->SatTorDs) ;
    }


}

