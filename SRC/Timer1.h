

#ifndef __TIMER_H__
#define __TIMER_H__


typedef struct
{
    short  Tmer100us;
    short  Tmer1ms;
    short  Tmer1ms25;
    short  Tmer1ms50;
    short  Tmer3ms;
    short  Tmer10ms;
    short  Tmer25ms;
    short  Tmer100ms;
    short  Tmer250ms;
    short  Tmer500ms;
    short  Tmer1s;
    short  Tmer1m;
    short  Tmer1h;
    short  Flag1ms;
    short  Flag3ms;
    short  Flag10ms;
    short  Flag25ms;
    short  Flag50ms;
    short  Flag100ms;
    short  Flag250ms;
    short  Flag500ms;
    short  Flag1s;
    short  Flag1m;
    short  Flag1h;
    short  Tmer1hour;
} TIMER;

#define TIMER_INITSTATE {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#endif // __TIMER_H__
//end of file
