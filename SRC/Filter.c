#include "ExternGlobals.h"

short Ids_filter1(short input);
short Ids_filter2(short input);
short Ids_filter3(short input);
short Ids_filter4(short input);
short Ids_filter5(short input);
short Ids_filter6(short input);
short  Moving_Average_Window_Filter(short input, short CHANNEL_ID,short width);
short  Moving_Average_Window_Filter_4096(short input, short CHANNEL_ID,short width);
// ========================================================================
// �������ƣ�Ids_filter()
// ���������input
// ���������result
// ��    �룺��
// ��    ������
// �������������Ǿ�ֵ�˲��㷨�����ɼ�����ֵ����ȡ��ƽ��ֵ
// ========================================================================


//ÿ��ͨ��ֻ���˲�һ�����ݣ�a�����ʹ��ͨ��һ��b������Ͳ���ʹ��ͨ��һ��ʹ��ʱ��ע�ⲻҪ�ظ�ʹ��
//�˲�������widthΪ0-127��IDΪ0-7

short  Moving_Average_Window_Filter(short input, short CHANNEL_ID,short width)
{
    static int   sum[8] = {0};
    static short buffer[8][128];
    static short point_a1[8] = {0};
    static short flag1[8] = {0};
    short reslut[8] = {0};
    short temp = buffer[CHANNEL_ID][point_a1[CHANNEL_ID]];
    sum[CHANNEL_ID] = sum[CHANNEL_ID] + input;
    buffer[CHANNEL_ID][point_a1[CHANNEL_ID]] = input;
    point_a1[CHANNEL_ID]++;

    if(flag1[CHANNEL_ID] == 0)
    {
        reslut[CHANNEL_ID] = (short)(sum[CHANNEL_ID] / point_a1[CHANNEL_ID]);//δ��128��������
        if(point_a1[CHANNEL_ID] >= width)          
        {
            flag1[CHANNEL_ID] = 1;
            point_a1[CHANNEL_ID] = 0;
        }
    }
    else  //��128��������
    {
        sum[CHANNEL_ID] = sum[CHANNEL_ID] - temp;
        reslut[CHANNEL_ID] = (short)(sum[CHANNEL_ID] / width);

        if(point_a1[CHANNEL_ID] >= width)
            point_a1[CHANNEL_ID] = 0;
    }

    return reslut[CHANNEL_ID];
}


short  Moving_Average_Window_Filter_4096(short input, short CHANNEL_ID,short width)
{
    static int   sum[4] = {0};
    static short buffer[4][4096];
    static short point_a1[4] = {0};
    static short flag1[4] = {0};
    short reslut[4] = {0};
    short temp = buffer[CHANNEL_ID][point_a1[CHANNEL_ID]];
    sum[CHANNEL_ID] = sum[CHANNEL_ID] + input;
    buffer[CHANNEL_ID][point_a1[CHANNEL_ID]] = input;
    point_a1[CHANNEL_ID]++;

    if(flag1[CHANNEL_ID] == 0)
    {
        reslut[CHANNEL_ID] = (short)(sum[CHANNEL_ID] / point_a1[CHANNEL_ID]);//δ��128��������
        if(point_a1[CHANNEL_ID] >= width)          
        {
            flag1[CHANNEL_ID] = 1;
            point_a1[CHANNEL_ID] = 0;
        }
    }
    else  //��128��������
    {
        sum[CHANNEL_ID] = sum[CHANNEL_ID] - temp;
        reslut[CHANNEL_ID] = (short)(sum[CHANNEL_ID] / width);

        if(point_a1[CHANNEL_ID] >= width)
            point_a1[CHANNEL_ID] = 0;
    }

    return reslut[CHANNEL_ID];
}

short Ids_filter1(short input)
{
    static int   sum = 0;
    static short buffer[128];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 128)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 128);

        if(point_a1 >= 128)
            point_a1 = 0;
    }

    return reslut;
}
short Ids_filter2(short input)
{
    static int   sum = 0;
    static short buffer[128];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 128)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 128);

        if(point_a1 >= 128)
            point_a1 = 0;
    }

    return reslut;
}

short Ids_filter3(short input)
{
    static int   sum = 0;
    static short buffer[128];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 128)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 128);

        if(point_a1 >= 128)
            point_a1 = 0;
    }

    return reslut;
}
short Ids_filter4(short input)
{
    static int   sum = 0;
    static short buffer[32];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 32)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 32);

        if(point_a1 >= 32)
            point_a1 = 0;
    }

    return reslut;
}
short Ids_filter5(short input)
{
    static int   sum = 0;
    static short buffer[128];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 128)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 128);

        if(point_a1 >= 128)
            point_a1 = 0;
    }

    return reslut;
}
short Ids_filter6(short input)
{
    static int   sum = 0;
    static short buffer[128];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 128)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 128);

        if(point_a1 >= 128)
            point_a1 = 0;
    }

    return reslut;
}
short Ids_filter7(short input)
{
    static int   sum = 0;
    static short buffer[ 4096];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 4096)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum /  4096);

        if(point_a1 >=  4096)
            point_a1 = 0;
    }

    return reslut;
}
short Ids_filter8(short input)
{
    static int   sum = 0;
    static short buffer[ 4096];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >=  4096)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum /  4096);

        if(point_a1 >=  4096)
            point_a1 = 0;
    }

    return reslut;
}

short Ids_filter9(short input)
{
    static int   sum = 0;
    static short buffer[8];
    static short point_a1 = 0;
    static short flag1 = 0;
    short reslut = 0;
    short temp = buffer[point_a1];
    sum = sum + input;
    buffer[point_a1] = input;
    point_a1++;

    if(flag1 == 0)
    {
        reslut = (short)(sum / point_a1);//δ��128��������
        if(point_a1 >= 8)          
        {
            flag1 = 1;
            point_a1 = 0;
        }
    }
    else  //��128��������
    {
        sum = sum - temp;
        reslut = (short)(sum / 8);

        if(point_a1 >= 8)
            point_a1 = 0;
    }

    return reslut;
}