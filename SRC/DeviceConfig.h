//DeviceConfig.h
#ifndef __DEVICECONFIG_H__
#define __DEVICECONFIG_H__

#define ABAB   0


/*********************PCBA参数*********************/
#define MAIN_FREQUENCY	    (72000000l*2) 	    // 主频
//motor 1
#define PWM_FREQUENCY1		16000l		    // 电机载频  

#define ADC_CHANNEL_Number 								5

#define LED1_ON			    GPIO_ResetBits(GPIOC,GPIO_PIN_13);     // 绿灯亮
#define LED1_OFF			  GPIO_SetBits(GPIOC,GPIO_PIN_13);
#define LED2_ON			    GPIO_ResetBits(GPIOC,GPIO_PIN_14);     // 红灯亮
#define LED2_OFF			  GPIO_SetBits(GPIOC,GPIO_PIN_14);

#define M_ENABLE0 1
#define M_DISABLE0 0



#define FIR_ORDER 10


#define M_AD_SAMPLE_WINDOW     0x3F
#define M_SYS_ERR_INIT         0xAA
#define M_READ_ERROR           0x01
#define M_TURE                 0x01
#define M_FAULT                0x00

#define	Abs(b) (((b)>0) ? (b) : (0-(b)))

#define M_SYSVARNUM_MAX        100
#define M_PROCESSVARNUM_MAX    124
#define M_MOTORVARNUM_MAX      100

#define M_SYSERR_NULL                         0x0000   //
#define M_SYSERR_IPM                          0x0001  //系统1
#define M_SYSERR_OV                           0x0002  //过压
#define M_SYSERR_DV                           0x0003  //低压
#define M_SYSERR_CODER                        0x0004  //编码器没有标定零点
#define M_SYSERR_CODERNESS                    0x0005  //编码器读取错误
#define M_SYSERR_CURRENTSENSOR                0x0006  //电流传感器
#define M_SYSERR_LACKPHASE                    0x0007  //缺相
#define M_SYSERR_ROTOR_LOCKED                 0x0008  //堵转  
#define M_SYSERR_OVER_LOAD                    0x0009  //过载
#define M_SYSERR_OVER_SPEED                   0x000a  //电机超速故障
#define M_SYSERR_ARRESTER                     0x000f  //制动回路故障
#define M_SYSERR_ARRESTER2                    0x0012  //制动回路故障
#define M_SYSERR_LOCATED1                     0x0013  //系统定位故障1
#define M_SYSERR_LOCATED2                     0x0014  //系统定位故障2
#define M_SYSERR_LOCATED3                     0x0015  //系统定位故障3
#define M_SYSERR_COMM                         0x0016  //上位机通信故障   
#define M_SYSERR_POWEROFF                     0x0021  //交流电源掉电
#define M_SYSERR_SYSVARRESET                  0x0030  //系统复位参数
#define M_SYSERR_OVERTolerance1               0x0031  //超差1 
#define M_SYSERR_OVERTolerance2               0x0032  //超差2
#define M_SYSERR_OVERTolerance3               0x0033  //系统复位参数
#define M_SYSERR_OVERTolerance4               0x0034  //系统复位参数

/*****************HMI*******************************/
#define  M_LED_NULL        0x00
#define  M_LED_TAIYAJIAO   0x80
#define  M_LED_CUT         0x40
#define  M_LED_SOFTSPEED   0x20
#define  M_LED_HALTANGLE   0x10
#define  M_LED_TIEG        0x08
#define  M_LED_N           0x0080
#define  M_LED_P           0x0040
#define  M_LED_F           0x0020
#define  M_LED_SWM         0x0010
#define  M_LED_SBM         0x0008
#define  M_LED_EBM         0x0004
#define  M_KEY_TAIYAJIAO   0x6
#define  M_KEY_CUT         0x5
#define  M_KEY_SOFTSPEED   0x4
#define  M_KEY_HALTANGLE   0x3
#define  M_KEY_TIEG        0x2
#define  M_KEY_SWM         0x1
#define  M_KEY_SBM         0x8
#define  M_KEY_EBM         0x7
#define  M_KEY_OK          0x1
#define  M_KEY_P           0x8
#define  M_PKEY_ADD         17
#define  M_PKEY_TIEG        22
#define  M_PKEY_SENSOR      3
#define  M_PKEY_CUT         8
#define  M_PKEY_HALTANGLE   13
#define  M_PKEY_SWM1        0x6
#define  M_PKEY_SWM2        21
#define  M_PKEY_SWM3        12
#define  M_PKEY_SWM4        0x1
#define  M_PKEY_SWM5        16
#define  M_PKEY_SWM6        7
#define  M_PKEY_SBM         18
#define  M_PKEY_EBM         19
#define  M_PKEY_F           11
#define  M_PKEY_P           2
#define  M_PKEY_ADD1        23
#define  M_PKEY_ADD2        4
#define  M_PKEY_ADD3        9
#define  M_PKEY_ADD4        14
#define  M_PKEY_SUB1        5
#define  M_PKEY_SUB2        10
#define  M_PKEY_SUB3        15
#define  M_PKEY_SUB4        20

#define M_SEWMODEL_MAX 6
#define M_SEWSEGMENTNUM_MAX 4

#define   M_DEBUG_IDLE          0
#define   M_DEBUG_PASSWORD      1
#define   M_DEBUG_N             2
#define   M_DEBUG_P             3
#define   M_DEBUG_F             4
#define   M_DEBUG_MONITOR       5
#define   M_DEBUG_SIX           6
#define   M_DEBUG_SEVEN         7
#define   M_MACHINE_TYPE        8
#define   M_SOFTWARE_VERSION    9
#define   M_MACHINE_TEN         10
#define   M_DEBUG_SWM           21
#define   M_DEBUG_SBM           22
#define   M_DEBUG_EBM           23
#define   M_DEBUG_SYSERR        25
#define   M_PKEY_SPEED          26
#define   M_DEBUG_SYN           27
#define   M_DEBUG_RESET         28
#define   M_DEBUG_FUN           29
#define   M_DEBUG_COMKEY        30
#define   M_DEBUG_VIE           31
#define   M_DEBUG_USERVIE       32
#define   M_MODEL_DATA_2        35
#define   M_MODEL_DATA_1        36

/**********************HMI*************************/


/********************Motor************************/
#define M_MOTOR_IDLE            0x01
#define M_MOTOR_RUNING          0x02
#define M_MOTOR_ARREST          0x03
#define M_MOTOR_STOPING         0x04
#define M_MOTOR_PROHALT         0x05
#define M_MOTOR_HALT            0x06
#define M_MOTOR_PROCUTING       0x07
#define M_MOTOR_CUTING          0x08
#define M_MOTOR_CCW             0x09
#define M_MOTOR_CCW2            0x0a
#define M_MOTOR_INIT            0x0b
#define M_MOTOR_INIT_PROHALT    0x0c


/**********************************************/

#define IDLE                    0
#define BUY                     2
#define ADD_XRDY                3
#define DATA_XRDY               4
#define DELAY1                  5
#define READ_DATA               6
#define DELAY                   7
#define FINISH                  8
#define ADD_STOP                9
#define ADD_FINISH              10
#define START                   11
#define ADD_DELAY               12


#endif
//end of file
