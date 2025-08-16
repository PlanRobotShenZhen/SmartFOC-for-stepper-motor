#include "n32g45x.h"

#define A_on 	GPIO_ResetBits(GPIOA,GPIO_PIN_11)
#define A_off GPIO_SetBits(GPIOA,GPIO_PIN_11)

#define B_on 	GPIO_ResetBits(GPIOA,GPIO_PIN_10)
#define B_off GPIO_SetBits(GPIOA,GPIO_PIN_10)

#define C_on 	GPIO_ResetBits(GPIOA,GPIO_PIN_9)
#define C_off GPIO_SetBits(GPIOA,GPIO_PIN_9)

#define D_on 	GPIO_ResetBits(GPIOA,GPIO_PIN_8)
#define D_off GPIO_SetBits(GPIOA,GPIO_PIN_8)

//extern void delay_Ms();
void Motor_drive(void);
void Wave_drive()//µ¥Ïò¼¤Àø
	{
A_on;
		
B_off;
		
		delay_Ms(500);
A_off;		
		
		
		
C_on;
		
D_off;
		
		delay_Ms(500);
C_off;		
		
		
		
B_on;
		
A_off;
delay_Ms(500);
	B_off;	
		
D_on;
		
C_off;		
	delay_Ms(500);	
D_off;

}
void Motor_drive(){

	Wave_drive();
	

	}












