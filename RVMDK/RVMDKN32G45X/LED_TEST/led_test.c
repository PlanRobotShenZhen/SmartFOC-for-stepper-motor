#include "LED_TEST.h"
#include "n32g45x.h"


extern void delay_Ms(uint16_t nms);


void LED_Green_Flash(void)
{
    LED1_ON;
    delay_Ms(100);
    LED1_OFF;
    delay_Ms(100);
}

void LED_Red_Flash(void)
{
    LED2_ON;
   // delay_Ms(100);
   // LED2_OFF;
   // delay_Ms(100);
}

void LED_Yellow_Flash(void)
{
    LED1_ON;
    LED2_ON;
 
  
    
}





