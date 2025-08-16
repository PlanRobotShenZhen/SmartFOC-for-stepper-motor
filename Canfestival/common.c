#include "common.h"

//--------------------------------delay start --------------------------------
//static uint8_t  us_count = 0; //us时基
//static uint16_t ms_count = 0; //ms时基

/*
 *   delay_init()
 *   计算说明：
 *   systick 为24位自减定时器，每一个时钟都会都会减1，这里就以21M为例来举例下
 *
 *   - 24位数据最大能表示：2^24 - 1 = 16777215,
 *   - 1/21M = t/16777215;则可以计算出单次最大能表示的时间为 16777215/21000000 = 0.798915s
 *   所以如果延时超过上面的时间，就加个for循环。
 *
 *  上面是stm32f407板子上的时钟，目前stm32f030时钟为48M，则计算方法如下
 *  1/6M = t/16777216,则可以计算出一次溢出之前最大可以表示16.777216s.这是很长时间了。
 *
 */
//void delay_init()
//{
//    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//    us_count = SystemCoreClock/8/1000000;   //1M的时钟源,则，1us需要的计数是6
//    ms_count = (uint16_t)us_count * 1000;        //1ms的装载值
//}

/*
 *  delay_us:延时多个us
 *
 */
//void delay_us(uint32_t nus)
//{
//    uint32_t ctrl_val;
//    SysTick->LOAD = nus * us_count; //设置reload寄存器数值
//    SysTick->VAL = 0x00;            //初始化value寄存器中的数值为0
//    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //启动定时器，此时LOAD寄存器中的数值为加载到VALUE寄存器中，进行递减
//    do {
//            ctrl_val = SysTick->CTRL;
//    } while((ctrl_val & (1 << SysTick_CTRL_ENABLE_Pos)) &&
//           !(ctrl_val & (1 << SysTick_CTRL_COUNTFLAG_Pos)));
//    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //关闭systick
//    SysTick->VAL = 0X00;       //重新清零
//}

/*
 *  delay_ms:延时多个ms
 *
 */
//void delay_xms(uint16_t nms)
//{
//    uint32_t ctrl_val;
//    SysTick->LOAD = (uint32_t)nms * ms_count;//设置reload寄存器数值
//    SysTick->VAL = 0x00;                //初始化value寄存器中的数值为0
//    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;//启动定时器，此时LOAD寄存器中的数值为加载到VALUE寄存器中，进行递减
//    do {
//            ctrl_val = SysTick->CTRL;
//    } while((ctrl_val & (1 << SysTick_CTRL_ENABLE_Pos)) &&
//            !(ctrl_val & (1 << SysTick_CTRL_COUNTFLAG_Pos)));
//    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  //关闭systick
//    SysTick->VAL = 0X00;                        //重新清零
//} 

/*
 *    delay_ms:延时n ms
 *    这里为了预防超过最大的时间量程798915us,所以循环多次
 */
//void delay_ms(uint16_t nms)
//{
//    uint8_t repeat = nms/500;
//    uint16_t remain=nms % 500;
//    while(repeat) {
//        delay_xms(500);
//        repeat--;
//    }
//    if(remain)
//        delay_xms(remain);
//}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  SYS_DEBUG(("%s,line:%d",file,line));
  while (1)
  {
  }
}
#endif
//----------------------------------delay end ----------------------------------
