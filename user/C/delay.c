 /******************************************************************************
 * @file    delay.c
 * @version V1.0.0
 * @date    16-2-2017
 * @brief   delay function
 ******************************************************************************/
#include "public.h"
static unsigned char  fac_us=0;
static unsigned short int fac_ms=0;  
void delay_Configuration(unsigned char SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	
	fac_us=SYSCLK/8;		    
	fac_ms=(unsigned short int)(fac_us*1000);
}	
void delay_ms(unsigned short int nms)
{         
	 u32 temp;     
	 SysTick->LOAD=(u32)(nms*fac_ms);
	 SysTick->VAL =0x00;           
	 SysTick->CTRL=0x01;           
	 do
	 {
			temp=SysTick->CTRL;
	 }
	 while(temp&0x01&&!(temp&(1<<16)));  
	 SysTick->CTRL=0x00;       
	 SysTick->VAL =0X00;            
}  
void delay_us(unsigned int nus)
{  
	 unsigned int temp;       
	 SysTick->LOAD=nus*fac_us;       
	 SysTick->VAL=0x00;        
	 SysTick->CTRL=0x01 ;      
	 do
	 {
			temp=SysTick->CTRL;
	 }
	 while(temp&0x01&&!(temp&(1<<16)));  
	 SysTick->CTRL=0x00;       
	 SysTick->VAL =0X00;       
}

					 	    								   






































