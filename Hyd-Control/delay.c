#include "delay.h"

void delay_1ms(void)
{
	unsigned int i;
	for(i=1;i<(unsigned int)(XTAL*143-2);i++);
}

void delay_ms(unsigned int ms)
{
  unsigned int i;
  for(i=0;i<ms;i++)
  delay_1ms();
}

void delay_1us(void)
{
   unsigned int i;
   for(i=0;i<(unsigned int)XTAL;i++)
   asm("nop");
}

void delay_us(unsigned int us)
{  
	unsigned int i;
	for(i=0;i<us;i++)
	delay_1us();
}
