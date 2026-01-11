#include "LPC17xx.h"

#define IN1 (1<<0)   // P0.0
#define IN2 (1<<1)   // P0.1
#define IN3 (1<<2)   // P0.2
#define IN4 (1<<3)   // P0.3

void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i=0;i<ms;i++)
        for(j=0;j<3000;j++);
}

int main(void)
{
    // Configure P0.0–P0.3 as GPIO
    LPC_PINCON->PINSEL0 &= 0xFFFFFF00;
    LPC_GPIO0->FIODIR |= (IN1 | IN2 | IN3 | IN4);
	
while (1)
{
    LPC_GPIO0->FIOCLR = IN1 | IN2 | IN3 | IN4;
    LPC_GPIO0->FIOSET = IN1;
    delay_ms(5);

    LPC_GPIO0->FIOCLR = IN1 | IN2 | IN3 | IN4;
    LPC_GPIO0->FIOSET = IN2;
    delay_ms(5);

    LPC_GPIO0->FIOCLR = IN1 | IN2 | IN3 | IN4;
    LPC_GPIO0->FIOSET = IN3;
    delay_ms(5);

    LPC_GPIO0->FIOCLR = IN1 | IN2 | IN3 | IN4;
    LPC_GPIO0->FIOSET = IN4;
    delay_ms(5);
}

}
