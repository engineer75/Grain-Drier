#include "lpc17xx.h"

#define DC_MOTOR_PIN 20   // Pin number (P1.20)

void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 6000; j++);
}

int main(void)
{
    /* 1. Configure P1.20 as GPIO
       P1.20 uses PINSEL3 bits 9:8 */
    LPC_PINCON->PINSEL3 &= ~(3 << 8);

    /* 2. Set P1.20 as OUTPUT */
    LPC_GPIO1->FIODIR |= (1 << DC_MOTOR_PIN);

    while (1)
    {
        /* MOTOR ON */
        LPC_GPIO1->FIOSET = (1 << DC_MOTOR_PIN);
        delay_ms(3000);

        /* MOTOR OFF */
        LPC_GPIO1->FIOCLR = (1 << DC_MOTOR_PIN);
        delay_ms(3000);
    }
}
