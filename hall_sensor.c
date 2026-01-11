#include "LPC17xx.h"
#include <stdio.h>

/* ---------------- CONFIG ---------------- */
#define HALL_PIN        (1UL << 10)   /* P2.10 */
#define PULSES_PER_REV  1
#define MIN_PULSE_US    10000UL       /* 10 ms debounce */
#define MAX_RPM         5000UL
#define UART_BAUD       9600UL

/* ---------------- GLOBALS ---------------- */
volatile uint32_t last_time = 0;
volatile uint32_t period_us = 0;
volatile uint8_t  rpm_ready = 0;

/* ---------------- UART0 ---------------- */
void UART0_Init(void)
{
    uint32_t pclk, divisor;

    LPC_SC->PCONP |= (1UL << 3);          /* UART0 power */
    LPC_SC->PCLKSEL0 &= ~(3UL << 6);      /* PCLK = CCLK / 4 */

    pclk = SystemCoreClock / 4;
    divisor = pclk / (16UL * UART_BAUD);

    LPC_PINCON->PINSEL0 &= ~(0xFUL << 4);
    LPC_PINCON->PINSEL0 |=  (0x5UL << 4); /* P0.2 TXD0, P0.3 RXD0 */

    LPC_UART0->LCR = 0x83;                /* 8N1, DLAB = 1 */
    LPC_UART0->DLL = divisor & 0xFF;
    LPC_UART0->DLM = (divisor >> 8) & 0xFF;
    LPC_UART0->LCR = 0x03;                /* DLAB = 0 */
}

void UART0_SendChar(char ch)
{
    while (!(LPC_UART0->LSR & (1UL << 5)));
    LPC_UART0->THR = ch;
}

void UART0_Print(const char *s)
{
    while (*s)
    {
        UART0_SendChar(*s++);
    }
}

/* ---------------- TIMER1 (1 µs) ---------------- */
void Timer1_Init(void)
{
    uint32_t pclk;

    LPC_SC->PCONP |= (1UL << 2);          /* Timer1 power */
    LPC_SC->PCLKSEL0 &= ~(3UL << 4);      /* PCLK = CCLK / 4 */

    pclk = SystemCoreClock / 4;

    LPC_TIM1->TCR = 0x02;                 /* Reset */
    LPC_TIM1->PR  = (pclk / 1000000UL) - 1;
    LPC_TIM1->TCR = 0x01;                 /* Enable */
}

/* ---------------- HALL SENSOR ---------------- */
void Hall_Init(void)
{
    LPC_PINCON->PINSEL4 &= ~(3UL << 20);  /* P2.10 GPIO */
    LPC_PINCON->PINMODE4 |=  (2UL << 20); /* Pull-down */

    LPC_GPIO2->FIODIR &= ~HALL_PIN;       /* Input */

    LPC_GPIOINT->IO2IntClr = HALL_PIN;
    LPC_GPIOINT->IO2IntEnR = HALL_PIN;    /* Rising edge */

    NVIC_EnableIRQ(EINT3_IRQn);
}

/* ---------------- GPIO ISR ---------------- */
void EINT3_IRQHandler(void)
{
    uint32_t now, diff;

    if (LPC_GPIOINT->IO2IntStatR & HALL_PIN)
    {
        now  = LPC_TIM1->TC;
        diff = now - last_time;           /* Handles wraparound */

        if (diff >= MIN_PULSE_US)
        {
            period_us = diff;
            last_time = now;
            rpm_ready = 1;
        }

        LPC_GPIOINT->IO2IntClr = HALL_PIN;
    }
}

/* ---------------- MAIN ---------------- */
int main(void)
{
    char buf[40];
    uint32_t rpm;
    uint32_t p;

    SystemInit();
    UART0_Init();
    Timer1_Init();
    Hall_Init();

    UART0_Print("\r\nStable RPM Measurement\r\n");

    while (1)
    {
        if (rpm_ready)
        {
            __disable_irq();
            p = period_us;
            rpm_ready = 0;
            __enable_irq();

            rpm = (60000000UL / p) / PULSES_PER_REV;

            if (rpm <= MAX_RPM)
            {
                snprintf(buf, sizeof(buf),
                         "Speed: %lu RPM\r\n", rpm);
                UART0_Print(buf);
            }
        }
    }
}
