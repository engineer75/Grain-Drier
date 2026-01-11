#include <RTL.h>
#include "LPC17xx.h"
#include <stdio.h>

/* ---------------- CONFIG ---------------- */
#define HALL_PIN        (1UL << 10)   /* P2.10 */
#define PULSES_PER_REV  1
#define MIN_PULSE_US    10000UL       /* 10 ms debounce */
#define MAX_RPM         5000UL
#define UART_BAUD       9600UL

/* ---------------- UART0 ---------------- */
void UART0_Init(void)
{
    uint32_t pclk, divisor;

    LPC_SC->PCONP |= (1UL << 3);
    LPC_SC->PCLKSEL0 &= ~(3UL << 6);

    pclk = SystemCoreClock / 4;
    divisor = pclk / (16UL * UART_BAUD);

    LPC_PINCON->PINSEL0 &= ~(0xFUL << 4);
    LPC_PINCON->PINSEL0 |=  (0x5UL << 4);

    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = divisor & 0xFF;
    LPC_UART0->DLM = divisor >> 8;
    LPC_UART0->FCR = 0x07;
    LPC_UART0->LCR = 0x03;
}

void UART0_SendChar(char ch)
{
    while (!(LPC_UART0->LSR & (1UL << 5)));
    LPC_UART0->THR = ch;
}

void UART0_Print(const char *s)
{
    while (*s) UART0_SendChar(*s++);
}

/* ---------------- TIMER1 (1 µs) ---------------- */
void Timer1_Init(void)
{
    uint32_t pclk;

    LPC_SC->PCONP |= (1UL << 2);
    LPC_SC->PCLKSEL0 &= ~(3UL << 4);

    pclk = SystemCoreClock / 4;

    LPC_TIM1->TCR = 0x02;
    LPC_TIM1->PR  = (pclk / 1000000UL) - 1;
    LPC_TIM1->TCR = 0x01;
}

/* ---------------- HALL PIN ---------------- */
void Hall_Init(void)
{
    LPC_PINCON->PINSEL4 &= ~(3UL << 20);
    LPC_PINCON->PINMODE4 &= ~(3UL << 20);
    LPC_PINCON->PINMODE4 |=  (2UL << 20);

    LPC_GPIO2->FIODIR &= ~HALL_PIN;
}

/* ---------------- RPM TASK ---------------- */
__task void RPM_Task(void)
{
    char buf[40];
    uint32_t last = 0, now, diff,i;
    uint32_t rpm, rpm_avg;
    uint32_t prev_pin = 0, curr_pin;
    uint8_t  first_edge = 1;

    /* averaging */
    uint32_t rpm_buf[4] = {0};
    uint8_t  idx = 0;
    uint8_t  count = 0;

    UART0_Print("Stable RPM Measurement\r\n");

    for (;;)
    {
        os_dly_wait(1);   /* ~10 ms */

        curr_pin = LPC_GPIO2->FIOPIN & HALL_PIN;

        if (curr_pin && !prev_pin)
        {
            now = LPC_TIM1->TC;

            if (first_edge)
            {
                last = now;
                first_edge = 0;
            }
            else
            {
                diff = now - last;

                /* HARD DEBOUNCE */
                if (diff < MIN_PULSE_US)
                    goto next;

                last = now;

                rpm = (60000000UL / diff) / PULSES_PER_REV;

                /* PHYSICAL LIMITS */
                if (rpm > MAX_RPM || rpm < 50)
                    goto next;

                /* MOVING AVERAGE (4 samples) */
                rpm_buf[idx] = rpm;
                idx = (idx + 1) & 3;
                if (count < 4) count++;

                rpm_avg = 0;
                for (i = 0; i < count; i++)
                    rpm_avg += rpm_buf[i];
                rpm_avg /= count;

                snprintf(buf, sizeof(buf),
                         "Speed: %lu RPM\r\n", rpm_avg);
                UART0_Print(buf);
            }
        }

    next:
        prev_pin = curr_pin;
    }
}


/* ---------------- INIT TASK ---------------- */
__task void Init_Task(void)
{
    Timer1_Init();
    Hall_Init();

    os_tsk_create(RPM_Task, 1);
    os_tsk_delete_self();
}

/* ---------------- MAIN ---------------- */
int main(void)
{
    SystemInit();

    UART0_Init();
    UART0_Print("\r\nRTX starting...\r\n");

    os_sys_init(Init_Task);

    while (1);
}
