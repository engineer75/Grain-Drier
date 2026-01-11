#include <LPC17xx.h>
#include <rtl.h>
#include <stdio.h>

/* -------- CALIBRATION -------- */
#define ADC_DRY         3800
#define ADC_WET         3500
#define ADC_MIN_VALID   50
#define ADC_MAX_VALID   4100
#define STUCK_LIMIT     10

/* -------- UART -------- */
void UART0_Init (void) {
    LPC_SC->PCONP |= (1UL << 3);
    LPC_SC->PCLKSEL0 &= ~(3UL << 6);      // PCLK = CCLK/4

    LPC_PINCON->PINSEL0 &= ~(0xF << 4);
    LPC_PINCON->PINSEL0 |=  (0x5 << 4);   // P0.2 TX, P0.3 RX

    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 162;                 // 9600 baud @ 25MHz
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 0x03;
}

void uart_putc (char c) {
    while (!(LPC_UART0->LSR & (1 << 5)));
    LPC_UART0->THR = c;
}

void uart_puts (const char *s) {
    while (*s) uart_putc(*s++);
}

/* -------- ADC -------- */
void ADC_Init (void) {
    LPC_SC->PCONP |= (1 << 12);

    LPC_PINCON->PINSEL1 &= ~(3 << 14);
    LPC_PINCON->PINSEL1 |=  (1 << 14);    // P0.23 AD0.0

    LPC_ADC->ADCR =
        (1 << 0)  |
        (5 << 8)  |
        (1 << 21);
}

U16 ADC_Read (void) {
    LPC_ADC->ADCR &= ~(7 << 24);
    LPC_ADC->ADCR |=  (1 << 24);

    while (!(LPC_ADC->ADGDR & (1UL << 31)));
    return (LPC_ADC->ADGDR >> 4) & 0x0FFF;
}

/* -------- RTX TASK -------- */
__task void Soil_Task (void) {
    U16 adc, last = 0;
    U8  stuck = 0;
    float v;
    int moisture;
    char buf[64];

    uart_puts("\r\nRTX STARTED\r\n");

    for (;;) {

        adc = ADC_Read();

        if (adc < ADC_MIN_VALID || adc > ADC_MAX_VALID) {
            sprintf(buf, "ADC:%4d SENSOR ERROR\r\n", adc);
            uart_puts(buf);
            os_dly_wait(50);   // 500 ms
            continue;
        }

        if (adc == last) {
            if (++stuck >= STUCK_LIMIT) {
                uart_puts("SENSOR STUCK\r\n");
                os_dly_wait(50);
                continue;
            }
        } else {
            stuck = 0;
        }

        last = adc;

        v = (adc * 3.3f) / 4095.0f;
        moisture = ((ADC_DRY - adc) * 100) / (ADC_DRY - ADC_WET);

        if (moisture < 0)   moisture = 0;
        if (moisture > 100) moisture = 100;

        sprintf(buf,
                "ADC:%4d  V:%.2f  M:%3d%%\r\n",
                adc, v, moisture);
        uart_puts(buf);

        os_dly_wait(100);   // 1 second
    }
}

/* -------- MAIN -------- */
int main (void) {
    SystemInit();
    UART0_Init();
    ADC_Init();

    uart_puts("BOOT OK\r\n");

    os_sys_init(Soil_Task);

    for (;;);   // never reached
}
