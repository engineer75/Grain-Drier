#include <LPC17xx.h>
#include <stdio.h>

/* -------- CALIBRATION -------- */
#define ADC_DRY  3800
#define ADC_WET  3500
#define ADC_MIN_VALID   50
#define ADC_MAX_VALID   4100
#define STUCK_LIMIT     10

/* -------------------------------- */

void delay_ms(uint32_t ms);


void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 6000; j++);
}

/* -------- UART -------- */
void UART0_Init(void)
{
    LPC_SC->PCONP |= (1UL << 3);          // Power UART0
    LPC_SC->PCLKSEL0 &= ~(3UL << 6);      // PCLK = CCLK/4 = 25 MHz

    LPC_PINCON->PINSEL0 &= ~(0xFUL << 4);
    LPC_PINCON->PINSEL0 |=  (0x5UL << 4); // P0.2 TX, P0.3 RX

    LPC_UART0->LCR = 0x83;                // 8N1, DLAB=1
    LPC_UART0->DLM = 0;
    LPC_UART0->DLL = 162;                 // 9600 baud
    LPC_UART0->LCR = 0x03;                // DLAB=0
}

void uart_putc(char c)
{
    while (!(LPC_UART0->LSR & (1UL << 5)));
    LPC_UART0->THR = c;
}

void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

/* -------- ADC -------- */
void ADC_Init(void)
{
    LPC_SC->PCONP |= (1 << 12);   // Power ADC

    /* P0.23 as AD0.0 */
    LPC_PINCON->PINSEL1 &= ~(3 << 14);
    LPC_PINCON->PINSEL1 |=  (1 << 14);

    LPC_ADC->ADCR =
        (1 << 0) |        // Select AD0.0
        (5 << 8) |        // CLKDIV = 5 ? ADC clk = 25MHz / 6 ˜ 4.16MHz
        (1 << 21);        // ADC enable
}

uint16_t ADC_Read(void)
{
    LPC_ADC->ADCR &= ~(7 << 24);   // Clear START bits
    LPC_ADC->ADCR |=  (1 << 24);   // Start conversion

    while (!(LPC_ADC->ADGDR & (1UL << 31)));  // Wait DONE

    return (LPC_ADC->ADGDR >> 4) & 0x0FFF;
}

/* -------- MAIN -------- */
int main(void)
{
    uint16_t adc_value, last_adc = 0;
    float voltage;
    int moisture_percent;
    char buffer[80];
    uint8_t stuck_count = 0;

	SystemInit();
    UART0_Init();
    ADC_Init();

    uart_puts("Soil Moisture Monitoring (with Error Detection)\r\n");

    while (1)
    {
        adc_value = ADC_Read();

        /* -------- ERROR: DISCONNECTED / OPEN -------- */
        if (adc_value < ADC_MIN_VALID || adc_value > ADC_MAX_VALID)
        {
            sprintf(buffer,
                    "ADC:%4d  ERROR: SENSOR DISCONNECTED\r\n",
                    adc_value);
            uart_puts(buffer);
            delay_ms(500);
            continue;
        }

        /* -------- ERROR: STUCK VALUE -------- */
        if (adc_value == last_adc)
        {
            stuck_count++;
            if (stuck_count >= STUCK_LIMIT)
            {
                uart_puts("ERROR: SENSOR NOT RESPONDING\r\n");
                delay_ms(500);
                continue;
            }
        }
        else
        {
            stuck_count = 0;
        }

        last_adc = adc_value;

        /* -------- NORMAL OPERATION -------- */
        voltage = (adc_value * 3.3f) / 4095.0f;

        moisture_percent =
            ((ADC_DRY - adc_value) * 100) /
            (ADC_DRY - ADC_WET);

        if (moisture_percent < 0)   moisture_percent = 0;
        if (moisture_percent > 100) moisture_percent = 100;

        sprintf(buffer,
                "ADC:%4d  V:%.2f  Moisture:%3d%% ",
                adc_value, voltage, moisture_percent);
        uart_puts(buffer);

        if (moisture_percent < 30)
            uart_puts("DRY\r\n");
        else if (moisture_percent < 70)
            uart_puts("MOIST\r\n");
        else
            uart_puts("WET\r\n");

        delay_ms(1000);
    }
}
