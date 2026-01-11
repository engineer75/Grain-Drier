#include <LPC17xx.h>
#include <rtl.h>
#include <stdio.h>
#include "lcd.h"

/* ================= LCD RTOS SAFE LAYER ================= */

OS_MUT lcd_mutex;

/* Move LCD cursor */
static void lcd_goto(uint8_t pos)
{
    temp1 = pos;
    lcd_com();
}

/* Init LCD once */
void lcd_safe_init(void)
{
    os_mut_init(&lcd_mutex);
    lcd_init();
    clr_disp();
}

/* Write fixed LCD line (0 or 1) */
void lcd_write_line(uint8_t line, const char *text)
{
    char buf[17];
    int i;

    for (i = 0; i < 16; i++)
        buf[i] = ' ';
    buf[16] = '\0';

    for (i = 0; i < 16 && text[i]; i++)
        buf[i] = text[i];

    os_mut_wait(&lcd_mutex, 0xFFFF);

    if (line == 0)
        lcd_goto(0x80);
    else
        lcd_goto(0xC0);

    lcd_puts((unsigned char *)buf);

    os_mut_release(&lcd_mutex);
}

/* ================= ADC ================= */

#define ADC_DRY 3800
#define ADC_WET 3500

void ADC_Init(void) {
    LPC_SC->PCONP |= (1 << 12);         // ADC power
    LPC_SC->PCLKSEL0 &= ~(3 << 24);     // ADC PCLK = CCLK/4

    LPC_PINCON->PINSEL1 &= ~(3 << 14);
    LPC_PINCON->PINSEL1 |=  (1 << 14); // P0.23 AD0.0
    LPC_PINCON->PINMODE1 |= (2 << 14); // no pull-up/down

    LPC_ADC->ADCR =
        (1 << 0) |                     // AD0.0
        (5 << 8) |                     // clkdiv
        (1 << 21);                     // ADC enable
}

uint16_t ADC_Read(void) {
    LPC_ADC->ADCR |= (1 << 24);         // start
    while (!(LPC_ADC->ADGDR & (1UL << 31)));
    LPC_ADC->ADCR &= ~(7 << 24);        // stop
    return (LPC_ADC->ADGDR >> 4) & 0x0FFF;
}

/* ================= DHT11 ================= */

#define DHT_PORT LPC_GPIO0
#define DHT_PIN  16
#define DHT_MASK (1 << DHT_PIN)

static void delay_us(uint32_t us)
{
    us *= 25;
    while (us--) __NOP();
}

static void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

static void dht_output(void) { DHT_PORT->FIODIR |=  DHT_MASK; }
static void dht_input(void)  { DHT_PORT->FIODIR &= ~DHT_MASK; }

static int dht_wait(int level, uint32_t timeout_us)
{
    while (timeout_us--)
    {
        if (((DHT_PORT->FIOPIN & DHT_MASK) != 0) == level)
            return 1;
        delay_us(1);
    }
    return 0;
}

uint8_t DHT11_Read(uint8_t *h, uint8_t *t)
{
    uint8_t data[5] = {0};
    uint32_t i, j;

    dht_output();
    DHT_PORT->FIOCLR = DHT_MASK;
    delay_ms(20);
    DHT_PORT->FIOSET = DHT_MASK;
    delay_us(30);
    dht_input();

    if (!dht_wait(0, 100)) return 0;
    if (!dht_wait(1, 100)) return 0;
    if (!dht_wait(0, 100)) return 0;

    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            data[i] <<= 1;
            if (!dht_wait(1, 100)) return 0;
            delay_us(30);
            if (DHT_PORT->FIOPIN & DHT_MASK)
                data[i] |= 1;
            if (!dht_wait(0, 100)) return 0;
        }
    }

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
        return 0;

    *h = data[0];
    *t = data[2];
    return 1;
}

/* ================= TASKS ================= */

__task void DHT_Task(void)
{
    uint8_t h, t;
    char buf[17];

    while (1)
    {
        if (DHT11_Read(&h, &t))
            sprintf(buf, "Hum:%2u%% Tmp:%2uC", h, t);
        else
            sprintf(buf, "DHT11 ERROR");

        lcd_write_line(0, buf);
        os_dly_wait(100);
    }
}

__task void Soil_Task(void) {
    char buf[64];
    int adc, moisture;

    //uart_puts("HW080 Running...\r\n");

    while (1) {
        adc = ADC_Read();

        moisture = ((ADC_DRY - adc) * 100) / (ADC_DRY - ADC_WET);
        if (moisture < 0) moisture = 0;
        if (moisture > 100) moisture = 100;

        sprintf(buf, "Moist:%d", moisture);
         lcd_write_line(1, buf);

        os_dly_wait(100);
    }
}

/* ================= INIT TASK ================= */

__task void init_task(void)
{
    lcd_safe_init();
    ADC_Init();
    dht_input();

    os_tsk_create(DHT_Task, 1);
    os_tsk_create(Soil_Task, 2);

    os_tsk_delete_self();
}

/* ================= MAIN ================= */

int main(void)
{
    SystemInit();

    LPC_PINCON->PINSEL1 &= ~(3 << 0);
    LPC_PINCON->PINMODE1 |=  (2 << 0);

    os_sys_init(init_task);
    while (1);
}
