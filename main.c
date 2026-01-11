#include <LPC17xx.h>
#include <rtl.h>
#include <stdio.h>

/* ================= UART ================= */
OS_MUT uart_mutex;

void UART0_Init(void) {
    LPC_SC->PCONP |= (1 << 3);          // UART0 power
    LPC_SC->PCLKSEL0 &= ~(3 << 6);      // PCLK = CCLK/4

    LPC_PINCON->PINSEL0 &= ~(0xF << 4);
    LPC_PINCON->PINSEL0 |=  (0x5 << 4); // P0.2 TXD0, P0.3 RXD0

    LPC_UART0->LCR = 0x83;              // 8N1 + DLAB
    LPC_UART0->DLL = 162;               // 9600 baud @ 25 MHz
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 0x03;              // DLAB off
}

static void uart_putc(char c) {
    while (!(LPC_UART0->LSR & (1 << 5)));
    LPC_UART0->THR = c;
}

void uart_puts(const char *s) {
    os_mut_wait(&uart_mutex, 0xFFFF);
    while (*s) uart_putc(*s++);
    os_mut_release(&uart_mutex);
}

/* ================= ADC (HW080) ================= */
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

static void delay_us(uint32_t us) {
    us *= 25;                          // ~100 MHz
    while (us--) __NOP();
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

static void dht_output(void) { DHT_PORT->FIODIR |=  DHT_MASK; }
static void dht_input(void)  { DHT_PORT->FIODIR &= ~DHT_MASK; }

static int dht_wait(int level, uint32_t timeout_us) {
    while (timeout_us--) {
        if (((DHT_PORT->FIOPIN & DHT_MASK) != 0) == level)
            return 1;
        delay_us(1);
    }
    return 0;
}

uint8_t DHT11_Read(uint8_t *h, uint8_t *t) {
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

    __disable_irq();

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            data[i] <<= 1;

            if (!dht_wait(1, 100)) goto error;
            delay_us(30);

            if (DHT_PORT->FIOPIN & DHT_MASK)
                data[i] |= 1;

            if (!dht_wait(0, 100)) goto error;
        }
    }

	
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
        return 0;

    *h = data[0];
    *t = data[2];
    return 1;

	error: __enable_irq();
    return 0;
}

/* ================= TASKS ================= */
__task void DHT_Task(void) {
    uint8_t h, t;
    char buf[64];

  //  uart_puts("DHT Running...\r\n");

    while (1) {
        if (DHT11_Read(&h, &t))
            sprintf(buf, "DHT11 -> Hum:%u%% Temp:%uC\r\n", h, t);
        else
            sprintf(buf, "DHT11 -> ERROR\r\n");

        uart_puts(buf);
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

        sprintf(buf, "HW080 -> ADC:%d Moist:%d%%\r\n", adc, moisture);
        uart_puts(buf);

        os_dly_wait(100);
    }
}

/* ================= INIT ================= */
__task void init_task(void) {
    os_mut_init(&uart_mutex);

    os_tsk_create(DHT_Task, 1);
    os_tsk_create(Soil_Task, 2);

    os_tsk_delete_self();
}

/* ================= MAIN ================= */
int main(void) {
    SystemInit();

    /* P0.16 GPIO, no pull-up/down */
    LPC_PINCON->PINSEL1 &= ~(3 << 0);
    LPC_PINCON->PINMODE1 &= ~(3 << 0);
    LPC_PINCON->PINMODE1 |=  (2 << 0);

    UART0_Init();
    ADC_Init();
    dht_input();

    os_sys_init(init_task);
    while (1);
}
