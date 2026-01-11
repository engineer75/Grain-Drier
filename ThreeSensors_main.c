#include <LPC17xx.h>
#include <rtl.h>
#include <stdio.h>

/* =========================================================
   ------------------- KERNEL OBJECTS -----------------------
   ========================================================= */
OS_MUT uart_mutex;

/* =========================================================
   ---------------------- UART0 -----------------------------
   ========================================================= */
void UART0_Init(void) {
    LPC_SC->PCONP |= (1 << 3);          // UART0 power
    LPC_SC->PCLKSEL0 &= ~(3 << 6);      // PCLK = CCLK/4

    LPC_PINCON->PINSEL0 &= ~(0xF << 4);
    LPC_PINCON->PINSEL0 |=  (0x5 << 4); // P0.2 TXD0, P0.3 RXD0

    LPC_UART0->LCR = 0x83;              // 8N1 + DLAB
    LPC_UART0->DLL = 162;               // 9600 baud @ 25 MHz
    LPC_UART0->DLM = 0;
    LPC_UART0->FCR = 0x07;              // FIFO enable
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

/* =========================================================
   ---------------------- TIMER1 ----------------------------
   ========================================================= */
void Timer1_Init(void) {
    LPC_SC->PCONP |= (1 << 2);          // Timer1 power
    LPC_SC->PCLKSEL0 &= ~(3 << 4);      // CCLK/4

    LPC_TIM1->TCR = 0x02;               // Reset
    LPC_TIM1->PR  = (SystemCoreClock / 4 / 1000000) - 1;
    LPC_TIM1->TC  = 0;
    LPC_TIM1->TCR = 0x01;               // Enable
}

/* =========================================================
   -------------------- HALL SENSOR -------------------------
   ========================================================= */
#define HALL_PIN (1 << 10)              // P2.10

void Hall_Init(void) {
    LPC_PINCON->PINSEL4 &= ~(3 << 20);  // GPIO
    LPC_PINCON->PINMODE4 |=  (2 << 20); // Pull-down
    LPC_GPIO2->FIODIR &= ~HALL_PIN;
}

__task void RPM_Task(void) {
    uint32_t last = 0, now, diff;
    uint32_t prev = 0, curr;
    char buf[40];

    while (1) {
        os_dly_wait(1);                 // 10 ms

        curr = LPC_GPIO2->FIOPIN & HALL_PIN;
        if (curr && !prev) {
            now  = LPC_TIM1->TC;
            diff = now - last;
            last = now;

            if (diff > 10000) {         // noise reject
                uint32_t rpm = 60000000UL / diff;
                sprintf(buf, "RPM -> %lu\r\n", rpm);
                uart_puts(buf);
            }
        }
        prev = curr;
    }
}

/* =========================================================
   ---------------------- ADC -------------------------------
   ========================================================= */
#define ADC_DRY 3800
#define ADC_WET 3500

void ADC_Init(void) {
    LPC_SC->PCONP |= (1 << 12);         // ADC power
    LPC_SC->PCLKSEL0 &= ~(3 << 24);     // CCLK/4

    LPC_PINCON->PINSEL1 &= ~(3 << 14);
    LPC_PINCON->PINSEL1 |=  (1 << 14);  // P0.23 AD0.0
    LPC_PINCON->PINMODE1 |= (2 << 14);  // no pull

    LPC_ADC->ADCR =
        (1 << 0) |                     // AD0.0
        (5 << 8) |                     // clkdiv
        (1 << 21);                     // enable
}

uint16_t ADC_Read(void) {
    LPC_ADC->ADCR |= (1 << 24);         // start
    while (!(LPC_ADC->ADGDR & (1UL << 31)));
    LPC_ADC->ADCR &= ~(7 << 24);        // stop
    return (LPC_ADC->ADGDR >> 4) & 0x0FFF;
}

__task void Soil_Task(void) {
    char buf[48];
    int adc, moisture;

    while (1) {
        adc = ADC_Read();

        moisture = ((ADC_DRY - adc) * 100) / (ADC_DRY - ADC_WET);
        if (moisture < 0)   moisture = 0;
        if (moisture > 100) moisture = 100;

        sprintf(buf, "Soil -> ADC:%d %d%%\r\n", adc, moisture);
        uart_puts(buf);
        os_dly_wait(100);
    }
}

/* =========================================================
   ---------------------- DHT11 -----------------------------
   ========================================================= */
#define DHT_PORT LPC_GPIO0
#define DHT_PIN  16
#define DHT_MASK (1 << DHT_PIN)

static void delay_us(uint32_t us) {
    us *= 25;
    while (us--) __NOP();
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

static void dht_out(void) { DHT_PORT->FIODIR |=  DHT_MASK; }
static void dht_in(void)  { DHT_PORT->FIODIR &= ~DHT_MASK; }

static int dht_wait(int level, uint32_t t) {
    while (t--) {
        if (((DHT_PORT->FIOPIN & DHT_MASK) != 0) == level)
            return 1;
        delay_us(1);
    }
    return 0;
}

uint8_t DHT11_Read(uint8_t *h, uint8_t *t) {
    uint8_t d[5] = {0};
    uint32_t i, j;

    dht_out();
    DHT_PORT->FIOCLR = DHT_MASK;
    delay_ms(20);
    DHT_PORT->FIOSET = DHT_MASK;
    delay_us(30);
    dht_in();

    if (!dht_wait(0, 100)) return 0;
    if (!dht_wait(1, 100)) return 0;
    if (!dht_wait(0, 100)) return 0;

    __disable_irq();                   // REQUIRED

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            d[i] <<= 1;
            if (!dht_wait(1, 100)) goto exit;
            delay_us(30);
            if (DHT_PORT->FIOPIN & DHT_MASK) d[i] |= 1;
            if (!dht_wait(0, 100)) goto exit;
        }
    }

exit:
    __enable_irq();                    // ALWAYS restore

    if ((uint8_t)(d[0]+d[1]+d[2]+d[3]) != d[4])
        return 0;

    *h = d[0];
    *t = d[2];
    return 1;
}

__task void DHT_Task(void) {
    uint8_t h, t;
    char buf[48];

    while (1) {
        if (DHT11_Read(&h, &t))
            sprintf(buf, "DHT -> H:%u%% T:%uC\r\n", h, t);
        else
            sprintf(buf, "DHT -> ERROR\r\n");

        uart_puts(buf);
        os_dly_wait(100);
    }
}

/* =========================================================
   ---------------------- INIT TASK -------------------------
   ========================================================= */
__task void init_task(void) {
    os_mut_init(&uart_mutex);

    UART0_Init();
    ADC_Init();
    Timer1_Init();
    Hall_Init();

    dht_in();

    os_tsk_create(RPM_Task, 3);
    os_tsk_create(DHT_Task, 2);
    os_tsk_create(Soil_Task, 1);

    os_tsk_delete_self();
}

/* =========================================================
   ------------------------- MAIN ---------------------------
   ========================================================= */
int main(void) {
    SystemInit();

    LPC_PINCON->PINSEL1 &= ~(3 << 0);   // P0.16 GPIO
    LPC_PINCON->PINMODE1 |=  (2 << 0);  // no pull

    os_sys_init(init_task);
    while (1);
}
