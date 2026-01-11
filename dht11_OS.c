#include <LPC17xx.h>
#include <RTL.h>
#include <stdio.h>

#define DHT_PORT LPC_GPIO0
#define DHT_PIN  16
#define DHT_MASK (1 << DHT_PIN)

void delay_us(uint32_t us) {
    us *= 25;
	while(us--) __NOP();// Approx 1us at 100MHz core clock (adjust if needed)
}

void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

void UART0_Init(void)
{
    LPC_SC->PCONP |= (1 << 3);
    LPC_PINCON->PINSEL0 &= ~0xF0;
    LPC_PINCON->PINSEL0 |=  0x50;       // P0.2 TX, P0.3 RX

    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLM = 0;
    LPC_UART0->DLL = 162;               // 9600 bps @ 25MHz PCLK
    LPC_UART0->LCR = 0x03;
}

void uart_putc(char c) {
    while (!(LPC_UART0->LSR & (1 << 5)));
    LPC_UART0->THR = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static void dht_set_output(void) {
    DHT_PORT->FIODIR |= DHT_MASK;
}

static void dht_set_input(void) {
    DHT_PORT->FIODIR &= ~DHT_MASK;
}

static uint8_t wait_level(uint8_t level)
{
    while (((DHT_PORT->FIOPIN & DHT_MASK) ? 1 : 0) != level)
    {
        delay_us(1);
    }
    return 1;
}


static uint8_t dht_read_bit(void)
{
    if (!wait_level(1)) return 0;
    delay_us(40);                         // RTX-safe sampling point
    return (DHT_PORT->FIOPIN & DHT_MASK) ? 1 : 0;
}

int DHT11_Read(uint8_t *h_int, uint8_t *h_dec, uint8_t *t_int, uint8_t *t_dec)
{
    uint8_t data[5] = {0};
    uint32_t i, j;

    // MCU start signal
    dht_set_output();
    DHT_PORT->FIOCLR = DHT_MASK;
    delay_ms(20);        // >=18 ms
    DHT_PORT->FIOSET = DHT_MASK;
    delay_us(30);
    dht_set_input();

    // DHT response
    if (DHT_PORT->FIOPIN & DHT_MASK) return 0;
    while (!(DHT_PORT->FIOPIN & DHT_MASK));
    while ( (DHT_PORT->FIOPIN & DHT_MASK));

    // Read 40 bits
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            data[i] <<= 1;
            data[i] |= dht_read_bit();
            while (DHT_PORT->FIOPIN & DHT_MASK);   // wait for low
        }
    }

    // checksum
    if ((data[0] + data[1] + data[2] + data[3]) != data[4])
        return 0;

    *h_int = data[0];
    *h_dec = data[1];
    *t_int = data[2];
    *t_dec = data[3];

    return 1;
}
__task void dht_task(void){
	uint8_t h, h_dec, t, t_dec;
	int ok;
	char buf[64];
	uart_puts("DHT11 test running...\r\n");
	uart_puts(buf);

    while (1)
    {
		tsk_lock();
		ok = DHT11_Read(&h, &h_dec, &t, &t_dec);
		tsk_unlock();
        if (ok) {
            sprintf(buf, "Humidity: %d.%d %%  Temp: %d.%d C\r\n",
                    h, h_dec, t, t_dec);
			uart_puts(buf);
            
        } else {
            uart_puts("DHT11 read error\r\n");
        }
        os_dly_wait(100);
    }
}

__task void init_task(void){
	os_tsk_create(dht_task,1);
	os_tsk_delete_self();
}

int main(void)
{
    SystemInit();
    UART0_Init();
	
	LPC_PINCON->PINMODE1 &= ~(3UL << 0);   // P0.16 pull-up
	dht_set_input();
	
	os_sys_init(init_task);
	return 0;
}
