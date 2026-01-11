#include <LPC17xx.h>
#include <rtl.h>
#include <stdio.h>

/* ---------------- GLOBALS ---------------- */
OS_MUT uart_mutex;

/* ---------------- UART ---------------- */
void UART0_Init(void){
    LPC_SC->PCONP |= (1<<3);
    LPC_SC->PCLKSEL0 &= ~(3<<6);
    LPC_PINCON->PINSEL0 = (LPC_PINCON->PINSEL0 & ~(0xF<<4)) | (0x5<<4);
    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 162;
    LPC_UART0->DLM = 0;
    LPC_UART0->FCR = 0x07;
    LPC_UART0->LCR = 0x03;
}

static void uart_putc(char c){
    while(!(LPC_UART0->LSR & (1<<5)));
    LPC_UART0->THR = c;
}

void uart_puts(const char *s){
    os_mut_wait(&uart_mutex, 0xFFFF);
    while(*s){
        uart_putc(*s++);
    }
    os_mut_release(&uart_mutex);
}

/* ---------------- TIMER1 ---------------- */
void Timer1_Init(void){
    LPC_SC->PCONP |= (1<<2);
    LPC_SC->PCLKSEL0 &= ~(3<<4);
    LPC_TIM1->TCR = 0x02;
    LPC_TIM1->PR  = (SystemCoreClock/4/1000000)-1;
    LPC_TIM1->TC  = 0;
    LPC_TIM1->TCR = 0x01;
}

/* ---------------- HALL SENSOR ---------------- */
#define HALL_PIN (1<<10)

void Hall_Init(void){
    LPC_PINCON->PINSEL4 &= ~(3<<20);
    LPC_PINCON->PINMODE4 |= (2<<20);
    LPC_GPIO2->FIODIR &= ~HALL_PIN;
}

__task void RPM_Task(void){
    uint32_t last;
    uint32_t prev;
    uint32_t curr;
    uint32_t now;
    uint32_t diff;
    uint32_t rpm;
    char buf[40];

    last = 0;
    prev = 0;

    while(1){
        os_dly_wait(1);
        curr = LPC_GPIO2->FIOPIN & HALL_PIN;

        if(curr && !prev){
            now = LPC_TIM1->TC;
            diff = now - last;
            last = now;

            if(diff > 10000){
                rpm = 60000000UL / diff;
                sprintf(buf, "Speed : %lu\r\n", rpm);
                uart_puts(buf);
            }
        }
        prev = curr;
    }
}

/* ---------------- ADC ---------------- */
#define ADC_DRY   3800
#define ADC_WET   3500
#define ADC_RANGE (ADC_DRY - ADC_WET)

void ADC_Init(void){
    LPC_SC->PCONP |= (1<<12);
    LPC_SC->PCLKSEL0 &= ~(3<<24);
    LPC_PINCON->PINSEL1 |= (1<<14);
    LPC_PINCON->PINMODE1 |= (2<<14);
    LPC_ADC->ADCR = (1<<0)|(5<<8)|(1<<21);
}

static uint16_t ADC_Read(void){
    LPC_ADC->ADCR |= (1<<24);
    while(!(LPC_ADC->ADGDR & (1UL<<31)));
    LPC_ADC->ADCR &= ~(7<<24);
    return (uint16_t)((LPC_ADC->ADGDR >> 4) & 0xFFF);
}

static int Moisture_Percent(uint16_t adc){
    int m;
    m = ((ADC_DRY - adc) * 100) / ADC_RANGE;
    if(m < 0) m = 0;
    if(m > 100) m = 100;
    return m;
}

__task void Moisture_Task(void){
    int m;
    char buf[48];

    while(1){
        m = Moisture_Percent(ADC_Read());
        sprintf(buf, "Moisture : %d%%\r\n", m);
        uart_puts(buf);
        os_dly_wait(50);
    }
}

/* ---------------- DHT11 ---------------- */
#define DHT_PORT LPC_GPIO0
#define DHT_PIN  16
#define DHT_MASK (1<<DHT_PIN)

static void delay_us(uint32_t us){
    us *= 25;
    while(us--) __NOP();
}

static void delay_ms(uint32_t ms){
    while(ms--) delay_us(1000);
}

static void dht_out(void){
    DHT_PORT->FIODIR |= DHT_MASK;
}

static void Dht_Init(void){
    DHT_PORT->FIODIR &= ~DHT_MASK;
}

static int dht_wait(int lvl, uint32_t t){
    while(t--){
        if(((DHT_PORT->FIOPIN & DHT_MASK) != 0) == lvl){
            return 1;
        }
        delay_us(1);
    }
    return 0;
}

uint8_t DHT11_Read(uint8_t *h, uint8_t *t){
    uint8_t d[5];
    uint32_t i;
    uint32_t j;

    for(i=0;i<5;i++) d[i]=0;

    dht_out();
    DHT_PORT->FIOCLR = DHT_MASK;
    delay_ms(20);
    DHT_PORT->FIOSET = DHT_MASK;
    delay_us(30);
    Dht_Init();

    if(!dht_wait(0,100) || !dht_wait(1,100) || !dht_wait(0,100))
        return 0;

    __disable_irq();
    for(i=0;i<5;i++){
        for(j=0;j<8;j++){
            d[i] <<= 1;
            if(!dht_wait(1,100)) goto exit;
            delay_us(30);
            if(DHT_PORT->FIOPIN & DHT_MASK) d[i] |= 1;
            if(!dht_wait(0,100)) goto exit;
        }
    }
exit:
    __enable_irq();

    if((uint8_t)(d[0]+d[1]+d[2]+d[3]) != d[4])
        return 0;

    *h = d[0];
    *t = d[2];
    return 1;
}

__task void DHT_Task(void){
    uint8_t h;
    uint8_t t;
    char buf[48];

    while(1){
        if(DHT11_Read(&h,&t))
            sprintf(buf,"Hum:%u%% | Temp:%uC\r\n",h,t);
        else
            sprintf(buf,"DHT11(Temp) ERROR\r\n");

        uart_puts(buf);
        os_dly_wait(50);
    }
}

/* ---------------- STEPPER ---------------- */
#define IN1 (1<<4)
#define IN2 (1<<5)
#define IN3 (1<<6)
#define IN4 (1<<7)
#define STEPS 125
#define CLR_ALL (IN1|IN2|IN3|IN4)

void Stepper_Init(void){
    LPC_PINCON->PINSEL0 &= ~(0xFF<<8);
    LPC_GPIO0->FIODIR |= CLR_ALL;
}

static void step_delay_us(uint32_t us){
    uint32_t s;
    s = LPC_TIM1->TC;
    while((LPC_TIM1->TC - s) < us);
}

static void step_seq(uint32_t mask){
    LPC_GPIO0->FIOCLR = CLR_ALL;
    LPC_GPIO0->FIOSET = mask;
    step_delay_us(2000);
}

static void step_cw(void){
    step_seq(IN1|IN2);
    step_seq(IN2|IN3);
    step_seq(IN3|IN4);
    step_seq(IN4|IN1);
}

static void step_ccw(void){
    step_seq(IN4|IN1);
    step_seq(IN3|IN4);
    step_seq(IN2|IN3);
    step_seq(IN1|IN2);
}

__task void Stepper_Task(void){
    uint32_t i;

    while(1){
        for(i=0;i<STEPS;i++) step_cw();
        os_dly_wait(50);
        for(i=0;i<STEPS;i++) step_ccw();
        os_dly_wait(50);
    }
}

/* ---------------- DC MOTOR ---------------- */
#define DC_MOTOR_PIN (1<<0)
#define MOISTURE_MIN 5

void DCMotor_Init(void){
    LPC_GPIO2->FIODIR |= DC_MOTOR_PIN;
    LPC_GPIO2->FIOSET  = DC_MOTOR_PIN;
}

__task void DCMotor_Task(void){
    int m;
    char buf[50];

    while(1){
        m = Moisture_Percent(ADC_Read());

        if(m > MOISTURE_MIN){
            LPC_GPIO2->FIOCLR = DC_MOTOR_PIN;
            sprintf(buf,"Fan : ON\r\n");
        }else{
            LPC_GPIO2->FIOSET = DC_MOTOR_PIN;
            sprintf(buf,"Fan : OFF\r\n");
        }
        uart_puts(buf);
        os_dly_wait(50);
    }
}

/* ---------------- INIT ---------------- */
__task void init_task(void){
    os_mut_init(&uart_mutex);

    UART0_Init();
    ADC_Init();
    Timer1_Init();
    Hall_Init();
    Stepper_Init();
    Dht_Init();
    DCMotor_Init();

    os_tsk_create(RPM_Task,3);
    os_tsk_create(DHT_Task,2);
    os_tsk_create(Moisture_Task,2);
    os_tsk_create(DCMotor_Task,2);
    os_tsk_create(Stepper_Task,1);

    os_tsk_delete_self();
}

/* ---------------- MAIN ---------------- */
int main(void){
    SystemInit();
    LPC_PINCON->PINSEL1 &= ~(3<<0);
    LPC_PINCON->PINMODE1 |= (2<<0);
    os_sys_init(init_task);
    while(1);
}
