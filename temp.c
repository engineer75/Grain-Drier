#include <rtl.h>
#include "LPC17xx.h"
#include <stdint.h>
#include <stdio.h>

/* =========================================================
   TASK PRIORITIES (OPTIMIZED)
   ========================================================= */
#define PRIO_SGP30     0   // Highest priority
#define PRIO_SENSOR    2   // Medium priority

/* =========================================================
   GLOBALS
   ========================================================= */
volatile uint16_t sgp_eco2 = 0;
volatile uint16_t sgp_tvoc = 0;
OS_MUT i2c_mutex;

/* =========================================================
   STATUS TYPE
   ========================================================= */
typedef enum {
    STATUS_LOW = 0,
    STATUS_NORMAL,
    STATUS_HIGH
} sensor_status_t;

const char* status_str(sensor_status_t s) {
    return (s == STATUS_NORMAL) ? "NORMAL" :
           (s == STATUS_HIGH)   ? "HIGH"   : "LOW";
}

/* =========================================================
   THRESHOLDS
   ========================================================= */
#define CO2_LOW   400
#define CO2_HIGH  1000
#define PM25_LOW  0
#define PM25_HIGH 35
#define HUM_LOW   30
#define HUM_HIGH  70
#define TEMP_LOW  18
#define TEMP_HIGH 35
#define ECO2_LOW  400
#define ECO2_HIGH 1000
#define TVOC_LOW  0
#define TVOC_HIGH 220
#define LED_BLINK_DELAY 100

/* =========================================================
   LED DEFINITIONS
   ========================================================= */
#define LED_CO2_NORMAL    (1<<0)
#define LED_CO2_ALERT     (1<<1)
#define LED_PM_NORMAL     (1<<2)
#define LED_PM_ALERT      (1<<3)
#define LED_HUM_NORMAL    (1<<4)
#define LED_HUM_ALERT     (1<<5)
#define LED_TEMP_NORMAL   (1<<6)
#define LED_TEMP_ALERT    (1<<7)
#define LED_TVOC_NORMAL   (1<<8)
#define LED_TVOC_ALERT    (1<<9)
#define LED_ECO2_NORMAL   (1<<18)
#define LED_ECO2_ALERT    (1<<19)

/* =========================================================
   UART0 : DEBUG
   ========================================================= */
void UART0_Init(void) {
    LPC_SC->PCONP |= (1 << 3);
    LPC_PINCON->PINSEL0 &= ~0xF0;
    LPC_PINCON->PINSEL0 |=  0x50;
    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 162;
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 0x03;
}

int fputc(int ch, FILE *f) {
    while (!(LPC_UART0->LSR & (1 << 5)));
    LPC_UART0->THR = ch;
    return ch;
}

/* =========================================================
   UART2 : MH-Z19C
   ========================================================= */
void UART2_Init(void) {
    LPC_SC->PCONP |= (1 << 24);
    LPC_PINCON->PINSEL0 &= ~(0xF << 20);
    LPC_PINCON->PINSEL0 |=  (0x5 << 20);
    LPC_UART2->LCR = 0x83;
    LPC_UART2->DLL = 162;
    LPC_UART2->DLM = 0;
    LPC_UART2->LCR = 0x03;
}

static void UART2_Send(uint8_t b) {
    while (!(LPC_UART2->LSR & (1 << 5)));
    LPC_UART2->THR = b;
}

static uint8_t UART2_Read(void) {
    while (!(LPC_UART2->LSR & 1));
    return LPC_UART2->RBR;
}

uint16_t MHZ19C_ReadCO2(void) {
    uint8_t cmd[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};
    uint8_t rsp[9];
    int i;

    for (i = 0; i < 9; i++) UART2_Send(cmd[i]);
    for (i = 0; i < 9; i++) rsp[i] = UART2_Read();

    if (rsp[0] != 0xFF || rsp[1] != 0x86) return 0;
    return ((uint16_t)rsp[2] << 8) | rsp[3];
}

/* =========================================================
   UART3 : PMS5003
   ========================================================= */
void UART3_Init(void) {
    LPC_SC->PCONP |= (1 << 25);
    LPC_PINCON->PINSEL0 &= ~0x0F;
    LPC_PINCON->PINSEL0 |=  0x0A;
    LPC_UART3->LCR = 0x83;
    LPC_UART3->DLL = 162;
    LPC_UART3->DLM = 0;
    LPC_UART3->LCR = 0x03;
    LPC_UART3->FCR = 0x07;
}

uint8_t UART3_Read(void) {
    while (!(LPC_UART3->LSR & 1));
    return LPC_UART3->RBR;
}

uint16_t PMS5003_Read_PM25(void) {
    uint8_t buf[32];
    uint16_t sum = 0;
    int i;

    do { buf[0] = UART3_Read(); } while (buf[0] != 0x42);
    buf[1] = UART3_Read();
    if (buf[1] != 0x4D) return 0;

    for (i = 2; i < 32; i++) buf[i] = UART3_Read();
    if ((((uint16_t)buf[2] << 8) | buf[3]) != 28) return 0;

    for (i = 0; i < 30; i++) sum += buf[i];
    if (sum != (((uint16_t)buf[30] << 8) | buf[31])) return 0;

    return (((uint16_t)buf[12] << 8) | buf[13]);
}

/* =========================================================
   DHT11
   ========================================================= */
#define DHT_MASK (1 << 16)

void delay_us(uint32_t us) {
    uint32_t cnt = (SystemCoreClock / 1000000) * us / 5;
    while (cnt--) __NOP();
}

void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

int DHT11_Read(uint8_t *h, uint8_t *t) {
    uint8_t d[5] = {0};
    int i, j;

    __disable_irq();
    LPC_GPIO0->FIODIR |= DHT_MASK;
    LPC_GPIO0->FIOCLR = DHT_MASK;
    delay_ms(20);
    LPC_GPIO0->FIOSET = DHT_MASK;
    delay_us(30);
    LPC_GPIO0->FIODIR &= ~DHT_MASK;

    while (LPC_GPIO0->FIOPIN & DHT_MASK);
    while (!(LPC_GPIO0->FIOPIN & DHT_MASK));
    while (LPC_GPIO0->FIOPIN & DHT_MASK);

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            while (!(LPC_GPIO0->FIOPIN & DHT_MASK));
            delay_us(40);
            d[i] <<= 1;
            if (LPC_GPIO0->FIOPIN & DHT_MASK) d[i] |= 1;
            while (LPC_GPIO0->FIOPIN & DHT_MASK);
        }
    }
    __enable_irq();

    if ((d[0]+d[1]+d[2]+d[3]) != d[4]) return 0;
    *h = d[0];
    *t = d[2];
    return 1;
}

/* =========================================================
   I2C + SGP30
   ========================================================= */
#define SGP30_ADDR 0x58

void I2C0_Init(void) {
    LPC_SC->PCONP |= (1 << 7);
    LPC_PINCON->PINSEL1 &= ~((3 << 22) | (3 << 24));
    LPC_PINCON->PINSEL1 |=  ((1 << 22) | (1 << 24));
    LPC_I2C0->I2SCLH = 60;
    LPC_I2C0->I2SCLL = 60;
    LPC_I2C0->I2CONSET = (1 << 6);
}

void I2C_Start(void) {
    LPC_I2C0->I2CONSET = (1 << 5);
    while (!(LPC_I2C0->I2CONSET & (1 << 3)));
    LPC_I2C0->I2CONCLR = (1 << 5);
}

void I2C_Stop(void) {
    LPC_I2C0->I2CONSET = (1 << 4);
    LPC_I2C0->I2CONCLR = (1 << 3);
}

void I2C_Write(uint8_t d) {
    LPC_I2C0->I2DAT = d;
    LPC_I2C0->I2CONCLR = (1 << 3);
    while (!(LPC_I2C0->I2CONSET & (1 << 3)));
}

uint8_t I2C_Read(uint8_t ack) {
    if (ack) LPC_I2C0->I2CONSET = (1 << 2);
    else     LPC_I2C0->I2CONCLR = (1 << 2);
    LPC_I2C0->I2CONCLR = (1 << 3);
    while (!(LPC_I2C0->I2CONSET & (1 << 3)));
    return LPC_I2C0->I2DAT;
}

uint8_t sgp_crc(uint8_t *d) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

void SGP30_ReadIAQ(void) {
    uint8_t b[6];

    I2C_Start();
    I2C_Write(SGP30_ADDR << 1);
    I2C_Write(0x20);
    I2C_Write(0x08);
    I2C_Stop();

    os_dly_wait(20);

    I2C_Start();
    I2C_Write((SGP30_ADDR << 1) | 1);
    for (int i = 0; i < 6; i++) b[i] = I2C_Read(i < 5);
    I2C_Stop();

    if (sgp_crc(b) == b[2] && sgp_crc(&b[3]) == b[5]) {
        sgp_eco2 = ((uint16_t)b[0] << 8) | b[1];
        sgp_tvoc = ((uint16_t)b[3] << 8) | b[4];
    }
}

/* =========================================================
   TASKS (PRIORITY OPTIMIZED)
   ========================================================= */

__task void SGP30_Task(void) {
    uint8_t warmup = 0;
    os_dly_wait(100);

    os_mut_wait(&i2c_mutex, 0xFFFF);
    I2C_Start();
    I2C_Write(SGP30_ADDR << 1);
    I2C_Write(0x20);
    I2C_Write(0x03);
    I2C_Stop();
    os_mut_release(&i2c_mutex);

    while (1) {
        os_mut_wait(&i2c_mutex, 0xFFFF);
        SGP30_ReadIAQ();
        os_mut_release(&i2c_mutex);

        if (warmup < 15) {
            sgp_eco2 = 0;
            sgp_tvoc = 0;
            warmup++;
        }
        os_dly_wait(100);
    }
}

__task void Sensor_Task(void) {
    uint16_t co2, pm25 = 0, last_pm25 = 0;
    uint8_t h = 0, t = 0;
    uint8_t b1,b2,b3,b4,b5,b6;
    static uint8_t blink = 0;

    while (1) {
        blink ^= 1;

        co2 = MHZ19C_ReadCO2();
        pm25 = PMS5003_Read_PM25();
        if (pm25) last_pm25 = pm25;
        pm25 = last_pm25;
        DHT11_Read(&h, &t);

        LED_Control(co2,CO2_LOW,CO2_HIGH,&LPC_GPIO2->FIOSET,&LPC_GPIO2->FIOCLR,LED_CO2_NORMAL,LED_CO2_ALERT,&b1);
        LED_Control(pm25,PM25_LOW,PM25_HIGH,&LPC_GPIO2->FIOSET,&LPC_GPIO2->FIOCLR,LED_PM_NORMAL,LED_PM_ALERT,&b2);
        LED_Control(h,HUM_LOW,HUM_HIGH,&LPC_GPIO2->FIOSET,&LPC_GPIO2->FIOCLR,LED_HUM_NORMAL,LED_HUM_ALERT,&b3);
        LED_Control(t,TEMP_LOW,TEMP_HIGH,&LPC_GPIO2->FIOSET,&LPC_GPIO2->FIOCLR,LED_TEMP_NORMAL,LED_TEMP_ALERT,&b4);
        LED_Control(sgp_eco2,ECO2_LOW,ECO2_HIGH,&LPC_GPIO1->FIOSET,&LPC_GPIO1->FIOCLR,LED_ECO2_NORMAL,LED_ECO2_ALERT,&b5);
        LED_Control(sgp_tvoc,TVOC_LOW,TVOC_HIGH,&LPC_GPIO2->FIOSET,&LPC_GPIO2->FIOCLR,LED_TVOC_NORMAL,LED_TVOC_ALERT,&b6);

        os_dly_wait(LED_BLINK_DELAY);
        os_tsk_pass();   // yield to higher priority task
    }
}

/* =========================================================
   INIT
   ========================================================= */
__task void init_task(void) {
    UART0_Init();
    UART2_Init();
    UART3_Init();
    I2C0_Init();
    LED_Init();

    os_mut_init(&i2c_mutex);

    os_tsk_create(SGP30_Task, PRIO_SGP30);
    os_tsk_create(Sensor_Task, PRIO_SENSOR);

    os_tsk_delete_self();
}

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();
    os_sys_init(init_task);
    while (1);
}