#include <LPC17xx.h>
#include <rtl.h>
#include <stdio.h>
#include "lcd.h"

/* =========================================================
   ------------------- KERNEL OBJECTS -----------------------
   ========================================================= */
OS_MUT uart_mutex;
OS_MUT lcd_mutex;
OS_MUT data_mutex;

/* =========================================================
   ------------------- SHARED DATA --------------------------
   ========================================================= */
volatile int      g_moisture = 0;
volatile uint8_t  g_temp     = 0;
volatile uint8_t  g_hum      = 0;
volatile uint32_t g_rpm      = 0;
volatile uint8_t  g_dht_ok   = 0;
volatile uint8_t  g_adc_ok   = 0;
volatile uint8_t  g_fan_on   = 0;

/* =========================================================
   ---------------------- UART0 -----------------------------
   ========================================================= */
void UART0_Init(void) {
    LPC_SC->PCONP |= (1 << 3);
    LPC_PINCON->PINSEL0 &= ~(0xF << 4);
    LPC_PINCON->PINSEL0 |=  (0x5 << 4);

    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 162;
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 0x03;
}

void uart_puts(const char *s) {
    os_mut_wait(&uart_mutex, 0xFFFF);
    while (*s) {
        while (!(LPC_UART0->LSR & (1 << 5)));
        LPC_UART0->THR = *s++;
    }
    os_mut_release(&uart_mutex);
}

/* =========================================================
   ---------------------- ADC -------------------------------
   ========================================================= */
#define ADC_DRY   3800
#define ADC_WET   3500
#define ADC_RANGE (ADC_DRY - ADC_WET)

void ADC_Init(void) {
    LPC_SC->PCONP |= (1 << 12);
    LPC_PINCON->PINSEL1 &= ~(3 << 14);
    LPC_PINCON->PINSEL1 |=  (1 << 14);
    LPC_ADC->ADCR = (1 << 0) | (5 << 8) | (1 << 21);
}

uint16_t ADC_Read(void) {
    LPC_ADC->ADCR |= (1 << 24);
    while (!(LPC_ADC->ADGDR & (1UL << 31)));
    return (LPC_ADC->ADGDR >> 4) & 0x0FFF;
}

/* =========================================================
   ---------------------- TIMER1 ----------------------------
   ========================================================= */
void Timer1_Init(void) {
    LPC_SC->PCONP |= (1 << 2);
    LPC_TIM1->PR  = (SystemCoreClock / 1000000) - 1;
    LPC_TIM1->TC  = 0;
    LPC_TIM1->TCR = 1;
}

/* =========================================================
   -------------------- HALL SENSOR -------------------------
   ========================================================= */
#define HALL_PIN (1 << 10)

void Hall_Init(void) {
    LPC_PINCON->PINSEL4 &= ~(3 << 20);
    LPC_PINCON->PINMODE4 &= ~(3 << 20);   // enable pull-up
    LPC_GPIO2->FIODIR  &= ~HALL_PIN;
}

__task void RPM_Task(void) {
    uint32_t last = 0, now;
    uint8_t prev = 0, curr;

    while (1) {
        curr = (LPC_GPIO2->FIOPIN & HALL_PIN) ? 1 : 0;

        if (curr && !prev) {
            now = LPC_TIM1->TC;
            if ((now > last) && ((now - last) > 1000)) {
                os_mut_wait(&data_mutex, 0xFFFF);
                g_rpm = 60000000UL / (now - last);
                os_mut_release(&data_mutex);
                last = now;
            }
        }
        prev = curr;
        os_dly_wait(1);
    }
}

/* =========================================================
   ---------------------- DHT11 -----------------------------
   ========================================================= */
#define DHT_PIN (1 << 16)
#define DHT_TIMEOUT 1000

static void delay_us(uint32_t us) {
    uint32_t start = LPC_TIM1->TC;
    while ((LPC_TIM1->TC - start) < us);
}

static int wait_for_level(uint32_t level, uint32_t timeout) {
    uint32_t start = LPC_TIM1->TC;
    while (((LPC_GPIO0->FIOPIN & DHT_PIN) ? 1 : 0) != level) {
        if ((LPC_TIM1->TC - start) > timeout) return 0;
    }
    return 1;
}

uint8_t DHT11_Read(uint8_t *h, uint8_t *t) {
    uint8_t d[5] = {0};
    int i, j;

    LPC_GPIO0->FIODIR |= DHT_PIN;
    LPC_GPIO0->FIOCLR = DHT_PIN;
    delay_us(18000);
    LPC_GPIO0->FIOSET = DHT_PIN;
    delay_us(30);
    LPC_GPIO0->FIODIR &= ~DHT_PIN;

    if (!wait_for_level(0, DHT_TIMEOUT)) return 0;
    if (!wait_for_level(1, DHT_TIMEOUT)) return 0;
    if (!wait_for_level(0, DHT_TIMEOUT)) return 0;

    tsk_lock();
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            if (!wait_for_level(1, DHT_TIMEOUT)) { tsk_unlock(); return 0; }
            delay_us(30);
            d[i] <<= 1;
            if (LPC_GPIO0->FIOPIN & DHT_PIN) d[i] |= 1;
            if (!wait_for_level(0, DHT_TIMEOUT)) { tsk_unlock(); return 0; }
        }
    }
    tsk_unlock();

    if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return 0;

    *h = d[0];
    *t = d[2];
    return 1;
}

__task void DHT_Task(void) {
    uint8_t h, t, ok;
    while (1) {
        ok = DHT11_Read(&h, &t);
        os_mut_wait(&data_mutex, 0xFFFF);
        g_dht_ok = ok;
        if (ok) { g_hum = h; g_temp = t; }
        os_mut_release(&data_mutex);
        os_dly_wait(100);
    }
}

/* =========================================================
   -------------------- STEPPER -----------------------------
   ========================================================= */
#define IN1 (1 << 4)
#define IN2 (1 << 5)
#define IN3 (1 << 6)
#define IN4 (1 << 7)

void Stepper_Init(void) {
    LPC_PINCON->PINSEL0 &= ~(0xFF << 8);
    LPC_GPIO0->FIODIR  |= IN1 | IN2 | IN3 | IN4;
}

static void step(uint32_t a, uint32_t b) {
    LPC_GPIO0->FIOCLR = IN1 | IN2 | IN3 | IN4;
    LPC_GPIO0->FIOSET = a | b;
    os_dly_wait(2);
}

__task void Stepper_Task(void) {
    while (1) {
        step(IN1, IN2);
        step(IN2, IN3);
        step(IN3, IN4);
        step(IN4, IN1);
        os_dly_wait(50);
    }
}

/* =========================================================
   -------------------- DC MOTOR ----------------------------
   ========================================================= */
#define DC_MOTOR_PIN (1 << 0)
#define MOISTURE_TSHLD 5

void DCMotor_Init(void) {
    LPC_GPIO2->FIODIR |= DC_MOTOR_PIN;
    LPC_GPIO2->FIOSET = DC_MOTOR_PIN;
}

__task void DCMotor_Task(void) {
    while (1) {
        os_mut_wait(&data_mutex, 0xFFFF);
        if (g_moisture > MOISTURE_TSHLD) {
            LPC_GPIO2->FIOCLR = DC_MOTOR_PIN;
            g_fan_on = 1;
        } else {
            LPC_GPIO2->FIOSET = DC_MOTOR_PIN;
            g_fan_on = 0;
        }
        os_mut_release(&data_mutex);
        os_dly_wait(50);
    }
}

/* =========================================================
   -------------------- ADC TASK ----------------------------
   ========================================================= */
__task void ADC_Task(void) {
    int adc, m;
    while (1) {
        adc = ADC_Read();
        os_mut_wait(&data_mutex, 0xFFFF);
        if (adc < ADC_WET || adc > ADC_DRY) {
            g_adc_ok = 0;
        } else {
            m = (adc - ADC_WET) * 100 / ADC_RANGE;
            if (m < 0) m = 0;
            if (m > 100) m = 100;
            g_moisture = m;
            g_adc_ok = 1;
        }
        os_mut_release(&data_mutex);
        os_dly_wait(20);
    }
}

/* =========================================================
   ---------------------- LCD -------------------------------
   ========================================================= */
void lcd_safe_init(void) {
    lcd_init();
    clr_disp();
}

__task void LCD_Task(void) {
    char buf[17];
    int m;
    uint8_t fan;

    while (1) {
        os_mut_wait(&data_mutex, 0xFFFF);
        m = g_moisture;
        fan = g_fan_on;
        os_mut_release(&data_mutex);

        os_mut_wait(&lcd_mutex, 0xFFFF);
        temp1 = 0x80; lcd_com();
        sprintf(buf, "GRAIN: %s ", (m < MOISTURE_TSHLD) ? "DRY" : "WET");
        lcd_puts((unsigned char *)buf);

        temp1 = 0xC0; lcd_com();
        sprintf(buf, "FAN: %s ", fan ? "ON" : "OFF");
        lcd_puts((unsigned char *)buf);
        os_mut_release(&lcd_mutex);

        os_dly_wait(100);
    }
}

/* =========================================================
   ---------------------- UART TASK -------------------------
   ========================================================= */
__task void UART_Task(void) {
    char buf[120];
    while (1) {
        os_mut_wait(&data_mutex, 0xFFFF);
        sprintf(buf,
            "Moisture : %d %%\r\n"
            "DHT11    : Temp=%u C  Hum=%u %%\r\n"
            "RPM      : %lu\r\n"
            "----------------------------\r\n",
            g_moisture, g_temp, g_hum, g_rpm);
        os_mut_release(&data_mutex);
        uart_puts(buf);
        os_dly_wait(100);
    }
}

/* =========================================================
   ---------------------- INIT TASK -------------------------
   ========================================================= */
__task void init_task(void) {
    os_mut_init(&uart_mutex);
    os_mut_init(&lcd_mutex);
    os_mut_init(&data_mutex);

    UART0_Init();
    ADC_Init();
    Timer1_Init();
    Hall_Init();
    Stepper_Init();
    DCMotor_Init();
    lcd_safe_init();

    os_tsk_create(ADC_Task,     3);
    os_tsk_create(DHT_Task,     2);
    os_tsk_create(RPM_Task,     2);
    os_tsk_create(DCMotor_Task, 2);
    os_tsk_create(Stepper_Task, 1);
    os_tsk_create(LCD_Task,     2);
    os_tsk_create(UART_Task,    1);

    os_tsk_delete_self();
}

/* =========================================================
   ------------------------- MAIN ---------------------------
   ========================================================= */
int main(void) {
    SystemInit();
    os_sys_init(init_task);
    while (1);
}
