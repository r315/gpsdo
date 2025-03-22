#include <stdint.h>
#include "board.h"
#include "syscalls.h"
#include "uart.h"
#include "i2c.h"
#include "system_gd32e23x.h"
#include "io_expander.h"


#define ON  1
#define OFF 0
#define RTC_BDCTL_RTCSRC_LXTAL    (1 << 8)

typedef struct {
    union{
        volatile uint32_t CTL0;
        struct {
            volatile uint32_t IRC8MEN : 1;
            volatile uint32_t IRC8MSTB : 1;
            volatile uint32_t rsvd0 : 1;
            volatile uint32_t IRC8MADJ : 5;
            volatile uint32_t IRC8MCALIB : 8;
            volatile uint32_t HXTALEN : 1;
            volatile uint32_t HXTALSTB : 1;
            volatile uint32_t HXTALBPS : 1;
            volatile uint32_t CLKMEN : 1;
            volatile uint32_t rsvd1 : 4;
            volatile uint32_t PLLEN : 1;
            volatile uint32_t PLLSTB : 1;
        };
    }CTL0_BIT;
    volatile uint32_t CFG0;
    volatile uint32_t INT;
    volatile uint32_t APB2RST;
    volatile uint32_t APB1RST;
    volatile uint32_t AHBEN;
    volatile uint32_t APB2EN;
    volatile uint32_t APB1EN;
    volatile uint32_t BDCTL;
    volatile uint32_t AHBRST;
    volatile uint32_t CFG1;
    volatile uint32_t CFG2;
    volatile uint32_t CTL1;
    uint32_t rsvd[0x100 - 0x34];
    volatile uint32_t VKEY;
    volatile uint32_t DSV;
}Rcu_Type;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t SMCFG;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t CHCTL1;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t RSV0;
    volatile uint32_t CH0CV;
    volatile uint32_t CH1CV;
    volatile uint32_t CH2CV;
    volatile uint32_t CH3CV;
    volatile uint32_t RSV1;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSVD2[43];
    volatile uint32_t CFG;
}Timer_Type;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t rsvd_1;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t rsvd_2[3];
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
}TimerBasic_Type;

#define RCU_CMSIS       ((Rcu_Type *)RCU_BASE)
#define TMR2            ((Timer_Type*)TIMER2)
#define TMR5            ((TimerBasic_Type*)TIMER5)
#define TMR13           ((Timer_Type*)TIMER13)
#define TMR14           ((Timer_Type*)TIMER14)
#define TMR16           ((Timer_Type*)TIMER16)


static void (*tim2_cb)(uint32_t);
static volatile uint32_t ticms;
static serialbus_t uartbus_a, uartbus_b;
static i2cbus_t i2cbus;

static void serial_a_init(void) { UART_Init(&uartbus_a); }
uint32_t serial_a_available(void) { return UART_Available(&uartbus_a); }
uint32_t serial_a_read(uint8_t *buf, uint32_t len) { return UART_Read(&uartbus_a, buf, len); }
uint32_t serial_a_write(const uint8_t *buf, uint32_t len) { return UART_Write(&uartbus_a, buf, len); }

static stdout_ops_t stdout_ops_serial = {
    .init = serial_a_init,
    .available = serial_a_available,
    .read = serial_a_read,
    .write = serial_a_write
};

uint32_t serial_b_available(void) { return UART_Available(&uartbus_b); }
uint32_t serial_b_read(uint8_t *buf, uint32_t len) { return UART_Read(&uartbus_b, buf, len); }
uint32_t serial_b_write(const uint8_t *buf, uint32_t len) { return UART_Write(&uartbus_b, buf, len); }

void delay_ms(uint32_t ms)
{
    volatile uint32_t end = ticms + ms;
    while (ticms < end){ }
}

uint32_t ElapsedTicks(uint32_t start_ticks)
{
	int32_t delta = GetTick() - start_ticks;
    return (delta < 0) ? -delta : delta;
}

inline uint32_t GetTick(void)
{
    return ticms;
}

void SysTick_Handler(void)
{
    ticms++;
}

void board_init(void)
{
    LED1_PIN_INIT;

    system_clock_config();

	SystemCoreClockUpdate();

	SysTick_Config(SystemCoreClock / 1000);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);

    uartbus_a.bus = UART_BUS0;
    uartbus_a.speed = 115200;

    uartbus_b.bus = UART_BUS1;
    uartbus_b.speed = 9600;

    UART_Init(&uartbus_b);

    i2cbus.speed = 100000;
    i2cbus.bus_num = I2C_BUS3;

    I2C_Init(&i2cbus);

    redirect_stdout(&stdout_ops_serial);
}

void SW_Reset(void)
{
    NVIC_SystemReset();
}

void __debugbreak(void)
{
    asm volatile
    (
        "bkpt #01 \n"
    );
}

void system_clock_config(void)
{
    system_clock_72m_irc8m();

    uint32_t timeout = 0xFFFF;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL0 |= RCU_CTL0_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do{
        timeout--;
        stab_flag = (RCU_CTL0 & RCU_CTL0_HXTALSTB);
    }while((0U == stab_flag) && timeout);

    /* if fail */
    if(0U == (RCU_CTL0 & RCU_CTL0_HXTALSTB)){
        return;
    }
    /* HXTAL is stable, enable output */

    RCU_CMSIS->CFG0 = (RCU_CMSIS->CFG0 & ~(7 << 24)) | (6 << 24);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_2);
}

i2cbus_t *board_i2c_get(void)
{
    return &i2cbus;
}

int32_t board_trim_irc(int8_t adj)
{
    uint16_t irc8mcal = RCU_CMSIS->CTL0_BIT.IRC8MCALIB;
    uint8_t irc8madj = RCU_CMSIS->CTL0_BIT.IRC8MADJ;

    if(adj > 0){
        irc8madj += adj;

        if(irc8madj > 31){
            if(irc8mcal < 256) {
                irc8mcal++;
                RCU_CMSIS->CTL0_BIT.IRC8MCALIB = irc8mcal;
                irc8madj = 16;
            }else{
                goto exit;
            }
            RCU_CMSIS->CTL0_BIT.IRC8MADJ = irc8madj;
            goto exit;
        }

        RCU_CMSIS->CTL0_BIT.IRC8MADJ = irc8madj;
    }

    if(adj < 0){
        irc8madj += adj;

        if(irc8madj < 0){
            if(irc8mcal > 0) {
                irc8mcal--;
                RCU_CMSIS->CTL0_BIT.IRC8MCALIB = irc8mcal;
                irc8madj = 16;
            }else{
                goto exit;
            }
            RCU_CMSIS->CTL0_BIT.IRC8MADJ = irc8madj;
            goto exit;
        }

        RCU_CMSIS->CTL0_BIT.IRC8MADJ = irc8madj;
    }

exit:
    return irc8mcal << 8 | irc8madj;
}

void board_frequency_measurement_start(void(*cb)(uint32_t))
{
    Timer_Type *tmr;
    tim2_cb = cb;

    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER14);
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_enable(RCU_TIMER14RST);
    rcu_periph_reset_disable(RCU_TIMER2RST);
    rcu_periph_reset_disable(RCU_TIMER14RST);

    // Configure PB2 for TIMER2_ETI
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_2);
    // Configure PA6 for TIMER2_CH0
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_6);
    // Configure PB14 for TIMER14_CH0
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_14);

    tmr = TMR14;

    tmr->SMCFG =
        TIMER_SMCFG_SMC |   // Select external clock mode 0
        (1 << 4);           // ITI1 as clock source

    tmr->CHCTL0 =
        (1 << 0);           // CH0 capture on CI0FE0
    tmr->CHCTL2 =
        TIMER_CHCTL2_CH0EN; // Enable CH0

    tmr->PSC = 0;           // No prescaller
    tmr->CAR = 0xFFFF;      // Full count
    tmr->CNT = 0xF800;
    tmr->CTL0 = 1;

    tmr = TMR2;

    tmr->SMCFG =
        TIMER_SMCFG_SMC1 |  // Enable external clock mode
        TIMER_SMCFG_SMC |   // Select external clock mode 0
        TIMER_SMCFG_TRGS;   // ETIFP as clock source

    tmr->CTL1 =
        (2 << 4);           // Select UPE as Trigger event

    tmr->CHCTL0 =
        (1 << 0);           // CH0 capture on CI0FE0
    tmr->CHCTL2 =
        TIMER_CHCTL2_CH0EN; // Enable CH0

    tmr->PSC = 0;           // No prescaller
    tmr->CAR = 0xFFFF;      // Full count
    tmr->DMAINTEN =
        TIMER_DMAINTEN_CH0IE; // Interrut on capture

    NVIC_EnableIRQ(TIMER2_IRQn);
    tmr->CTL0 = 1;

    // Configure elaborated PPS led
    rcu_periph_clock_enable(RCU_TIMER5);

    TMR5->CTL0 = TIMER_CTL0_SPM;
    TMR5->PSC = (rcu_clock_freq_get(CK_APB1) / 10000UL) - 1;
    TMR5->CAR = 1000;  // on for ~100ms
    TMR5->DMAINTEN =
        TIMER_DMAINTEN_UPIE; // Interrut on update event
    NVIC_EnableIRQ(TIMER5_IRQn);
}

void board_frequency_measurement_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_enable(RCU_TIMER14RST);
    rcu_periph_reset_enable(RCU_TIMER5RST);
}

void TIMER2_IRQHandler(void)
{
    pps_led_set(ON);
    TMR5->CTL0 |= TIMER_CTL0_CEN;

    if(tim2_cb){
        tim2_cb(TMR14->CH0CV << 16 | TMR2->CH0CV);
    }
    TMR2->INTF = 0;
}

void TIMER5_IRQHandler(void)
{
    pps_led_set(OFF);
    TMR5->INTF = 0;
}

void dac_init(void)
{
    Timer_Type *tmr = TMR13;

    rcu_periph_clock_enable(RCU_TIMER13);
    rcu_periph_reset_enable(RCU_TIMER13RST);
    rcu_periph_reset_disable(RCU_TIMER13RST);
    // Configure PA7 for TIMER13_CH0
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_7);

    tmr->PSC = 0;           // No prescaller
    tmr->CAR = DAC_MAX_VAL;
    tmr->CH0CV =  tmr->CAR >> 1; // 50%
    tmr->CHCTL0 =
        (6 << 4) |          // PWM0 mode
        (0 << 0);           // CH0 output mode
    tmr->CHCTL2 =
        TIMER_CHCTL2_CH0EN; // Enable CH0
    tmr->CTL0 = 1;          // Start Timer
}

void dac_duty_set(uint16_t duty)
{
    Timer_Type *tmr = TMR13;

    if(duty > DAC_MAX_VAL){
        return;
    }

    tmr->CH0CV = duty;
}

uint16_t dac_duty_get(void)
{
    return ((Timer_Type *)TMR13)->CH0CV;
}

uint32_t dac_voltage_get(void)
{
    //TODO read adc
    return 0;
}

uint8_t pps_init(void)
{
    uint8_t retry = 10;
    rcu_periph_clock_enable(RCU_RTC);
    rcu_periph_clock_enable(RCU_PMU);

    // Reset to ensure that is stable
    PMU_CTL |= PMU_CTL_BKPWEN;
    RCU_BDCTL |= RCU_BDCTL_BKPRST;
    RCU_BDCTL &= ~RCU_BDCTL_BKPRST;
    // Enable Low speed xtal
    RCU_BDCTL |= RCU_BDCTL_LXTALEN;

    do{
        delay_ms(50);
        if(!(--retry)){
            return 0;
        }
    }while(!(RCU_BDCTL & RCU_BDCTL_LXTALSTB));
    // Enable RTC and use LXTAL
    RCU_BDCTL |= RCU_BDCTL_RTCEN | RTC_BDCTL_RTCSRC_LXTAL;
    // Enable 1Hz output on PC13
    rtc_alter_output_config(RTC_CALIBRATION_1HZ, RTC_ALARM_OUTPUT_PP);

    return 1;
}

void pps_led_set(uint8_t state)
{
    static uint8_t color = 1;

    if(state){
        IOEXP_Clr(&i2cbus, color & 7);
        color = (color == 7) ? 1 : color + 1;
    }else
        IOEXP_Set(&i2cbus, 7);
}

