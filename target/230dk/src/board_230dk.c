#include <stdint.h>
#include "board.h"
#include "syscalls.h"
#include "uart.h"
#include "i2c.h"
#include "system_gd32e23x.h"

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

#define RCU_CMSIS       ((Rcu_Type *)RCU_BASE)
#define TMR2            ((Timer_Type*)TIMER2)
#define TMR14           ((Timer_Type*)TIMER14)


static void (*tim2_cb)(uint32_t);
static volatile uint32_t ticms;
static serialbus_t uartbus_a, uartbus_b;

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

void SysTick_Handler(void){
    ticms++;
    //LED1_TOGGLE;
}

void delay_ms(uint32_t ms){
    volatile uint32_t end = ticms + ms;
    while (ticms < end){ }
}

uint32_t ElapsedTicks(uint32_t start_ticks){
	int32_t delta = GetTick() - start_ticks;
    return (delta < 0) ? -delta : delta;
}

inline uint32_t GetTick(void)
{
    return ticms;
}

void board_init(void)
{
    i2cbus_t i2cbus = {
        .speed = 100000,
        .bus_num = I2C_BUS1
    };

    LED1_PIN_INIT;

    system_clock_config();

	SystemCoreClockUpdate();

	SysTick_Config(SystemCoreClock / 1000);

    uartbus_a.bus = UART_BUS0;
    uartbus_a.speed = 115200;

    uartbus_b.bus = UART_BUS1;
    uartbus_b.speed = 9600;

    UART_Init(&uartbus_b);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_2);

    I2C_Init(&i2cbus);

    redirect_stdout(&stdout_ops_serial);

    rcu_periph_clock_enable(RCU_GPIOB);
}

void SW_Reset(void)
{
    NVIC_SystemReset();
}

void __debugbreak(void){
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

void board_config_output(uint32_t frequency)
{
#if 0
    rcu_periph_clock_enable(RCU_TIMER2);
    uint32_t clock = rcu_clock_freq_get(CK_APB1);

    timer_deinit(TIMER2);
    timer_prescaler_config(TIMER2, (clock / 1000000UL) - 1, TIMER_PSC_RELOAD_UPDATE);
    timer_autoreload_value_config(TIMER2, frequency - 1);

    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, (frequency >> 1) - 1);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_state_config(TIMER2, TIMER_CH_1, TIMER_CCX_ENABLE);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_7);

    timer_enable(TIMER2);

    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);

    TMR5->ctrl1 = 0;
    TMR5->div = (clocks.apb1_freq / 500UL) - 1;
    TMR5->pr = 1000UL -1;
    TMR5->c1dt = 500 - 1;
    TMR5->cctrl_bit.c1en = 1;
    TMR5->cm1_output_bit.c1c = 0; // output
    TMR5->cm1_output_bit.c1octrl = 6; // PWM

    TMR5->ctrl1_bit.tmren = 1;

    GPIOA->cfglr = (GPIOA->cfglr & ~(0x0F << 0)) | (0x0A << 0); // PA0
#endif
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

void board_frequency_measurement_start(void)
{
    Timer_Type *tmr;

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
    // Configure PA2 for TIMER14_CH0
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_2);

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
}

void board_frequency_measurement_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_enable(RCU_TIMER14RST);
}

void board_frequency_measurement_cb(void(*cb)(uint32_t))
{
    tim2_cb = cb;
}

void TIMER2_IRQHandler(void)
{
    if(tim2_cb){
        tim2_cb(TMR14->CH0CV << 16 | TMR2->CH0CV);
    }
    TMR2->INTF = 0;
}