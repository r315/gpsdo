#include <stdint.h>
#include "board.h"
#include "syscalls.h"
#include "uart.h"
#include "i2c.h"
#include "system_gd32e23x.h"
#include "io_expander.h"

#define ON  1
#define OFF 0
#define RTC_BDCTL_RTCSRC_LXTAL      (1 << 8)
#define HXTAL_MAX_FREQ              32000000UL
#define HXTAL_MIN_FREQ              4000000UL
#define SYS_CLK_MAX_FREQ            80000000UL

enum clock_err{
    CLOCK_OK = 0,
    CLOCK_INVALID,
    CLOCK_NOT_STABLE,
    CLOCK_PLL_NOT_STABLE,
    CLOCK_SWITCH_FAIL
};

struct pll_cfg{
    uint32_t xtal;
    uint32_t clk;
    uint8_t mul;
    uint8_t div;
};

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
    volatile uint32_t CREP;
    volatile uint32_t CH0CV;
    volatile uint32_t CH1CV;
    volatile uint32_t CH2CV;
    volatile uint32_t CH3CV;
    volatile uint32_t CCHP;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSVD2[43];
    volatile uint32_t CFG;
}Timer_Advanced;

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
}Timer_TypeL0;

typedef struct {
    volatile uint32_t CTL0;     // 00
    volatile uint32_t RSV1[2];
    volatile uint32_t DMAINTEN; // 0C
    volatile uint32_t INTF;     // 10
    volatile uint32_t SWEVG;    // 14
    volatile uint32_t CHCTL0;   // 18
    volatile uint32_t RSV2;
    volatile uint32_t CHCTL2;   // 20
    volatile uint32_t CNT;      // 24
    volatile uint32_t PSC;      // 28
    volatile uint32_t CAR;      // 2C
    volatile uint32_t RSV3;
    volatile uint32_t CH0CV;    // 34
    volatile uint32_t RSV4[7];
    volatile uint32_t IRMP;     // 50
    volatile uint32_t RSV5[43];
    volatile uint32_t CFG;
}Timer_TypeL2;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t SMCFG;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t RSV1;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t CREP;
    volatile uint32_t CH0CV;
    volatile uint32_t CH1CV;
    volatile uint32_t RSV2[3];
    volatile uint32_t CCHP;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSVD2[43];
    volatile uint32_t CFG;
}Timer_TypeL3;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t RSV1;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t RSV2;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t CREP;
    volatile uint32_t CH0CV;
    volatile uint32_t RSV3[4];
    volatile uint32_t CCHP;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSV4[43];
    volatile uint32_t CFG;
}Timer_TypeL4;

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
#define TMR0            ((Timer_Advanced*)TIMER0)
#define TMR2            ((Timer_TypeL0*)TIMER2)
#define TMR5            ((TimerBasic_Type*)TIMER5)
#define TMR13           ((Timer_TypeL2*)TIMER13)
#define TMR14           ((Timer_TypeL3*)TIMER14)
#define TMR15           ((Timer_TypeL4*)TIMER15)
#define TMR16           ((Timer_TypeL4*)TIMER16)

static void (*tim2_cb)(uint32_t);
static void (*tim0_cb)(uint32_t);
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

    dac_init();
    adc_init();
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



/**
 * @brief finds the higiest sys_clk
 *
 *  sys_clk = xtal*pllmul/predv
 *
 *  predv [2,16]
 *  pllmul [2,32]
 *  xtal 4000000 to 32000000
 *
 * @param cfg struct pll_cfg
 * @return uint8_t
 */
void calculate_max_sys_clk(struct pll_cfg *cfg)
{
    cfg->clk = 0;
    cfg->div = 0;
    cfg->mul = 0;

    for (uint8_t pllmul = 2; pllmul <= 32; pllmul++) {
        uint32_t product = cfg->xtal * pllmul;
        // The optimal `predv` is the smallest value that maximizes sys_clk
        for (uint8_t predv = 2; predv <= 16; predv++) {
            uint32_t clk = product / predv;

            if(clk > SYS_CLK_MAX_FREQ){
                continue;
            }

            if (clk > cfg->clk) {
                cfg->clk = clk;
                cfg->mul = pllmul;
                cfg->div = predv;
            }

            // Since `predv` only increases the denominator, we can break early
            if (clk < cfg->clk) {
                break;
            }
        }
    }
}

/**
 * @brief Tries to configure the higest system clock
 * from a given xtal
 *

 * @param xtal
 * @return uint8_t
 */
uint8_t system_clock_from_xtal(uint32_t xtal)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    if (xtal > HXTAL_MAX_FREQ || HXTAL_MIN_FREQ > xtal){
        return CLOCK_INVALID;
    }

    /* Enable high speed crystal */
    RCU_CTL0 |= RCU_CTL0_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than timeout */
    do{
        timeout++;
        stab_flag = (RCU_CTL0 & RCU_CTL0_HXTALSTB);
    }
    while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail, return without changing clock */
    if(!(RCU_CTL0 & RCU_CTL0_HXTALSTB)){
        return CLOCK_NOT_STABLE;
    }

    /* HXTAL is stable */
    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT_2;

    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV1;

    struct pll_cfg cfg = {
        .xtal = xtal
    };

    calculate_max_sys_clk(&cfg);

    /* pll multiplier */
    RCU_CFG0 = (RCU_CFG0 & ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLDV)) |
               RCU_PLLSRC_HXTAL | CFG0_PLLMF(cfg.mul - 2) | ((cfg.mul > 16) ? RCU_CFG0_PLLMF4 : 0);

    /* pll prediv */
    RCU_CFG1 = (RCU_CFG1 & ~(RCU_CFG1_PREDV)) | CFG1_PREDV(cfg.div - 1);

    /* enable PLL */
    RCU_CTL0 |= RCU_CTL0_PLLEN;

    /* wait until PLL is stable */
    timeout = 0;
    do{
        timeout++;
        stab_flag = RCU_CTL0 & RCU_CTL0_PLLSTB;
    }while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    if(!(RCU_CTL0 & RCU_CTL0_PLLSTB)){
        return CLOCK_PLL_NOT_STABLE;
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
     timeout = 0;
    do{
        timeout++;
        stab_flag = RCU_CFG0 & RCU_CFG0_SCSS;
    }while((RCU_SCSS_PLL != stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    if(RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS)){
        return CLOCK_SWITCH_FAIL;
    }

    SystemCoreClock = cfg.clk;

    return CLOCK_OK;
}

void system_clock_output_enable()
{
    RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_CKOUTSEL) | RCU_CKOUTSRC_HXTAL;
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_2);
}

/**
 * @brief Overriding weak function
 * implemented in system_xxxx.c
 */
void system_clock_config(void)
{
    //system_clock_72m_irc8m();
    system_clock_from_xtal(HXTAL_VALUE);

    //SystemCoreClockUpdate();

    system_clock_output_enable();
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

void frequency_measurement_start(void(*cb)(uint32_t))
{
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

    TMR14->SMCFG =
        TIMER_SMCFG_SMC |           // Select external clock mode 0
        TIMER_SMCFG_TRGSEL_ITI1;    // ITI1 as clock source

    TMR14->CHCTL0 =
        (1 << 0);           // CH0 capture on CI0FE0
    TMR14->CHCTL2 =
        TIMER_CHCTL2_CH0EN; // Enable CH0

    TMR14->PSC = 0;           // No prescaller
    TMR14->CAR = 0xFFFF;      // Full count
    //TMR14->CNT = 0xF800;
    TMR14->CTL0 = TIMER_CTL0_CEN;

    TMR2->SMCFG =
        TIMER_SMCFG_SMC1 |  // Enable external clock mode
        TIMER_SMCFG_SMC |   // Select external clock mode 0
        TIMER_SMCFG_TRGS;   // ETIFP as clock source

    TMR2->CTL1 =
        (2 << 4);           // Select UPE as Trigger event

    TMR2->CHCTL0 =
        (1 << 0);           // CH0 capture on CI0FE0
    TMR2->CHCTL2 =
        TIMER_CHCTL2_CH0EN; // Enable CH0

    TMR2->PSC = 0;           // No prescaller
    TMR2->CAR = 0xFFFF;      // Full count
    TMR2->DMAINTEN =
        TIMER_DMAINTEN_CH0IE; // Interrut on capture

    NVIC_EnableIRQ(TIMER2_IRQn);
    TMR2->CTL0 = 1;

    // Configure elaborated PPS led
    rcu_periph_clock_enable(RCU_TIMER5);

    TMR5->CTL0 = TIMER_CTL0_SPM;
    TMR5->PSC = (rcu_clock_freq_get(CK_APB1) / 10000UL) - 1;
    TMR5->CAR = 1000;  // on for ~100ms
    TMR5->DMAINTEN =
        TIMER_DMAINTEN_UPIE; // Interrut on update event
    NVIC_EnableIRQ(TIMER5_IRQn);
}

void frequency_measurement_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_enable(RCU_TIMER14RST);
    rcu_periph_reset_enable(RCU_TIMER5RST);
}

void phase_measurement_start(void(*cb)(uint32_t))
{
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_reset_enable(RCU_TIMER0RST);
    rcu_periph_reset_disable(RCU_TIMER0RST);

    tim0_cb = cb;

    // Configure PA12 for TIMER0_ETI
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_12);

    // Configure PA11 for TIMER0_CH3
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_11);

    uint32_t tmr0_clk = rcu_clock_freq_get(CK_APB2);

    TMR0->PSC = tmr0_clk / 1000000UL;      // Count us
    TMR0->CAR = 5000;                      // 5ms max
    TMR0->SMCFG =
        TIMER_SLAVE_MODE_EVENT |           // Enable timer on event
        TIMER_SMCFG_TRGSEL_ETIFP;          // Trigger event on ETIFP (TIMER0_ETI pin)

    TMR0->CHCTL1 = (1 << 8);               // CH3 Input capture
    TMR0->CHCTL2 = TIMER_CHCTL2_CH3EN;     // Enable CH3

    TMR0->DMAINTEN = TIMER_DMAINTEN_CH3IE; // Enable interrupt

    TMR0->CTL0 = TIMER_CTL0_SPM;           // Single pulse

    NVIC_EnableIRQ(TIMER0_Channel_IRQn);
}

void phase_measurement_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER0RST);
}

void TIMER0_Channel_IRQHandler(void)
{
    if(tim0_cb){
        tim0_cb(TMR0->CH3CV);
    }
    TMR0->INTF = 0;
}

void TIMER2_IRQHandler(void)
{
    led_set(LED_PPS, ON);
    TMR5->CTL0 |= TIMER_CTL0_CEN;

    if(tim2_cb){
        tim2_cb(TMR14->CH0CV << 16 | TMR2->CH0CV);
    }
    TMR2->INTF = 0;
}

void TIMER5_IRQHandler(void)
{
    led_set(LED_PPS, OFF);
    TMR5->INTF = 0;
}

void dac_init(void)
{
    Timer_TypeL2 *tmr = TMR13;

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
    if(duty > DAC_MAX_VAL){
        return;
    }

    TMR13->CH0CV = duty;
}

uint16_t dac_duty_get(void)
{
    return TMR13->CH0CV;
}

uint32_t dac_voltage_get(void)
{
    //TODO read adc
    return 0;
}

void adc_init(void)
{
    rcu_periph_clock_enable(RCU_ADC);
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
     /* ADC channel length config */
    adc_channel_length_config(ADC_INSERTED_CHANNEL, 2);
    /* ADC temperature sensor channel config */
    adc_inserted_channel_config(0U, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5);
    /* ADC internal reference voltage channel config */
    adc_inserted_channel_config(1U, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_INSERTED_CHANNEL, ADC_EXTTRIG_INSERTED_NONE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC SCAN function enable */
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();
    adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
    /* enable ADC interface */
    adc_enable();
    delay_ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
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

void led_set(enum led_tag tag, uint8_t state)
{
    static uint8_t color = 1;

    switch(tag){
        case LED_PPS:
            if(state){
                IOEXP_Clr(&i2cbus, color & 7);
                color = (color == 7) ? 1 : color + 1;
            }else
                IOEXP_Set(&i2cbus, 7);
            break;

        case LED_LOCK:
        case LED_ALM:
        default:
            break;
    }
}

uint32_t vref_get(void)
{
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
    /* delay a time in milliseconds */
    delay_ms(10U);

    uint32_t vref_value = (ADC_IDATA1 * 3300 / 4096);
    return vref_value;
}

