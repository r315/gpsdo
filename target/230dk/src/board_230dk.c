#include <stdint.h>
#include <math.h>
#include "board.h"
#include "uart.h"
#include "i2c.h"
#include "system_gd32e23x.h"
#include "io_expander.h"
#include "gd32e23x_def.h"

#define HXTAL_MAX_FREQ              32000000UL
#define HXTAL_MIN_FREQ              4000000UL
#define SYS_CLK_MAX_FREQ            80000000UL
#define EDGE_DETECTOR_TIMEOUT       50000 /* ms */

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

struct adc_channel{
    uint8_t rank;
    uint8_t channel;
    uint32_t sample_time;
};

static void (*counter_cb)(uint32_t);
static void (*tdc_cb)(int32_t);
static volatile uint32_t ticms;
static serialbus_t uartbus_a, uartbus_b;
static i2cbus_t i2cbus;
static ioexp_t *ioexp = &pcf8574_ioexp;

int serial_a_available(void) { return UART_Available(&uartbus_a); }
int serial_a_read(char *buf, int len) { return UART_Read(&uartbus_a, buf, len); }
int serial_a_write(const char *buf, int len) { return UART_Write(&uartbus_a, buf, len); }

stdinout_t serial_a_ops = {
    .available = serial_a_available,
    .read = serial_a_read,
    .write = serial_a_write
};

int serial_b_available(void) { return UART_Available(&uartbus_b); }
int serial_b_read(char *buf, int len) { return UART_Read(&uartbus_b, buf, len); }
int serial_b_write(const char *buf, int len) { return UART_Write(&uartbus_b, buf, len); }

stdinout_t serial_b_ops = {
    .available = serial_b_available,
    .read = serial_b_read,
    .write = serial_b_write
};

static const struct adc_channel board_adc_channels[] ={
    {0U, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5}, // internal temperature sensor
    {1U, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5}, // 1.2V internal reference
    {2U, THM_ADC_CH, ADC_SAMPLETIME_239POINT5},     // external temperature sensor
};

/**
 * @brief finds the higiest possible system clock
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
static void system_clock_max(struct pll_cfg *cfg)
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
static uint8_t system_clock_from_xtal(uint32_t xtal)
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

    system_clock_max(&cfg);

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

/**
 * @brief Overriding weak function
 * implemented in system_xxxx.c
 */
void system_clock_config(void)
{
    system_clock_from_xtal(HXTAL_VALUE);

    system_clock_output(OFF);
}

void delay_ms(uint32_t ms)
{
    volatile uint32_t end = ticms + ms;
    while (ticms < end){ }
}

uint32_t elapsed_ms(uint32_t start_ms)
{
	int32_t delta = get_ms() - start_ms;
    return (delta < 0) ? -delta : delta;
}

inline uint32_t get_ms(void)
{
    return ticms;
}

/**
 * @brief
 * @param
 */
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

    UART_Init(&uartbus_a);

    uartbus_b.bus = UART_BUS1;
    uartbus_b.speed = 9600;

    UART_Init(&uartbus_b);

    i2cbus.speed = 100000;
    i2cbus.bus_num = I2C_BUS3;

    I2C_Init(&i2cbus);

    dac_init();
    adc_init();
}

void board_reset(void)
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
 * @brief Enables ouput of system clock to pin
 *
 * @param src   clock to be output
 */
void system_clock_output(uint8_t src)
{
    if(!src){
        RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_CKOUTSEL);
        gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
        return;
    }

    RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_CKOUTSEL) | (src << 24);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);
}

/**
 * @brief Get i2c bus instance for this board
 *
 * @param
 * @return
 */
i2cbus_t *board_i2c_get(void)
{
    return &i2cbus;
}

uint16_t board_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t size)
{
    return I2C_Write(&i2cbus, dev_addr, data, size);
}

uint16_t board_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t size)
{
    return I2C_Read(&i2cbus, dev_addr, data, size);
}

#if ENABLE_IRC8_TRIM
/**
 * @brief   Trim internal rc oscillator
 * @param   adj
 * @return
 */
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
#endif
/**
 * @brief Frequency measurement is archived by
 * cascading timer 2 and timer 14 to obtain a 32bit counter
 * and perform triggered captures.
 *
 * The counter counts cycles from input clock and
 * a second clock (1HZ) triggers the capture, which then
 * calls a callback with the captured value.
 * Main application calculates the difference between
 * captures to get the input frequency.
 *
 * @param cb callback for capture with captured value
 */
void counter_start(void(*cb)(uint32_t))
{
    counter_cb = cb;

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
    rcu_periph_reset_disable(RCU_TIMER5RST);

    TMR5->CTL0 = TIMER_CTL0_SPM;    // TODO: Check if Single pulse better
    TMR5->PSC = (rcu_clock_freq_get(CK_APB1) / 10000UL) - 1;
    TMR5->CAR = 1000;  // on for ~100ms
    TMR5->DMAINTEN =
        TIMER_DMAINTEN_UPIE; // Interrut on update event
    NVIC_EnableIRQ(TIMER5_IRQn);
}

void counter_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_enable(RCU_TIMER14RST);
    rcu_periph_reset_enable(RCU_TIMER5RST);
    NVIC_DisableIRQ(TIMER2_IRQn);
    NVIC_DisableIRQ(TIMER5_IRQn);
}

/**
 * @brief Time-to-digital converter, measures the time interval between two events
 * and converts that duration into a digital value.
 *
 * Due to timer limitations, this implementation cannot measure events that are more than 50ms apart.
 * After each measurement, a callback is called with the time in [us] between the two events.
 *
 * The implementation consists in a timer configured to operate in event mode triggered by ETI
 * and single pulse. Timer is started by a rising edge on ETI (First event) and stops at update event.
 * The second event triggers a capture at the rising edge of CHx and due to external DFF, the captured
 * value remains in CHxCV register.
 *
 * The callback is invoked on the update even handler and the value of CHxCV is passed as parameter.
 *
 * @param cb  Callback to be invoked with measured time in [us].
 */
void tdc_start(void(*cb)(int32_t))
{
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_reset_enable(RCU_TIMER0RST);
    rcu_periph_reset_disable(RCU_TIMER0RST);

    // Configure PA12 for TIMER0_ETI
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_12);

    // Configure PA11 for TIMER0_CH3
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_11);

    // Configure PA15 for DFF reset
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_bit_set(GPIOA, GPIO_PIN_15);

    tdc_cb = cb;

    TMR0->PSC = rcu_clock_freq_get(CK_APB2) / 1000000UL - 1;      // Count us
    TMR0->CAR = EDGE_DETECTOR_TIMEOUT;      // 50ms between resets
    TMR0->SMCFG =
        TIMER_SLAVE_MODE_EVENT |            // Enable timer on event
        TIMER_SMCFG_TRGSEL_ETIFP;           // Trigger event on ETIFP (TIMER0_ETI pin)

    TMR0->CHCTL1 = (1 << 8);                // CH3 Input capture
    TMR0->CHCTL2 = TIMER_CHCTL2_CH3EN;      // Enable CH3

    TMR0->DMAINTEN = TIMER_DMAINTEN_UPIE;
                     //TIMER_DMAINTEN_CH3IE;
    TMR0->CTL0 = TIMER_CTL0_SPM;            // Single pulse ensures that timer starts at rising edge of first clock

    //NVIC_EnableIRQ(TIMER0_Channel_IRQn);    // Interrupt for passing measured value
    NVIC_EnableIRQ(TIMER0_BRK_UP_TRG_COM_IRQn); // Interrupt for reseting external DFF's
}

void tdc_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER0RST);
    //NVIC_DisableIRQ(TIMER0_Channel_IRQn);
    NVIC_DisableIRQ(TIMER0_BRK_UP_TRG_COM_IRQn);
}

/**
 * @brief Divides input clock using TIMER2
 * and outputs resulting clock on PB0
 * @param div even division
 */
void div_start(uint32_t div)
{
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_disable(RCU_TIMER2RST);

    // Configure PB2 for TIMER2_ETI
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_2);
    // Configure PB0 for TIMER2_CH2
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_0);

    TMR2->SMCFG =
        TIMER_SMCFG_SMC1 |  // Enable external clock mode
        TIMER_SMCFG_SMC |   // Select external clock mode 0
        TIMER_SMCFG_TRGS;   // ETIFP as clock source

    TMR2->CHCTL1 =
        (0 << 0) |          // CH2 as output
        (6 << 4);           // PWM0 mode
    TMR2->CHCTL2 =
        TIMER_CHCTL2_CH2EN; // Enable CH2

    uint16_t rem = div >> 16;
    TMR2->PSC = rem;
    TMR2->CAR = (div / (rem + 1)) - 1;
    TMR2->CH2CV = TMR2->CAR >> 1;

    TMR2->CTL0 |= TIMER_CTL0_CEN;
}

void div_stop(void)
{
    rcu_periph_reset_enable(RCU_TIMER2RST);
    // Configure PB0 for TIMER2_CH2
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
}

/**
 * @brief Initialize PWM signal to be used in DAC
 *
 * @param
 */
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

/**
 * @brief VDAC Value, this value is applied as
 * duty of the PWM signal
 *
 * @param value
 */
void dac_value_set(uint16_t value)
{
    if(value > DAC_MAX_VAL){
        return;
    }

    TMR13->CH0CV = value;
}

/**
 * @brief Get current dac value (PWM duty)
 * @param
 * @return
 */
uint16_t dac_value_get(void)
{
    return TMR13->CH0CV;
}

/**
 * @brief Get DAC output voltage.
 * This should be DAC output read by ADC.
 *
 * @param
 * @return
 */
uint32_t dac_voltage_get(void)
{
    //TODO read adc
    return 0;
}

/**
 * @brief Configures inserted (preemptive) ADC channels
 * with software trigger
 *
 * @param channels      Channels
 * @param nchannels
 */
static void adc_ll_init(const struct adc_channel *channels, uint8_t nchannels)
{
    rcu_periph_clock_enable(RCU_ADC);
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);  // 18MHz clock

    /* ADC channel length config */
    adc_channel_length_config(ADC_INSERTED_CHANNEL, nchannels);

    for(int i = 0; i < nchannels; i++){
        /* ADC channel config */
        adc_inserted_channel_config(
            channels[i].rank,
            channels[i].channel,
            channels[i].sample_time
        );
    }

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_INSERTED_CHANNEL, ADC_EXTTRIG_INSERTED_NONE);
    adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC SCAN function enable */
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();
    /* enable ADC interface */
    adc_enable();
    delay_ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
}

void adc_init(void)
{
    adc_ll_init(board_adc_channels, 3);
}

/**
 * @brief Start ADC acquisition trggered by software
 *
 * @param wait 0: return from function, otherwise Wait for acquisition end.
 *
 * @return 1 if started or conversion ended, 0 on timeout while waiting
 */
uint8_t adc_acquire_start(uint8_t wait)
{
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
    /* wait for conversion */
    if(wait){
        uint32_t timeout = 0xFFFF;
        do{
            if (--timeout == 0) {
                return 0;
            }
        }while (adc_flag_get(ADC_FLAG_EOC) == RESET && timeout);
    }

    return 1;
}

/**
 * @brief Get raw data from previous acquisition
 *
 * @param ch ADC channel number 0-9
 * @return
 */
uint16_t adc_acquire_get(uint8_t ch)
{
    switch(ch){
        case THM_ADC_CH: return ADC_IDATA2;
        case ADC_CHANNEL_16: return ADC_IDATA0;
        case ADC_CHANNEL_17:return ADC_IDATA1;
        default: return 0;
    }
}

/**
 * @brief Get voltage on analog pin usind ADC
 *
 * @param ch ADC channel (0-9)
 * @return Voltage in mV
 */
uint32_t adc_voltage_get(uint8_t ch)
{
    return (adc_acquire_get(ch) * VDDA) / ADC_RES;
}
/**
 * @brief Obtain external temperature using adc and k type thermistor
 * @param
 * @return tmeprature in degre C
 */
float temp_get(void)
{
    int voltage = adc_acquire_get(THM_ADC_CH);

    if (voltage == 0)
        return -273.15f;

    // Calculate thermistor resistance
    float r_ntc = (THM_R_FIXED * voltage) / (VCC5V0 - voltage);

    // Apply Beta formula to get temperature in Kelvin
    float temp_k = 1.0f / ( (1.0f / THM_T0) + (1.0f / THM_BETA) * log(r_ntc / THM_R0) );

    // Convert Kelvin to Celsius
    return temp_k - 273.15f;
}

/**
 * @brief Generates PPS signal from external 32768Hz oscillator
 * using RTC peripheral.
 */
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
    RCU_BDCTL |= RCU_BDCTL_RTCEN | RCU_BDCTL_RTCSRC_LXTAL;
    // Enable 1Hz output on PC13
    rtc_alter_output_config(RTC_CALIBRATION_1HZ, RTC_ALARM_OUTPUT_PP);
    // Output mode
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    return 1;
}

void pps_select(uint8_t input)
{
    //TODO
}

void output_select(uint8_t input)
{
    //TODO
}

void led_set(enum led_tag tag, uint8_t state)
{
    switch(tag){
        case LED_PPS:
            if(state)
                ioexp->clr(&i2cbus, (state << LED_PPS_Pos) & LED_PPS_MASK);
            else
                ioexp->set(&i2cbus, LED_PPS_MASK);
            break;

        case LED_LOCK:
            if(state)
                ioexp->clr(&i2cbus, (state << LED_LOCK_Pos) & LED_LOCK_MASK);
            else
                ioexp->set(&i2cbus, LED_LOCK_MASK);
            break;
        case LED_ALM:
            if(state)
                ioexp->clr(&i2cbus, (state << LED_ALM_Pos) & LED_ALM_MASK);
            else
                ioexp->set(&i2cbus, LED_ALM_MASK);
            break;
        default:
            break;
    }
}

uint8_t settings_load(uint8_t *data, uint8_t len)
{
    // Read from eeprom
    return 0;
}

uint8_t settings_save(const uint8_t *data, uint8_t len)
{
    // Write to eeprom
    return 0;
}

/**
 * Interrupt handlers
 */
void SysTick_Handler(void)
{
    ticms++;
}

void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
    gpio_bit_reset(GPIOA, GPIO_PIN_15);

    if(tdc_cb){
        if(!TMR0->CH3CV){
            // No second event
            tdc_cb(gpio_input_bit_get(GPIOA, GPIO_PIN_11) ?
                EDGE_DETECTOR_TIMEOUT : -EDGE_DETECTOR_TIMEOUT);
        }else{
            tdc_cb(TMR0->CH3CV);
        }
    }

    TMR0->CH3CV = 0;
    TMR0->INTF = 0;

    gpio_bit_set(GPIOA, GPIO_PIN_15);
}

void TIMER0_Channel_IRQHandler(void)
{
    TMR0->INTF = 0;
}

void TIMER2_IRQHandler(void)
{
    if(counter_cb){
        counter_cb(TMR14->CH0CV << 16 | TMR2->CH0CV);
    }
    TMR2->INTF = 0;
    TMR5->CTL0 |= TIMER_CTL0_CEN;
}

void TIMER5_IRQHandler(void)
{
    led_set(LED_PPS, OFF);
    TMR5->INTF = 0;
}