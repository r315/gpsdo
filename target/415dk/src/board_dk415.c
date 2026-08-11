#include <stdio.h>
#include "at32f4xx.h"
#include "board.h"
#include "stdinout.h"
#include "debug.h"
#include "uart.h"
#include "flash.h"
#include "dma.h"
#include "i2c.h"
#include "pwm.h"
#include "clock.h"
#include "io_expander.h"
#include "io_expander/pcf8574.h"

#ifdef ENABLE_DEBUG_BOARD
#define DBG_TAG "BOARD : "
#define DBG_BOARD_INF(...) DBG_INF(DBG_TAG __VA_ARGS__)
#define DBG_BOARD_WRN(...) DBG_WRN(DBG_TAG __VA_ARGS__)
#define DBG_BOARD_ERR(...) DBG_ERR(DBG_TAG __VA_ARGS__)
#define DBG_BOARD_PRINT(...) DBG_PRINT(__VA_ARGS__)
#else
#define DBG_BOARD_INF(...)
#define DBG_BOARD_WRN(...)
#define DBG_BOARD_ERR(...)
#define DBG_BOARD_PRINT(...)
#endif

#define ADC_STATUS_RDY          (1 << 0)
#define ADC_STATUS_DIV          (1 << 1)
#define ADC_STATUS_CAL          (1 << 2)
#define ADC_STATUS_RES          (1 << 3)
#define ADC_CR2_EXSEL_SWSTART   (ADC_CTRL2_EXTREN | (7 << 17))
#define ADC_CR2_JEXSEL_JSWSTART (ADC_CTRL2_JEXTREN | (7 << 12))

#define TMR_SMC_TRGSEL_EXT      (7 << 4)
#define TMR_SMC_SMSEL_EXT       (7 << 0)
#define TMR_CCM1_C1C_C1IFP1     (1 << 0)
#define TMR_CCM1_C1C_C2IFP1     (2 << 0)
#define TMR_CCM1_C1C_STCI       (3 << 0)
#define TMR_CCM1_C2C_C1IFP1     (1 << 8)
#define TMR_CCM1_C2C_C2IFP1     (2 << 8)
#define TMR_CCM1_C2C_STCI       (3 << 8)


typedef struct adc {
    uint32_t calibration_code;
    float resolution;
} adc_t;

static uint32_t systicks;

extern uint32_t _seeprom, _eeeprom; // declared on linker script
static uint8_t eep_buf[EEPROM_SZ];

static nvdata_t dk415_nvdata = {
    .sector = {
        .start = (uint32_t)&_seeprom,
        .end = (uint32_t)&_eeeprom,
        .init = NULL,
        .read = FLASH_Read,
        .write = FLASH_Write,
        .erase = Flash_EraseSector},
    .nvb = {.size = 0, .next = NULL, .data = NULL}};

static serialbus_t uart1;
static serialbus_t uart2;

int uart1_available(void) { return UART_Available(&uart1); }
int uart1_read(char *buf, int len) { return UART_Read(&uart1, (uint8_t *)buf, len); }
int uart1_write(const char *buf, int len) { return UART_Write(&uart1, (const uint8_t *)buf, len); }
int uart2_available(void) { return UART_Available(&uart2); }
int uart2_read(char *buf, int len) { return UART_Read(&uart2, (uint8_t *)buf, len); }
int uart2_write(const char *buf, int len) { return UART_Write(&uart2, (const uint8_t *)buf, len); }

stdinout_t cli_serial = {
    .available = uart1_available,
    .read = uart1_read,
    .write = uart1_write};

stdinout_t gps_serial = {
    .available = uart2_available,
    .read = uart2_read,
    .write = uart2_write};

#ifdef ENABLE_SPI
static spibus_t hspi;
#endif

#ifdef ENABLE_COUNT
static uint32_t count_last_cap;
static void (*counterHandler)(uint32_t);
#endif

#ifdef ENABLE_I2C
static i2cbus_t i2cbus;
#endif

#ifdef ENABLE_ADC
static adc_t hadc;
#endif

#ifdef ENABLE_IOEXPANDER
static ioexp_t *ioexp = &pcf8574_ioexp;
#endif
/**
 * @brief Initialyze system time bases
 * */
static void systicksInit(void)
{
    /** 1ms time base using cortex-M systick
     * for software timers
     */
    systicks = 0;
    SysTick_Config(CLOCK_Get(CLOCK_CPU) / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);
}

uint32_t millis(void)
{
    return systicks;
}

void delayMs(uint32_t ms)
{
    uint32_t timeout = systicks + ms;
    while (systicks < timeout)
    {
    }
}

void mco(enum mco_src src, enum mco_div div)
{
    CLOCK_Output(div | src);
}

void ledSet(enum led_tag tag, uint8_t state)
{
#if defined(ENABLE_I2C) && defined(ENABLE_IOEXPANDER)
    switch (tag)
    {
    case LED_PPS:
        if (state)
            ioexp->clr(&i2cbus, (state << LED_PPS_Pos) & LED_PPS_MASK);
        else
            ioexp->set(&i2cbus, LED_PPS_MASK);
        break;

    case LED_LOCK:
        if (state)
            ioexp->clr(&i2cbus, (state << LED_LOCK_Pos) & LED_LOCK_MASK);
        else
            ioexp->set(&i2cbus, LED_LOCK_MASK);
        break;
    case LED_ALM:
        if (state)
            ioexp->clr(&i2cbus, (state << LED_ALM_Pos) & LED_ALM_MASK);
        else
            ioexp->set(&i2cbus, LED_ALM_MASK);
        break;
    case LED_RGB:
        ioexp->write(&i2cbus, state);
        break;
    default:
        break;
    }
#else
    (void)tag;
    (void)state;
#endif
}

/**
 * @brief Initializes eeprom data structures
 *
 * @param size  number of bytes requested by
 *              application for eeprom
 * @return      Pointer to eeprom data bytes
 */
uint8_t *eepromInit(uint16_t size)
{
    if (!dk415_nvdata.nvb.size)
    {
        dk415_nvdata.nvb.data = eep_buf;
        dk415_nvdata.nvb.size = NVDATA_BLK_SZ(size);

        if (dk415_nvdata.nvb.size > EEPROM_SZ)
        {
            dk415_nvdata.nvb.size = EEPROM_SZ;
            DBG_BOARD_WRN("eep_buf should be bigger");
        }

        NV_Init(&dk415_nvdata);
    }

    return dk415_nvdata.nvb.data;
}

uint32_t eepromWrite(uint16_t offset, const uint8_t *data, uint32_t count)
{
    return NV_Write(offset, data, count);
}

uint32_t eepromSave(void)
{
    return NV_Sync();
}

uint32_t eepromErase(void)
{
    return NV_Erase();
}

#ifdef ENABLE_I2C
uint16_t i2cWrite(uint8_t addr, const uint8_t *data, uint16_t size)
{
    return I2C_Write(&i2cbus, addr, data, size);
}

uint16_t i2cRead(uint8_t addr, uint8_t *data, uint16_t size)
{
    return I2C_Read(&i2cbus, addr, data, size);
}
#endif

#ifdef ENABLE_SPI
/**
 * @brief
 * @param data
 */
void spiWrite(uint8_t data)
{
    SPI_Xchg(&hspi, &data, 1);
}

/**
 * @brief
 * @param
 * @return
 */
uint8_t spiRead(void)
{
    uint8_t data = 0xFF;
    SPI_Xchg(&hspi, &data, 1);
    return data;
}
#endif

#ifdef ENABLE_COUNT
/**
 * @brief   Frequency counter, direct count implementation.
 *          Timer counts input signal on rising edges and
 *          an stable external 1Hz triggers capture of timer.
 *
 *          An interrupt is generated with capture event and
 *          count is given be the difference of captured value with last.
 *          finally count is passed to application
 *          trough a callback.
 *
 * @param
 * @return  1: on success
 */
uint32_t countInit(void)
{
    #ifdef BOARD_DK415
    GPIO_Config(PA_15, GPIO_REMAP_PA15_TMR2_EXT);
    GPIO_Config(PB_3, GPI_PD);
    #else
    GPIO_Config(PA_0, GPI_PD);
    GPIO_Config(PA_1, GPI_PD);
    #endif
    /**
     * Configure 32bit external clock counter
     * */
    CLOCK_Enable(TMR2, ENABLE);
    // Enable 32bit with plus mode
    TMR2->CTRL1 = TMR_CTRL1_PMEN;
    // Slave mode, external clock mode B
    TMR2->SMC = TMR_SMC_ECLKEN |
                TMR_SMC_SMSEL_EXT |
                TMR_SMC_TRGSEL_EXT;
    // Count 32bit
    TMR2->AR = 0xFFFFFFFF;
    // Start counting
    TMR2->CTRL1 |= TMR_CTRL1_CNTEN;

    /**
     * Configure capture mode with interrupt
     * */
    TMR2->CCM1 = TMR_CCM1_C2C_C1IFP1;
    // Generate interrupt on capture
    TMR2->DIE = TMR_DIE_C2IE;
    // Enable interrupt
    NVIC_EnableIRQ(TMR2_GLOBAL_IRQn);
    return 1;
}
/**
 * @brief
 * @param cb
 */
void countStart(void (*cb)(uint32_t))
{
    if(cb){
        counterHandler = cb;
        TMR2->CCE = TMR_CCE_C2EN;
    }
}

void countStop(void)
{
    TMR2->CCE = 0;
}
#endif

#ifdef ENABLE_PPS
/**
 * @brief PPS generation from external 32768kHz oscillator
 * @param
 * @return
 */
static uint32_t ppsInit(void)
{
    uint8_t retry = 10;
    CLOCK_Enable(PWR, ENABLE);
    // Enable write on ERTC
    PWR->CTRL |= PWR_CTRL_DBP;
    // Reset to ensure that is stable
    RCC->BDC |= RCC_BDC_BDRST;
    RCC->BDC &= ~RCC_BDC_BDRST;
    // Enable external low speed oscillator with bypass
    RCC->BDC |= RCC_BDC_LSEEN | RCC_BDC_LSEBYPS;

    do{
        delayMs(50);
        if(!(--retry)){
            return 0;
        }
    }while(!(RCC->BDC & RCC_BDC_LSESTBL));

    // Enable RTC and use LEXT
    RCC->BDC |= RCC_BDC_ERTCEN | RCC_BDC_ERTCSEL_LSE;

    /* Configure ERTC */

    /* disable the write protection */
    ERTC->WPR = 0xCA;
    ERTC->WPR = 0x53;

    ERTC->CTRL &= ~(ERTC_CTRL_CALOE | ERTC_CTRL_OSEL | ERTC_CTRL_OPOL | ERTC_CTRL_CALSEL);
    ERTC->CTRL |= ERTC_CTRL_CALOE | ERTC_CTRL_CALSEL;

    /* alarm output push pull */
    ERTC->TPAF |= ERTC_TPAF_ALAOUTTYPE;

    /* enable the write protection */
    ERTC->WPR = 0;

    // Enable 1Hz output on PC13
    GPIO_Config(PC_13, GPO_MS_AF);
    return 1;
}
#endif

#ifdef ENABLE_DAC
/**
 * @brief 1 Channel PWM for DAC
 *
 * @param
 */
static void dacInit(void)
{
    CLOCK_Enable(PWM_TIM, ENABLE);

    PWM_TIM->CTRL1 = 0;
    PWM_TIM->DIV = (CLOCK_Get(CLOCK_CLK4) / 1000000) - 1;   // 1MHz clock, counts units of 1us
    PWM_TIM->AR = DAC_MAX_VAL;

    PWM_TIM->CCM1 = (6 << 4);
    PWM_TIM->CCE = TMR_CCE_C1EN;

    PWM_TIM->CC1 = PWM_TIM->AR >> 2;    // 25%

    // Enable timer
    PWM_TIM->CTRL1 = TMR_CTRL1_CNTEN;
}

void dacEnable(uint8_t en)
{
    GPIO_Config(PA_7, en ? (GPIO_REMAP_PA7_TMR11_CH1 | GPO_HS_AF) : GPI_FLOAT);
}

uint16_t dacValueGet()
{
    return PWM_TIM->CC1;
}

void dacValueSet(uint16_t val)
{
    PWM_TIM->CC1 = val & DAC_MAX_VAL;
}
#endif

#ifdef ENABLE_ADC
/**
 * @brief Configure sample time for one adc channel
 *
 * */
static void adcSampleTime(uint16_t ch, uint16_t time)
{
    uint32_t shift;
    __IO uint32_t *smpt;

    if (ch > 17){
        return;
    }

    if (ch < 10){
        smpt = &ADC1->SMPT2;
    }else{
        ch = ch % 10;
        smpt = &ADC1->SMPT1;
    }

    shift = (3 * ch);
    *smpt = (*smpt & ~(7 << shift)) | (time << shift);
}

/**
 * @brief calibrate ADC and get resolution based
 * on 1.20V internal reference
 * */
static uint32_t adcCalibrate(void)
{
    ADC1->CTRL2 |= ADC_CTRL2_CAL; // Perform ADC calibration

    while (ADC1->CTRL2 & ADC_CTRL2_CAL)
        asm volatile("nop");
    // Get calibration code
    hadc.calibration_code = ADC1->RDOR;

    /**
     * Measure resolution by performing conversion
     * of internal reference using preemptive channels
     * */
    ADC1->JSQ = (ADC_CHANNEL_INTRV << 15);
    // Start conversion
    ADC1->CTRL2 |= ADC_CTRL2_JSWSTR;
    while (!(ADC1->STS & ADC_STS_JEC))
        asm volatile("nop");

    /**
     * Check if conversion is within range
     *
     * VINTRV 1.17V  1.20V  1.23V
     *
     * with VCCA = 3.3V
     * 1450 < ADC < 1530
     */
    uint32_t vref_raw = ADC1->JDOR1;
    if(1530 < vref_raw || 1450 > vref_raw){
        return 0;
    }

    // Compute resolution
    hadc.resolution = ADC_INTRV_VALUE / vref_raw;
    return 1;
}


/**
 * @brief   Configure ADC to read 4 channels
 *          The 4 channels are read with preemptive channels,
 *          whit this implementation each analog channel has his own
 *          data register and DMA is not necessary.
 * */
static uint32_t adcInit(void)
{
    GPIO_Config(PA_4, GPI_ANALOG);
    GPIO_Config(PA_5, GPI_ANALOG);
    GPIO_Config(PA_6, GPI_ANALOG);

    RCC->APB2EN |= RCC_APB2EN_ADC1EN; // Enable and reset ADC1
    RCC->APB2RST |= RCC_APB2EN_ADC1EN;
    RCC->APB2RST &= ~RCC_APB2EN_ADC1EN;
    /* Configure ADC for software trigger */
    ADC1->CTRL2 = ADC_CR2_JEXSEL_JSWSTART |
                  ADC_CTRL2_ADON;
    delayMs(20);
    /* Configure Sample time to maximum */
    adcSampleTime(ADC_SEQ_4, 7);
    adcSampleTime(ADC_SEQ_3, 7);
    adcSampleTime(ADC_SEQ_2, 7);
    adcSampleTime(ADC_SEQ_1, 7);
    // wake up Vrefint
    ADC1->CTRL2 |= ADC_CTRL2_TSREF;
    // Perform start up calibration
    if(!adcCalibrate()){
        return 0;
    }
    /* Configure channel sequence scan */
    ADC1->CTRL1 |= ADC_CTRL1_SCN;
    ADC1->JSQ = (ADC_SEQ_4 << 15) |
                (ADC_SEQ_3 << 10) |
                (ADC_SEQ_2 << 5) |
                (ADC_SEQ_1 << 0) |
                ((ADC_SEQ_LEN - 1) << 20);

    return 1;
}

/**
 * @brief   Conversion is started with software
 *          trigger and resolts are giben by JDORx
 *          registers.
 * */
uint8_t adcConversionStart(uint8_t wait)
{
    ADC1->STS = 0;
    ADC1->CTRL2 |= ADC_CTRL2_JSWSTR;

    if(wait){
        uint32_t timeout = 0xFFFF;
        do{
            if (--timeout == 0) {
                return 0;
            }
        }while ((ADC1->STS & ADC_STS_JEC) == RESET && timeout);
    }

    return 1;
}

/**
 * @brief
 * @param ch
 * @return
 */
uint16_t adcConversionGet(uint8_t ch)
{
    if(ADC1->STS & ADC_STS_JEC){
        switch(ch){
        case ADC_SEQ_1:
            return ADC1->JDOR1;
        case ADC_SEQ_2:
            return ADC1->JDOR2;
        case ADC_SEQ_3:
            return ADC1->JDOR3;
        case ADC_SEQ_4:
            return ADC1->JDOR4;
        default: break;
        }
    }

    return 0xFFFF;
}

/**
 * @brief
 * @param ch
 * @return
 */
int32_t adcVoltageGet(uint8_t ch)
{
    return adcConversionGet(ch) * hadc.resolution;
}
#endif

int main(void)
{
    // Enable gpio peripherals
    RCC->APB2EN |= RCC_APB2EN_GPIOAEN | RCC_APB2EN_GPIOBEN |
                   RCC_APB2EN_GPIOCEN | RCC_APB2EN_GPIOFEN | RCC_APB2EN_AFIOEN;
    // Configure basic gpio
    LED_INIT;
#ifdef ENABLE_DEBUG
    DBG_PIN_INIT;
#endif

    // Configure cpu and bus clock's
    SystemConfigClockSrc(RCC_CFG_SYSCLKSEL_HSI);
    uint32_t sclk = SystemConfigPll(RCC_CFG_PLLRC_HSE, HSE_VALUE, 150000000);

    if (sclk)
    {
        SystemConfigClockSrc(RCC_CFG_SYSCLKSEL_PLL);
        SystemCoreClockUpdate();
    }

    uart1.bus = UART_BUS0;
    uart1.speed = 115200;
    UART_Init(&uart1);
    uart1_write("\e[2J\r", 5);

#if defined(ENABLE_DEBUG)
    dbg_init(&cli_serial);
    DBG_BOARD_INF("Debug started");
#endif

    if(!sclk){
        DBG_BOARD_ERR("PLL Config fail");
    }

    systicksInit();
    DBG_BOARD_INF("Systick started");

#ifdef ENABLE_BUZZER
    tone_pwm_init_t tone_init;
    tone_init.ch = BUZ_TIM_CH;
    tone_init.pin = BUZ_PIN;
    tone_init.pin_idle = BUZ_PIN_IDLE;
    tone_init.tmr = BUZ_TIM;
    tone_init.volume = BUZ_DEFAULT_VOLUME;

    if (TONE_PwmInit(&tone_init) == TONE_IDLE) {
        DBG_BOARD_WRN("Buzzer Initialized");
    }
#endif

#ifdef ENABLE_SPI
    hspi.bus = SPI_BUS1;
    hspi.freq = 4000;
    hspi.cfg = SPI_MODE0;

    if (SPI_Init(&hspi) == SPI_OK){
        DBG_BOARD_INF("SPI Initialized");
    }else{
        DBG_BOARD_ERR("SPI fail");
    }
#endif

#ifdef ENABLE_DAC
    dacInit();
    dacEnable(1);
    DBG_BOARD_INF("DAC Initialized");
#endif

#ifdef ENABLE_ADC
    if(adcInit()){
        DBG_BOARD_INF("ADC Initialized");
    }else{
        DBG_BOARD_ERR("ADC Fail");
    }
#endif

#ifdef ENABLE_I2C
    i2cbus.addr = 0;
    i2cbus.speed = 100;
    i2cbus.bus_num = I2C_BUS3;
    i2cbus.cfg = I2C_CFG_DMA | I2C_CFG_PINS;

    if (I2C_Init(&i2cbus) == I2C_SUCCESS)
    {
        DBG_BOARD_INF("I2C Initialized");
    }
    else
    {
        DBG_BOARD_ERR("I2C fail");
    }
#endif

#ifdef ENABLE_PPS
    if(!ppsInit()){
        DBG_BOARD_ERR("Fail to start pps");
    }
#endif

#ifdef ENABLE_COUNT
    countInit();
    DBG_BOARD_INF("Fcount Initialized");
#endif

    /* Log reset reason
    // LPRSTF - bit7, Low power reset
    // WWDTRSTF - bit 6, window watchdog
    // WDTRSTF - bit 5, independent watchdog
    // SWRSTF - bit 4, Software reset
    // PORRSTF - bit 3, Power on reset
    // NRSTF - bit 2, nRST pin
    // RSTFC - bit 0, Reset flags clear
    */
    if (RCC->CTRLSTS)
    {
        uint8_t reset = RCC->CTRLSTS >> 24;
        RCC->CTRLSTS = RCC_CTRLSTS_RSTFC;
        DBG_BOARD_WRN("Reset reason: 0x%x ", reset);
    }

    extern void app(void);

    app();

    return 0;
}

/**
 * Interrupt handlers
 */

void SysTick_Handler(void)
{
    systicks++;
}

void TMR2_GLOBAL_IRQHandler(void)
{
    uint32_t cap = TMR2->CC2;
    TMR2->STS = 0;
    counterHandler(cap - count_last_cap);
    count_last_cap = cap;
}
