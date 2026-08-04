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

typedef struct adc
{
    volatile uint16_t status;
    uint16_t result[2];
    uint32_t calibration_code;
    float resolution;
    float vdiv_racio;
    float sense_resistor;
    uint32_t battery_voltage;
    uint32_t battery_current;
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

// static ioexp_t *ioexp = &pcf8574_ioexp;

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

#ifdef ENABLE_PWM
#ifdef ENABLE_DC_MOTOR
static pwmchip_t pwm2;
#endif
static void pwmOutInit(void);
#endif

#ifdef ENABLE_I2C
static i2cbus_t i2cbus;
#endif

#ifdef ENABLE_ADC
#define ADC_RDY (1 << 0)
#define ADC_DIV (1 << 1)
#define ADC_CAL (1 << 2)
#define ADC_RES (1 << 3)
#define ADC_CR2_EXTSEL_SWSTART (15 << 17)
#define ADC_SEQ_LEN 2

static adc_t hadc;
static dmatype_t adcdma;
static void adcInit(void);
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
    SysTick_Config(SystemCoreClock / 1000);
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

void mco(enum mco src)
{
    (void)src;
}

void led_set(enum led_tag tag, uint8_t state)
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
uint16_t board_i2c_write(uint8_t addr, const uint8_t *data, uint16_t size)
{
    return I2C_Write(&i2cbus, addr, data, size);
}

uint16_t board_i2c_read(uint8_t addr, uint8_t *data, uint16_t size)
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
void count_start(void (*cb)(uint32_t))
{
    (void)cb;
}

void count_stop(void)
{
}

#ifdef ENABLE_PWM_OUTPUT
/**
 * @brief 4 Channel PWM output generation
 *
 * @param
 */
static void pwmOutInit(void)
{
    // TODO: fix clock enable
    // RCC->APB1EN |= RCC_APB1EN_TMR4EN;
    // CLOCK_Enable((uint32_t)PWM_TIM, ENABLE);

    PWM_TIM->CTRL1 = TMR_CTRL1_DIR | TMR_CTRL1_ARPEN; // Down count and buffered ARR
    PWM_TIM->DIV = (SystemCoreClock / 1000000) - 1;   // 1MHz clock, counts units of 1us
#ifdef ENABLE_DC_MOTOR
    PWM_TIM->CCM1 = (7 << 12); // PWM mode 2 on channels 2 and 3
    PWM_TIM->CCM2 = (7 << 4);
    PWM_TIM->CCE = TMR_CCE_C2NEN | TMR_CCE_C3NEN | // Enable complementary outputs
                   TMR_CCE_C2NP | TMR_CCE_C3NP;    // Invert output
    PWM_TIM->CC2 = PWM_PULSE_W_DEFAULT;
    PWM_TIM->CC3 = PWM_PULSE_W_DEFAULT;
#else
    PWM_TIM->CCM1 = (7 << 12) | (7 << 4); // PWM mode 2 all channels
    PWM_TIM->CCM2 = (7 << 12) | (7 << 4);
    PWM_TIM->CCE = TMR_CCE_C1EN | TMR_CCE_C2EN | // Enable all channels
                   TMR_CCE_C3EN | TMR_CCE_C4EN |
                   TMR_CCE_C1P | TMR_CCE_C2P | // Invert them
                   TMR_CCE_C3P | TMR_CCE_C4P;
    PWM_TIM->CC1 = PWM_PULSE_W_DEFAULT;
    PWM_TIM->CC2 = PWM_PULSE_W_DEFAULT;
    PWM_TIM->CC3 = PWM_PULSE_W_DEFAULT;
    PWM_TIM->CC4 = PWM_PULSE_W_DEFAULT;
#endif
    // Force high state by having CNT > CCRx
    PWM_TIM->AR = PWM_PERIOD;
    PWM_TIM->CNT = PWM_PERIOD;
    // Enable timer
    PWM_TIM->CTRL1 = TMR_CTRL1_CNTEN;

    if (PWM_TIM == TMR1)
    {
        PWM_TIM->BRKDT = TMR_BRKDT_MOEN; // Main output enable
    }

#ifdef ENABLE_DC_MOTOR
    AFIO->MAP |= (1 << 6); // Remap TMR1 to PB0/1

    GPIO_Config(PB_0, GPO_LS_AF);
    GPIO_Config(PB_1, GPO_LS_AF);

    pwm2.chip = PWM_4;
    pwm2.frequency = PWM2_FREQUENCY;

    PWM_Init(&pwm2);

    PWM_Enable(&pwm2, 1, PWM_PIN_ON);
    PWM_Enable(&pwm2, 2, PWM_PIN_ON);
    PWM_Enable(&pwm2, 3, PWM_PIN_ON);
    PWM_Enable(&pwm2, 4, PWM_PIN_ON);
#endif

    GPIO_Config(PB_6, GPO_LS_AF);
    GPIO_Config(PB_7, GPO_LS_AF);
    GPIO_Config(PB_8, GPO_LS_AF);
    GPIO_Config(PB_9, GPO_LS_AF);
}

void pwmOut(const uint16_t *data, uint8_t nch)
{
    for (int i = 0; i < nch; i++)
    {
        pwmChannelSet(data[i], i);
    }
}

void pwmChannelSet(uint16_t us, uint8_t ch)
{
    if (ch >= PWM_NCH || us < PWM_PULSE_W_MIN || us > PWM_PULSE_W_MAX)
    {
        return;
    }

#ifdef ENABLE_DC_MOTOR
    if (ch < 2)
    {
        // CH 1 & 2
        if (us > 1450 && us < 1550)
        {
            dcMotorBreak(ch);
        }
        else
        {
            /**
             * % = (x - min) * 100 / (max - min)
             */
            uint8_t percent, dir;
            if (us < 1550)
            {
                percent = ((1450 - us) * 100) / 400;
                dir = 0;
            }
            else
            {
                percent = ((us - 1550) * 100) / 400;
                dir = 1;
            }

            dcMotorMove(ch, dir, percent);
        }
        return;
    }

    ch -= 1; // CC2/3
#endif

    ((uint32_t *)&PWM_TIM->CC1)[ch] = us;
}

void pwmFrequencySet(uint32_t freq)
{
#ifdef ENABLE_DC_MOTOR
    PWM_FrequencySet(&pwm2, freq);
#else
    (void)freq;
#endif
}
#endif

#ifdef ENABLE_ADC
/**
 * @brief Configure sample time for one adc channel
 *
 * */
static void adcSampleTime(uint16_t ch, uint16_t time)
{
    if (ch > 17)
    { // Max 17 channels
        return;
    }

    if (ch < 10)
    {
        ADC1->SMPT2 = (ADC1->SMPT2 & (7 << (3 * ch))) | (time << (3 * ch)); // Sample time for channels AN9-0
    }
    else
    {
        ADC1->SMPT1 = (ADC1->SMPT1 & (7 << (3 * (ch % 10)))) | (time << (3 * (ch % 10))); // Sample time for channels AN17-10
    }
}

/**
 * @brief calibrate ADC and get resolution based
 * on 1.20V internal reference
 * */
uint32_t adcCalibrate(void)
{
    uint32_t bsqr3;

    ADC1->CTRL2 |= ADC_CTRL2_CAL; // Perform ADC calibration

    while (ADC1->CTRL2 & ADC_CTRL2_CAL)
    {
        asm volatile("nop");
    }

    hadc.calibration_code = ADC1->RDOR;
    // Set calibration flag
    hadc.status |= ADC_CAL;
    // select VREFINT channel for first conversion
    bsqr3 = ADC1->RSQ3;
    ADC1->RSQ3 = (HW_VREFINT_CHANNEL << 0);
    // wake up Vrefint
    ADC1->CTRL2 |= ADC_CTRL2_TSREF;
    DelayMs(5);
    // Start conversion
    ADC1->CTRL2 |= ADC_CTRL2_SWSTR;
    while (!(ADC1->STS & ADC_STS_EC))
    {
        asm volatile("nop");
    }
    // Compute resolution
    hadc.resolution = VREFINT_VALUE / ADC1->RDOR;
    // power down VREFINT
    ADC1->CTRL2 &= ~ADC_CTRL2_TSREF;
    ADC1->RSQ3 = bsqr3;
    // Set resolution flag
    hadc.status |= ADC_RES;
    return 1;
}

/**
 * @brief Set voltage divider racio, used to measure battery voltage
 *
 * @param r : Racio = R2/(R1+R2)
 * */
void adcSetVdivRacio(float r)
{
    hadc.vdiv_racio = r;
    hadc.status |= ADC_DIV;
}

/**
 * @brief Set current sense resistor value for current calculation
 *
 * @param rs : Sense resistor value in ohms
 * */
void adcSetSenseResistor(float rs)
{
    hadc.sense_resistor = rs * ISENSE_GAIN;
}

/**
 * @brief get current voltage divider racio, used to measure battery voltage
 *
 * @return : R2/(R1+R2)
 * */
float adcGetVdivRacio(void)
{
    return hadc.vdiv_racio;
}

/**
 * @brief Get current sense resistor value for current calculation
 *
 * @return : Sense resistor value in ohms
 * */
float adcGetSenseResistor(void)
{
    return hadc.sense_resistor / ISENSE_GAIN;
}

static void adcEocHandler(void)
{
    DMA_Cancel(&adcdma);
    hadc.battery_voltage = (float)(hadc.result[0] * hadc.resolution) / hadc.vdiv_racio;
    hadc.battery_current = (hadc.result[1] * hadc.resolution) / hadc.sense_resistor;
    hadc.status |= ADC_RDY;
}

/**
 * @brief Configure ADC for a HW_VBAT_CHANNEL channel in interrupt mode and initiates a convertion.
 *  After convertion the result is stored locally through the interrupt
 *
 * PA0/AN0 is the default channel
 * */
static void adcInit(void)
{
    HW_VBAT_CH_INIT;

    RCC->APB2EN |= RCC_APB2EN_ADC1EN; // Enable and reset ADC1
    RCC->APB2RST |= RCC_APB2EN_ADC1EN;
    RCC->APB2RST &= ~RCC_APB2EN_ADC1EN;

    ADC1->CTRL2 = ADC_CR2_EXTSEL_SWSTART | // Select software trigger,
                  ADC_CTRL2_ADON;          // Enable ADC
    DelayMs(20);
    // Configure Sample time for the used channles
    adcSampleTime(HW_VBAT_CHANNEL, 6);    // Sample time, 6 => 71.5 cycles.
    adcSampleTime(HW_ISENSE_CHANNEL, 6);  // Sample time, 6 => 71.5 cycles.
    adcSampleTime(HW_VREFINT_CHANNEL, 3); // Sample time 3 => 28.5 cycles.
    // Perform start up calibration
    adcCalibrate();
    // Configure channels to be converted and enable scan mode
    ADC1->RSQ3 = (HW_ISENSE_CHANNEL << 5) | (HW_VBAT_CHANNEL << 0);
    ADC1->RSQ1 = (1 << 20); // Two channels on sequence
    ADC1->CTRL1 |= ADC_CTRL1_SCN;
    // Configure DMA
    adcdma.dir = DMA_DIR_P2M;
    adcdma.ssize = DMA_DATA_SIZE_16;
    adcdma.dsize = DMA_DATA_SIZE_16;
    adcdma.src = (void *)&ADC1->RDOR;
    adcdma.dst = (void *)&hadc.result[0];
    adcdma.eot = adcEocHandler;
    adcdma.len = ADC_SEQ_LEN;

    DMA_Config(&adcdma, DMA1_REQ_ADC1);

    ADC1->CTRL2 |= ADC_CTRL2_DMAEN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    hadc.status = 0;
}

/**
 * @brief
 * */
static void adcStartConversion(void)
{
    DMA_Start(&adcdma);
    hadc.status &= ~ADC_RDY;
    ADC1->CTRL2 |= ADC_CTRL2_SWSTR;
}

/**
 * @brief Get current adc resolution (mV/step)
 * */
float adcGetResolution(void)
{
    return hadc.resolution;
}
#endif

int main(void)
{
    // Enable gpio peripherals
    RCC->APB2EN |= RCC_APB2EN_GPIOAEN | RCC_APB2EN_GPIOBEN |
                   RCC_APB2EN_GPIOCEN | RCC_APB2EN_AFIOEN;
    // Configure basic gpio
    LED_INIT;
#ifdef ENABLE_DEBUG
    DBG_PIN_INIT;
#endif

    // Configure cpu and bus clock's
    // SystemInit();

    SystemConfigClockSrc(0);
    if (SystemConfigPll(1, HSE_VALUE, 150000000))
    {
        CLOCK_Output(0b0110, 0b1001, 1);
        SystemConfigClockSrc(2);
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

    systicksInit();
    DBG_BOARD_INF("Systick started");

    // ptInit(&pt1);
    // DBG_BOARD_INF("Main tick timer started");

#ifdef ENABLE_BUZZER
    tone_pwm_init_t tone_init;
    tone_init.ch = BUZ_TIM_CH;
    tone_init.pin = BUZ_PIN;
    tone_init.pin_idle = BUZ_PIN_IDLE;
    tone_init.tmr = BUZ_TIM;
    tone_init.volume = BUZ_DEFAULT_VOLUME;

    if (TONE_PwmInit(&tone_init) == TONE_IDLE)
    {
        DBG_BOARD_WRN("Buzzer Initialized");
    }
#endif

#ifdef ENABLE_SPI
    hspi.bus = CC25_SPU_BUS;
    hspi.freq = 4000;
    hspi.cfg = SPI_MODE0;

    if (SPI_Init(&hspi) == SPI_OK)
    {
        SPI_PINS_INIT;
        DBG_BOARD_INF("SPI Initialized");
    }
    else
    {
        DBG_BOARD_ERR("SPI failed");
    }
#endif

#ifdef ENABLE_PWM_OUTPUT
    pwmOutInit();
    DBG_BOARD_INF("PWM output started");
#endif

#ifdef ENABLE_SBUS_OUTPUT
    sbusInit();
    DBG_BOARD_INF("SBUS initialized");
#endif

#ifdef ENABLE_AUX_ENCODER
    auxEncoderInit();
#endif

#ifdef ENABLE_BATTERY_MONITOR
    adcInit();
#endif

#ifdef ENABLE_I2C
    i2cbus.addr = 0;
    i2cbus.speed = 100;
    i2cbus.bus_num = I2C_BUS3;
    i2cbus.cfg = I2C_CFG_DMA | I2C_CFG_PINS;

    if (I2C_Init(&i2cbus) == I2C_SUCCESS)
    {
        GPIO_Config(PB_10, GPO_MS_AF_OD);
        GPIO_Config(PB_11, GPO_MS_AF_OD);
        DBG_BOARD_INF("I2C Init ok");
    }
    else
    {
        DBG_BOARD_ERR("I2C Init fail");
    }
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
