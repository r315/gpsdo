#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "i2c.h"
#include "gd32e23x_gpio.h"
#include "pcf8574.h"
#include "stdinout.h"

#define FALSE           0
#define TRUE            1
#define OFF             FALSE
#define ON              TRUE

#define LED1_PIN_INIT  \
        gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);

#define LED1_OFF        gpio_bit_write(GPIOA, GPIO_PIN_8, SET)
#define LED1_ON         gpio_bit_write(GPIOA, GPIO_PIN_8, RESET)
#define LED1_TOGGLE     gpio_bit_toggle(GPIOA, GPIO_PIN_8)

#define LED_PPS_Pos     0
#define LED_PPS_MASK    (7 << LED_PPS_Pos)
#define LED_LOCK_Pos    3
#define LED_LOCK_MASK   (1 << LED_LOCK_Pos)
#define LED_ALM_Pos     4
#define LED_ALM_MASK    (1 << LED_ALM_Pos)

#define DAC_MAX_VAL     0x0FFF      // 12bit DAC

#define VCC5V0          4420          // OXCO supply
#define VDDA            3300          // ADC reference voltage in mv
#define VREF            2500          // Vref from LT1009
#define ADC_RES         4096          // 12-bit ADC resolution

#define THM_ADC_CH      ADC_CHANNEL_5
#define THM_R_FIXED     9840.0        // Pull-up resistor value in ohms
#define THM_BETA        3950.0        // Beta parameter of the NTC thermistor
#define THM_T0          298.15        // 25°C in Kelvin
#define THM_R0          10000.0       // Resistance at 25°C (10kΩ)

#define TSENSE_ADC_CH   ADC_CHANNEL_16
#define VREFINT_ADC_CH  ADC_CHANNEL_17

enum led_tag{
    LED_PPS = 0,
    LED_LOCK,
    LED_ALM
};

enum mco {
    MCO_PLL,
    MCO_OTHER
};

typedef struct boardcfg {
    uint32_t enabled;       // Bit mask with enabled features
    uint8_t output_clock;   // Selects 10M outout
    uint8_t pps_clock;      // Selects PPS output
    uint8_t sys_clk;        // Clock output select for PA8
    uint8_t rsvd;
    uint32_t gps_baud;
    #if ENABLE_IRC8_TRIM
    uint32_t irc8_trim;
    #endif
    uint16_t dac_duty;
    uint16_t vdda;
    uint16_t vcc5v0;
}boardcfg_t;

/* board low level */
void board_init(void);
#if ENABLE_IRC8_TRIM
int32_t board_trim_irc(int8_t adj);
#endif
void board_i2c_init(void);
uint16_t i2cWrite(uint8_t dev_addr, const uint8_t *data, uint16_t size);
uint16_t i2cRead(uint8_t dev_addr, uint8_t *data, uint16_t size);
void board_reset(void);

/* Time related functions */
void delayMs(uint32_t ms);
uint32_t elapsed_ms(uint32_t start_ms);
uint32_t millis(void);

/* System functions */
void __debugbreak(void);
void mco(uint8_t src);

/* Frequency counter */
void countStart(void(*cb)(uint32_t));
void countStop(void);
void tdc_start(void(*cb)(int32_t));
void tdc_stop(void);
void tdc_select(uint8_t input);
void div_start(uint32_t div);
void div_stop(void);

/* DAC */
void dacInit(void);
void dacValueSet(uint16_t value);
uint16_t dacValueGet(void);
uint32_t dacVoltageGet(void);

/* ADC */
void adc_init(void);
uint8_t adcConversionStart(uint8_t wait);
uint16_t adcConversionGet(uint8_t ch);
int32_t adcVoltageGet(uint8_t ch);

/* Temperature sensor */
float temp_get(void);

/* PPS Generation */
void ppsSelect(uint8_t input);

/* LED */
void ledSet(enum led_tag, uint8_t state);

/* Main ouput */
void output_select(uint8_t in);

/* EEPROM */
uint8_t settings_load(uint8_t *data, uint8_t len);
uint8_t settings_save(const uint8_t *data, uint8_t len);

/* Global varaiables */
extern stdinout_t cli_serial;
extern stdinout_t gps_serial;

#ifdef __cplusplus
}
#endif

#endif