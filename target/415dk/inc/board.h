#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f4xx.h"
#include "gpio_at32f4xx.h"
#include "tone.h"
#include "dma_at32f4xx.h"
#include "spi.h"
#include "gpio.h"
#include "stdinout.h"
#include "tone.h"
#include "nvdata.h"

/* General symbols */
#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#define GPO_SET(_IO, _PIN)      _IO->BSRE = (1 << _PIN)
#define GPO_CLR(_IO, _PIN)      _IO->BRE = (1 << _PIN)
#define GPO_TOGGLE(_IO, _PIN)   _IO->ODR ^= (1<<_PIN)

#define RAM_CODE                __attribute__((section(".ram_code")))

#define FALSE                   0
#define TRUE                    1
#define OFF                     FALSE
#define ON                      TRUE

#define LED_PIN                 PA_8
#define LED_INIT                GPIO_Config(LED_PIN, GPO_LS);
#define LED_OFF                 GPIO_Write(LED_PIN, GPIO_PIN_HIGH)
#define LED_ON                  GPIO_Write(LED_PIN, GPIO_PIN_LOW)
#define LED_TOGGLE              GPIO_Toggle(LED_PIN)

#define DBG_PIN                 PC_15
#define DBG_PIN_INIT            GPIO_Config(DBG_PIN, GPO_LS)
#define DBG_PIN_HIGH            GPIO_Write(DBG_PIN, GPIO_PIN_HIGH)
#define DBG_PIN_LOW             GPIO_Write(DBG_PIN, GPIO_PIN_LOW)
#define DBG_PIN_TOGGLE          GPIO_Toggle(DBG_PIN)

/* IO Expander */
#define LED_PPS_B_Pos           2
#define LED_PPS_B_MASK          (1 << LED_PPS_B_Pos)
#define LED_PPS_G_Pos           3
#define LED_PPS_G_MASK          (1 << LED_PPS_G_Pos)
#define LED_PPS_R_Pos           4
#define LED_PPS_R_MASK          (1 << LED_PPS_R_Pos)
#define LED_LOCK_Pos            1
#define LED_LOCK_MASK           (1 << LED_LOCK_Pos)
#define LED_ALM_Pos             0
#define LED_ALM_MASK            (1 << LED_ALM_Pos)
#define LED_ALL_MASK            (0x1F)
#define LED_PPS_RGB_MASK        (0x1C)

/* This is not eeprom size, but it should be greater than requested eeprom size and be 4byte aligned */
#define EEPROM_BUF_SIZE         128

#define board_reset             NVIC_SystemReset

#define DAC_MAX_VAL             0x0FFF      // 12bit DAC

#define ADC_CHANNEL_4           4
#define ADC_CHANNEL_5           5
#define ADC_CHANNEL_6           6
#define ADC_CHANNEL_16          16
#define ADC_CHANNEL_17          17
#define ADC_CHANNEL_TS          ADC_CHANNEL_16
#define ADC_CHANNEL_INTRV       ADC_CHANNEL_17
#define ADC_CHANNEL_TS_EXT      ADC_CHANNEL_5
#define ADC_CHANNEL_OXCO_VCC    ADC_CHANNEL_4
#define ADC_CHANNEL_DAC_FB      ADC_CHANNEL_6
#define ADC_INTRV_VALUE         1200.0f
#define ADC_RESOLUTION          4096          // 12-bit ADC resolution
#define ADC_VDDA                3300          // ADC reference voltage in mv

#define THM_R_FIXED             9840.0        // Pull-up resistor value in ohms
#define THM_BETA                3950.0        // Beta parameter of the NTC thermistor
#define THM_T0                  298.15        // 25°C in Kelvin
#define THM_R0                  10000.0       // Resistance at 25°C (10kΩ)

#define ADC_SEQ_1               ADC_CHANNEL_TS_EXT
#define ADC_SEQ_2               ADC_CHANNEL_DAC_FB
#define ADC_SEQ_3               ADC_CHANNEL_OXCO_VCC
#define ADC_SEQ_4               ADC_CHANNEL_INTRV
#define ADC_SEQ_LEN             4

/* IO Pins */
#define CLK_SEL_PIN             PB_5    // Route SOC CLK or LO ro CLK output
#define CNT_SEL_PIN             PB_4    // Route CLK IN or CLKO to counter gate
#define PPS_SEL_PIN             PA_1    // Route SOC PPS or GPS PPS ro PPS output
#define GPS_EN_PIN              PB_9    // Enable GPS module
#define GPS_OUT_EN_PIN          PC_13   // Enable GPS PPS output to Ext clock input

enum led_tag{
    LED_LOCK = 0,
    LED_ALM,
    LED_PPS_RGB,
    LED_PPS_R,
    LED_PPS_G,
    LED_PPS_B,
    LED_ALL
};

enum mco_src {
    MCO_NONE = 0,
    MCO_LICK = 2,
    MCO_LEXT,
    MCO_SCLK,
    MCO_HICK,
    MCO_HEXT,
    MCO_PLL_2,
    MCO_PLL_4 = 12,
    MCO_USB,
    MCO_ADC
};

enum mco_div {
    MCO_DIV_1   = 0,
    MCO_DIV_2   = (8 << 4),
    MCO_DIV_4   = (9 << 4),
    MCO_DIV_8   = (10 << 4),
    MCO_DIV_16  = (11 << 4),
    MCO_DIV_64  = (12 << 4),
    MCO_DIV_128 = (13 << 4),
    MCO_DIV_256 = (14 << 4),
    MCO_DIV_512 = (15 << 4)
};

enum pps_sel {
    PPS_SEL_SOC = 0,
    PPS_SEL_GPS,
    PPS_GPS_OUT_EN,
    PPS_CPS_OUT_DIS
};

enum clk_sel {
    CLK_SEL_SOC = 0,
    CLK_SEL_LO
};

enum gate_sel {
    GATE_SEL_CLKO,
    GATE_SEL_EXT
};

enum count_sel {
    COUNT_SEL_OUT = 0,
    COUNT_SEL_EXT
};

enum count_cfg {
    COUNT_CFG_RECIPROCAL = 0,
    COUNT_CFG_DIRECT
};

typedef float countres_t;

/* Global varaiables */
extern stdinout_t cli_serial;
extern stdinout_t gps_serial;

/* Exported functions */
void delayMs(uint32_t ms);
uint32_t millis(void);
/* EEPROM API */
uint8_t* eepromInit(uint16_t size);
uint32_t eepromWrite(uint16_t offset, const uint8_t *data, uint32_t count);
uint32_t eepromSave(void);
uint32_t eepromErase(void);

void mco(enum mco_src src, enum mco_div div);
void ledSet(enum led_tag tag, uint8_t state);
enum clk_sel clockSelectGet(void);

#ifdef ENABLE_COUNT
void countEocSet(void(*cb)(countres_t));
void countGateTimeSet(uint32_t ms);
void countStart(void);
void countStop(void);
uint32_t countConfig(enum count_cfg cfg);
void countGateSelect(enum gate_sel sel);
#endif

uint16_t i2cWrite(uint8_t dev_addr, const uint8_t *data, uint16_t size);
uint16_t i2cRead(uint8_t dev_addr, uint8_t *data, uint16_t size);

#ifdef ENABLE_DAC
uint16_t dacValueGet(void);
void dacValueSet(uint16_t);
void dacEnable(uint8_t en);
#endif

#ifdef ENABLE_ADC
uint8_t adcConversionStart(uint8_t wait);
uint16_t adcConversionGet(uint8_t ch);
int32_t adcVoltageGet(uint8_t ch);
#endif

#ifdef ENABLE_PPS
void ppsSelect(enum pps_sel sel);
void ppsSync(void);
void ppsPulseSet(uint16_t unit);
void ppsInt(void (*cb)(void));
enum pps_sel ppsSelectGet(void);
uint8_t ppsGpsOutEnGet(void);
#endif

#ifdef ENABLE_GPS
void gpsEnableSet(uint8_t en);
#endif

void clockSelect(enum clk_sel sel);
void clockFrequencySet(uint32_t freq);

#ifdef ENABLE_TERMISTOR
#endif

#ifdef __cplusplus
}
#endif

#endif