#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f4xx.h"
#include "gpio_at32f4xx.h"
#include "tone_at32f4xx.h"
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

#define LED_PPS_Pos             0
#define LED_PPS_MASK            (7 << LED_PPS_Pos)
#define LED_LOCK_Pos            3
#define LED_LOCK_MASK           (1 << LED_LOCK_Pos)
#define LED_ALM_Pos             4
#define LED_ALM_MASK            (1 << LED_ALM_Pos)

/* This is not eeprom size, but it should be greater than requested eeprom size and be 4byte aligned */
#define EEPROM_SZ               128

#define board_reset             NVIC_SystemReset


enum led_tag{
    LED_PPS = 0,
    LED_LOCK,
    LED_ALM
};

enum mco {
    MCO_PLL,
    MCO_OTHER
};


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

void mco(enum mco src);
void led_set(enum led_tag tag, uint8_t state);
void count_start(void(*cb)(uint32_t));
void count_stop(void);

#ifdef ENABLE_PPS
uint8_t pps_init(void);
void pps_select(uint8_t input);
#endif

#ifdef ENABLE_TERMISTOR
#endif

#ifdef __cplusplus
}
#endif

#endif