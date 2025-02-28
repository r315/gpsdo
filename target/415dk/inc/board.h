#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f4xx.h"
#include "gpio_at32f4xx.h"
#include "gpio.h"

#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#ifndef BOARD_415DK
#define BOARD_415DK
#endif

#define LED1_PIN_INIT   GPIOA->CTRLH = (GPIOA->CTRLH & ~0x000F) | 0x0002

#define LED1_OFF        GPIOA->BSRE = (1 << 8)
#define LED1_ON         GPIOA->BRE  = (1 << 8)
#define LED1_TOGGLE     GPIOA->ODT = GPIOA->IDT ^ (1 << 8)

#define DBG_PIN_INIT    LED1_PIN_INIT
#define DBG_PIN_TOGGLE  LED1_TOGGLE

#define USER_BUTTON 1

#define I2C_TIMEOUT                      0xFFFFFFFF
#define I2Cx_SPEED                       100000
#define I2Cx_ADDRESS                     0x00
#define I2Cx_PORT                        I2C1
#define I2Cx_CLK                         CRM_I2C1_PERIPH_CLOCK
#define I2Cx_SCL_GPIO_PIN                GPIO_PINS_8
#define I2Cx_SCL_GPIO_PORT               GPIOB
#define I2Cx_SDA_GPIO_PIN                GPIO_PINS_9
#define I2Cx_SDA_GPIO_PORT               GPIOB


void board_init(void);
void delay_ms(uint32_t ms);
uint32_t ElapsedTicks(uint32_t start_ticks);
uint32_t GetTick(void);
void SW_Reset(void);
void __debugbreak(void);

void board_config_output(uint32_t frequency);

void board_frequency_measurement_start(void);
void board_frequency_measurement_stop(void);
void board_frequency_measurement_cb(void(*cb)(uint32_t));


#ifdef __cplusplus
}
#endif

#endif