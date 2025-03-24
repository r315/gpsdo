#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "i2c.h"
#include "gd32e23x_gpio.h"

#define LED1_PIN_INIT  \
        gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);

#define LED1_OFF    gpio_bit_write(GPIOA, GPIO_PIN_8, SET)
#define LED1_ON     gpio_bit_write(GPIOA, GPIO_PIN_8, RESET)
#define LED1_TOGGLE gpio_bit_toggle(GPIOA, GPIO_PIN_8)

#define DAC_MAX_VAL     0x0FFF      // 12bit DAC
#define LED_PPS_MASK    (3 << 1)

enum led_tag{
    LED_PPS = 0,
    LED_LOCK,
    LED_ALM
};


void board_init(void);
void delay_ms(uint32_t ms);
uint32_t ElapsedTicks(uint32_t start_ticks);
uint32_t GetTick(void);
void SW_Reset(void);
void __debugbreak(void);

void board_config_output(uint32_t frequency);
int32_t board_trim_irc(int8_t adj);

uint32_t serial_a_available(void);
uint32_t serial_a_read(uint8_t *buf, uint32_t len);
uint32_t serial_a_write(const uint8_t *buf, uint32_t len);

uint32_t serial_b_available(void);
uint32_t serial_b_read(uint8_t *buf, uint32_t len);
uint32_t serial_b_write(const uint8_t *buf, uint32_t len);

void serial_passtrough(void);

void frequency_measurement_start(void(*cb)(uint32_t));
void frequency_measurement_stop(void);
void phase_measurement_start(void(*cb)(uint32_t));
void phase_measurement_stop(void);

void dac_init(void);
void dac_duty_set(uint16_t duty);
uint16_t dac_duty_get(void);
uint32_t dac_voltage_get(void);
void adc_init(void);

i2cbus_t *board_i2c_get(void);

uint8_t pps_init(void);

void led_set(enum led_tag, uint8_t state);
uint32_t vref_get(void);

#ifdef __cplusplus
}
#endif

#endif