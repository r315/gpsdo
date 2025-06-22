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

#define DAC_MAX_VAL     0x0FFF      // 12bit DAC
#define LED_PPS_MASK    (3 << 1)

#define VCC5V0          4420          // OXCO supply
#define VDDA            3300          // ADC reference voltage in mv
#define VREF            2500          // Vref from LT1009
#define ADC_RES         4096          // 12-bit ADC resolution

#define THM_ADC_CH      ADC_CHANNEL_5
#define THM_R_FIXED     9840.0        // Pull-up resistor value in ohms
#define THM_BETA        3950.0        // Beta parameter of the NTC thermistor
#define THM_T0          298.15        // 25°C in Kelvin
#define THM_R0          10000.0       // Resistance at 25°C (10kΩ)

enum led_tag{
    LED_PPS = 0,
    LED_LOCK,
    LED_ALM
};

/* board low level */
void board_init(void);
#if ENABLE_IRC8_TRIM
int32_t board_trim_irc(int8_t adj);
#endif
i2cbus_t* board_i2c_get(void);
uint16_t board_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t size);
uint16_t board_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t size);
void board_reset(void);

/* Time related functions */
void delay_ms(uint32_t ms);
uint32_t elapsed_ms(uint32_t start_ms);
uint32_t get_ms(void);

/* System functions */
void __debugbreak(void);
void system_clock_output(uint8_t en);

/* Frequency counter */
void counter_start(void(*cb)(uint32_t));
void counter_stop(void);
void phase_start(void(*cb)(uint32_t));
void phase_stop(void);
void phase_reset(void);
void phase_select(uint8_t input);

/* DAC */
void dac_init(void);
void dac_duty_set(uint16_t duty);
uint16_t dac_duty_get(void);
uint32_t dac_voltage_get(void);

/* ADC */
void adc_init(void);
uint16_t adc_get(uint8_t ch);
uint32_t adc_voltage_get(uint16_t raw);

/* Temperature sensor */
float temp_get(void);

/* PPS Generation */
uint8_t pps_init(void);
void pps_select(uint8_t input);

/* LED */
void led_set(enum led_tag, uint8_t state);

/* Main ouput */
void output_select(uint8_t in);

/* EEPROM */
uint8_t settings_load(uint8_t *data, uint8_t len);
uint8_t settings_save(const uint8_t *data, uint8_t len);

/* Global varaiables */
extern stdinout_t serial_a_ops;
extern stdinout_t serial_b_ops;

#ifdef __cplusplus
}
#endif

#endif