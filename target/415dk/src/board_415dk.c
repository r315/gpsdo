#include <stdint.h>
#include "board.h"
#include "syscalls.h"
#include "serial.h"
#include "i2c.h"

static serialbus_t uartbus_a;
static volatile uint32_t ticms;

static void serial_init(void)
{
    UART_Init(&uartbus_a);
    GPIO_Config(PA_9, GPIO_USART1_TX);
    GPIO_Config(PA_10, GPIO_USART1_RX);
}

static uint32_t serial_available(void)
{
    return UART_Available(&uartbus_a);
}

static uint32_t serial_read(uint8_t *buf, uint32_t len)
{
    return UART_Read(&uartbus_a, buf, len);
}

static uint32_t serial_write(const uint8_t *buf, uint32_t len)
{
    return UART_Write(&uartbus_a, buf, len);
}

static stdout_ops_t stdout_ops_serial = {
    .init = serial_init,
    .available = serial_available,
    .read = serial_read,
    .write = serial_write
};

void SysTick_Handler(void)
{
    ticms++;
}

void delay_ms(uint32_t ms)
{
    volatile uint32_t end = ticms + ms;
    while (ticms < end){ }
}

uint32_t ElapsedTicks(uint32_t start_ticks)
{
    return ticms - start_ticks;
}

inline uint32_t GetTick(void)
{
    return ticms;
}

void board_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);

    LED1_PIN_INIT;

	SystemInit();

	SystemCoreClockUpdate();

	SysTick_Config((SystemCoreClock / 1000) - 1);

    uartbus_a.bus = UART_BUS0;
    uartbus_a.speed = 115200;

    redirect_stdout(&stdout_ops_serial);

    I2C_Init(I2C_BUS0);
}

void SW_Reset(void){
    NVIC_SystemReset();
}

void __debugbreak(void){
	 asm volatile
    (
        "bkpt #01 \n"
    );
}

void board_config_output(uint32_t frequency)
{

}

void board_frequency_measurement_start(void)
{

}

void board_frequency_measurement_stop(void)
{

}

void board_frequency_measurement_cb(void(*cb)(uint32_t))
{

}

void TMR5_GLOBAL_IRQHandler(void)
{

}
