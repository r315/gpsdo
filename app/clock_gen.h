#ifndef CLOCK_GEN_H
#define CLOCK_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

// Include necessary headers
#include <stdint.h>

typedef enum {
    CLOCK_OK = 0,
    CLOCK_FAIL,

}clockres_t;


// Function prototypes
/**
 * @brief Initializes the clock generator.
 *
 * @return int CLOCK_GEN_SUCCESS on success, CLOCK_GEN_ERROR on failure.
 */
clockres_t clock_gen_init(void);

/**
 * @brief Sets the frequency of the clock generator.
 *
 * @param frequency Desired frequency in Hz.
 * @return int CLOCK_GEN_SUCCESS on success, CLOCK_GEN_ERROR on failure.
 */
clockres_t clock_gen_set_frequency(uint8_t clk, uint32_t frequency);

/**
 * @brief Gets the current frequency of the clock generator.
 *
 * @return uint32_t Current frequency in Hz.
 */
uint32_t clock_gen_get_frequency(uint8_t clk);

/**
 * @brief Shuts down the clock generator.
 *
 * @return int CLOCK_GEN_SUCCESS on success, CLOCK_GEN_ERROR on failure.
 */
clockres_t clock_gen_shutdown(uint8_t clk);

#ifdef __cplusplus
}
#endif

#endif // CLOCK_GEN_H