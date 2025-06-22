
#include "board.h"
#include "si5351.h"
#include "clock_gen.h"
#include "cli_simple.h"

static i2cbus_t *i2c ;

static uint8_t parseFreq(uint64_t *out, char *str)
{
    uint64_t freq = 0;
    int32_t value;

    char *ptr = str;

    while(*ptr != '\0') {
        if(*ptr == '.'){
            *(ptr++) = '\0';
            // and truncate decimal part into two digits, assuming
            // that buffer has at least three more bytes after '.'
            ptr[2] = '\0';
            // ensure two digits
            if(ptr[1] == '\0'){
                ptr[1] = '0';
            }
            break;
        }
        ptr++;
    }

    // Parse integer part
    CLI_Ia2i(str, &value);
    freq = value * SI5351_FREQ_MULT;
    // Parse decimal part
    value = 0;
    CLI_Ia2i(ptr, &value);
    freq += value;

    if(freq > SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT){
        return 0;
    }

    *out = freq;
    return 1;
}

clockres_t clock_gen_init(void)
{
    i2c = board_i2c_get();

    if (i2c == NULL){
        return CLOCK_FAIL;
    }

    if(Si5351_DeviceStatusGet().status.REVID == 0){
        if(Si5351_Init(i2c, 0, 0) == 0){
            return CLOCK_FAIL;
        }
    }

    return CLOCK_OK;
}

clockres_t clock_gen_set_frequency(uint8_t clk, uint32_t frequency)
{
    if(Si5351_FreqSet((enum si5351_clock)clk, frequency)){
        return CLOCK_FAIL;
    }

    if(Si5351_DeviceStatusGet().status.LOL_A == 1){
        Si5351_ClockPwrSet((enum si5351_clock)clk, 1);
        Si5351_OutputEnableSet((enum si5351_clock)clk, 1);
    }

    return CLOCK_OK;
}

clockres_t clock_gen_shutdown(uint8_t clk)
{
    Si5351_ClockPwrSet((enum si5351_clock)clk, 0);
    Si5351_OutputEnableSet((enum si5351_clock)clk, 0);

    return CLOCK_OK;
}