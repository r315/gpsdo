
#include <stdint.h>
#include <stdio.h>
#include "board.h"
#include "gpsdo.h"
#include "logger.h"

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
    board_init();

    dac_init();


    gpsdo();

    while(1)
    {
    }
}

