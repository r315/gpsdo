
#include <stdint.h>
#include <stdio.h>
#include "board.h"
#include "gpsdo.h"

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
    board_init();

    gpsdo();

    while(1)
    {
    }
}

