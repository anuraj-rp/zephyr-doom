#include <stdio.h>
#include <zephyr/kernel.h>

#include "gamepad.h"

// extern int doom_main(int argc, char const * const *argv);
// extern void spi_lcd_init() ;


int main(void)
{
    printf("Hello Doom! %s\n", CONFIG_BOARD);
    jsInit();
}
