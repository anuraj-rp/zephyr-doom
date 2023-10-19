#include <stdio.h>
#include <zephyr/kernel.h>

#include "gamepad.h"

// extern int doom_main(int argc, char const * const *argv);
// extern void spi_lcd_init() ;

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

void blink(const struct led *led, uint32_t sleep_ms, uint32_t id)
{
	while(1)
	{
		printk("Blink Thread\n");
		k_msleep(200);
	}
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink, NULL, NULL, NULL, PRIORITY, 0, 0);

int main(void)
{
    printf("Hello Doom! %s\n", CONFIG_BOARD);
    jsInit();
    return 0;
}
