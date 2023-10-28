#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "gamepad.h"

// extern int doom_main(int argc, char const * const *argv);
// extern void spi_lcd_init() ;

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

static const struct gpio_dt_spec led = 
	GPIO_DT_SPEC_GET(DT_NODELABEL(blinking_led), gpios);


// void button_pressed(const struct device* dev, 
// 					struct gpio_callback *cb,
// 					uint32_t pins)
// {
// 	int ret;
// 	ret = gpio_pin_toggle_dt(&led);
// 	if(ret != 0){
// 		printk("Could not toggle LED\n");
// 	}
// }

void blink(void)
{
	int ret;


	// if(!gpio_is_ready_dt(&led));
	// {
	// 	printf("Led not ready\n");
	// 	while (1);
	// }
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if(ret != 0)
	{
		printf("Error!\n");
		while(1);
	}

	while(1)
	{
		// printk("Blink Thread\n");
		int ret;
		ret = gpio_pin_toggle_dt(&led);
		if(ret != 0){
			printk("Could not toggle LED\n");
		}
		k_msleep(200);
	}
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink, NULL, NULL, NULL, PRIORITY, 0, 2000);

int main(void)
{
    printf("Hello Doom! %s\n", CONFIG_BOARD);
    jsInit();
    return 0;
}
