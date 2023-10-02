#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "spi_lcd.h"

#define LED_BLUE    2


void gpio_toggler(struct k_timer *timer_info)
{
    gpio_pin_toggle(gpio_dev_p, LED_BLUE);
};

K_TIMER_DEFINE(gpio_timer, gpio_toggler, NULL);

int main(void)
{
    if(!device_is_ready(gpio_dev_p))
	{
		printf("GPIO CTRL not ready!");
        return;
	}
	int ret;
	ret = gpio_pin_configure(gpio_dev_p, LED_BLUE, GPIO_OUTPUT_ACTIVE);
    ret = gpio_pin_configure(gpio_dev_p, ILI9341_DC, GPIO_OUTPUT_ACTIVE);
    ret = gpio_pin_configure(gpio_dev_p, ILI9341_RESET, GPIO_OUTPUT_INACTIVE);
    k_timer_start(&gpio_timer, K_MSEC(200), K_MSEC(200));
    spi_lcd_init();
    // while(1)
    // {
    //     printf("Hello World! %s\n", CONFIG_BOARD);
    //     k_msleep(500);
    // }

    return 0;
}
