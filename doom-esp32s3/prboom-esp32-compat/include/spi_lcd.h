#ifndef SPI_LCD_H_
#define SPI_LCD_H_

// ili9341 driver for test taken from - https://github.com/anuraj-rp/doom-espidf

#define ILI9341_DC          7
#define ILI9341_RESET       6
#define ILI9341_DATA        1
#define ILI9341_CMD         0

#define ILI9341_STACK_SIZE  8192

extern const struct device *gpio_dev_p;

void spi_lcd_wait_finish();
void spi_lcd_send(uint16_t *scr);
void spi_lcd_init();

#endif //SPI_LCD_H_