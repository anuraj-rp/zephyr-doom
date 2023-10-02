```c
/*
 /*
 * ┌─────────────────────────────────┐
 * │                                 │
 * │ VCC                             │
 * │ GND                             │
 * │ CS                              │
 * │ RESET                 SD_CS     │
 * │ DC                              │
 * │ SDI(MOSI)             SD_MOSI   │
 * │ SCK                             │
 * │ LED                   SD_MISO   │
 * │ SDO(MISO)                       │
 * │ T_CLK                 SD_SCK    │
 * │ T_CS                            │
 * │ T_DIN                           │
 * │ T_DO                            │
 * │ T_IRQ                           │
 * └─────────────────────────────────┘
 */
```

TFT Manal - https://web.mit.edu/6.115/www/document/TFT_User_Manual.pdf

• GND - Ground
• CS - Chip select (active low)
• RESET - Reset (active low)
• DC - Data/Command select. When the DC line is low, data received by the LCD
is interpreted as commands. When this DC line is high, data is interpreted as data
(arguments to commands, pixel data, etc.)
• SDI - Serial data input (mosi)
• SDO - Serial data output (miso)
• SCK - Serial clock input
• LED - This pin is used to control the intensity of the backlight. If not controlled by
an analog voltage, connecting this pin to 3.3V will set the display to full brightness

pins on esp32s3

cs -    10
mosi -  11
sck  -  12
miso -  13
dc   -   7
reset-   6