# SSD1327-ESP
A minimal driver written in C for the SSD1327 display controller.
## Usage
### SPI
```c
#define SSD1327_IMPLEMENTATION
#define COM_BACKEND COM_BACKEND_SPI
#include "ssd1327_driver.h"

display_t display = {
    .PIN_MOSI = 23,
    .PIN_CLK = 18,
    .PIN_CS = 5,
    .PIN_DC = 16,
    .PIN_RST = 17,
    .DISPLAY_WIDTH = 128,
    .DISPLAY_HEIGHT = 128,
    .Contrast = 0x7F,
};

void app_main(void) {
    oled_setup(&display);
    static uint8_t buffer[128][128] = {0};
    uint8_t *fb[128];
    for (int i = 0; i < 128; i++) fb[i] = buffer[i];
    oled_flush_fb(fb, &display);
}
```
### I²C
```c
#define SSD1327_IMPLEMENTATION
#define COM_BACKEND COM_BACKEND_I2C
#include "ssd1327_driver.h"

display_t display = {
    .PIN_SDA = 21,
    .PIN_SCL = 22,
    .PIN_RST = 17,
    .i2c_port = I2C_NUM_0,
    .i2c_address = 0x3C,
    .DISPLAY_WIDTH = 128,
    .DISPLAY_HEIGHT = 128,
    .Contrast = 0x7F,
};

void app_main(void) {
    oled_setup(&display);
    static uint8_t buffer[128][128] = {0};
    uint8_t *fb[128];
    for (int i = 0; i < 128; i++) fb[i] = buffer[i];
    oled_flush_fb(fb, &display);
}

```
