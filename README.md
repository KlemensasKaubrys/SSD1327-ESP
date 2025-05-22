# SSD1327-ESP
A minimal driver written in C for the SSD1327 display controller.
## Usage
### SPI
```c
#define SSD1327_IMPLEMENTATION
#define COM_BACKEND COM_BACKEND_SPI
#include "ssd1327_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "SSD1327"
#define WIDTH 128
#define HEIGHT 96

display_t display = {
    .PIN_MOSI = 23,
    .PIN_CLK = 18,
    .PIN_CS = 5,
    .PIN_DC = 16,
    .PIN_RST = 17,
    .DISPLAY_WIDTH = WIDTH,
    .DISPLAY_HEIGHT = HEIGHT,
    .Contrast = 0x7F,
};

void app_main(void) {
    oled_setup(&display);
    uint8_t **fb = fb_alloc(HEIGHT, WIDTH);
    fb_clear(fb, HEIGHT, WIDTH);

    while (1) {
    	oled_flush_fb(fb, &display);
    	vTaskDelay(pdMS_TO_TICKS(100));
    }
    fb_free(fb, HEIGHT);
}
```
### I²C
```c
#define SSD1327_IMPLEMENTATION
#define COM_BACKEND COM_BACKEND_I2C
#include "ssd1327_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "SSD1327"
#define WIDTH 128
#define HEIGHT 96

display_t display = {
    .PIN_SDA = 21,
    .PIN_SCL = 22,
    .PIN_RST = 17,
    .i2c_port = I2C_NUM_0,
    .i2c_address = 0x3C,
    .DISPLAY_WIDTH = WIDTH,
    .DISPLAY_HEIGHT = HEIGHT,
    .Contrast = 0x7F,
};

void app_main(void) {
    oled_setup(&display);
    uint8_t **fb = fb_alloc(HEIGHT, WIDTH);
    fb_clear(fb, HEIGHT, WIDTH);

    while (1) {
    	oled_flush_fb(fb, &display);
    	vTaskDelay(pdMS_TO_TICKS(100));
    }
    fb_free(fb, HEIGHT);
}

```
