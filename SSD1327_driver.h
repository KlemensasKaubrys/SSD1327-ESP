#ifndef SSD1327_DRIVER_H_
#define SSD1327_DRIVER_H_

#define COM_BACKEND_SPI 0
#define COM_BACKEND_I2C 1

#ifndef COM_BACKEND
#define COM_BACKEND COM_BACKEND_SPI
#endif // COM_BACKEND

#include <stdint.h>
#include <stddef.h>

typedef struct display_t display_t;

void oled_setup(display_t *display);
void oled_reset(display_t *display);
void oled_flush_fb(uint8_t **fb, display_t *display);
void oled_cmd(display_t *display, uint8_t cmd);
void oled_data(display_t *display, uint8_t *data, size_t len);
void oled_init(display_t *display);

#ifdef SSD1327_IMPLEMENTATION
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#if COM_BACKEND == COM_BACKEND_SPI
#include "driver/spi_master.h"

struct display_t {
    int PIN_MOSI;
    int PIN_CLK;
    int PIN_CS;
    int PIN_DC;
    int PIN_RST;
    int DISPLAY_HEIGHT;
    int DISPLAY_WIDTH;
    int Contrast;
    spi_device_handle_t spi;
};

void oled_cmd(display_t *display, uint8_t cmd) {
    gpio_set_level(display->PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(display->spi, &t);
}

void oled_data(display_t *display, uint8_t *data, size_t len) {
    gpio_set_level(display->PIN_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data
    };
    spi_device_transmit(display->spi, &t);
}

void oled_setup(display_t *display) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << display->PIN_DC) | (1ULL << display->PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .mosi_io_num = display->PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = display->PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8000000,
        .mode = 0,
        .spics_io_num = display->PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &display->spi));
    oled_init(display);
}

#elif COM_BACKEND == COM_BACKEND_I2C

#include "driver/i2c.h"

#define I2C_MASTER_FREQ_HZ  400000
#define I2C_TX_BUF_DISABLE  0
#define I2C_RX_BUF_DISABLE  0

struct display_t {
    int PIN_SDA;
    int PIN_SCL;
    int PIN_RST;
    int DISPLAY_HEIGHT;
    int DISPLAY_WIDTH;
    int Contrast;
    uint8_t i2c_address;
    i2c_port_t i2c_port;
};

void oled_cmd(display_t *display, uint8_t cmd) {
    i2c_cmd_handle_t cmd_hdl = i2c_cmd_link_create();
    i2c_master_start(cmd_hdl);
    i2c_master_write_byte(cmd_hdl, (display->i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_hdl, 0x80, true);
    i2c_master_write_byte(cmd_hdl, cmd, true);
    i2c_master_stop(cmd_hdl);
    i2c_master_cmd_begin(display->i2c_port, cmd_hdl, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd_hdl);
}

void oled_data(display_t *display, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd_hdl = i2c_cmd_link_create();
    i2c_master_start(cmd_hdl);
    i2c_master_write_byte(cmd_hdl, (display->i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_hdl, 0x40, true);
    i2c_master_write(cmd_hdl, data, len, true);
    i2c_master_stop(cmd_hdl);
    i2c_master_cmd_begin(display->i2c_port, cmd_hdl, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd_hdl);
}

void oled_setup(display_t *display) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << display->PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = display->PIN_SDA,
        .scl_io_num = display->PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(display->i2c_port, &conf);
    i2c_driver_install(display->i2c_port, I2C_MODE_MASTER,
                       I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);

    oled_init(display);
}

#endif // SSD1327_COM_BACKEND

void oled_reset(display_t *display) {
    gpio_set_level(display->PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(display->PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void oled_init(display_t *display) {
    oled_reset(display);
    oled_cmd(display, 0xAE);                                                   // Display OFF
    oled_cmd(display, 0x81); oled_cmd(display, display->Contrast);             // Contrast
    oled_cmd(display, 0xA0); oled_cmd(display, 0x53);                          // Remap
    oled_cmd(display, 0xA1); oled_cmd(display, 0x00);                          // Start line
    oled_cmd(display, 0xA2); oled_cmd(display, 128 - display->DISPLAY_HEIGHT); // Display vertical offset
    oled_cmd(display, 0xA4);                                                   // Display follows RAM
    oled_cmd(display, 0xA8); oled_cmd(display, display->DISPLAY_HEIGHT - 1);   // Multiplex ratio
    oled_cmd(display, 0xB1); oled_cmd(display, 0xF1);                          // Precharge phase length
    oled_cmd(display, 0xB3); oled_cmd(display, 0x00);                          // Clock divider
    oled_cmd(display, 0xAB); oled_cmd(display, 0x01);                          // Enable internal voltage regulator
    oled_cmd(display, 0xB6); oled_cmd(display, 0x0F);                          // Precharge voltage
    oled_cmd(display, 0xBE); oled_cmd(display, 0x0F);                          // VCOMH voltage level
    oled_cmd(display, 0xBC); oled_cmd(display, 0x08);                          // First precharge period
    oled_cmd(display, 0xD5); oled_cmd(display, 0x62);                          // Second precharge period
    oled_cmd(display, 0xAF);                                                   // Display ON
}

void oled_flush_fb(uint8_t **fb, display_t *display) {
    oled_cmd(display, 0x15); oled_cmd(display, 0x00); oled_cmd(display, (display->DISPLAY_WIDTH / 2) - 1); // Columns
    oled_cmd(display, 0x75); oled_cmd(display, 0x00); oled_cmd(display, display->DISPLAY_HEIGHT - 1); // Rows
    
    for (int y = 0; y < display->DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < display->DISPLAY_WIDTH; x += 2) {
            uint8_t packet = (fb[y][x] << 4) | (fb[y][x + 1] & 0x0F);
            oled_data(display, &packet, 1);
        }
    }
}

#endif // SSD1327_IMPLEMENTATION

#endif // SSD1327_DRIVER_H_