// ssd1306.h
// This header is designed for use with ESP-IDF I2C master APIs
// Edwin vd Oetelaar, december 2024
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
// #include "driver/i2c_master.h"

    // SSD1306 device descriptor
    typedef struct
    {
        uint16_t device_address;             // I2C device address (2 bytes)
        //i2c_master_dev_handle_t i2c_port;    // I2C port handle (assumed pointer, 4 bytes)
        //i2c_master_dev_handle_t *dev_handle; // Pointer to device on I2C bus (4 bytes)

        uint8_t width;                       // Display width in pixels (1 byte)
        uint8_t height;                      // Display height in pixels (1 byte)
        uint8_t pages;                       // Total pages (1 byte)
        uint8_t external_vcc;                // External VCC flag (1 byte)

        uint8_t buffer[1024];                // Pixel buffer (1024 bytes)
    } ssd1306_handle_t;

    // Initialization/free function
    void ssd1306_init(ssd1306_handle_t *dev, uint8_t width, uint8_t height, uint8_t external_vcc);
    void ssd1306_free(ssd1306_handle_t **dev);

    // Basic commands
    void ssd1306_poweroff(ssd1306_handle_t *dev);
    void ssd1306_poweron(ssd1306_handle_t *dev);
    void ssd1306_contrast(ssd1306_handle_t *dev, uint8_t contrast);
    void ssd1306_invert(ssd1306_handle_t *dev, uint8_t invert);

    // Drawing / updating
    void ssd1306_fill(ssd1306_handle_t *dev, uint8_t color);
    void ssd1306_set_pixel(ssd1306_handle_t *dev, unsigned int x, unsigned int y, uint8_t color);

    // Copy bitmap from buffer to device (blit)
    void ssd1306_show(ssd1306_handle_t *dev);
    uint8_t ssd1306_printFixed6(ssd1306_handle_t *dev, uint8_t xpos, uint8_t y, uint8_t color, const char *str);
    uint8_t ssd1306_printFixed8(ssd1306_handle_t *dev, uint8_t xpos, uint8_t ypos, uint8_t color, const char *str);
    uint8_t ssd1306_printFixed16(ssd1306_handle_t *dev, uint8_t xpos, uint8_t ypos, uint8_t color, const char *str);
    // Low-level I2C write
    // static void ssd1306_write_cmd(ssd1306_handle_t *dev, uint8_t cmd);
    // static void ssd1306_write_data(ssd1306_handle_t *dev, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif
