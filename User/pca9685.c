#if 0
/*
 * pca9685.c
 * handles i2c IO extenders, used for prorail LED bars
 *  Created on: May 26, 2024
 *      Author: oetelaar edwin@oetelaar.com
 */

//#include <freertos/FreeRTOS.h>
//#include <freertos/semphr.h>
//#include <driver/i2c_master.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
//#include "esp_err.h"
#include <errno.h>
//#include "esp_log.h"
//#include "esp_system.h"
#include "pca9685.h"
#include <stddef.h>

static bool pca9685_write_u8(pca9685_handle_t *handle, uint8_t address, uint8_t value)
{
    // check params
    assert(handle != NULL);

    uint8_t data[] = {address, value};

    ed_err_t ret = i2c_master_transmit(*(handle->dev_handle), data, sizeof(data), pdMS_TO_TICKS(1000));

    return (ESP_OK == ret);
}

static bool pca9685_write_data(pca9685_handle_t *handle,
                               uint8_t address,
                               uint8_t *txdata,
                               size_t length)
{
    assert(handle != NULL);
    if (length == 0 || length > 4)
        return false;

    uint8_t data[5];
    data[0] = address;

    memcpy(data + 1, txdata, length);
    ed_err_t ret = i2c_master_transmit(*(handle->dev_handle), data, length + 1, pdMS_TO_TICKS(1000));

    return (ESP_OK == ret);
}

static bool pca9685_read_u8(pca9685_handle_t *handle, uint8_t address, uint8_t *dest)
{
    // printf("tst r_u8: %p %x %p\n", handle, address, dest);

    assert(handle != NULL);

    ed_err_t ret = i2c_master_transmit_receive(*(handle->dev_handle), &address, 1, dest, 1, pdMS_TO_TICKS(1000));
    // printf("ret r_u8: %d\n", ret);

    return (ESP_OK == ret);
}

bool pca9685_init(pca9685_handle_t *handle)
{
    // Controleer de geldigheid van de i2c_handle
    assert(handle != NULL);

    // Stel standaardwaarden in voor de mode registers (Auto-Increment, Sleep, TOTEM POLE)
    uint8_t mode1_reg_default_value = 0b00010000u; // is NO sleep+auto incre, was auto incr and sleep 0b00110000u; //
    uint8_t mode2_reg_default_value = 0b00000100u; // OUTDRV=1 0x04

    if (handle->inverted)
    {
        mode2_reg_default_value |= 0b00010000u; // INVRT=1
    }

    // Schrijf de standaardwaarden naar de mode registers
    if (!pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg_default_value))
        return false;

    if (!pca9685_write_u8(handle, PCA9685_REGISTER_MODE2, mode2_reg_default_value))
        return false;

    // Zet alle kanalen uit om mee te beginnen
    uint8_t data[4] = {0x00, 0x00, 0xFF, 0x1F}; // ON_TIME=0x000, OFF_TIME = 0xFFF + FLAG 0x10
    if (!pca9685_write_data(handle, PCA9685_REGISTER_ALL_LED_ON_L, data, 4))
        return false;

    // Stel de PWM frequentie in op 1000 Hz
    if (!pca9685_set_pwm_frequency(handle, 1000.0f))
        return false;

    // Wek het apparaat op
    if (!pca9685_wakeup(handle))
        return false;

    return true;
}

bool pca9685_wakeup(pca9685_handle_t *handle)
{
    assert(handle != NULL);

    uint8_t mode1_reg;
    if (!pca9685_read_u8(handle, PCA9685_REGISTER_MODE1, &mode1_reg))
    {
        return false;
    }

    bool restart_required = PCA9685_READ_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
    PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
    PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);

    if (!pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg))
    {
        return false;
    }

    if (restart_required)
    {
        vTaskDelay(1);
        PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
        return pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg);
    }

    return true;
}

bool pca9685_set_pwm_frequency(pca9685_handle_t *handle, float frequency)
{
    assert(handle != NULL);
    // Controleer de geldigheid van de frequentie
    if (frequency < 24.0f || frequency > 1526.0f)
    {
        return false;
    }
    uint8_t prescaler = (uint8_t)(roundf(25000000.0f / (4096.0f * frequency)) - 1.0f);
    // printf("resulting value=%f\n", 25000000u / 4096u / (prescaler -0.5));
    uint8_t old_mode;
    pca9685_read_u8(handle, PCA9685_REGISTER_MODE1, &old_mode);
    uint8_t mode = (old_mode & 0x7F) | 0x10;
    pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode);
    pca9685_write_u8(handle, PCA9685_REGISTER_PRESCALER, prescaler);
    pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, old_mode);
    vTaskDelay(1);
    mode = old_mode | 0xa1;
    pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode);

    return true;
}

ed_err_t pca9685_set_channel_pwm_times(pca9685_handle_t *handle, unsigned channel, unsigned on_time,
                                        unsigned off_time)
{
    assert(handle != NULL);

    if (channel > 15 || on_time > 4096 || off_time > 4096)
    {
        return false;
    }

    uint8_t data[4] = {(uint8_t)on_time, (uint8_t)(on_time >> 8u), (uint8_t)off_time,
                       (uint8_t)(off_time >> 8u)};

    return pca9685_write_data(handle, PCA9685_REGISTER_LED0_ON_L + channel * 4, data, 4);
}

/**
 * @brief Stel de duty cycle in voor een PCA9685 kanaal.
 *
 * @param handle Pointer naar de PCA9685 handle.
 * @param channel Het kanaalnummer (0-15).
 * @param duty_cycle De gewenste duty cycle (0.0 - 1.0).
 * @return true Als de operatie succesvol was.
 * @return false Als er een fout optrad, zoals een ongeldige duty cycle of NULL handle.
 */
bool pca9685_set_channel_duty_cycle(pca9685_handle_t *handle, unsigned channel, float duty_cycle)
{
    assert(handle != NULL);

    if (duty_cycle < 0.0f || duty_cycle > 1.0f || channel > 15)
    {
        return false;
    }

    unsigned on_time, off_time;

    if (duty_cycle == 0.0f)
    {
        return pca9685_set_channel_pwm_times(handle, channel, 0, 4096); // Altijd uit
    }
    else if (duty_cycle == 1.0f)
    {
        return pca9685_set_channel_pwm_times(handle, channel, 4096, 0); // Altijd aan
    }
    else
    {
        unsigned required_on_time = (unsigned)roundf(4095 * duty_cycle);
        on_time = (channel == 0) ? 0 : (channel * 256) - 1;
        off_time = (on_time + required_on_time) & 0xfffu;
        return pca9685_set_channel_pwm_times(handle, channel, on_time, off_time);
    }
}

/**
 * @brief Zet een PCA9685 kanaal aan of uit.
 *
 * @param handle Pointer naar de PCA9685 handle.
 * @param channel Het kanaalnummer (0-15).
 * @param onoff De gewenste toestand (1 = aan, 0 = uit).
 * @return true Als de operatie succesvol was.
 * @return false Als er een fout optrad, zoals een ongeldige handle of kanaalnummer.
 */
bool pca9685_set_channel_onoff(pca9685_handle_t *handle, unsigned channel, uint8_t onoff)
{
    assert(handle != NULL);

    if (channel > 15)
    {
        return false;
    }

    uint8_t data_on[4] = {0, 16, 0, 0};
    uint8_t data_off[4] = {0, 0, 16, 0};
    uint8_t *p = (onoff) ? data_on : data_off;

    return pca9685_write_data(handle, PCA9685_REGISTER_LED0_ON_L + channel * 4, p, 4);
}

/**
 * @brief Stel een LED-balk in op een PCA9685, waarbij een bepaald aantal LEDs wordt ingeschakeld.
 *
 * @param handle Pointer naar de PCA9685 handle.
 * @param number_of_led Het aantal LEDs dat moet worden ingeschakeld (0-16).
 * @return true Als de operatie succesvol was.
 * @return false Als er een fout optrad, zoals een ongeldige handle of een ongeldig aantal LEDs.
 */
bool pca9685_set_led_bar(pca9685_handle_t *handle, unsigned number_of_led)
{
    assert(handle != NULL);

    if (number_of_led > 16)
    {
        return false;
    }

    for (unsigned i = 0; i < 16; i++)
    {
        // pca9685_set_channel_onoff(handle, i, (i < number_of_led) ? 1 : 0); // werkt dit niet???

        if (!pca9685_set_channel_pwm_times(handle, i,
                                           (number_of_led > i) ? 4096 : 0,
                                           (number_of_led > i) ? 0 : 4096))
            return false;
    }

    return true;
}

bool pca_set_servo_channel(pca9685_handle_t *handle, unsigned chan, unsigned microsec)
{
    assert(handle != NULL);
    int on_time;
    int off_time;
    // total time is 1/60 sec == 4096
    // 1/60 of second is 0.0166666
    // 4.07 us per bit (12 bits)
    // 1500 / 4 => 368 is the on_time
    on_time = 16; // microsec >> 2;
    off_time = 16 + (microsec >> 2);
    ed_err_t ret = pca9685_set_channel_pwm_times(handle, chan, on_time, off_time);
    return (ret != ESP_OK);
}

/* end of file Edwin was here */
#endif