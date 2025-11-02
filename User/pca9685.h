/* pca9685.h : oetelaar */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "bsp.h"
#include <stdio.h>
#include <stdbool.h>

#ifndef PCA9685_I2C_TIMEOUT
#define PCA9685_I2C_TIMEOUT 10 /* ticks timeout */
#endif

#define PCA9685_SET_BIT_MASK(BYTE, MASK) ((BYTE) |= (uint8_t)(MASK))
#define PCA9685_CLEAR_BIT_MASK(BYTE, MASK) ((BYTE) &= (uint8_t)(~(uint8_t)(MASK)))
#define PCA9685_READ_BIT_MASK(BYTE, MASK) ((BYTE) & (uint8_t)(MASK))

/**
 * Registers addresses.
 */
typedef enum {
    PCA9685_REGISTER_MODE1 = 0x00,
    PCA9685_REGISTER_MODE2 = 0x01,
    PCA9685_REGISTER_LED0_ON_L = 0x06,
    PCA9685_REGISTER_ALL_LED_ON_L = 0xfa,
    PCA9685_REGISTER_ALL_LED_ON_H = 0xfb,
    PCA9685_REGISTER_ALL_LED_OFF_L = 0xfc,
    PCA9685_REGISTER_ALL_LED_OFF_H = 0xfd,
    PCA9685_REGISTER_PRESCALER = 0xfe
} pca9685_register_t;

/**
 * Bit masks for the mode 1 register.
 */
typedef enum {
    PCA9685_REGISTER_MODE1_SLEEP = (1u << 4u),
    PCA9685_REGISTER_MODE1_RESTART = (1u << 7u)
} pca9685_register_mode1_t;

/**
 * Structure defining a handle describing a PCA9685 device.
 */
typedef struct
{
    // i2c_master_dev_handle_t *dev_handle;  // Handle to the I2C bus for the device
    uint16_t device_address;              // I2C device address
    bool inverted;                        // Set to true to drive inverted
} pca9685_handle_t;

// Initialize PCA9685, reset registers, set PWM frequency to 1000Hz, turn off all channels, and wake up
bool pca9685_init (pca9685_handle_t *handle);

// Test if PCA9685 is in sleep mode
bool pca9685_is_sleeping (pca9685_handle_t *handle, bool *sleeping);

// Put PCA9685 into sleep mode
bool pca9685_sleep (pca9685_handle_t *handle);

// Wake up PCA9685 from sleep mode
bool pca9685_wakeup (pca9685_handle_t *handle);

// Set PWM frequency (24-1526 Hz) for all channels
bool pca9685_set_pwm_frequency (pca9685_handle_t *handle, float frequency);

// Set PWM on/off times (0-4096) for a specific channel (0-15)
ed_err_t pca9685_set_channel_pwm_times (pca9685_handle_t *handle, unsigned channel, unsigned on_time, unsigned off_time);

// Set PWM duty cycle (0.0-1.0) for a specific channel
bool pca9685_set_channel_duty_cycle (pca9685_handle_t *handle, unsigned channel, float duty_cycle);

// Set the number of LEDs based on LED bar configuration
bool pca9685_set_led_bar (pca9685_handle_t *handle, unsigned number_of_led);

// Set up a servo on the given channel (500-2500 µs pulse width, center at 1500 µs)
bool pca_set_servo_channel (pca9685_handle_t *handle, unsigned chan, unsigned microsec);

// Turn a channel ON/OFF
bool pca9685_set_channel_onoff (pca9685_handle_t *handle, unsigned channel, uint8_t onoff);

#ifdef __cplusplus
}
#endif
