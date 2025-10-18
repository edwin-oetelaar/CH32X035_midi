/*********************************************************************
 * @file        channels.h
 * @author      E vd Oetelaar
 * @version     V1.0.0
 * @date        2025-10-18
 * @brief       Defines all available channels on the analog multiplexer.
 *              Includes a sentinel value for the total channel count.
 ***********************************************************************/

#ifndef __CHANNELS_H
#define __CHANNELS_H

/**
 * @brief Defines all available channels on the analog multiplexer.
 * @note    The values (0-15) correspond directly to the binary pattern
 *          needed for the multiplexer's selector pins (S0-S3).
 */
typedef enum {
    // --- Main Potentiometers (Channels 0-7) ---
    CH_POT1 = 0,  ///< Channel for main potentiometer 1
    CH_POT2 = 1,  ///< Channel for main potentiometer 2
    CH_POT3 = 2,  ///< Channel for main potentiometer 3
    CH_POT4 = 3,  ///< Channel for main potentiometer 4
    CH_POT5 = 4,  ///< Channel for main potentiometer 5
    CH_POT6 = 5,  ///< Channel for main potentiometer 6
    CH_POT7 = 6,  ///< Channel for main potentiometer 7
    CH_POT8 = 7,  ///< Channel for main potentiometer 8

    // --- Other Controls (Channels 8-10) ---
    CH_SLIDER = 8,     ///< Channel for the main slider potentiometer
    CH_BTN_RIGHT = 9,  ///< Channel for the right button
    CH_BTN_LEFT = 10,  ///< Channel for the left button

    // --- Unused / Reserved Channels (Channels 11-15) ---
    CH_UNUSED1 = 11,  ///< Reserved channel, not currently used
    CH_UNUSED2 = 12,  ///< Reserved channel, not currently used
    CH_UNUSED3 = 13,  ///< Reserved channel, not currently used
    CH_UNUSED4 = 14,  ///< Reserved channel, not currently used
    CH_UNUSED5 = 15,  ///< Reserved channel, not currently used

    // --- Sentinel Value ---
    /**
     * @brief Total number of channels on the multiplexer.
     * @note    This is NOT a valid channel itself. It's a sentinel value
     *          used to define the size of arrays and as a loop boundary.
     *          This makes the code robust against future changes.
     */
    CHANNEL_COUNT = 16

} channel_t;

/**
 * @brief A lookup table to convert channel_t enums to human-readable strings.
 * @note    The index of the array corresponds to the channel_t enum value.
 *          This is perfect for debugging and logging.
 */
static const char* const channel_names[CHANNEL_COUNT] = {
    [CH_POT1]      = "CH_POT1",
    [CH_POT2]      = "CH_POT2",
    [CH_POT3]      = "CH_POT3",
    [CH_POT4]      = "CH_POT4",
    [CH_POT5]      = "CH_POT5",
    [CH_POT6]      = "CH_POT6",
    [CH_POT7]      = "CH_POT7",
    [CH_POT8]      = "CH_POT8",
    [CH_SLIDER]    = "CH_SLIDER",
    [CH_BTN_RIGHT] = "CH_BTN_RIGHT",
    [CH_BTN_LEFT]  = "CH_BTN_LEFT",
    [CH_UNUSED1]   = "CH_UNUSED1",
    [CH_UNUSED2]   = "CH_UNUSED2",
    [CH_UNUSED3]   = "CH_UNUSED3",
    [CH_UNUSED4]   = "CH_UNUSED4",
    [CH_UNUSED5]   = "CH_UNUSED5"
};

/**
 * @brief Safely retrieves the string name for a given channel.
 * @param channel The channel enum value.
 * @return A pointer to the constant string name. Returns "UNKNOWN" if the
 *         channel is out of bounds. This function is 'inline' for performance.
 */
static inline const char* get_channel_name(channel_t channel) {
    if (channel >= 0 && channel < CHANNEL_COUNT) {
        return channel_names[channel];
    } else {
        return "UNKNOWN";
    }
}

#endif  // __CHANNELS_H