#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STATE_1,       ///< Set the multiplexer to the next channel.
    STATE_2,       ///< Wait a few ticks for the MUX to settle.
    STATE_STDC,    ///< Trigger the ADC conversion.
    STATE_WALETE,  ///< Wait for the ADC conversion to finish.
    STATE_REC,     ///< Read the value from the ADC and store it.
    STATE_INVALID  ///< Prepare for the next channel or signal completion.
} ed_keyscan_sm_state_t;

/**
 * @brief Structure to hold the state and data for the pot scanner state machine.
 */
typedef struct {
    ed_keyscan_sm_state_t current_state;  ///< The current state of the FSM.
                                          // uint8_t current_channel;          ///< The channel currently being scanned (0-15).
                                          // uint16_t values[16];              ///< Array to store the 16 ADC results.
                                          // bool scan_complete;               ///< Flag to indicate a full scan is finished.
                                          // uint8_t delay_counter;            ///< Internal counter for handling wait states.
} ed_keyscan_t;

/**
 * @brief Initializes the pot scanner state machine.
 * @param p A pointer to the state machine instance.
 */
void ed_keyscan_init (ed_keyscan_t *p);

/**
 * @brief Advances the state machine by one step.
 * @note This function is designed to be called periodically, e.g., from a timer interrupt.
 * @param p A pointer to the state machine instance.
 */
void ed_keyscan_next (ed_keyscan_t *p);


/**
 * @brief De-initializes the state machine.
 * @param p A pointer to a pointer to the state machine instance.
 */
void ed_keyscan_deinit (ed_keyscan_t **p);
