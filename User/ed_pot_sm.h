#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Defines the states for the ADC/MUX scanning state machine.
 */
typedef enum {
    STATE_SET_MUX,          ///< Set the multiplexer to the next channel.
    STATE_WAIT_MUX_SETTLE,  ///< Wait a few ticks for the MUX to settle.
    STATE_START_ADC,        ///< Trigger the ADC conversion.
    STATE_WAIT_ADC_COMPLETE,///< Wait for the ADC conversion to finish.
    STATE_READ_ADC,         ///< Read the value from the ADC and store it.
    STATE_NEXT_CHANNEL      ///< Prepare for the next channel or signal completion.
} ed_pot_sm_state_t;

/**
 * @brief Structure to hold the state and data for the pot scanner state machine.
 */
typedef struct {
    ed_pot_sm_state_t current_state;  ///< The current state of the FSM.
    uint8_t current_channel;          ///< The channel currently being scanned (0-15).
    uint16_t values[16];              ///< Array to store the 16 ADC results.
    bool scan_complete;               ///< Flag to indicate a full scan is finished.
    uint8_t delay_counter;            ///< Internal counter for handling wait states.
} ed_pot_sm_t;

/**
 * @brief Initializes the pot scanner state machine.
 * @param p A pointer to the state machine instance.
 */
void ed_pot_sm_init(ed_pot_sm_t *p);

/**
 * @brief Advances the state machine by one step.
 * @note This function is designed to be called periodically, e.g., from a timer interrupt.
 * @param p A pointer to the state machine instance.
 */
void ed_pot_sm_next(ed_pot_sm_t *p);

/**
 * @brief De-initializes the state machine.
 * @param p A pointer to a pointer to the state machine instance.
 */
void ed_pot_sm_deinit(ed_pot_sm_t **p);

/**
 * @brief Checks if a full scan of all 16 channels is complete.
 * @param p A pointer to the state machine instance.
 * @return True if the scan is complete, false otherwise.
 */
bool ed_pot_sm_is_scan_complete(const ed_pot_sm_t *p);