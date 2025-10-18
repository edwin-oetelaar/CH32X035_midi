#include "bsp.h"
#include "ed_potscan.h"
#include "channels.h"

// Define how many timer ticks to wait for the MUX to settle.
// This is a tunable parameter. Start with a small value.
#define MUX_SETTLE_DELAY_TICKS 5

void ed_pot_sm_init (ed_pot_sm_t *p) {
    if (p == NULL)
        return;

    p->current_state = STATE_SET_MUX;
    p->current_channel = 0;
    p->scan_complete = false;
    p->delay_counter = 0;

    // Clear the results array
    for (int i = 0; i < 16; i++) {
        p->values[i] = 0;
    }
}

void ed_pot_sm_deinit (ed_pot_sm_t **p) {
    // In this implementation, we don't allocate memory dynamically,
    // so we just NULL the pointer to signify de-initialization.
    if (p != NULL) {
        *p = NULL;
    }
}

void ed_pot_sm_next (ed_pot_sm_t *p) {
    if (p == NULL)
        return;

    ed_pot_sm_state_t next_state = p->current_state;  // no change by default

    switch (p->current_state) {
    case STATE_SET_MUX:
        set_active_mux (p->current_channel);
        next_state = STATE_WAIT_MUX_SETTLE;
        p->delay_counter = 0;
        break;

    case STATE_WAIT_MUX_SETTLE:
        if (p->delay_counter >= MUX_SETTLE_DELAY_TICKS) {
            next_state = STATE_START_ADC;
        } else {
            p->delay_counter++;
        }
        break;

    case STATE_START_ADC:
        // 1. Configure which channel to convert (Channel 9, Rank 1)
        // The sample time can be adjusted based on the source impedance.
        ADC_RegularChannelConfig (ADC_PORT, ADC_CHANNEL, 1, ADC_SampleTime_11Cycles);

        // 2. Start the software conversion

        ADC_SoftwareStartConvCmd (ADC_PORT, ENABLE);
        next_state = STATE_WAIT_ADC_COMPLETE;
        break;

    case STATE_WAIT_ADC_COMPLETE:
        // Check if the End of Conversion flag is set
        if (ADC_GetFlagStatus (ADC_PORT, ADC_FLAG_EOC) != RESET) {
            next_state = STATE_READ_ADC;
        }
        // If not, we just stay in this state and check again on the next tick.
        break;

    case STATE_READ_ADC:
        p->values[p->current_channel] = ADC_GetConversionValue (ADC_PORT);
        next_state = STATE_NEXT_CHANNEL;
        break;

    case STATE_NEXT_CHANNEL:
        p->current_channel++;
        if (p->current_channel >= CHANNEL_COUNT) {
            // Scan is complete
            p->current_channel = 0;
            p->scan_complete = true;
        }
        // Loop back to start the next channel (or the first one again)
        next_state = STATE_SET_MUX;
        break;

    default:
        // Should not happen, but for safety, reset to a known state.
        next_state = STATE_SET_MUX;
        break;
    }
    p->current_state = next_state;  // make explicit here
}

bool ed_pot_sm_is_scan_complete (const ed_pot_sm_t *p) {
    if (p == NULL)
        return false;
    return p->scan_complete;
}