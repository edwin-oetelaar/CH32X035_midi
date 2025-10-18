#include "bsp.h"
#include "ed_keyscan.h"

// Define how many timer ticks to wait for the MUX to settle.
// This is a tunable parameter. Start with a small value.
#define MUX_SETTLE_DELAY_TICKS 5

void ed_keyscan_init (ed_keyscan_t *p) {
    if (p == NULL)
        return;

    // p->current_state = STATE_SET_MUX;
    // p->current_channel = 0;
    // p->scan_complete = false;
    // p->delay_counter = 0;

    // // Clear the results array
    // for (int i = 0; i < 16; i++) {
    //     p->values[i] = 0;
    // }
}

void ed_keyscan_deinit (ed_keyscan_t **p) {
    // In this implementation, we don't allocate memory dynamically,
    // so we just NULL the pointer to signify de-initialization.
    if (p != NULL) {
        *p = NULL;
    }
}

void ed_keyscan_next (ed_keyscan_t *p) {
    if (p == NULL)
        return;
    ed_keyscan_sm_state_t next_state = p->current_state;  // no change by default
    p->current_state = next_state;                        // make explicit here
}
