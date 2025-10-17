#pragma once

// my structs for holding state and state transitions

typedef struct {
    uint8_t current_state;
    uint8_t next_state;
    uint8_t ch;
    uint16_t value[16];
} AnalogPots_t;

ed_pot_sm_init(AnalogPots_t *p); // could do some alloc
ed_pot_sm_next(AnalogPots_t *p); // state transition
ed_pot_sm_deinit(AnalogPots_t **p); // ref to pointer to clean up

