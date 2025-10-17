#include "uart_buffer.h"

// --- TX Queue Implementation ---
void UartTxQueue_Init(UartTxQueue_t* q) {
    q->head = 0;
    q->tail = 0;
    q->is_active = false;
}

bool UartTxQueue_Push(UartTxQueue_t* q, uint8_t data) {
    uint16_t next_head = (q->head + 1) % UART_TX_BUFFER_SIZE;
    if (next_head == q->tail) return false; // Queue is vol
    q->buffer[q->head] = data;
    q->head = next_head;
    return true;
}

bool UartTxQueue_Pop(UartTxQueue_t* q, uint8_t* data) {
    if (q->head == q->tail) return false; // Queue is leeg
    *data = q->buffer[q->tail];
    q->tail = (q->tail + 1) % UART_TX_BUFFER_SIZE;
    return true;
}

bool UartTxQueue_IsEmpty(UartTxQueue_t* q) {
    return (q->head == q->tail);
}

bool UartTxQueue_IsActive(UartTxQueue_t* q) {
    return q->is_active;
}

void UartTxQueue_SetActive(UartTxQueue_t* q, bool active) {
    q->is_active = active;
}

// --- RX Queue Implementation ---
void UartRxQueue_Init(UartRxQueue_t* q) {
    q->head = 0;
    q->tail = 0;
}

bool UartRxQueue_Push(UartRxQueue_t* q, uint8_t data) {
    uint16_t next_head = (q->head + 1) % UART_RX_BUFFER_SIZE;
    if (next_head == q->tail) return false; // Queue is vol
    q->buffer[q->head] = data;
    q->head = next_head;
    return true;
}

bool UartRxQueue_Pop(UartRxQueue_t* q, uint8_t* data) {
    if (q->head == q->tail) return false; // Queue is leeg
    *data = q->buffer[q->tail];
    q->tail = (q->tail + 1) % UART_RX_BUFFER_SIZE;
    return true;
}

bool UartRxQueue_IsEmpty(UartRxQueue_t* q) {
    return (q->head == q->tail);
}

uint16_t UartRxQueue_GetCount(UartRxQueue_t* q) {
    return (q->head - q->tail + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
}