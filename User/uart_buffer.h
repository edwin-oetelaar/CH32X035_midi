#pragma once

#include <stdint.h>
#include <stdbool.h>

// --- Generic TX Queue ---
#define UART_TX_BUFFER_SIZE 64

typedef struct {
    uint8_t buffer[UART_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile bool is_active; // Tracks if the TX interrupt is currently active
} UartTxQueue_t;

// --- Generic RX Queue ---
#define UART_RX_BUFFER_SIZE 64

typedef struct {
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UartRxQueue_t;


// --- TX Queue Function Prototypes ---
void UartTxQueue_Init(UartTxQueue_t* q);
bool UartTxQueue_Push(UartTxQueue_t* q, uint8_t data);
bool UartTxQueue_Pop(UartTxQueue_t* q, uint8_t* data);
bool UartTxQueue_IsEmpty(UartTxQueue_t* q);
bool UartTxQueue_IsActive(UartTxQueue_t* q);
void UartTxQueue_SetActive(UartTxQueue_t* q, bool active);

// --- RX Queue Function Prototypes ---
void UartRxQueue_Init(UartRxQueue_t* q);
bool UartRxQueue_Push(UartRxQueue_t* q, uint8_t data);
bool UartRxQueue_Pop(UartRxQueue_t* q, uint8_t* data);
bool UartRxQueue_IsEmpty(UartRxQueue_t* q);
uint16_t UartRxQueue_GetCount(UartRxQueue_t* q);