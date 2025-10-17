/*********************************************************************
 * @file        bsp.h
 * @author      Jouw Naam
 * @version     V1.0.0
 * @date        2024-05-21
 * @brief       Board Support Package (BSP) header for the CH32X035 project.
 *              Defines all hardware configurations, pin mappings, and provides
 *              function prototypes for initialization and control.
***********************************************************************/
#pragma once

#ifndef __BSP_H
#define __BSP_H

#include "ch32x035.h"
#include "uart_buffer.h"

// --- Timer Definities ---

#define SCAN_TIMER            TIM3
#define SCAN_TIMER_IRQn       TIM3_IRQn
#define SCAN_TIMER_FREQ       2000 // 2000 Hz


//=============================================================================
//== LED Configuration =====================================================
//=============================================================================

#define LED1_GPIO_PORT         GPIOA
#define LED1_GPIO_PIN          GPIO_Pin_0

#define LED2_GPIO_PORT         GPIOA
#define LED2_GPIO_PIN          GPIO_Pin_1


//=============================================================================
//== MIDI UART Configuration (USART2) =======================================
//=============================================================================

#define MIDI_UART              USART2
#define MIDI_UART_IRQn         USART2_IRQn
#define MIDI_UART_BAUDRATE     31250

#define MIDI_TX_GPIO_PORT      GPIOA
#define MIDI_TX_GPIO_PIN       GPIO_Pin_2

#define MIDI_RX_GPIO_PORT      GPIOA
#define MIDI_RX_GPIO_PIN       GPIO_Pin_3


//=============================================================================
//== Debug UART Configuration (USART1) ======================================
//=============================================================================

#define DEBUG_UART             USART1
#define DEBUG_UART_IRQn        USART1_IRQn
#define DEBUG_UART_BAUDRATE    115200

#define DEBUG_TX_GPIO_PORT     GPIOB
#define DEBUG_TX_GPIO_PIN      GPIO_Pin_10

#define DEBUG_RX_GPIO_PORT     GPIOB
#define DEBUG_RX_GPIO_PIN      GPIO_Pin_11


//=============================================================================
//== SPI Configuration (SPI1) ================================================
//=============================================================================

#define SPI_PORT               SPI1

#define SPI_CS_GPIO_PORT       GPIOA
#define SPI_CS_GPIO_PIN        GPIO_Pin_4

#define SPI_SCK_GPIO_PORT      GPIOA
#define SPI_SCK_GPIO_PIN       GPIO_Pin_5

#define SPI_MISO_GPIO_PORT     GPIOA
#define SPI_MISO_GPIO_PIN      GPIO_Pin_6

#define SPI_MOSI_GPIO_PORT     GPIOA
#define SPI_MOSI_GPIO_PIN      GPIO_Pin_7


//=============================================================================
//== I2C Configuration (I2C1) ================================================
//=============================================================================

#define I2C_PORT               I2C1

#define I2C_SCL_GPIO_PORT      GPIOA
#define I2C_SCL_GPIO_PIN       GPIO_Pin_10

#define I2C_SDA_GPIO_PORT      GPIOA
#define I2C_SDA_GPIO_PIN       GPIO_Pin_11


//=============================================================================
//== ADC Configuration (ADC1, Channel 9) =====================================
//=============================================================================

#define ADC_PORT               ADC1
#define ADC_CHANNEL            ADC_Channel_9

#define ADC_GPIO_PORT          GPIOB
#define ADC_GPIO_PIN           GPIO_Pin_1


//=============================================================================
//== Analog Mux Selector Configuration =======================================
//=============================================================================

#define MUX_S0_GPIO_PORT       GPIOA
#define MUX_S0_GPIO_PIN        GPIO_Pin_15

#define MUX_S1_GPIO_PORT       GPIOA
#define MUX_S1_GPIO_PIN        GPIO_Pin_16

#define MUX_S2_GPIO_PORT       GPIOA
#define MUX_S2_GPIO_PIN        GPIO_Pin_17

#define MUX_S3_GPIO_PORT       GPIOA
#define MUX_S3_GPIO_PIN        GPIO_Pin_18


//=============================================================================
//== NVIC Priority Configuration =============================================
//=============================================================================

// NOTE: These are examples. Adjust priorities based on your application's needs.
// A lower number means a higher priority.

#define TIMER_IRQ_PRIORITY     0  // Highest priority for time-critical tasks
#define UART_IRQ_PRIORITY      1  // Lower priority, can be preempted
#define SPI_IRQ_PRIORITY       1  // Same as UART, can be adjusted
#define ADC_IRQ_PRIORITY       1  // Lower priority



//=============================================================================
//== Function Prototypes =====================================================
//=============================================================================

/**
 * @brief Initializes all configured hardware peripherals.
 *        This function should be called once at the start of main().
 */
void BSP_Init(void);

/**
 * @brief Initializes the GPIO pins for the status LEDs.
 */
void BSP_LED_Init(void);

/**
 * @brief Initializes USART2 for MIDI communication.
 */
void BSP_MIDI_UART_Init(void);

/**
 * @brief Initializes USART1 for debug communication (printf).
 */
void BSP_Debug_UART_Init(void);

/**
 * @brief Initializes SPI1 for communication with peripherals.
 */
void BSP_SPI_Init(void);

/**
 * @brief Initializes I2C1 for communication with peripherals.
 */
void BSP_I2C_Init(void);

/**
 * @brief Initializes ADC1 for reading analog signals.
 */
void BSP_ADC_Init(void);

/**
 * @brief Initializes the GPIO pins for the analog multiplexer selectors.
 */
void BSP_Mux_Init(void);

void BSP_Timer_Init(void);
/**
 * @brief Initializes ADC1 for reading analog signals.
 */
void BSP_ADC_Init(void);

/**
 * @brief Performs a single ADC conversion on the configured channel.
 * @return The 12-bit ADC conversion result (0-4095).
 */
uint16_t BSP_ADC_Read(void);


// --- Functie Prototypes (toevoegingen) ---
void MIDI_SendByte_NonBlocking(uint8_t byte);

void UART1_SendByte_NonBlocking(uint8_t byte);
// set mux

/**
 * @brief Defines the available analog multiplexer channels.
 */
typedef enum {
    CH_POT1 = 0,  ///< Channel for potentiometer 1
    CH_POT2 = 1,  ///< Channel for potentiometer 2
    // Voeg hier eventueel meer kanalen toe
    CH_TEMP_SENSOR = 2,
    CH_LIGHT_SENSOR = 3
} channel_t;

/**
 * @brief Sets the active channel on the analog multiplexer.
 * @param channel The channel to select. Must be of type channel_t.
 */
void set_active_mux(const channel_t channel);
void BSP_ADC_Diagnostic_Run(void);
//extern UartRxQueue_t midiRxQueue;      // Nieuw
//extern UartRxQueue_t debugRxQueue;     // Nieuw

// --- Globale Queue Instanties ---
extern UartTxQueue_t midiTxQueue;
extern UartRxQueue_t midiRxQueue;
extern UartTxQueue_t debugTxQueue;
extern UartRxQueue_t debugRxQueue;

// --- Generieke UART API ---
// Functies die de juiste queue selecteren op basis van de USART_TypeDef* pointer.

// TX API
void UART_TX_Push(USART_TypeDef* uart, uint8_t data);

// RX API
bool UART_RX_HasData(USART_TypeDef* uart);
uint16_t UART_RX_GetDataCount(USART_TypeDef* uart);
bool UART_RX_ReadByte(USART_TypeDef* uart, uint8_t* data);

// Interrupt Handlers (worden aangeroepen vanuit de ISRs)
void UART_TX_HandleIrq(USART_TypeDef* uart);
void UART_RX_HandleIrq(USART_TypeDef* uart);

// Interrupt Handler voor timer3, die 2000 Hz gaat
void TIM3_Update_HandleIrq(void *p);

#endif // __BSP_H