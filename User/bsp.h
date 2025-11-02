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
#include "channels.h"



typedef int32_t ed_err_t;

/* Definitions for error constants. */

#define ED_OK          (0)
#define ED_FAIL        (-1)

/* smart arrays size macros, from linux kernel */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]) + __must_be_array(arr))
/* &a[0] degrades to a pointer: a different type from an array */
#define __must_be_array(a) BUILD_BUG_ON_ZERO(__same_type((a), &(a)[0]))
#define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))
#define BUILD_BUG_ON_ZERO(e) (sizeof(struct { int:(-!!(e)); }))


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

// #define I2C_PORT               I2C1

// #define I2C_SCL_GPIO_PORT      GPIOA
// #define I2C_SCL_GPIO_PIN       GPIO_Pin_10

// #define I2C_SDA_GPIO_PORT      GPIOA
// #define I2C_SDA_GPIO_PIN       GPIO_Pin_11

// Kies twee vrije BitBang pinnen. PA10 en PA11 zijn hier een goede keuze.
#define I2C_BB_SCL_GPIO_PORT   GPIOA
#define I2C_BB_SCL_GPIO_PIN    GPIO_Pin_10

#define I2C_BB_SDA_GPIO_PORT   GPIOA
#define I2C_BB_SDA_GPIO_PIN    GPIO_Pin_11


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
//void BSP_hardware_I2C_Init(void);
void BSP_software_I2C_Init(void);
ed_err_t bsp_i2c_master_transmit (const uint8_t dev_id, const uint8_t *buf, uint8_t len);
/**
 * @brief Scans the I2C bus for connected devices.
 * @note    This function iterates through all possible 7-bit I2C addresses
 *          and reports any devices that respond with an ACK.
 *          The results are printed to the debug UART.
 * @return  none
 */
void BSP_I2C_BB_Scan(void);

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

/**
 * @brief Sets the active channel on the analog multiplexer.
 * @param channel The channel to select. Must be of type channel_t.
 */
void set_active_mux(const channel_t channel);
void systick_init(void);
void BSP_ADC_Diagnostic_Run(void);

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
// Simple macro functions to give a arduino-like functions to call
// millis() reads the incremented systick variable
// micros() reads the raw SysTick Count, and divides it by the number of
// ticks per microsecond (WARN: This only uses the lower 32 bits of the SysTick)
extern volatile uint32_t systick_millis;
#define millis() (systick_millis)
#define micros() (SysTick->CNT / SYSTICK_ONE_MICROSECOND)

#endif // __BSP_H