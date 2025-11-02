#include "bsp.h"
#include "ed_potscan.h"

// --- Definitie van de Globale Queues ---
UartTxQueue_t midiTxQueue;
UartRxQueue_t midiRxQueue;
UartTxQueue_t debugTxQueue;
UartRxQueue_t debugRxQueue;

// globale state

volatile uint32_t counter = 0;

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)HSI_VALUE / 1000)
// Number of ticks elapsed per microsecond (48 when using 48MHz Clock)
#define SYSTICK_ONE_MICROSECOND ((uint32_t)HSI_VALUE / 1000000)
#define SYSTICK_CTLR_SWIE (1 << 31)
#define SYSTICK_CTLR_INIT (1 << 5)
#define SYSTICK_CTLR_MODE (1 << 4)
#define SYSTICK_CTLR_STRE (1 << 3)
#define SYSTICK_CTLR_STCLK (1 << 2)
#define SYSTICK_CTLR_STIE (1 << 1)
#define SYSTICK_CTLR_STE (1 << 0)
#define SYSTICK_SR_CNTIF (1 << 0)


// Incremented in the SysTick IRQ - in this example once per millisecond
volatile uint32_t systick_millis;

/*
 * Initialises the SysTick to trigger an IRQ with auto-reload, using HCLK/1 as
 * its clock source
 */
void systick_init (void) {
    // Reset any pre-existing configuration
    SysTick->CTLR = 0x0000;

    // Set the SysTick Compare Register to trigger in 1 millisecond
    SysTick->CMP = SysTick->CNT + SYSTICK_ONE_MILLISECOND;

    systick_millis = 0x00000000;

    // Set the SysTick Configuration
    // NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
    // busywait delay funtions used by ch32v003_fun.
    SysTick->CTLR |= SYSTICK_CTLR_STE |   // Enable Counter
                     SYSTICK_CTLR_STIE |  // Enable Interrupts
                     SYSTICK_CTLR_STCLK;  // Set Clock Source to HCLK/1

    // Enable the SysTick IRQ
    NVIC_EnableIRQ (SysTicK_IRQn);
}

/*
 * SysTick ISR - must be lightweight to prevent the CPU from bogging down.
 * Increments Compare Register and systick_millis when triggered (every 1ms)
 */

void SysTick_Handler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

void SysTick_Handler (void) {
    // Set the SysTick Compare Register to trigger in 1 millisecond
    SysTick->CMP = SysTick->CNT + SYSTICK_ONE_MILLISECOND;

    // Clear the trigger state for the next IRQ
    SysTick->SR = 0x00000000;

    // Increment the milliseconds count
    systick_millis++;
}

// --- Implementatie van de Generieke API ---

void UART_TX_Push (USART_TypeDef *uart, uint8_t data) {
    UartTxQueue_t *queue = NULL;
    if (uart == MIDI_UART) {
        queue = &midiTxQueue;
    } else if (uart == DEBUG_UART) {
        queue = &debugTxQueue;
    }
    if (queue) {
        if (UartTxQueue_Push (queue, data)) {
            // Als de verzendketen nog niet loopt, start hem.
            if (!UartTxQueue_IsActive (queue)) {
                UartTxQueue_SetActive (queue, true);
                USART_ITConfig (uart, USART_IT_TXE, ENABLE);
            }
        } else {
            printf ("Error: UART TX Queue is full!\r\n");
        }
    }
}

bool UART_RX_HasData (USART_TypeDef *uart) {
    UartRxQueue_t *queue = (uart == MIDI_UART) ? &midiRxQueue : &debugRxQueue;
    return !UartRxQueue_IsEmpty (queue);
}

uint16_t UART_RX_GetDataCount (USART_TypeDef *uart) {
    UartRxQueue_t *queue = (uart == MIDI_UART) ? &midiRxQueue : &debugRxQueue;
    return UartRxQueue_GetCount (queue);
}

bool UART_RX_ReadByte (USART_TypeDef *uart, uint8_t *data) {
    UartRxQueue_t *queue = (uart == MIDI_UART) ? &midiRxQueue : &debugRxQueue;
    return UartRxQueue_Pop (queue, data);
}

// --- Implementatie van de Interrupt Handlers ---

void UART_TX_HandleIrq (USART_TypeDef *uart) {
    UartTxQueue_t *queue = (uart == MIDI_UART) ? &midiTxQueue : &debugTxQueue;
    uint8_t dataToSend;
    if (UartTxQueue_Pop (queue, &dataToSend)) {
        USART_SendData (uart, dataToSend);
    } else {
        // Queue is leeg, stop de verzendketen.
        UartTxQueue_SetActive (queue, false);
        USART_ITConfig (uart, USART_IT_TXE, DISABLE);
    }
}

void UART_RX_HandleIrq (USART_TypeDef *uart) {
    UartRxQueue_t *queue = (uart == MIDI_UART) ? &midiRxQueue : &debugRxQueue;
    uint8_t received_byte = (u8)USART_ReceiveData (uart);  // cast down to 8 bit
    UartRxQueue_Push (queue, received_byte);
}

/*********************************************************************
 * @brief   Initializes the GPIO pins for the status LEDs (LED1 on PA0, LED2 on PA1).
 * @note    Configures the pins as push-pull outputs and sets them to a
 *          known initial state (off).
 * @return  none
 */

void BSP_LED_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 1. Enable the peripheral clock for GPIOA
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);

    // 2. Configure the GPIO pins for the LEDs
    // Configure both LED pins in a single initialization call for efficiency.
    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN | LED2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // Push-Pull output mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (LED1_GPIO_PORT, &GPIO_InitStructure);

    // 3. Set the LEDs to a known initial state (off)
    // This ensures the LEDs are not on when the system starts.
    GPIO_ResetBits (LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN);
}

/*
void BSP_LED_Init(void)
{
    // 1. Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 2. Configure both LED pins in one call using direct struct initialization
    GPIO_Init(LED1_GPIO_PORT, &(GPIO_InitTypeDef){
        .GPIO_Pin   = LED1_GPIO_PIN | LED2_GPIO_PIN,
        .GPIO_Mode  = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz
    });

    // 3. Turn off both LEDs (initial state)
    GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN);
}*/
/*********************************************************************
 * @brief   Initializes the GPIO pins for the analog multiplexer selectors.
 * @note    Configures PA15, PA16, PA17, and PA18 as push-pull outputs.
 *          Sets all pins to a known initial state (low) to select channel 0.
 * @return  none
 */
/*
void BSP_Mux_Init(void)
{
   // 1. Enable GPIOA clock (idempotent ? safe to call multiple times)
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

   // 2. Configure all 4 MUX pins in one call using direct struct initialization
   GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
       .GPIO_Pin   = MUX_S0_GPIO_PIN | MUX_S1_GPIO_PIN | MUX_S2_GPIO_PIN | MUX_S3_GPIO_PIN,
       .GPIO_Mode  = GPIO_Mode_Out_PP,
       .GPIO_Speed = GPIO_Speed_50MHz
   });

   // 3. Set MUX to channel 0 (0000) ? all pins low
   GPIO_ResetBits(GPIOA, MUX_S0_GPIO_PIN | MUX_S1_GPIO_PIN | MUX_S2_GPIO_PIN | MUX_S3_GPIO_PIN);
}*/

void BSP_Mux_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 1. Enable the peripheral clock for GPIOA
    // Note: If another BSP function (like BSP_LED_Init) already enabled this,
    // it's safe to call it again. Enabling an already enabled clock has no effect.
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);

    // 2. Configure the GPIO pins for the MUX selectors
    // Configure all four MUX pins in a single initialization call for efficiency.
    GPIO_InitStructure.GPIO_Pin = MUX_S0_GPIO_PIN | MUX_S1_GPIO_PIN | MUX_S2_GPIO_PIN | MUX_S3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // Push-Pull output mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (MUX_S0_GPIO_PORT, &GPIO_InitStructure);

    // 3. Set the MUX to a known initial state (channel 0)
    // This ensures a predictable starting point for the first ADC reading.
    // Channel 0 corresponds to binary 0000, so all pins are set low.
    GPIO_ResetBits (MUX_S0_GPIO_PORT, MUX_S0_GPIO_PIN | MUX_S1_GPIO_PIN | MUX_S2_GPIO_PIN | MUX_S3_GPIO_PIN);
}

void BSP_software_I2C_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = I2C_BB_SDA_GPIO_PIN | I2C_BB_SCL_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // we change this when needed, i2c needs pull ups
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (I2C_BB_SDA_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits (I2C_BB_SDA_GPIO_PORT, I2C_BB_SDA_GPIO_PIN | I2C_BB_SCL_GPIO_PIN);
}

// // static uint8_t SDA_READ (void) { return GPIO_ReadInputDataBit (I2C_BB_SDA_GPIO_PORT, I2C_BB_SDA_GPIO_PIN); }
// #define GPIO_MODE_INPUT 0x4  // Floating input (0b0100)

// // static uint8_t SCL_READ (void) { return GPIO_ReadInputDataBit (I2C_BB_SCL_GPIO_PORT, I2C_BB_SCL_GPIO_PIN); }
// void set_gpio_input (GPIO_TypeDef *gpio, uint8_t pin) {
//     if (pin <= 7) {  // CFGLR voor pinnen 0-7
//         uint32_t tempreg = gpio->CFGLR;
//         tempreg &= ~(0xF << (pin * 4));
//         tempreg |= (GPIO_MODE_INPUT << (pin * 4));
//         gpio->CFGLR = tempreg;
//     } else if (pin <= 15) {  // CFGHR voor pinnen 8-15
//         uint32_t tempreg = gpio->CFGHR;
//         tempreg &= ~(0xF << ((pin - 8) * 4));
//         tempreg |= (GPIO_MODE_INPUT << ((pin - 8) * 4));
//         gpio->CFGHR = tempreg;
//     }
// }

// --- Supersnelle Low-level GPIO Primitives ---
// Deze macros schrijven direct naar het BSRR register.
#define I2C_BB_SCL_PIN_POS (10)  // Omdat I2C_BB_SCL_GPIO_PIN = GPIO_Pin_10
#define I2C_BB_SDA_PIN_POS (11)  // Omdat I2C_BB_SDA_GPIO_PIN = GPIO_Pin_11

static inline void SCL_HIGH (void) {
    // to understand look at 8.3.1.2 Port Configuration Register High [15:8] (GPIOx_CFGHR) (x=A/B/C)
    register uint32_t tempreg = GPIOA->CFGHR;
    tempreg &= ~(0x0Fu << 8);
    tempreg |= (0x04u << 8);  // float input
    GPIOA->CFGHR = tempreg;
}

static inline void SCL_LOW (void) {
    register uint32_t tempreg = GPIOA->CFGHR;
    tempreg |= (0x03u << 8);  // output
    GPIOA->CFGHR = tempreg;
}

static inline void SDA_HIGH (void) {
    register uint32_t tempreg = GPIOA->CFGHR;
    tempreg &= ~(0x0Fu << 12);
    tempreg |= (0x04u << 12);
    GPIOA->CFGHR = tempreg;
}

static inline void SDA_LOW (void) {
    register uint32_t tempreg = GPIOA->CFGHR;
    tempreg |= (0x03u << 12);
    GPIOA->CFGHR = tempreg;
}

// Lezen kan ook direct van het Input Data Register (IDR)
static inline uint8_t SDA_READ (void) {
    return (GPIOA->INDR & I2C_BB_SDA_GPIO_PIN) ? 1 : 0;
}

// Lezen kan ook direct van het Input Data Register (IDR)
static inline uint8_t SCL_READ (void) {
    return (GPIOA->INDR & I2C_BB_SCL_GPIO_PIN) ? 1 : 0;
}

#define I2C_BB_DELAY_NOPS() \
    {                       \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
        __NOP();            \
    }

/**
 * @brief Schrijft ижижn byte naar de bus en leest de ACK.
 * @param data De byte die verstuurd moet worden.
 * @return True als een ACK werd ontvangen, False als NACK.
 */
static bool I2C_BitBang_WriteByte (uint8_t data) {
    // Stuur 8 bits, van MSB naar LSB

    for (uint8_t i = 0; i < 8; i++) {
        // SCL_LOW();  // Bereid de klok voor
        if (data & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }  // Zet data bit
        __NOP();
        __NOP();
        __NOP();
        SCL_HIGH();  // Slave leest de bit op de hoge flank
        I2C_BB_DELAY_NOPS();
        I2C_BB_DELAY_NOPS();
        SCL_LOW();   //
        data <<= 1;  // Volgende bit
    }

    // Lees de ACK/NACK

    SDA_HIGH();  // free bus for ack/nack
    I2C_BB_DELAY_NOPS();
    SCL_HIGH();
    I2C_BB_DELAY_NOPS();
    uint8_t ack = SDA_READ();  // Lees de SDA-lijn
    SCL_LOW();
    I2C_BB_DELAY_NOPS();
    SDA_LOW();
    // // ACK is een lage spanning (0), NACK is hoog (1)
    return (ack == 0);
}

/**
 * @brief Genereert een START conditie op de bus.
 */
static bool I2C_BitBang_Start (uint8_t addr) {
    // pre: sda=hi scl=hi
    // SDA_HIGH();
    // SCL_HIGH();
    I2C_BB_DELAY_NOPS();
    SDA_LOW();  // START: SDA gaat laag terwijl SCL hoog is
    I2C_BB_DELAY_NOPS();
    SCL_LOW();
    I2C_BB_DELAY_NOPS();
    return I2C_BitBang_WriteByte ((u8)(addr << 1));
    // post SDA=low SCL=low
}

/**
 * @brief Genereert een STOP conditie op de bus.
 */
static void I2C_BitBang_Stop (void) {
    // pre: SCL is low en SDA is low
    I2C_BB_DELAY_NOPS();
    SCL_HIGH();
    I2C_BB_DELAY_NOPS();
    SDA_HIGH();  // STOP: SDA gaat hoog terwijl SCL hoog is
    // post: sda=hi scl=hi
}

// static void I2C_BitBang_Restart (uint8_t addr) {
//     //  SCL_LOW();
//     SDA_HIGH();  // Zorg dat SDA laag is
//     I2C_BB_DELAY_NOPS();
//     SCL_HIGH();  // STOP: SCL gaat hoog terwijl SDA laag is
//     I2C_BB_DELAY_NOPS();
//     SDA_HIGH();
//     I2C_BitBang_Start (addr);
// }

ed_err_t bsp_i2c_master_transmit (const uint8_t dev_id, const uint8_t *buf, uint8_t len) {
    I2C_BitBang_Start (dev_id);
    while (len) {
        I2C_BitBang_WriteByte (*buf);
        len--;
        buf++;
    }                    // write buffer
    I2C_BitBang_Stop();  // stop transmission
    return ED_OK;
}

/*********************************************************************
 * @brief   Scans the I2C bus for connected devices.
 * @note    Iterates through all possible 7-bit I2C addresses (1-127)
 *          and reports any devices that respond with an ACK.
 *          The results are printed to the debug UART.
 * @return  none
 */
void BSP_I2C_BB_Scan (void) {
    uint8_t found_devices = 0;
    // uint16_t timeout;

    // first make stable high
    I2C_BitBang_Stop();
    // check if SDA and SCL are high, if not bus is busy
    while (1)
        if (SCL_READ() == Bit_SET && SDA_READ() == Bit_SET) {
            // ok
            break;
        } else {
            printf ("x");
            Delay_Ms (100);
        }

    //  u8 j = 0;

    printf ("\033[1;1H"
            "--- Starting I2C Bus Scan ---\r\n");
    // printf ("Scanning addresses 0x01 to 0x7F...\r\n");
    // while (I2C_GetFlagStatus (I2C_PORT, I2C_FLAG_BUSY) != RESET);  // block on i2c busy
    //  Loop through all possible 7-bit addresses
    for (uint8_t addr = 8; addr < 128; addr++) {
        bool ack = I2C_BitBang_Start (addr);
        if (ack) {
            // If we reach here, an ACK was received. A device is present!
            printf ("I2C device found at address: 0x%02X\r\n", addr);
            found_devices++;
        }

        // stop_scan:
        //  5. Generate a STOP condition to release the bus
        //  I2C_GenerateSTOP (I2C_PORT, ENABLE);
        I2C_BitBang_Stop();
        // Small delay to ensure the bus is free before the next attempt
        Delay_Ms (1);
    }

    printf ("--- Scan Complete ---\r\n");
    if (found_devices == 0) {
        printf ("No I2C devices found.\r\n");
    } else {
        printf ("Found %u device(s) in total.\r\n", found_devices);
    }
    printf ("-----------------------\r\n\r\n");
}

/*********************************************************************
 * @brief   Initializes USART1 for debug communication (e.g., for printf).
 * @note    Configures USART1 on PB10 (TX) and PB11 (RX) for a standard
 *          115200 baud rate. The NVIC is also configured for potential
 *          interrupt-driven debug operations.
 * @return  none
 */
void BSP_Debug_UART_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    // 1. Enable the peripheral clocks
    // Enable clock for GPIOB (for TX and RX pins)
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB, ENABLE);
    // Enable clock for USART1 (also located on APB2 bus)
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);

    // 2. Configure the USART1 GPIO pins (PB10-TX, PB11-RX)
    // Configure both TX and RX pins in a single initialization call
    GPIO_InitStructure.GPIO_Pin = DEBUG_TX_GPIO_PIN | DEBUG_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (DEBUG_TX_GPIO_PORT, &GPIO_InitStructure);

    // Re-configure the RX pin as an input. This is safer than setting both.
    GPIO_InitStructure.GPIO_Pin = DEBUG_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // Input mode for RX
    GPIO_Init (DEBUG_RX_GPIO_PORT, &GPIO_InitStructure);

    // 3. Configure the USART1 peripheral itself
    USART_InitStructure.USART_BaudRate = DEBUG_UART_BAUDRATE;                        // 115200 for debug
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 8 data bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 stop bit
    USART_InitStructure.USART_Parity = USART_Parity_No;                              // No parity
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // No flow control
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                  // Enable TX and RX

    // Apply the configuration to USART1
    USART_Init (DEBUG_UART, &USART_InitStructure);

    // 4. Configure the NVIC for USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_UART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    // BELANGRIJK: Schakel de Receive Data Register Not Empty interrupt in.
    USART_ITConfig (DEBUG_UART, USART_IT_RXNE, ENABLE);


    // 5. Enable the USART1 peripheral
    USART_Cmd (DEBUG_UART, ENABLE);
}

/*********************************************************************
 * @brief   Initializes USART2 for MIDI communication.
 * @note    Configures USART2 on PA2 (TX) and PA3 (RX) for the standard
 *          MIDI baud rate of 31250. The NVIC is also configured to allow
 *          for interrupt-driven operation.
 * @return  none
 */
void BSP_MIDI_UART_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    // 1. Enable the peripheral clocks
    // Enable clock for GPIOA (for TX and RX pins)
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);
    // Enable clock for USART2 (located on APB1 bus)
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART2, ENABLE);

    // 2. Configure the USART2 GPIO pins (PA2-TX, PA3-RX)
    // Configure both TX and RX pins in a single initialization call
    GPIO_InitStructure.GPIO_Pin = MIDI_TX_GPIO_PIN | MIDI_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (MIDI_TX_GPIO_PORT, &GPIO_InitStructure);

    // Re-configure the RX pin as an input. This is safer than setting both.
    GPIO_InitStructure.GPIO_Pin = MIDI_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // Input mode for RX
    GPIO_Init (MIDI_RX_GPIO_PORT, &GPIO_InitStructure);


    // 3. Configure the USART2 peripheral itself
    USART_InitStructure.USART_BaudRate = MIDI_UART_BAUDRATE;                         // 31250 for MIDI
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 8 data bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 stop bit
    USART_InitStructure.USART_Parity = USART_Parity_No;                              // No parity
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // No flow control
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                  // Enable TX and RX

    // Apply the configuration to USART2
    USART_Init (MIDI_UART, &USART_InitStructure);

    // 4. Configure the NVIC for USART2 interrupts
    NVIC_InitStructure.NVIC_IRQChannel = MIDI_UART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    // BELANGRIJK: Schakel de Receive Data Register Not Empty interrupt in.
    // Dit zorgt ervoor dat de ISR wordt aangeroepen zodra er een byte binnenkomt.
    USART_ITConfig (MIDI_UART, USART_IT_RXNE, ENABLE); /* moeten we dit doen? Of pas als we interrupts willen handlen ?*/

    // 5. Enable the USART2 peripheral
    USART_Cmd (MIDI_UART, ENABLE);
}

/*********************************************************************
 * @fn      BSP_SPI_Init
 *
 * @brief   Initialiseert SPI1
 *
 * @return  none
 */

void BSP_SPI_Init (void) {
    SPI_InitTypeDef SPI_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable klokken voor SPI1 en GPIOA
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_SPI1, ENABLE);

    // --- SPI Pinnen (Alternate Function) ---
    GPIO_InitStructure.GPIO_Pin = SPI_CS_GPIO_PIN | SPI_SCK_GPIO_PIN | SPI_MISO_GPIO_PIN | SPI_MOSI_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // SCK als push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (SPI_CS_GPIO_PORT, &GPIO_InitStructure);

    // SPI Configuratie
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                           // Clock is laag in idle
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                         // Data wordt vastgelegd op de eerste flank
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                            // We sturen NSS zelf (via PL pin)
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // Relatief langzame klok voor stabiliteit
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init (SPI_PORT, &SPI_InitStructure);

    // Enable SPI
    SPI_Cmd (SPI_PORT, ENABLE);
}

/*********************************************************************
 * @fn      BSP_Timer_Init
 *
 * @brief   Initialiseert TIM3 voor een 2000 Hz interrupt.
 *
 * @return  none
 */
void BSP_Timer_Init (void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    // 1. Enable klok voor de timer
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);

    // 2. Configureer de timer basis
    // SystemCoreClock / (prescaler + 1) / (period + 1) = 2000
    // Voorbeeld: 48MHz / 48 / 500 = 2000
    TIM_TimeBaseStructure.TIM_Period = 500 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 48 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit (SCAN_TIMER, &TIM_TimeBaseStructure);

    // 3. BELANGRIJK: Forceer een update event om de prescaler en period te laden
    TIM_GenerateEvent (SCAN_TIMER, TIM_EventSource_Update);

    // 4. Enable de update interrupt
    TIM_ITConfig (SCAN_TIMER, TIM_IT_Update, ENABLE);

    // 5. Configureer de NVIC
    NVIC_InitStructure.NVIC_IRQChannel = SCAN_TIMER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    // 6. Start de timer
    TIM_Cmd (SCAN_TIMER, ENABLE);
}

/*********************************************************************
 * @brief   Initializes ADC1 for reading analog signals.
 * @note    Configures ADC1 for single conversion on Channel 9 (PB1).
 *          Includes a calibration cycle for accuracy.
 * @return  none
 */
void BSP_ADC_Init (void) {
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 1. Enable the peripheral clocks
    // Enable clock for GPIOB (for the ADC pin)
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB, ENABLE);
    // Enable clock for ADC1
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);

    // 2. Configure the ADC GPIO pin (PB1) as an analog input
    GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  // Analog Input mode
    GPIO_Init (ADC_GPIO_PORT, &GPIO_InitStructure);

    // 3. Configure the ADC peripheral itself
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   // Use ADC1 independently
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                        // Single channel, no scanning
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  // Convert on demand
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // Software trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               // Right-aligned data
    ADC_InitStructure.ADC_NbrOfChannel = 1;                              // Number of channels in scan mode

    ADC_Init (ADC_PORT, &ADC_InitStructure);

    // Enable the ADC
    ADC_Cmd (ADC_PORT, ENABLE);
    // slow it down, no need to run extreme speed
    ADC_CLKConfig (ADC_PORT, ADC_CLK_Div8);  // slow it down
}

/*********************************************************************
 * @brief   Performs a single ADC conversion on the configured channel.
 * @note    This is a blocking function. It waits until the conversion is complete.
 * @return  The 12-bit ADC conversion result (0-4095).
 */
uint16_t BSP_ADC_Read (void) {
    // 1. Configure which channel to convert (Channel 9, Rank 1)
    // The sample time can be adjusted based on the source impedance.
    ADC_RegularChannelConfig (ADC_PORT, ADC_CHANNEL, 1, ADC_SampleTime_11Cycles);

    // 2. Start the software conversion
    ADC_SoftwareStartConvCmd (ADC_PORT, ENABLE);

    // 3. Wait until the conversion is complete
    while (ADC_GetFlagStatus (ADC_PORT, ADC_FLAG_EOC) == RESET);

    // 4. Read the conversion result and return it
    return ADC_GetConversionValue (ADC_PORT);
}

/*********************************************************************
 * @brief   Sets the active channel on the analog multiplexer using bit patterns.
 * @param   channel The channel number to select (0-15).
 * @return  none
 */
void set_active_mux (const channel_t channel) {
    // Veiligheidscheck: beperk het kanaal tot het bereik 0-15.
    // De '& 0x0F' maskereert de 4 laagste bits, wat een effici?nte manier is
    // om te zorgen dat het kanaal nooit groter is dan 15.
    uint8_t ch = channel & 0x0F;

    // Zet elke pin direct op de juiste waarde (hoog of laag) gebaseerd op het bitpatroon.
    // De ternaire operator (conditie ? waarde_indien_waar : waarde_indien_onwaar)
    // is hier perfect voor.
    GPIO_WriteBit (MUX_S0_GPIO_PORT, MUX_S0_GPIO_PIN, (ch & 0x01) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit (MUX_S1_GPIO_PORT, MUX_S1_GPIO_PIN, (ch & 0x02) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit (MUX_S2_GPIO_PORT, MUX_S2_GPIO_PIN, (ch & 0x04) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit (MUX_S3_GPIO_PORT, MUX_S3_GPIO_PIN, (ch & 0x08) ? Bit_SET : Bit_RESET);
}

/*********************************************************************
 * @brief   Runs a full diagnostic on the ADC and MUX.
 * @note    This function iterates through all 16 MUX channels, reads the ADC
 *          value for each, and prints the results in a table. It also measures
 *          and prints the total time taken for the full scan using the SysTick timer.
 *          This is useful for determining if delays are needed between MUX switches.
 * @return  none
 */
void BSP_ADC_Diagnostic_Run (void) {
    uint16_t adc_results[16];
    uint32_t start_tick, end_tick, elapsed_ticks;
    float elapsed_us;

    printf ("\r\n--- Starting ADC/MUX Diagnostic Scan ---\r\n");

    // 1. Start de timing met SysTick
    // SysTick is een 24-bit down-counter. We lezen de huidige waarde.
    start_tick = counter;

    // 2. Lees alle 16 kanalen met maximale snelheid
    for (uint8_t channel = 0; channel < 16; channel++) {
        set_active_mux (channel);
        Delay_Us (2);
        adc_results[channel] = BSP_ADC_Read();
    }

    // 3. Stop de timing
    end_tick = counter;

    // 4. Bereken de verstreken tijd
    // Omdat SysTick een down-counter is, is start - end de verstreken tijd.
    // Dit werkt correct, zelfs als de counter een keer is "overgelopen" (wrap-around).
    elapsed_ticks = end_tick - start_tick;
    elapsed_us = (float)elapsed_ticks / (SystemCoreClock / 1000000.0f);

    // 5. Print de resultaten in een tabel
    printf ("Channel | ADC Value\r\n");
    printf ("--------|----------\r\n");
    for (uint8_t i = 0; i < 16; i++) {
        // Print kanaal 0-F en de bijbehorende ADC-waarde
        printf ("   %02X   |    %4u   \r\n", i, adc_results[i]);
    }

    // 6. Print de timing-informatie
    printf ("----------------------------------------\r\n");
    printf ("Total scan time for 16 channels:\r\n");
    printf ("  Ticks: %lu\r\n", (long)elapsed_ticks);
    printf ("  Microseconds: %.2f us\r\n", elapsed_us);
    printf ("  Average per channel: %.2f us\r\n", elapsed_us / 16.0f);
    printf ("--- Diagnostic Scan Complete ---\r\n\r\n");
}

/*********************************************************************
 * @fn      BSP_Init
 *
 * @brief   Initialiseert alle benodigde BSP-periferie.
 *
 * @return  none
 */
void BSP_Init (void) {
    // Initialiseer alle vier de queues
    UartTxQueue_Init (&midiTxQueue);
    UartRxQueue_Init (&midiRxQueue);
    UartTxQueue_Init (&debugTxQueue);
    UartRxQueue_Init (&debugRxQueue);
    // systick_init();
    BSP_LED_Init();
    BSP_software_I2C_Init();  // set up PA10 PA11 as LOW output PP, later we flip to input
    BSP_MIDI_UART_Init();
    BSP_Debug_UART_Init();
    BSP_SPI_Init();
    BSP_Mux_Init();
    BSP_ADC_Init();
    BSP_Timer_Init();
    printf ("BSP Initialized.\r\n");
}