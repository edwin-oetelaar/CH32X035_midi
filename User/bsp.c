#include "bsp.h"
#include "ed_pot_sm.h"

// --- Definitie van de Globale Queues ---
UartTxQueue_t midiTxQueue;
UartRxQueue_t midiRxQueue;
UartTxQueue_t debugTxQueue;
UartRxQueue_t debugRxQueue;

//globale state
ed_pot_sm_t global_pot_sm = {0};
volatile uint32_t counter = 0;

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
    uint8_t received_byte = USART_ReceiveData (uart);
    UartRxQueue_Push (queue, received_byte);
}


void TIM3_Update_HandleIrq (void *p) {
    (void)p;
    ed_pot_sm_next (&global_pot_sm);  // roep dit 2000x per seconde aan
    
    counter++;
    if (counter == 2000) {
        UART_TX_Push (DEBUG_UART, 'x');
        counter = 0;
    }
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

/*********************************************************************
 * @brief   Initializes the GPIO pins for the analog multiplexer selectors.
 * @note    Configures PA15, PA16, PA17, and PA18 as push-pull outputs.
 *          Sets all pins to a known initial state (low) to select channel 0.
 * @return  none
 */
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

/*********************************************************************
 * @brief   Initializes I2C1 for communication with peripherals.
 * @note    Configures I2C1 on PA13 (SCL) and PA14 (SDA) for a standard
 *          100kHz clock speed. The GPIO pins are configured in
 *          Open-Drain mode as required by the I2C specification.
 * @return  none
 */
void BSP_I2C_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitStructure = {0};

    // 1. Enable the peripheral clocks
    // Enable clock for GPIOA (for SCL and SDA pins)
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);
    // Enable clock for I2C1
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_I2C1, ENABLE);

    // 2. Configure the I2C GPIO pins (PA13-SCL, PA14-SDA)
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN | I2C_SDA_GPIO_PIN;
    // I2C requires Open-Drain configuration. External pull-up resistors are mandatory.
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // is dit OK, is dit Open Drain, hoe werkt dat?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    // 3. Configure the I2C peripheral itself
    // I2C clock speed. 100000 is a standard 100kHz speed.
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    // Set I2C mode to I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    // Duty cycle for fast mode (not critical for standard mode)
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    // The device's own address on the I2C bus. 0x00 is fine for a master-only device.
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    // Enable Acknowledgement (ACK) after receiving a byte
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    // Specify 7-bit or 10-bit addressing for own address
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    // Apply the configuration to I2C1
    I2C_Init (I2C_PORT, &I2C_InitStructure);

    // 4. Enable the I2C peripheral
    I2C_Cmd (I2C_PORT, ENABLE);
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

    BSP_LED_Init();
    BSP_I2C_Init();
    BSP_MIDI_UART_Init();
    BSP_Debug_UART_Init();
    BSP_SPI_Init();
    BSP_Mux_Init();
    BSP_ADC_Init();
    BSP_Timer_Init();
    printf ("BSP Initialized.\r\n");
}