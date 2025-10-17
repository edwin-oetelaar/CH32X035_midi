/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32x035_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/10/28
 * Description        : Main Interrupt Service Routines.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32x035_it.h"
#include "bsp.h"

void NMI_Handler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));
void HardFault_Handler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler (void) {
    while (1) {
    }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler (void) {
    NVIC_SystemReset();
    while (1) {
    }
}

// My stuff

void USART1_IRQHandler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

void USART1_IRQHandler (void) {
    if (USART_GetITStatus (DEBUG_UART, USART_IT_TXE) != RESET) {
        UART_TX_HandleIrq (DEBUG_UART);
    }
    if (USART_GetITStatus (DEBUG_UART, USART_IT_RXNE) != RESET) {
        UART_RX_HandleIrq (DEBUG_UART);
    }
}

void USART2_IRQHandler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

void USART2_IRQHandler (void) {
    if (USART_GetITStatus (MIDI_UART, USART_IT_TXE) != RESET) {
        UART_TX_HandleIrq (MIDI_UART);
    }
    if (USART_GetITStatus (MIDI_UART, USART_IT_RXNE) != RESET) {
        UART_RX_HandleIrq (MIDI_UART);
    }
}

void TIM3_IRQHandler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

void TIM3_IRQHandler (void) {
    if (TIM_GetITStatus (TIM3, TIM_IT_Update) != RESET) {
        TIM3_Update_HandleIrq (NULL);
        TIM_ClearITPendingBit (TIM3, TIM_IT_Update);
    }
}
