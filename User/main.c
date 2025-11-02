#include "bsp.h"
#include "debug.h"
#include "ed_potscan.h"
#include "ed_keyscan.h"
#include "channels.h"
#include <inttypes.h>
#include "ssd1306.h"
// declaratie van de globale state machine
ed_pot_sm_t g_pot_state = {0};
ed_keyscan_t g_keyscan_state = {0};

volatile uint32_t g_counter = 0;

void TIM3_Update_HandleIrq(void *p) {
    (void)p;
    ed_keyscan_next(&g_keyscan_state);
    ed_pot_sm_next(&g_pot_state);  // roep dit 2000x per seconde aan

    g_counter++;
    // if (g_counter == 2000) {
    //     UART_TX_Push (DEBUG_UART, 'x');
    //     g_counter = 0;
    // }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

ssd1306_handle_t dd = {.device_address = 0x3c, 0};
char buf[10];

int main(void) {

    // Initialiseer de state machine VOOR alle callbacks actief worden

    ed_pot_sm_init(&g_pot_state);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    USART_Printf_Init(115200);

    BSP_Init();

    printf("\033[20;1H");
    // while (1) {
    //     BSP_I2C_BB_Scan();
    //     Delay_Ms (1000);
    // }
    Delay_Ms(100); // nodig anders is i2c display niet klaar... 
    ssd1306_init(&dd, 128, 64, 0);
    ssd1306_poweron(&dd);
    ssd1306_fill(&dd, 1);
    ssd1306_show(&dd);
    Delay_Ms(2000);
    printf("\033[1;1H"
           "SystemClk:%" PRIu32 "\r\n",
           SystemCoreClock);
    printf("ChipID:%" PRIx32 "\r\n", DBGMCU_GetCHIPID());
    printf("GPIO Toggle TEST\r\n");
    u8 j = 0;
    u8 k = 0;
    while (1) {
        Delay_Ms(500);
        GPIO_WriteBit(GPIOA, GPIO_Pin_0,
                      (j == 0) ? (j = Bit_SET) : (j = Bit_RESET));
        //  read all adc values maximum speed
        //  storing in array
        //  printing out values in table
        //  to find out if we need delays between mux switch and adc reading
        //  and also time the whole 16 channel loop, using SysTick() calls
        //  BSP_ADC_Diagnostic_Run();
        // Check if the state machine has completed a full scan

        if (ed_pot_sm_is_scan_complete(&g_pot_state)) {
            snprintf(buf, 10, "aap %d", k++);
            ssd1306_printFixed16(&dd, 10, 10, 0, buf);
            ssd1306_show(&dd);
            printf("\033[5;1H"
                   "--- New Scan Complete ---\r\n");
            for (int i = 0; i < CHANNEL_COUNT; i++) {
                printf("%s: %4u\r\n", get_channel_name(i), g_pot_state.values[i]);
            }
            printf("%ld -------------------------\r\n", millis());

            // Reset the flag to allow the state machine to start a new scan
            g_pot_state.scan_complete = false;
        }
    }
}
