#include "bsp.h"
#include "debug.h"
#include "ed_pot_sm.h"

#include <inttypes.h>

// Externe declaratie van de globale state machine
extern ed_pot_sm_t global_pot_sm;


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main (void) {
    u8 j = 0;
// Initialiseer de state machine
    ed_pot_sm_init(&global_pot_sm);

    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
   
    USART_Printf_Init (115200);
    BSP_Init();

    printf ("SystemClk:%" PRIu32 "\r\n", SystemCoreClock);
    printf ("ChipID:%" PRIx32 "\r\n", DBGMCU_GetCHIPID());
    printf ("GPIO Toggle TEST\r\n");

    while (1) {
        Delay_Ms (500);
        GPIO_WriteBit (GPIOA, GPIO_Pin_0, (j == 0) ? (j = Bit_SET) : (j = Bit_RESET));
        // read all adc values maximum speed
        // storing in array 
        // printing out values in table
        // to find out if we need delays between mux switch and adc reading
        // and also time the whole 16 channel loop, using SysTick() calls
       //  BSP_ADC_Diagnostic_Run();
        // Check if the state machine has completed a full scan
        if (ed_pot_sm_is_scan_complete(&global_pot_sm))
        {
            printf("--- New Scan Complete ---\r\n");
            for (int i = 0; i < 16; i++) {
                printf("CH %02d: %4u\r\n", i, global_pot_sm.values[i]);
            }
            printf("-------------------------\r\n");

            // Reset the flag to allow the state machine to start a new scan
            global_pot_sm.scan_complete = false;
        }       
    }
}

