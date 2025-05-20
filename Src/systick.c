/* systick.c */
#include "systick.h"
#include "stm32f4xx.h"
#include "system_config.h"  // ADD THIS LINE

/* Global SysTick counter - increments every 1ms */
volatile uint32_t systick_counter = 0;

void SysTick_Init(void) {
    /* Configure SysTick for 1ms intervals */

    /* Load reload value for 1ms delay */
    SysTick->LOAD = (SystemCoreClock_Get() / 1000) - 1;  // CHANGE THIS LINE

    /* Rest of your code remains the same */
    SysTick->VAL = 0;

    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

// Rest of systick.c remains the same

void SysTick_Handler(void) {
    /* Increment counter every 1ms */
    systick_counter++;
}

void SysTick_Delay(uint32_t delay_ms) {
    uint32_t start_time = systick_counter;

    /* Wait until the delay time has passed */
    while ((systick_counter - start_time) < delay_ms) {
        /* Wait for the delay */
    }
}
