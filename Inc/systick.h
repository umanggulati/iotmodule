/* @systick.h */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/* Global SysTick counter - increments every 1ms */
extern volatile uint32_t systick_counter;

/**
 * @brief Initialize SysTick timer for 1ms intervals
 * @param None
 * @return None
 */
void SysTick_Init(void);

/**
 * @brief Get millisecond delay using SysTick
 * @param delay_ms: Delay in milliseconds
 * @return None
 */
void SysTick_Delay(uint32_t delay_ms);

#endif /* SYSTICK_H */
