/* system_config.c */
#include "system_config.h"

/* Global variable required by CMSIS */
uint32_t SystemCoreClock = SYSTEM_CLOCK_FREQ;

/* Simple function to get current system clock */
uint32_t SystemCoreClock_Get(void) {
    return SystemCoreClock;
}
