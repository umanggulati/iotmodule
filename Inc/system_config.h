/* system_config.h */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

/* Current Clock Configuration - HSI 16MHz (default after reset) */
#define SYSTEM_CLOCK_FREQ   16000000U

/* Function to get system clock frequency */
uint32_t SystemCoreClock_Get(void);

#endif /* SYSTEM_CONFIG_H */
