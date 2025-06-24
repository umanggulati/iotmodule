/**
 * @file system_config.h
 * @brief System configuration for STM32F429ZI
 * @desc Clock configuration and system parameters
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

/* ====================== System Clock Configuration ====================== */

/* STM32F429ZI default clock configuration */
#define SYSTEM_CLOCK_FREQ       16000000    /* 16 MHz HSI (default) */

/* Alternative clock configurations */
// #define SYSTEM_CLOCK_FREQ    180000000   /* 180 MHz (max frequency with PLL) */
// #define SYSTEM_CLOCK_FREQ     84000000   /* 84 MHz (balanced performance/power) */
// #define SYSTEM_CLOCK_FREQ     48000000   /* 48 MHz (good performance, lower power) */

/* ====================== Peripheral Clock Frequencies ====================== */

/* APB1 peripherals (USART2, USART3, I2C1, etc.) */
#define APB1_CLOCK_FREQ         (SYSTEM_CLOCK_FREQ / 1)    /* Same as system clock for simplicity */

/* APB2 peripherals (USART1, SPI1, etc.) */
#define APB2_CLOCK_FREQ         (SYSTEM_CLOCK_FREQ / 1)    /* Same as system clock for simplicity */

/* AHB peripherals (GPIO, DMA, etc.) */
#define AHB_CLOCK_FREQ          (SYSTEM_CLOCK_FREQ / 1)    /* Same as system clock */

/* ====================== Memory Configuration ====================== */

/* STM32F429ZI memory sizes */
#define FLASH_SIZE              (2048 * 1024)  /* 2MB Flash */
#define SRAM_SIZE               (256 * 1024)   /* 256KB SRAM */
#define CCM_SRAM_SIZE           (64 * 1024)    /* 64KB CCM SRAM */

/* Stack and heap sizes */
#define STACK_SIZE              (8 * 1024)     /* 8KB stack */
#define HEAP_SIZE               (16 * 1024)    /* 16KB heap */

/* ====================== Function Prototypes ====================== */

/**
 * @brief Get current system clock frequency
 * @return uint32_t: System clock frequency in Hz
 */
uint32_t SystemCoreClock_Get(void);

/**
 * @brief Initialize system clock (if needed)
 * @return None
 */
void SystemClock_Config(void);

/* ====================== CMSIS Compatibility ====================== */

/* Global variable for CMSIS compatibility */
extern uint32_t SystemCoreClock;

/* ====================== Utility Macros ====================== */

/* Convert frequency to period */
#define FREQ_TO_PERIOD_US(freq)     (1000000 / (freq))
#define FREQ_TO_PERIOD_MS(freq)     (1000 / (freq))

/* Convert time to ticks */
#define MS_TO_TICKS(ms)             ((ms) * (SYSTEM_CLOCK_FREQ / 1000))
#define US_TO_TICKS(us)             ((us) * (SYSTEM_CLOCK_FREQ / 1000000))

#endif /* SYSTEM_CONFIG_H */
