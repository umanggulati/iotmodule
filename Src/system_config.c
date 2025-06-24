/**
 * @file system_config.c
 * @brief System configuration implementation for STM32F429ZI
 * @desc Clock configuration and system parameters
 */

#include "system_config.h"
#include "stm32f4xx.h"

/* ====================== Global Variables ====================== */

/* Global variable required by CMSIS */
uint32_t SystemCoreClock = SYSTEM_CLOCK_FREQ;

/* ====================== Function Implementations ====================== */

/**
 * @brief Get current system clock frequency
 * @return uint32_t: System clock frequency in Hz
 */
uint32_t SystemCoreClock_Get(void) {
    return SystemCoreClock;
}

/**
 * @brief Initialize system clock (optional - currently using default HSI)
 * @return None
 * @note This function can be expanded to configure PLL for higher frequencies
 */
void SystemClock_Config(void) {
    /* Currently using default HSI clock (16 MHz) */
    /* This function can be expanded to configure PLL for higher performance */

    /* Example PLL configuration for 180 MHz (commented out for stability):
     *
     * // Enable HSE
     * RCC->CR |= RCC_CR_HSEON;
     * while (!(RCC->CR & RCC_CR_HSERDY));
     *
     * // Configure PLL
     * RCC->PLLCFGR = (RCC_PLLCFGR_PLLSRC_HSE |
     *                 (8 << RCC_PLLCFGR_PLLM_Pos) |      // PLLM = 8
     *                 (360 << RCC_PLLCFGR_PLLN_Pos) |    // PLLN = 360
     *                 (0 << RCC_PLLCFGR_PLLP_Pos) |      // PLLP = 2
     *                 (7 << RCC_PLLCFGR_PLLQ_Pos));      // PLLQ = 7
     *
     * // Enable PLL
     * RCC->CR |= RCC_CR_PLLON;
     * while (!(RCC->CR & RCC_CR_PLLRDY));
     *
     * // Configure Flash latency
     * FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
     *
     * // Configure AHB, APB1, APB2 prescalers
     * RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 |   // AHB = SYSCLK
     *               RCC_CFGR_PPRE1_DIV4 |   // APB1 = SYSCLK/4 (45 MHz)
     *               RCC_CFGR_PPRE2_DIV2);   // APB2 = SYSCLK/2 (90 MHz)
     *
     * // Switch to PLL
     * RCC->CFGR |= RCC_CFGR_SW_PLL;
     * while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
     *
     * // Update SystemCoreClock
     * SystemCoreClock = 180000000;
     */

    /* For now, we're using the default HSI configuration for simplicity and stability */
    SystemCoreClock = SYSTEM_CLOCK_FREQ;
}
