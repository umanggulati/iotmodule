/**
 * @file power_management.c
 * @brief Power management implementation for STM32F429ZI
 * @desc Direct power control for IoT applications
 */

#include "power_management.h"
#include "stm32f4xx.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>

/* ====================== Static Variables ====================== */

/* Current power configuration */
static PowerConfig_t currentConfig;

/* Last detected wake-up source */
static WakeupSource_t lastWakeupSource = WAKEUP_SOURCE_NONE;

/* Current power mode */
static PowerMode_t currentPowerMode = POWER_MODE_RUN;

/* ====================== Private Function Prototypes ====================== */

static void ConfigureWakeupPinPA0(bool enable);
static bool ConfigureRTCWakeupSources(WakeupSource_t sources);
static bool ConfigureRTCWakeupTimer(uint32_t seconds);
static void EnablePVD(bool enable, PVDLevel_t level);
static bool IsVoltageScalingReady(void);
static void ClearWakeupFlags(void);
static void EnterSleepMode(void);
static void EnterStopMode(bool lowPowerRegulator, bool flashPowerDown);
static void EnterStandbyMode(void);
static bool ConfigureVoltageScaling(VoltageScale_t scale);
static WakeupSource_t DetermineWakeupSource(void);

/* ====================== Function Implementations ====================== */

/**
 * @brief Initialize power management subsystem
 */
void PowerMgmt_Init(void) {
    /* Enable power interface clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Set default configuration */
    memset(&currentConfig, 0, sizeof(PowerConfig_t));
    currentConfig.mode = POWER_MODE_RUN;
    currentConfig.voltageScale = VOLTAGE_SCALE_1; /* Highest performance by default */
    currentConfig.flashPowerDown = false;
    currentConfig.enableWakeupPin = false;
    currentConfig.enableBackupRegulator = false;
    currentConfig.enablePVD = false;
    currentConfig.pvdLevel = PVD_LEVEL_2V7;
    currentConfig.wakeupSources = WAKEUP_SOURCE_NONE;

    /* Apply default voltage scaling */
    ConfigureVoltageScaling(currentConfig.voltageScale);

    /* Debug message */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[POWER] Power management initialized, voltage scale: %d\r\n",
             (int)currentConfig.voltageScale);
    UART_SendString(debug_msg);
}

/**
 * @brief Configure power management settings
 */
bool PowerMgmt_Configure(const PowerConfig_t* config) {
    if (config == NULL) {
        return false;
    }

    /* Update configuration */
    memcpy(&currentConfig, config, sizeof(PowerConfig_t));

    /* Configure voltage scaling */
    if (!ConfigureVoltageScaling(config->voltageScale)) {
        UART_SendString("[POWER] Failed to set voltage scaling\r\n");
        return false;
    }

    /* Configure wake-up pin */
    ConfigureWakeupPinPA0(config->enableWakeupPin);

    /* Configure wake-up sources */
    if (!PowerMgmt_ConfigureWakeupSources(config->wakeupSources)) {
        UART_SendString("[POWER] Failed to configure wake-up sources\r\n");
        return false;
    }

    /* Configure PVD */
    PowerMgmt_ConfigurePVD(config->enablePVD, config->pvdLevel);

    /* Configure backup regulator if needed */
    if (config->enableBackupRegulator) {
        /* Enable backup domain access */
        PowerMgmt_EnableBackupAccess(true);

        /* Enable backup regulator */
        PWR->CSR |= PWR_CSR_BRE;

        /* Wait for backup regulator ready flag */
        while (!(PWR->CSR & PWR_CSR_BRR));
    } else {
        /* Disable backup regulator */
        PWR->CSR &= ~PWR_CSR_BRE;
    }

    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[POWER] Configuration updated: mode=%d, voltageScale=%d\r\n",
             (int)currentConfig.mode, (int)currentConfig.voltageScale);
    UART_SendString(debug_msg);

    return true;
}

/**
 * @brief Enter specified power mode
 */
/**
 * @brief Enter specified power mode
 * @param mode: Power mode to enter
 * @return None
 * @note Function may not return if entering Standby mode
 */
void PowerMgmt_EnterLowPowerMode(PowerMode_t mode) {
    /* Store current mode */
    currentPowerMode = mode;

    /* Clear all wake-up flags before entering low-power mode */
    ClearWakeupFlags();

    /* Enter specific power mode */
    switch (mode) {
        case POWER_MODE_RUN:
            /* Already in Run mode, nothing to do */
            break;

        case POWER_MODE_RUN_LP:
            /* Reduce system clock frequency for lower power */
            /* This would modify RCC->CFGR to adjust clock dividers */
            UART_SendString("[POWER] Entering Run Low-Power mode\r\n");
            break;

        case POWER_MODE_SLEEP:
            UART_SendString("[POWER] Entering Sleep mode\r\n");
            EnterSleepMode();

            /* Add this after EnterSleepMode returns - early confirmation of wake-up */
            UART_SendString("[POWER] Initial wake-up from Sleep mode detected\r\n");
            break;

        case POWER_MODE_STOP_MR:
            UART_SendString("[POWER] Entering Stop mode (Main Regulator)\r\n");
            EnterStopMode(false, false);

            /* Add this after EnterStopMode returns - early confirmation of wake-up */
            UART_SendString("[POWER] Initial wake-up from Stop mode detected\r\n");
            break;

        case POWER_MODE_STOP_LP:
            UART_SendString("[POWER] Entering Stop mode (Low-Power Regulator)\r\n");
            EnterStopMode(true, false);

            /* Add this after EnterStopMode returns - early confirmation of wake-up */
            UART_SendString("[POWER] Initial wake-up from Stop mode detected\r\n");
            break;

        case POWER_MODE_STOP_LP_FPD:
            UART_SendString("[POWER] Entering Stop mode (Low-Power Regulator with Flash Power-Down)\r\n");
            EnterStopMode(true, true);

            /* Add this after EnterStopMode returns - early confirmation of wake-up */
            UART_SendString("[POWER] Initial wake-up from Stop mode detected\r\n");
            break;

        case POWER_MODE_STANDBY:
            UART_SendString("[POWER] Entering Standby mode\r\n");
            EnterStandbyMode();
            /* Note: Should not return from Standby mode unless there's a reset */
            break;

        default:
            UART_SendString("[POWER] Invalid power mode\r\n");
            break;
    }

    /* After wake-up */
    if (mode != POWER_MODE_RUN && mode != POWER_MODE_RUN_LP) {
        /* Determine the wake-up source */
        lastWakeupSource = DetermineWakeupSource();

        /* Reset to RUN mode */
        currentPowerMode = POWER_MODE_RUN;

        /* Enhanced debug reporting */
        char debug_msg[150];
        snprintf(debug_msg, sizeof(debug_msg),
                 "[POWER] Complete wake-up from low-power mode, source: 0x%02X\r\n"
                 "[POWER] RTC_ISR=0x%08lX, EXTI_PR=0x%08lX, PWR_CSR=0x%08lX\r\n",
                 (unsigned int)lastWakeupSource, RTC->ISR, EXTI->PR, PWR->CSR);
        UART_SendString(debug_msg);
    }
}

/**
 * @brief Set voltage scaling
 */
bool PowerMgmt_SetVoltageScale(VoltageScale_t scale) {
    return ConfigureVoltageScaling(scale);
}

/**
 * @brief Configure wake-up sources
 */
bool PowerMgmt_ConfigureWakeupSources(WakeupSource_t sources) {
    /* Configure wake-up pin */
    ConfigureWakeupPinPA0((sources & WAKEUP_SOURCE_PIN) != 0);

    /* Configure RTC wake-up sources */
    bool result = ConfigureRTCWakeupSources(sources);

    /* CRITICAL: Additional explicit NVIC and EXTI configuration for RTC wakeup */
    if (sources & WAKEUP_SOURCE_RTC_WAKEUP) {
        /* Force-configure EXTI Line 22 (RTC Wake-up) for interrupt */
        EXTI->IMR |= (1 << 22);   /* Unmask interrupt */
        EXTI->RTSR |= (1 << 22);  /* Rising trigger */
        EXTI->FTSR &= ~(1 << 22); /* Disable falling trigger */

        /* Clear any pending EXTI line 22 interrupt */
        EXTI->PR = (1 << 22);

        /* Force-enable the RTC WKUP IRQ in the NVIC */
        NVIC_SetPriority(RTC_WKUP_IRQn, 0);
        NVIC_EnableIRQ(RTC_WKUP_IRQn);

        UART_SendString("[POWER] RTC wake-up EXTI and NVIC explicitly configured\r\n");
    }

    return result;
}

/**
 * @brief Set wake-up timer for Stop mode
 */
bool PowerMgmt_SetWakeupTimer(uint32_t seconds) {
    return ConfigureRTCWakeupTimer(seconds);
}

/**
 * @brief Enable/disable the programmable voltage detector (PVD)
 */
void PowerMgmt_ConfigurePVD(bool enable, PVDLevel_t level) {
    EnablePVD(enable, level);
}

/**
 * @brief Get last wake-up source after exiting low-power mode
 */
WakeupSource_t PowerMgmt_GetLastWakeupSource(void) {
    return lastWakeupSource;
}

/**
 * @brief Get current power consumption estimate
 * @note This is a very rough estimate based on datasheet typical values
 */
uint32_t PowerMgmt_GetPowerConsumptionEstimate(void) {
    uint32_t estimate = 0;

    /* Base consumption depends on power mode */
    switch (currentPowerMode) {
        case POWER_MODE_RUN:
            /* Run mode at max frequency ~100-150mA */
            estimate = 120000; /* 120mA in microamps */
            break;

        case POWER_MODE_RUN_LP:
            /* Run mode at reduced frequency ~30-80mA */
            estimate = 50000; /* 50mA */
            break;

        case POWER_MODE_SLEEP:
            /* Sleep mode ~10-25mA */
            estimate = 15000; /* 15mA */
            break;

        case POWER_MODE_STOP_MR:
            /* Stop mode with main regulator ~100-500µA */
            estimate = 250; /* 250µA */
            break;

        case POWER_MODE_STOP_LP:
        case POWER_MODE_STOP_LP_FPD:
            /* Stop mode with low-power regulator ~30-100µA */
            estimate = 50; /* 50µA */
            break;

        case POWER_MODE_STANDBY:
            /* Standby mode ~2-3µA */
            estimate = 3; /* 3µA */
            break;

        default:
            estimate = 0;
            break;
    }

    /* Adjust based on voltage scale */
    switch (currentConfig.voltageScale) {
        case VOLTAGE_SCALE_1:
            /* No adjustment, base estimate is for Scale 1 */
            break;

        case VOLTAGE_SCALE_2:
            /* ~20% reduction */
            estimate = (estimate * 80) / 100;
            break;

        case VOLTAGE_SCALE_3:
            /* ~40% reduction */
            estimate = (estimate * 60) / 100;
            break;
    }

    return estimate;
}

/**
 * @brief Enable backup domain access (RTC, backup registers)
 */
void PowerMgmt_EnableBackupAccess(bool enable) {
    if (enable) {
        /* Enable power interface clock if not already enabled */
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;

        /* Enable access to backup domain */
        PWR->CR |= PWR_CR_DBP;

        /* Wait for backup domain access */
        while (!(PWR->CR & PWR_CR_DBP));
    } else {
        /* Disable access to backup domain */
        PWR->CR &= ~PWR_CR_DBP;
    }
}

/**
 * @brief Clear wake-up flags
 */
void PowerMgmt_ClearWakeupFlags(void) {
    ClearWakeupFlags();
}

/**
 * @brief Get power mode status
 */
PowerMode_t PowerMgmt_GetCurrentMode(void) {
    return currentPowerMode;
}

/* ====================== Private Functions ====================== */

/**
 * @brief Configure PA0 as wake-up pin
 */
static void ConfigureWakeupPinPA0(bool enable) {
    if (enable) {
        /* Enable the wake-up pin functionality */
        PWR->CSR |= PWR_CSR_EWUP;
    } else {
        /* Disable the wake-up pin functionality */
        PWR->CSR &= ~PWR_CSR_EWUP;
    }
}

/**
 * @brief Configure RTC-based wake-up sources
 */
static bool ConfigureRTCWakeupSources(WakeupSource_t sources) {
    /* Enable backup domain access */
    PowerMgmt_EnableBackupAccess(true);

    /* Enable RTC clock if any RTC wake-up source is selected */
    if (sources & (WAKEUP_SOURCE_RTC_ALARM | WAKEUP_SOURCE_RTC_WAKEUP |
                  WAKEUP_SOURCE_RTC_TAMPER | WAKEUP_SOURCE_RTC_TIMESTAMP)) {

        /* Enable RTC clock (use LSI as clock source for simplicity) */
        RCC->CSR |= RCC_CSR_LSION; /* Enable LSI */
        while (!(RCC->CSR & RCC_CSR_LSIRDY)); /* Wait until LSI is ready */

        /* Select LSI as RTC clock source */
        RCC->BDCR &= ~RCC_BDCR_RTCSEL; /* Clear RTCSEL bits */
        RCC->BDCR |= RCC_BDCR_RTCSEL_1; /* Select LSI as RTC clock */

        /* Enable RTC clock */
        RCC->BDCR |= RCC_BDCR_RTCEN;
    }

    /* Configure EXTI lines for RTC wake-up sources */

    /* RTC Alarm (EXTI Line 17) */
    if (sources & WAKEUP_SOURCE_RTC_ALARM) {
        /* Configure EXTI Line 17 to be triggered by a rising edge */
        EXTI->RTSR |= (1 << 17); /* Rising trigger */
        EXTI->IMR |= (1 << 17);  /* Unmask interrupt */
    } else {
        EXTI->IMR &= ~(1 << 17); /* Mask interrupt */
    }

    /* RTC Tamper and Timestamp (EXTI Line 21) */
    if (sources & (WAKEUP_SOURCE_RTC_TAMPER | WAKEUP_SOURCE_RTC_TIMESTAMP)) {
        /* Configure EXTI Line 21 to be triggered by a rising edge */
        EXTI->RTSR |= (1 << 21); /* Rising trigger */
        EXTI->IMR |= (1 << 21);  /* Unmask interrupt */
    } else {
        EXTI->IMR &= ~(1 << 21); /* Mask interrupt */
    }

    /* RTC Wake-up (EXTI Line 22) */
    if (sources & WAKEUP_SOURCE_RTC_WAKEUP) {
        /* Configure EXTI Line 22 to be triggered by a rising edge */
        EXTI->RTSR |= (1 << 22); /* Rising trigger */
        EXTI->IMR |= (1 << 22);  /* Unmask interrupt */
    } else {
        EXTI->IMR &= ~(1 << 22); /* Mask interrupt */
    }

    return true;
}

/**
 * @brief Configure RTC wake-up timer with specified time period
 */
static bool ConfigureRTCWakeupTimer(uint32_t seconds) {
    /* Enable backup domain access */
    PowerMgmt_EnableBackupAccess(true);

    /* Enable RTC clock if not already enabled */
    if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
        /* Enable LSI */
        RCC->CSR |= RCC_CSR_LSION;
        while (!(RCC->CSR & RCC_CSR_LSIRDY)); /* Wait until LSI is ready */

        /* Select LSI as RTC clock source */
        RCC->BDCR &= ~RCC_BDCR_RTCSEL;
        RCC->BDCR |= RCC_BDCR_RTCSEL_1; /* Select LSI as RTC clock (01) */

        /* Enable RTC clock */
        RCC->BDCR |= RCC_BDCR_RTCEN;

        /* Reset RTC domain */
        RCC->BDCR |= RCC_BDCR_BDRST;
        RCC->BDCR &= ~RCC_BDCR_BDRST;
    }

    /* Disable RTC write protection */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Enter initialization mode */
    RTC->ISR |= RTC_ISR_INIT;
    while (!(RTC->ISR & RTC_ISR_INITF)); /* Wait until init flag is set */

    /* Configure RTC prescaler for LSI (roughly 32.768 kHz) */
    RTC->PRER = (127 << 16) | 255; /* Sync and async prescalers for ~1 Hz */

    /* Exit initialization mode */
    RTC->ISR &= ~RTC_ISR_INIT;

    /* Disable wake-up timer to configure it */
    RTC->CR &= ~RTC_CR_WUTE;

    /* Wait for wake-up timer write flag to be set */
    uint32_t timeout = 100000;
    while (!(RTC->ISR & RTC_ISR_WUTWF)) {
        if (timeout-- == 0) {
            return false;
        }
    }

    /* Set wake-up auto-reload value */
    /* LSI frequency is typically ~32 kHz */
    /* With WUCKSEL=4 (divider = 16), each count represents 0.5ms (approximately) */
    RTC->WUTR = seconds - 1; /* Convert seconds to number of 0.5ms periods */

    /* Select auto wake-up clock source */
    RTC->CR &= ~RTC_CR_WUCKSEL;
    RTC->CR |= (0x4 << RTC_CR_WUCKSEL_Pos); /* Select ck_spre (1Hz) with divider */

    RTC->ISR &= ~(RTC_ISR_WUTF | RTC_ISR_ALRAF | RTC_ISR_ALRBF | RTC_ISR_TSF | RTC_ISR_TSOVF);
    /* Clear EXTI line 22 pending bit as well */
    EXTI->PR = (1 << 22);


    /* Enable wake-up timer and its interrupt */
    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;

    /* Clear wake-up timer flag */
    RTC->ISR &= ~RTC_ISR_WUTF;

    /* Re-enable RTC write protection */
    RTC->WPR = 0xFF;

    /* Configure EXTI Line 22 (RTC Wake-up) for event detection */
    EXTI->IMR |= (1 << 22);  /* Unmask interrupt */
    EXTI->RTSR |= (1 << 22); /* Rising trigger */
    EXTI->FTSR &= ~(1 << 22); /* Disable falling trigger */

    /* Clear any pending EXTI line 22 interrupt */
    EXTI->PR = (1 << 22);

    /* Enable the RTC WKUP IRQ in the NVIC */
    NVIC_SetPriority(RTC_WKUP_IRQn, 0);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);

    UART_SendString("[POWER] RTC wake-up timer configured for wake-up in ");
    char debug_msg[10];
    snprintf(debug_msg, sizeof(debug_msg), "%lds\r\n", seconds);
    UART_SendString(debug_msg);

    return true;
}
static void EnablePVD(bool enable, PVDLevel_t level) {
    /* Configure PVD level */
    PWR->CR &= ~PWR_CR_PLS; /* Clear PLS bits */
    PWR->CR |= (level << PWR_CR_PLS_Pos); /* Set new PVD level */

    /* Enable/disable PVD */
    if (enable) {
        PWR->CR |= PWR_CR_PVDE;

        /* Configure EXTI Line 16 (PVD output) */
        EXTI->IMR |= (1 << 16);  /* Unmask interrupt */
        EXTI->RTSR |= (1 << 16); /* Rising trigger */
        EXTI->FTSR |= (1 << 16); /* Falling trigger */
    } else {
        PWR->CR &= ~PWR_CR_PVDE;
        EXTI->IMR &= ~(1 << 16); /* Mask interrupt */
    }
}

/**
 * @brief Check if voltage scaling is ready
 */
static bool IsVoltageScalingReady(void) {
    uint32_t timeout = 10000; /* Arbitrary timeout value */

    while (!(PWR->CSR & PWR_CSR_VOSRDY)) {
        if (timeout-- == 0) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Clear all wake-up flags
 */
static void ClearWakeupFlags(void) {
    /* Clear PWR wake-up flags */
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

    /* Clear EXTI pending bits */
    EXTI->PR = 0xFFFFFFFF;
}

/**
 * @brief Enter Sleep mode
 */
static void EnterSleepMode(void) {
    /* Clear SLEEPDEEP bit to enter Sleep mode instead of Stop mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* WFI instruction will cause the processor to enter Sleep mode */
    __WFI();
}

/**
 * @brief Enter Stop mode
 */
static void EnterStopMode(bool lowPowerRegulator, bool flashPowerDown) {
    /* Configure low-power regulator in Stop mode if requested */
    if (lowPowerRegulator) {
        PWR->CR |= PWR_CR_LPDS;
    } else {
        PWR->CR &= ~PWR_CR_LPDS;
    }

    /* Configure flash power-down in Stop mode if requested */
    if (flashPowerDown) {
        PWR->CR |= PWR_CR_FPDS;
    } else {
        PWR->CR &= ~PWR_CR_FPDS;
    }

    /* Clear PDDS bit to enter Stop mode */
    PWR->CR &= ~PWR_CR_PDDS;

    /* CRITICAL: Make sure RTC wake-up interrupt is properly set up */
    EXTI->IMR |= (1 << 22);   /* Unmask interrupt */
    EXTI->RTSR |= (1 << 22);  /* Rising trigger */
    EXTI->PR = (1 << 22);     /* Clear any pending interrupt */
    NVIC_EnableIRQ(RTC_WKUP_IRQn);

    /* Set SLEEPDEEP bit to enter Stop mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Add message right before entering stop mode */
    UART_SendString("[POWER] About to execute WFI instruction to enter Stop mode\r\n");

    /* Small delay to ensure UART transmission completes */
    for(volatile int i = 0; i < 10000; i++);

    /* Wait for any interrupt to wake up from Stop mode */
    __WFI();

    /* ---- Post wake-up code starts here ---- */

    /* Toggle LED immediately after wake-up as a visual indicator */
    GPIOB->ODR ^= (1 << 7); /* Toggle blue LED on PB7 */

    /* Clear SLEEPDEEP bit after waking up */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* If using HSE or PLL before Stop mode, you'd reconfigure it here */
    /* For now, we assume HSI is being used */

    /* Note: After Stop mode exit, system uses HSI clock
       If you were using a different clock source before,
       you would need to reconfigure it here */
}
static void EnterStandbyMode(void) {
    /* Set PDDS bit to enter Standby mode */
    PWR->CR |= PWR_CR_PDDS;

    /* Set SLEEPDEEP bit to enter deep sleep */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* WFI instruction will cause the processor to enter Standby mode */
    __WFI();

    /* The MCU will reset after waking up from Standby mode, so execution will not reach here */
}

/**
 * @brief Configure voltage scaling
 */
static bool ConfigureVoltageScaling(VoltageScale_t scale) {
    uint32_t vos_bits;

    /* Determine VOS bits based on scale */
    switch (scale) {
        case VOLTAGE_SCALE_1:
            vos_bits = PWR_CR_VOS; /* Both bits set for Scale 1 (11) */
            break;
        case VOLTAGE_SCALE_2:
            vos_bits = PWR_CR_VOS_1; /* Only bit 1 set for Scale 2 (10) */
            break;
        case VOLTAGE_SCALE_3:
            vos_bits = PWR_CR_VOS_0; /* Only bit 0 set for Scale 3 (01) */
            break;
        default:
            return false;
    }

    /* Update voltage scaling bits */
    PWR->CR &= ~PWR_CR_VOS; /* Clear VOS bits */
    PWR->CR |= vos_bits;    /* Set new VOS bits */

    /* Wait for voltage scaling to complete */
    return IsVoltageScalingReady();
}

/**
 * @brief Determine the wake-up source after waking from low-power mode
 */
static WakeupSource_t DetermineWakeupSource(void) {
    WakeupSource_t source = WAKEUP_SOURCE_NONE;
    char debug_msg[100];

    /* Debug output of relevant registers */
    snprintf(debug_msg, sizeof(debug_msg),
             "[POWER] Wake-up debugging: PWR_CSR=0x%08lX, EXTI_PR=0x%08lX, RTC_ISR=0x%08lX\r\n",
             PWR->CSR, EXTI->PR, RTC->ISR);
    UART_SendString(debug_msg);

    /* Check wake-up flag in PWR_CSR */
    if (PWR->CSR & PWR_CSR_WUF) {
        /* Check if wake-up was caused by PA0 (wake-up pin) */
        if (PWR->CSR & PWR_CSR_EWUP) {
            source |= WAKEUP_SOURCE_PIN;
        }
    }

    /* Check EXTI pending register for RTC-related wake-up even if PWR_CSR_WUF isn't set */
    /* RTC Wake-up (EXTI Line 22) */
    if (EXTI->PR & (1 << 22)) {
        source |= WAKEUP_SOURCE_RTC_WAKEUP;
    }

    /* RTC Alarm (EXTI Line 17) */
    if (EXTI->PR & (1 << 17)) {
        source |= WAKEUP_SOURCE_RTC_ALARM;
    }

    /* RTC Tamper and Timestamp (EXTI Line 21) */
    if (EXTI->PR & (1 << 21)) {
        /* Need to check RTC flags to distinguish between tamper and timestamp */
        source |= (WAKEUP_SOURCE_RTC_TAMPER | WAKEUP_SOURCE_RTC_TIMESTAMP);
    }

    /* If no source was identified via PWR_CSR or EXTI, check RTC registers directly */
    if (source == WAKEUP_SOURCE_NONE) {
        if (RTC->ISR & RTC_ISR_WUTF) {
            source |= WAKEUP_SOURCE_RTC_WAKEUP;
            UART_SendString("[POWER] Wake-up source identified from RTC_ISR directly\r\n");
        }
    }

    return source;
}
/* Add this to your power_management.c file */
void RTC_WKUP_IRQHandler(void) {
    /* Toggle a debug GPIO pin (one of your LEDs) - This confirms IRQ is being called
       even if UART isn't working yet */
    /* Green LED on PB0 */
    GPIOB->ODR ^= (1 << 0);  /* Toggle PB0 */

    /* Your existing debug output */
    UART_SendString("\r\n[IRQ] RTC_WKUP_IRQHandler entered\r\n");

    /* Rest of your handler code remains the same */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[IRQ] RTC_ISR=0x%08lX, EXTI_PR=0x%08lX\r\n",
             RTC->ISR, EXTI->PR);
    UART_SendString(debug_msg);

    /* Check if RTC wake-up interrupt occurred */
    if (RTC->ISR & RTC_ISR_WUTF) {
        UART_SendString("[IRQ] RTC Wake-up flag detected! Clearing flags...\r\n");

        /* Clear wake-up flag in RTC */
        RTC->ISR &= ~RTC_ISR_WUTF;

        /* Clear EXTI line 22 flag - CRITICAL for exiting Stop mode */
        EXTI->PR = (1 << 22);

        snprintf(debug_msg, sizeof(debug_msg),
                 "[IRQ] Flags cleared - RTC_ISR=0x%08lX, EXTI_PR=0x%08lX\r\n",
                 RTC->ISR, EXTI->PR);
        UART_SendString(debug_msg);
    } else {
        UART_SendString("[IRQ] Wake-up flag NOT detected! Check configuration.\r\n");
    }
}
/* Add a PendSV handler to help with wake-up */
void PendSV_Handler(void) {
    /* This handler is triggered by software in the RTC wake-up handler */
    /* It provides an additional mechanism to wake up the CPU */
    UART_SendString("[IRQ] PendSV_Handler - Additional wake-up mechanism\r\n");
}
