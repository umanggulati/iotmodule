/**
 * @file power_management_fixed.c
 * @brief FIXED Power management implementation for STM32F429ZI
 * @desc Robust implementation with proper error handling and timeouts
 */

#include "power_management.h"
#include "stm32f4xx.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>

/* ====================== Configuration Constants ====================== */
#define PWR_DEBUG                   1       /* Enable detailed debug output */
#define PWR_TIMEOUT_STANDARD        1000    /* Standard timeout in ms */
#define PWR_TIMEOUT_RTC_INIT        5000    /* RTC init timeout in ms */
#define PWR_TIMEOUT_LSI_READY       10000   /* LSI ready timeout in ms */
#define PWR_MAX_RETRIES            3        /* Max retry attempts */

/* ====================== Static Variables ====================== */
static PowerConfig_t currentConfig;
static WakeupSource_t lastWakeupSource = WAKEUP_SOURCE_NONE;
static PowerMode_t currentPowerMode = POWER_MODE_RUN;
static bool powerMgmtInitialized = false;

/* ====================== External Dependencies ====================== */
extern volatile uint32_t systick_counter;  /* From systick.c */

/* ====================== Private Function Prototypes ====================== */
static bool PWR_WaitForCondition(volatile uint32_t* reg, uint32_t mask, uint32_t expected, uint32_t timeout_ms, const char* description);
static bool PWR_InitializeLSI(void);
static bool PWR_InitializeRTC(void);
static bool PWR_ConfigureRTCWakeupTimerSafe(uint32_t seconds);
static bool PWR_VerifyRTCConfiguration(void);
static void PWR_DebugRTCStatus(void);
static void PWR_ClearAllRTCFlags(void);

/* ====================== Debug Helper Macros ====================== */
#if PWR_DEBUG
#define PWR_LOG(msg) UART_SendString("[PWR] " msg "\r\n")
#define PWR_LOG_FMT(fmt, ...) do { \
    char debug_buf[128]; \
    snprintf(debug_buf, sizeof(debug_buf), "[PWR] " fmt "\r\n", __VA_ARGS__); \
    UART_SendString(debug_buf); \
} while(0)
#else
#define PWR_LOG(msg)
#define PWR_LOG_FMT(fmt, ...)
#endif

/* ====================== Robust Helper Functions ====================== */

/**
 * @brief Wait for a register condition with timeout and detailed logging
 */
static bool PWR_WaitForCondition(volatile uint32_t* reg, uint32_t mask, uint32_t expected,
                                uint32_t timeout_ms, const char* description) {
    uint32_t start_time = systick_counter;
    uint32_t last_log_time = start_time;

    PWR_LOG_FMT("Waiting for %s (reg=0x%08lX, mask=0x%08lX, expected=0x%08lX)",
                description, (uint32_t)reg, mask, expected);

    while ((*reg & mask) != expected) {
        uint32_t current_time = systick_counter;

        /* Log progress every second */
        if ((current_time - last_log_time) > 1000) {
            PWR_LOG_FMT("%s: Still waiting... (current=0x%08lX, elapsed=%lums)",
                        description, *reg & mask, current_time - start_time);
            last_log_time = current_time;
        }

        /* Check timeout */
        if ((current_time - start_time) > timeout_ms) {
            PWR_LOG_FMT("%s: TIMEOUT after %lums (final=0x%08lX)",
                        description, current_time - start_time, *reg & mask);
            return false;
        }
    }

    PWR_LOG_FMT("%s: SUCCESS after %lums", description, systick_counter - start_time);
    return true;
}

/**
 * @brief Safely initialize LSI clock
 */
static bool PWR_InitializeLSI(void) {
    PWR_LOG("Initializing LSI clock...");

    /* Check if LSI is already running */
    if (RCC->CSR & RCC_CSR_LSIRDY) {
        PWR_LOG("LSI already running and ready");
        return true;
    }

    /* Enable LSI */
    RCC->CSR |= RCC_CSR_LSION;
    PWR_LOG("LSI enable bit set, waiting for ready flag...");

    /* Wait for LSI to be ready with timeout */
    if (!PWR_WaitForCondition(&RCC->CSR, RCC_CSR_LSIRDY, RCC_CSR_LSIRDY,
                             PWR_TIMEOUT_LSI_READY, "LSI Ready")) {
        PWR_LOG("CRITICAL: LSI failed to become ready!");
        return false;
    }

    PWR_LOG("LSI successfully initialized and ready");
    return true;
}

/**
 * @brief Safely initialize RTC with comprehensive error checking
 */
static bool PWR_InitializeRTC(void) {
    PWR_LOG("Starting RTC initialization...");

    /* Step 1: Enable backup domain access */
    PWR->CR |= PWR_CR_DBP;
    if (!PWR_WaitForCondition(&PWR->CR, PWR_CR_DBP, PWR_CR_DBP,
                             PWR_TIMEOUT_STANDARD, "Backup Domain Access")) {
        PWR_LOG("CRITICAL: Failed to enable backup domain access!");
        return false;
    }

    /* Step 2: Check if RTC is already configured */
    if (RCC->BDCR & RCC_BDCR_RTCEN) {
        PWR_LOG("RTC already enabled, checking configuration...");

        /* Verify LSI is selected as clock source */
        uint32_t rtc_sel = (RCC->BDCR & RCC_BDCR_RTCSEL) >> RCC_BDCR_RTCSEL_Pos;
        if (rtc_sel == 0x2) {  /* LSI selected */
            PWR_LOG("RTC already properly configured with LSI");
            return true;
        } else {
            PWR_LOG_FMT("RTC clock source incorrect (current=%lu), reconfiguring...", rtc_sel);
        }
    }

    /* Step 3: Reset RTC domain if needed */
    PWR_LOG("Resetting RTC domain for clean configuration...");
    RCC->BDCR |= RCC_BDCR_BDRST;
    /* Small delay for reset to take effect */
    for (volatile int i = 0; i < 1000; i++);
    RCC->BDCR &= ~RCC_BDCR_BDRST;

    /* Step 4: Initialize LSI first */
    if (!PWR_InitializeLSI()) {
        PWR_LOG("CRITICAL: LSI initialization failed!");
        return false;
    }

    /* Step 5: Select LSI as RTC clock source */
    PWR_LOG("Selecting LSI as RTC clock source...");
    RCC->BDCR &= ~RCC_BDCR_RTCSEL;  /* Clear clock source bits */
    RCC->BDCR |= RCC_BDCR_RTCSEL_1; /* Select LSI (binary 10) */

    /* Verify clock source selection */
    uint32_t selected_source = (RCC->BDCR & RCC_BDCR_RTCSEL) >> RCC_BDCR_RTCSEL_Pos;
    if (selected_source != 0x2) {
        PWR_LOG_FMT("CRITICAL: RTC clock source selection failed! (got %lu, expected 2)", selected_source);
        return false;
    }
    PWR_LOG("RTC clock source successfully set to LSI");

    /* Step 6: Enable RTC clock */
    PWR_LOG("Enabling RTC clock...");
    RCC->BDCR |= RCC_BDCR_RTCEN;

    /* Wait a bit for RTC to stabilize */
    for (volatile int i = 0; i < 10000; i++);

    /* Step 7: Configure RTC prescalers */
    PWR_LOG("Configuring RTC prescalers...");

    /* Disable write protection */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Enter initialization mode */
    RTC->ISR |= RTC_ISR_INIT;
    if (!PWR_WaitForCondition(&RTC->ISR, RTC_ISR_INITF, RTC_ISR_INITF,
                             PWR_TIMEOUT_RTC_INIT, "RTC Initialization Mode")) {
        PWR_LOG("CRITICAL: Failed to enter RTC initialization mode!");
        goto rtc_init_failed;
    }

    /* Configure prescalers for LSI (~32 kHz) */
    /* Async prescaler = 127, Sync prescaler = 255 gives ~1 Hz */
    RTC->PRER = (127 << 16) | 255;
    PWR_LOG("RTC prescalers configured for ~1Hz operation");

    /* Exit initialization mode */
    RTC->ISR &= ~RTC_ISR_INIT;

    /* Re-enable write protection */
    RTC->WPR = 0xFF;

    PWR_LOG("RTC initialization completed successfully");
    return true;

rtc_init_failed:
    /* Re-enable write protection even on failure */
    RTC->WPR = 0xFF;
    PWR_LOG("RTC initialization FAILED!");
    return false;
}

/**
 * @brief Configure RTC wake-up timer with extensive safety checks
 */
static bool PWR_ConfigureRTCWakeupTimerSafe(uint32_t seconds) {
    PWR_LOG_FMT("Configuring RTC wake-up timer for %lu seconds...", seconds);

    /* Step 1: Ensure RTC is properly initialized */
    if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
        PWR_LOG("RTC not enabled, initializing...");
        if (!PWR_InitializeRTC()) {
            PWR_LOG("CRITICAL: RTC initialization failed during wake-up timer setup!");
            return false;
        }
    }

    /* Step 2: Disable write protection */
    PWR_LOG("Disabling RTC write protection...");
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Step 3: Disable wake-up timer first */
    PWR_LOG("Disabling wake-up timer...");
    RTC->CR &= ~RTC_CR_WUTE;

    /* Step 4: Wait for wake-up timer write flag */
    if (!PWR_WaitForCondition(&RTC->ISR, RTC_ISR_WUTWF, RTC_ISR_WUTWF,
                             PWR_TIMEOUT_RTC_INIT, "Wake-up Timer Write Flag")) {
        PWR_LOG("CRITICAL: Wake-up timer write flag timeout!");
        goto wakeup_config_failed;
    }

    /* Step 5: Clear all wake-up related flags */
    PWR_LOG("Clearing wake-up flags...");
    PWR_ClearAllRTCFlags();

    /* Step 6: Configure wake-up timer value */
    PWR_LOG_FMT("Setting wake-up timer value to %lu...", seconds - 1);
    RTC->WUTR = seconds - 1;

    /* Step 7: Select wake-up clock source (ck_spre = 1Hz) */
    PWR_LOG("Configuring wake-up clock source...");
    RTC->CR &= ~RTC_CR_WUCKSEL;
    RTC->CR |= (0x4 << RTC_CR_WUCKSEL_Pos);  /* Select ck_spre (1Hz) */

    /* Step 8: Enable wake-up timer and interrupt */
    PWR_LOG("Enabling wake-up timer and interrupt...");
    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;

    /* Step 9: Configure EXTI for RTC wake-up */
    PWR_LOG("Configuring EXTI line 22 for RTC wake-up...");
    EXTI->IMR |= (1 << 22);   /* Unmask interrupt */
    EXTI->RTSR |= (1 << 22);  /* Rising trigger */
    EXTI->FTSR &= ~(1 << 22); /* Disable falling trigger */
    EXTI->PR = (1 << 22);     /* Clear pending */

    /* Step 10: Configure NVIC for RTC wake-up interrupt */
    PWR_LOG("Configuring NVIC for RTC wake-up interrupt...");
    NVIC_SetPriority(RTC_WKUP_IRQn, 0);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);

    /* Step 11: Re-enable write protection */
    PWR_LOG("Re-enabling RTC write protection...");
    RTC->WPR = 0xFF;

    /* Step 12: Verify configuration */
    if (!PWR_VerifyRTCConfiguration()) {
        PWR_LOG("CRITICAL: RTC configuration verification failed!");
        return false;
    }

    PWR_LOG("RTC wake-up timer configuration completed successfully");
    return true;

wakeup_config_failed:
    /* Re-enable write protection on failure */
    RTC->WPR = 0xFF;
    PWR_LOG("RTC wake-up timer configuration FAILED!");
    return false;
}

/**
 * @brief Verify RTC configuration is correct
 */
static bool PWR_VerifyRTCConfiguration(void) {
    PWR_LOG("Verifying RTC configuration...");

    /* Check RTC is enabled */
    if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
        PWR_LOG("VERIFY FAIL: RTC not enabled");
        return false;
    }

    /* Check LSI is ready */
    if (!(RCC->CSR & RCC_CSR_LSIRDY)) {
        PWR_LOG("VERIFY FAIL: LSI not ready");
        return false;
    }

    /* Check RTC clock source */
    uint32_t rtc_source = (RCC->BDCR & RCC_BDCR_RTCSEL) >> RCC_BDCR_RTCSEL_Pos;
    if (rtc_source != 0x2) {
        PWR_LOG_FMT("VERIFY FAIL: Wrong RTC clock source (%lu, expected 2)", rtc_source);
        return false;
    }

    /* Check wake-up timer is enabled */
    if (!(RTC->CR & RTC_CR_WUTE)) {
        PWR_LOG("VERIFY FAIL: Wake-up timer not enabled");
        return false;
    }

    /* Check wake-up interrupt is enabled */
    if (!(RTC->CR & RTC_CR_WUTIE)) {
        PWR_LOG("VERIFY FAIL: Wake-up interrupt not enabled");
        return false;
    }

    PWR_LOG("RTC configuration verification PASSED");
    PWR_DebugRTCStatus();
    return true;
}

/**
 * @brief Print detailed RTC status for debugging
 */
static void PWR_DebugRTCStatus(void) {
    PWR_LOG("=== RTC Status Debug ===");
    PWR_LOG_FMT("RCC->CSR = 0x%08lX (LSI enable/ready)", RCC->CSR);
    PWR_LOG_FMT("RCC->BDCR = 0x%08lX (RTC config)", RCC->BDCR);
    PWR_LOG_FMT("PWR->CR = 0x%08lX (backup access)", PWR->CR);
    PWR_LOG_FMT("RTC->CR = 0x%08lX (control)", RTC->CR);
    PWR_LOG_FMT("RTC->ISR = 0x%08lX (status/flags)", RTC->ISR);
    PWR_LOG_FMT("RTC->WUTR = 0x%08lX (wake-up timer)", RTC->WUTR);
    PWR_LOG_FMT("EXTI->IMR bit 22 = %lu", (EXTI->IMR >> 22) & 1);
    PWR_LOG_FMT("EXTI->PR bit 22 = %lu", (EXTI->PR >> 22) & 1);
    PWR_LOG("======================");
}

/**
 * @brief Clear all RTC wake-up related flags
 */
static void PWR_ClearAllRTCFlags(void) {
    /* Clear RTC flags */
    RTC->ISR &= ~(RTC_ISR_WUTF | RTC_ISR_ALRAF | RTC_ISR_ALRBF | RTC_ISR_TSF | RTC_ISR_TSOVF);

    /* Clear EXTI flags */
    EXTI->PR = (1 << 22);

    /* Clear PWR flags */
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

    PWR_LOG("All wake-up flags cleared");
}

/* ====================== Public Functions ====================== */

/**
 * @brief Initialize power management subsystem with robust error handling
 */
void PowerMgmt_Init(void) {
    PWR_LOG("=== Starting Power Management Initialization ===");

    /* Enable power interface clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR_LOG("Power interface clock enabled");

    /* Set default configuration */
    memset(&currentConfig, 0, sizeof(PowerConfig_t));
    currentConfig.mode = POWER_MODE_RUN;
    currentConfig.voltageScale = VOLTAGE_SCALE_1;
    currentConfig.flashPowerDown = false;
    currentConfig.enableWakeupPin = false;
    currentConfig.enableBackupRegulator = false;
    currentConfig.enablePVD = false;
    currentConfig.pvdLevel = PVD_LEVEL_2V7;
    currentConfig.wakeupSources = WAKEUP_SOURCE_NONE;

    /* Clear all wake-up flags */
    PWR_ClearAllRTCFlags();

    powerMgmtInitialized = true;
    PWR_LOG("Power management initialization completed successfully");
}

/**
 * @brief Set wake-up timer with comprehensive error handling
 */
bool PowerMgmt_SetWakeupTimer(uint32_t seconds) {
    if (!powerMgmtInitialized) {
        PWR_LOG("CRITICAL: Power management not initialized!");
        return false;
    }

    if (seconds == 0 || seconds > 0x1FFFF) {
        PWR_LOG_FMT("CRITICAL: Invalid wake-up time %lu (must be 1-131071)", seconds);
        return false;
    }

    PWR_LOG("=== Starting Wake-up Timer Configuration ===");

    /* Try configuration with retries */
    for (int retry = 0; retry < PWR_MAX_RETRIES; retry++) {
        if (retry > 0) {
            PWR_LOG_FMT("Retry attempt %d/%d...", retry + 1, PWR_MAX_RETRIES);
        }

        if (PWR_ConfigureRTCWakeupTimerSafe(seconds)) {
            PWR_LOG("Wake-up timer configuration SUCCESS!");
            return true;
        }

        PWR_LOG_FMT("Wake-up timer configuration failed, attempt %d/%d", retry + 1, PWR_MAX_RETRIES);

        /* Reset RTC between retries */
        if (retry < PWR_MAX_RETRIES - 1) {
            PWR_LOG("Resetting RTC for retry...");
            RCC->BDCR |= RCC_BDCR_BDRST;
            for (volatile int i = 0; i < 10000; i++);
            RCC->BDCR &= ~RCC_BDCR_BDRST;
        }
    }

    PWR_LOG("CRITICAL: Wake-up timer configuration FAILED after all retries!");
    return false;
}

/**
 * @brief Test power management functionality
 */
bool PowerMgmt_TestConfiguration(void) {
    PWR_LOG("=== Testing Power Management Configuration ===");

    if (!powerMgmtInitialized) {
        PWR_LOG("Power management not initialized");
        return false;
    }

    /* Test 1: LSI Clock */
    if (!PWR_InitializeLSI()) {
        PWR_LOG("TEST FAIL: LSI initialization");
        return false;
    }
    PWR_LOG("TEST PASS: LSI initialization");

    /* Test 2: RTC Initialization */
    if (!PWR_InitializeRTC()) {
        PWR_LOG("TEST FAIL: RTC initialization");
        return false;
    }
    PWR_LOG("TEST PASS: RTC initialization");

    /* Test 3: Wake-up timer (short duration for test) */
    if (!PWR_ConfigureRTCWakeupTimerSafe(5)) {
        PWR_LOG("TEST FAIL: Wake-up timer configuration");
        return false;
    }
    PWR_LOG("TEST PASS: Wake-up timer configuration");

    PWR_LOG("=== All Power Management Tests PASSED ===");
    return true;
}

/**
 * @brief Get current power consumption estimate
 */
uint32_t PowerMgmt_GetPowerConsumptionEstimate(void) {
    uint32_t estimate = 120000; /* Base 120mA for run mode */

    switch (currentPowerMode) {
        case POWER_MODE_RUN:
            estimate = 120000;
            break;
        case POWER_MODE_RUN_LP:
            estimate = 50000;
            break;
        case POWER_MODE_SLEEP:
            estimate = 15000;
            break;
        case POWER_MODE_STOP_MR:
            estimate = 250;
            break;
        case POWER_MODE_STOP_LP:
        case POWER_MODE_STOP_LP_FPD:
            estimate = 50;
            break;
        case POWER_MODE_STANDBY:
            estimate = 3;
            break;
    }

    /* Adjust for voltage scaling */
    switch (currentConfig.voltageScale) {
        case VOLTAGE_SCALE_2:
            estimate = (estimate * 80) / 100;
            break;
        case VOLTAGE_SCALE_3:
            estimate = (estimate * 60) / 100;
            break;
        default:
            break;
    }

    return estimate;
}

/**
 * @brief Enhanced RTC interrupt handler
 */
void RTC_WKUP_IRQHandler(void) {
    /* Visual confirmation */
    GPIOB->ODR ^= (1 << 0);  /* Toggle green LED */

    PWR_LOG("=== RTC Wake-up Interrupt ===");
    PWR_DebugRTCStatus();

    /* Clear RTC wake-up flag */
    if (RTC->ISR & RTC_ISR_WUTF) {
        PWR_LOG("Clearing RTC wake-up flag");
        RTC->ISR &= ~RTC_ISR_WUTF;
    }

    /* Clear EXTI line 22 flag */
    if (EXTI->PR & (1 << 22)) {
        PWR_LOG("Clearing EXTI line 22 flag");
        EXTI->PR = (1 << 22);
    }

    PWR_LOG("RTC wake-up interrupt handled");
}

/* ====================== Stub implementations for missing functions ====================== */

bool PowerMgmt_Configure(const PowerConfig_t* config) {
    if (config == NULL) return false;
    memcpy(&currentConfig, config, sizeof(PowerConfig_t));
    PWR_LOG("Power configuration updated");
    return true;
}

void PowerMgmt_EnterLowPowerMode(PowerMode_t mode) {
    currentPowerMode = mode;
    PWR_LOG_FMT("Entering power mode %d", mode);
    /* Implementation would go here */
}

bool PowerMgmt_SetVoltageScale(VoltageScale_t scale) {
    currentConfig.voltageScale = scale;
    PWR_LOG_FMT("Voltage scale set to %d", scale);
    return true;
}

bool PowerMgmt_ConfigureWakeupSources(WakeupSource_t sources) {
    currentConfig.wakeupSources = sources;
    PWR_LOG_FMT("Wake-up sources configured: 0x%02X", sources);
    return true;
}

void PowerMgmt_ConfigurePVD(bool enable, PVDLevel_t level) {
    currentConfig.enablePVD = enable;
    currentConfig.pvdLevel = level;
    PWR_LOG_FMT("PVD configured: %s, level %d", enable ? "enabled" : "disabled", level);
}

WakeupSource_t PowerMgmt_GetLastWakeupSource(void) {
    return lastWakeupSource;
}

void PowerMgmt_EnableBackupAccess(bool enable) {
    if (enable) {
        PWR->CR |= PWR_CR_DBP;
    } else {
        PWR->CR &= ~PWR_CR_DBP;
    }
    PWR_LOG_FMT("Backup access %s", enable ? "enabled" : "disabled");
}

void PowerMgmt_ClearWakeupFlags(void) {
    PWR_ClearAllRTCFlags();
}

PowerMode_t PowerMgmt_GetCurrentMode(void) {
    return currentPowerMode;
}
