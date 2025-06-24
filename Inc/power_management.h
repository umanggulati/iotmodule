/**
 * @file power_management_fixed.h
 * @brief FIXED Power management header with additional debugging functions
 */

#ifndef POWER_MANAGEMENT_FIXED_H
#define POWER_MANAGEMENT_FIXED_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ====================== Power Mode Definitions ====================== */

typedef enum {
    POWER_MODE_RUN = 0,
    POWER_MODE_RUN_LP,
    POWER_MODE_SLEEP,
    POWER_MODE_STOP_MR,
    POWER_MODE_STOP_LP,
    POWER_MODE_STOP_LP_FPD,
    POWER_MODE_STANDBY
} PowerMode_t;

typedef enum {
    VOLTAGE_SCALE_1 = 0,
    VOLTAGE_SCALE_2,
    VOLTAGE_SCALE_3
} VoltageScale_t;

typedef enum {
    WAKEUP_SOURCE_NONE = 0x00,
    WAKEUP_SOURCE_PIN = 0x01,
    WAKEUP_SOURCE_RTC_ALARM = 0x02,
    WAKEUP_SOURCE_RTC_WAKEUP = 0x04,
    WAKEUP_SOURCE_RTC_TAMPER = 0x08,
    WAKEUP_SOURCE_RTC_TIMESTAMP = 0x10,
    WAKEUP_SOURCE_ALL = 0x1F
} WakeupSource_t;

typedef enum {
    PVD_LEVEL_2V0 = 0,
    PVD_LEVEL_2V1,
    PVD_LEVEL_2V3,
    PVD_LEVEL_2V5,
    PVD_LEVEL_2V6,
    PVD_LEVEL_2V7,
    PVD_LEVEL_2V8,
    PVD_LEVEL_2V9
} PVDLevel_t;

typedef struct {
    PowerMode_t mode;
    VoltageScale_t voltageScale;
    bool flashPowerDown;
    bool enableWakeupPin;
    bool enableBackupRegulator;
    bool enablePVD;
    PVDLevel_t pvdLevel;
    WakeupSource_t wakeupSources;
} PowerConfig_t;

/* ====================== Core Functions ====================== */

/**
 * @brief Initialize power management with robust error handling
 */
void PowerMgmt_Init(void);

/**
 * @brief Configure power management settings
 */
bool PowerMgmt_Configure(const PowerConfig_t* config);

/**
 * @brief Enter specified power mode
 */
void PowerMgmt_EnterLowPowerMode(PowerMode_t mode);

/**
 * @brief Set voltage scaling
 */
bool PowerMgmt_SetVoltageScale(VoltageScale_t scale);

/**
 * @brief Configure wake-up sources
 */
bool PowerMgmt_ConfigureWakeupSources(WakeupSource_t sources);

/**
 * @brief Set wake-up timer with comprehensive error handling
 * @param seconds: Wake-up time in seconds (1-131071)
 * @return bool: true if successful, false if failed
 */
bool PowerMgmt_SetWakeupTimer(uint32_t seconds);

/**
 * @brief Test power management configuration
 * @return bool: true if all tests pass, false otherwise
 */
bool PowerMgmt_TestConfiguration(void);

/**
 * @brief Configure PVD
 */
void PowerMgmt_ConfigurePVD(bool enable, PVDLevel_t level);

/**
 * @brief Get last wake-up source
 */
WakeupSource_t PowerMgmt_GetLastWakeupSource(void);

/**
 * @brief Get power consumption estimate
 */
uint32_t PowerMgmt_GetPowerConsumptionEstimate(void);

/**
 * @brief Enable backup domain access
 */
void PowerMgmt_EnableBackupAccess(bool enable);

/**
 * @brief Clear wake-up flags
 */
void PowerMgmt_ClearWakeupFlags(void);

/**
 * @brief Get current power mode
 */
PowerMode_t PowerMgmt_GetCurrentMode(void);

#endif /* POWER_MANAGEMENT_FIXED_H */
