/**
 * @file power_management.h
 * @brief Power management subsystem for STM32F429ZI
 * @desc Direct power control for IoT applications
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ====================== Power Mode Definitions ====================== */

/**
 * @brief Power modes available in STM32F429ZI
 */
typedef enum {
    POWER_MODE_RUN = 0,       /* Normal run mode */
    POWER_MODE_RUN_LP,        /* Run mode with reduced clock speed */
    POWER_MODE_SLEEP,         /* Sleep mode - CPU stopped, peripherals running */
    POWER_MODE_STOP_MR,       /* Stop mode with main regulator */
    POWER_MODE_STOP_LP,       /* Stop mode with low-power regulator */
    POWER_MODE_STOP_LP_FPD,   /* Stop mode with low-power regulator and flash power down */
    POWER_MODE_STANDBY        /* Standby mode - lowest power consumption */
} PowerMode_t;

/**
 * @brief Voltage scaling options
 * @note Affects performance and power consumption
 */
typedef enum {
    VOLTAGE_SCALE_1 = 0,      /* Highest performance */
    VOLTAGE_SCALE_2,          /* Balance between performance and power */
    VOLTAGE_SCALE_3           /* Lowest power consumption, reduced frequency */
} VoltageScale_t;

/**
 * @brief Wake-up source options
 */
typedef enum {
    WAKEUP_SOURCE_NONE = 0x00,
    WAKEUP_SOURCE_PIN = 0x01,         /* PA0 wake-up pin */
    WAKEUP_SOURCE_RTC_ALARM = 0x02,   /* RTC Alarm A or B */
    WAKEUP_SOURCE_RTC_WAKEUP = 0x04,  /* RTC Wake-up event */
    WAKEUP_SOURCE_RTC_TAMPER = 0x08,  /* RTC Tamper event */
    WAKEUP_SOURCE_RTC_TIMESTAMP = 0x10, /* RTC Timestamp event */
    WAKEUP_SOURCE_ALL = 0x1F          /* All wake-up sources */
} WakeupSource_t;

/**
 * @brief PVD threshold levels
 */
typedef enum {
    PVD_LEVEL_2V0 = 0,        /* 2.0V threshold */
    PVD_LEVEL_2V1,            /* 2.1V threshold */
    PVD_LEVEL_2V3,            /* 2.3V threshold */
    PVD_LEVEL_2V5,            /* 2.5V threshold */
    PVD_LEVEL_2V6,            /* 2.6V threshold */
    PVD_LEVEL_2V7,            /* 2.7V threshold */
    PVD_LEVEL_2V8,            /* 2.8V threshold */
    PVD_LEVEL_2V9             /* 2.9V threshold */
} PVDLevel_t;

/**
 * @brief Power management configuration
 */
typedef struct {
    PowerMode_t mode;                /* Power mode selection */
    VoltageScale_t voltageScale;     /* Voltage scaling selection */
    bool flashPowerDown;             /* Whether to power down flash in Stop mode */
    bool enableWakeupPin;            /* Enable PA0 as wake-up pin */
    bool enableBackupRegulator;      /* Enable backup regulator for SRAM in standby */
    bool enablePVD;                  /* Enable Programmable Voltage Detector */
    PVDLevel_t pvdLevel;             /* PVD threshold level */
    WakeupSource_t wakeupSources;    /* Sources that can wake the MCU from low-power modes */
} PowerConfig_t;

/* ====================== Function Prototypes ====================== */

/**
 * @brief Initialize power management subsystem
 * @return None
 */
void PowerMgmt_Init(void);

/**
 * @brief Configure power management settings
 * @param config: Pointer to power configuration structure
 * @return true if successful, false if invalid configuration
 */
bool PowerMgmt_Configure(const PowerConfig_t* config);

/**
 * @brief Enter specified power mode
 * @param mode: Power mode to enter
 * @return None
 * @note Function may not return if entering Standby mode
 */
void PowerMgmt_EnterLowPowerMode(PowerMode_t mode);

/**
 * @brief Set voltage scaling
 * @param scale: Voltage scale to set
 * @return true if successful, false if invalid or not possible now
 */
bool PowerMgmt_SetVoltageScale(VoltageScale_t scale);

/**
 * @brief Configure wake-up sources
 * @param sources: Bit mask of wake-up sources
 * @return true if successful, false if invalid configuration
 */
bool PowerMgmt_ConfigureWakeupSources(WakeupSource_t sources);

/**
 * @brief Set wake-up timer for Stop mode
 * @param seconds: Wake-up time in seconds
 * @return true if successful, false otherwise
 */
bool PowerMgmt_SetWakeupTimer(uint32_t seconds);

/**
 * @brief Enable/disable the programmable voltage detector (PVD)
 * @param enable: true to enable, false to disable
 * @param level: Voltage threshold level
 * @return None
 */
void PowerMgmt_ConfigurePVD(bool enable, PVDLevel_t level);

/**
 * @brief Get last wake-up source after exiting low-power mode
 * @return Wake-up source that triggered exit from low-power mode
 */
WakeupSource_t PowerMgmt_GetLastWakeupSource(void);

/**
 * @brief Get current power consumption estimate
 * @return Estimated power consumption in microamps
 * @note This is an estimate based on mode and peripherals, not a measurement
 */
uint32_t PowerMgmt_GetPowerConsumptionEstimate(void);

/**
 * @brief Enable backup domain access (RTC, backup registers)
 * @param enable: true to enable, false to disable access
 * @return None
 */
void PowerMgmt_EnableBackupAccess(bool enable);

/**
 * @brief Clear wake-up flags
 * @return None
 */
void PowerMgmt_ClearWakeupFlags(void);

/**
 * @brief Get power mode status
 * @return Current power mode
 */
PowerMode_t PowerMgmt_GetCurrentMode(void);

#endif /* POWER_MANAGEMENT_H */
