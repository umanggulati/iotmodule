/**
 * @file i2c.h
 * @brief I2C driver implementation for STM32F429ZI
 * @desc Baremetal I2C1 driver for BMP280 sensor communication
 */

#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C1 Configuration for STM32F429ZI Development Board - Using PB8 (SCL) and PB9 (SDA) */
#define I2C_PORT               GPIOB
#define I2C_SCL_PIN            8
#define I2C_SDA_PIN            9
#define I2C_AF                 4  /* AF4 for I2C1 */

/* I2C Timing Configuration for 100kHz (Standard Mode) */
#define I2C_CLOCK_SPEED        100000  /* 100 kHz */
#define I2C_TIMEOUT_MS         1000    /* 1 second timeout */

/* Error codes */
typedef enum {
    I2C_OK = 0,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_BUSY,
    I2C_ERROR_NACK,
    I2C_ERROR_ARBITRATION_LOST,
    I2C_ERROR_BUS_ERROR,
    I2C_ERROR_OVERRUN,
    I2C_ERROR_INVALID_PARAM
} I2C_Error_t;

/* I2C Status */
typedef enum {
    I2C_STATE_READY = 0,
    I2C_STATE_BUSY_TX,
    I2C_STATE_BUSY_RX,
    I2C_STATE_ERROR
} I2C_State_t;

/* I2C Configuration Structure */
typedef struct {
    uint32_t clockSpeed;        /* Clock speed in Hz */
    uint8_t ownAddress;         /* Own address (7-bit) */
    bool enableGeneralCall;     /* Enable general call */
    bool enableClockStretching; /* Enable clock stretching */
} I2C_Config_t;

/* Device information for multi-device support */
typedef struct {
    uint8_t address;            /* 7-bit device address */
    char name[16];              /* Device name for debugging */
    bool isDetected;            /* Device detection status */
    uint32_t lastCommunication; /* Last successful communication timestamp */
    uint32_t errorCount;        /* Communication error count */
} I2C_Device_t;

/* I2C Statistics for monitoring */
typedef struct {
    uint32_t totalTransactions;
    uint32_t successfulTransactions;
    uint32_t timeoutErrors;
    uint32_t nackErrors;
    uint32_t busErrors;
    uint32_t maxTransactionTime;
    uint32_t averageTransactionTime;
} I2C_Stats_t;

/* ====================== Function Prototypes ====================== */

/**
 * @brief Initialize I2C1 peripheral
 * @param config: Pointer to I2C configuration structure
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_Init(const I2C_Config_t* config);

/**
 * @brief Deinitialize I2C1 peripheral
 * @return None
 */
void I2C_DeInit(void);

/**
 * @brief Check if I2C bus is ready
 * @return bool: true if ready, false if busy
 */
bool I2C_IsReady(void);

/**
 * @brief Get current I2C state
 * @return I2C_State_t: Current state
 */
I2C_State_t I2C_GetState(void);

/* ==================== Device Detection Functions ==================== */

/**
 * @brief Scan I2C bus for devices
 * @param deviceList: Array to store detected devices
 * @param maxDevices: Maximum number of devices to detect
 * @param detectedCount: Pointer to store number of detected devices
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_ScanBus(I2C_Device_t* deviceList, uint8_t maxDevices, uint8_t* detectedCount);

/**
 * @brief Check if device is present at given address
 * @param address: 7-bit device address
 * @return bool: true if device responds, false otherwise
 */
bool I2C_IsDevicePresent(uint8_t address);

/**
 * @brief Register a known device for monitoring
 * @param device: Pointer to device information structure
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_RegisterDevice(I2C_Device_t* device);

/* ==================== Basic Communication Functions ==================== */

/**
 * @brief Write data to I2C device
 * @param address: 7-bit device address
 * @param data: Pointer to data buffer
 * @param size: Number of bytes to write
 * @param timeout_ms: Timeout in milliseconds
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_Write(uint8_t address, const uint8_t* data, uint16_t size, uint32_t timeout_ms);

/**
 * @brief Read data from I2C device
 * @param address: 7-bit device address
 * @param data: Pointer to data buffer
 * @param size: Number of bytes to read
 * @param timeout_ms: Timeout in milliseconds
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_Read(uint8_t address, uint8_t* data, uint16_t size, uint32_t timeout_ms);

/**
 * @brief Write then read from I2C device (typical register access)
 * @param address: 7-bit device address
 * @param writeData: Pointer to write data buffer
 * @param writeSize: Number of bytes to write
 * @param readData: Pointer to read data buffer
 * @param readSize: Number of bytes to read
 * @param timeout_ms: Timeout in milliseconds
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_WriteRead(uint8_t address, const uint8_t* writeData, uint16_t writeSize,
                         uint8_t* readData, uint16_t readSize, uint32_t timeout_ms);

/* ==================== Register Access Functions ==================== */

/**
 * @brief Write to device register
 * @param address: 7-bit device address
 * @param reg: Register address
 * @param value: Value to write
 * @param timeout_ms: Timeout in milliseconds
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_WriteRegister(uint8_t address, uint8_t reg, uint8_t value, uint32_t timeout_ms);

/**
 * @brief Read from device register
 * @param address: 7-bit device address
 * @param reg: Register address
 * @param value: Pointer to store read value
 * @param timeout_ms: Timeout in milliseconds
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_ReadRegister(uint8_t address, uint8_t reg, uint8_t* value, uint32_t timeout_ms);

/**
 * @brief Read multiple bytes from device registers
 * @param address: 7-bit device address
 * @param reg: Starting register address
 * @param data: Pointer to data buffer
 * @param size: Number of bytes to read
 * @param timeout_ms: Timeout in milliseconds
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_ReadRegisters(uint8_t address, uint8_t reg, uint8_t* data, uint16_t size, uint32_t timeout_ms);

/* ==================== Error Handling Functions ==================== */

/**
 * @brief Clear I2C error flags and recover from error state
 * @return I2C_Error_t: Recovery result
 */
I2C_Error_t I2C_ClearErrors(void);

/**
 * @brief Reset I2C peripheral (hard reset)
 * @return I2C_Error_t: Reset result
 */
I2C_Error_t I2C_Reset(void);

/**
 * @brief Get last error details
 * @return I2C_Error_t: Last error code
 */
I2C_Error_t I2C_GetLastError(void);

/* ==================== Statistics and Monitoring ==================== */

/**
 * @brief Get I2C communication statistics
 * @param stats: Pointer to statistics structure
 * @return None
 */
void I2C_GetStatistics(I2C_Stats_t* stats);

/**
 * @brief Reset I2C statistics
 * @return None
 */
void I2C_ResetStatistics(void);

/**
 * @brief Print I2C status and statistics via UART
 * @return None
 */
void I2C_PrintStatus(void);

/* ==================== Power Management Integration ==================== */

/**
 * @brief Prepare I2C for low-power mode
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_PrepareForLowPower(void);

/**
 * @brief Restore I2C after waking from low-power mode
 * @return I2C_Error_t: Operation result
 */
I2C_Error_t I2C_RestoreFromLowPower(void);

#endif /* I2C_H */
