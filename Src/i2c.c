/* @file i2c.c
 * @brief Fixed I2C driver implementation for STM32F429ZI
 * @desc Corrected implementation with proper ADDR clearing, BTF usage, and ACK management
 */

#include "i2c.h"
#include "uart.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

/* ====================== Private Variables ====================== */

/* Current I2C state */
static I2C_State_t currentState = I2C_STATE_READY;

/* Last error code */
static I2C_Error_t lastError = I2C_OK;

/* I2C statistics */
static I2C_Stats_t i2cStats = {0};

/* Registered devices for monitoring */
static I2C_Device_t registeredDevices[8];
static uint8_t deviceCount = 0;

/* Configuration backup for low-power recovery */
static I2C_Config_t configBackup;

/* ====================== Private Function Prototypes ====================== */

static I2C_Error_t I2C_WaitForFlag(uint32_t flag, bool state, uint32_t timeout_ms);
static I2C_Error_t I2C_WaitForFlagWithError(uint32_t flag, bool state, uint32_t timeout_ms);
static void I2C_UpdateStats(bool success, uint32_t transactionTime);

/* ====================== Public Functions ====================== */

/**
 * @brief Initialize I2C1 peripheral
 */
I2C_Error_t I2C_Init(const I2C_Config_t* config) {
    if (config == NULL) {
        return I2C_ERROR_INVALID_PARAM;
    }

    /* Store configuration for recovery */
    memcpy(&configBackup, config, sizeof(I2C_Config_t));

    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  /* Enable GPIOB clock */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   /* Enable I2C1 clock */

    /* Reset I2C peripheral BEFORE GPIO configuration */
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    /* Configure GPIO pins PB8 (SCL) and PB9 (SDA) */
    /* Configure PB8 (SCL) */
    GPIOB->MODER &= ~(3u<<16);          /* Clear bits 17:16 for PB8 */
    GPIOB->MODER |= (2u<<16);           /* Set bits 17:16 to 10 for AF mode */

    /* Configure PB9 (SDA) */
    GPIOB->MODER &= ~(3u<<18);          /* Clear bits 19:18 for PB9 */
    GPIOB->MODER |= (2u<<18);           /* Set bits 19:18 to 10 for AF mode */

    /* Set open-drain output type for both pins */
    GPIOB->OTYPER |= (1<<8) | (1<<9);   /* Open drain for PB8 and PB9 */

    /* Set pull-up resistors for both pins */
    GPIOB->PUPDR &= ~((3u<<16) | (3u<<18)); /* Clear first */
    GPIOB->PUPDR |= (1<<16) | (1<<18);      /* Pull-up for PB8 and PB9 */

    /* Set alternate function AF4 for I2C1 on both pins */
    GPIOB->AFR[1] &= ~((0xF<<0) | (0xF<<4)); /* Clear AF for PB8 and PB9 */
    GPIOB->AFR[1] |= (0x04<<0) | (0x04<<4);  /* AF4 for PB8 (bits 3:0) and PB9 (bits 7:4) */

    /* Configure I2C timing */
    I2C1->CR2 = 16;              /* Set peripheral clock frequency to 16 MHz */
    I2C1->CCR = 80;              /* Clock control register for 100 kHz */
    I2C1->TRISE = 17;            /* Rise time configuration */
    I2C1->OAR1 |= (1 << 14);     /* Set own address register bit 14 */

    /* Configure general call and clock stretching */
    if (config->enableGeneralCall) {
        I2C1->CR1 |= I2C_CR1_ENGC;
    } else {
        I2C1->CR1 &= ~I2C_CR1_ENGC;
    }

    if (!config->enableClockStretching) {
        I2C1->CR1 |= I2C_CR1_NOSTRETCH;
    } else {
        I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;
    }

    /* Enable I2C peripheral */
    I2C1->CR1 |= I2C_CR1_PE;

    /* Initialize statistics */
    memset(&i2cStats, 0, sizeof(I2C_Stats_t));
    deviceCount = 0;

    /* Set state to ready */
    currentState = I2C_STATE_READY;
    lastError = I2C_OK;

    /* Debug message */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[I2C] Initialized - Clock: %lu Hz, Own Address: 0x%02X\r\n",
             config->clockSpeed, config->ownAddress);
    UART_SendString(debug_msg);

    return I2C_OK;
}

/**
 * @brief Deinitialize I2C1 peripheral
 */
void I2C_DeInit(void) {
    /* Disable I2C peripheral */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* Disable I2C clock */
    RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

    /* Reset GPIO pins PB8 and PB9 to default state */
    GPIOB->MODER &= ~((3u<<16) | (3u<<18));
    GPIOB->OTYPER &= ~((1<<8) | (1<<9));
    GPIOB->PUPDR &= ~((3u<<16) | (3u<<18));
    GPIOB->AFR[1] &= ~((0x0F<<0) | (0x0F<<4));

    currentState = I2C_STATE_READY;
    UART_SendString("[I2C] Deinitialized\r\n");
}

/**
 * @brief Check if I2C bus is ready
 */
bool I2C_IsReady(void) {
    return (currentState == I2C_STATE_READY) && !(I2C1->SR2 & I2C_SR2_BUSY);
}

/**
 * @brief Get current I2C state
 */
I2C_State_t I2C_GetState(void) {
    return currentState;
}

/**
 * @brief Write to device register
 */
I2C_Error_t I2C_WriteRegister(uint8_t address, uint8_t reg, uint8_t value, uint32_t timeout_ms) {
    uint32_t startTime = systick_counter;
    I2C_Error_t result = I2C_OK;

    /* Check if I2C is ready */
    if (!I2C_IsReady()) {
        lastError = I2C_ERROR_BUSY;
        return I2C_ERROR_BUSY;
    }

    currentState = I2C_STATE_BUSY_TX;

    /* Generate START condition */
    I2C1->CR1 |= I2C_CR1_START;

    /* Wait for SB flag */
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Send device address for write */
    I2C1->DR = (address << 1);  /* Write mode */

    /* Wait for ADDR flag with error checking */
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* CRITICAL: Clear ADDR by reading SR2 */
    (void)I2C1->SR2;

    /* Send register address */
    I2C1->DR = reg;

    /* Wait for BTF to ensure register is sent */
    result = I2C_WaitForFlag(I2C_SR1_BTF, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Send data value */
    I2C1->DR = value;

    /* Wait for BTF to ensure data is sent */
    result = I2C_WaitForFlag(I2C_SR1_BTF, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Generate STOP condition */
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Wait until BUSY flag is cleared */
    uint32_t timeout_start = systick_counter;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((systick_counter - timeout_start) > timeout_ms) {
            result = I2C_ERROR_TIMEOUT;
            goto cleanup;
        }
    }

cleanup:
    currentState = I2C_STATE_READY;

    /* Update statistics */
    uint32_t transactionTime = systick_counter - startTime;
    I2C_UpdateStats(result == I2C_OK, transactionTime);

    if (result != I2C_OK) {
        lastError = result;
    }

    return result;
}

/**
 * @brief Read from device register
 */
I2C_Error_t I2C_ReadRegister(uint8_t address, uint8_t reg, uint8_t* value, uint32_t timeout_ms) {
    uint32_t startTime = systick_counter;
    I2C_Error_t result = I2C_OK;

    if (value == NULL) {
        return I2C_ERROR_INVALID_PARAM;
    }

    /* Check if I2C is ready */
    if (!I2C_IsReady()) {
        lastError = I2C_ERROR_BUSY;
        return I2C_ERROR_BUSY;
    }

    currentState = I2C_STATE_BUSY_TX;

    /* Generate START condition */
    I2C1->CR1 |= I2C_CR1_START;

    /* Wait for SB */
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Send device address for write */
    I2C1->DR = (address << 1);

    /* Wait for ADDR with error checking */
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Clear ADDR */
    (void)I2C1->SR2;

    /* Send register address */
    I2C1->DR = reg;

    /* Wait for BTF */
    result = I2C_WaitForFlag(I2C_SR1_BTF, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Enable ACK before repeated start */
    I2C1->CR1 |= I2C_CR1_ACK;

    /* Generate repeated START */
    I2C1->CR1 |= I2C_CR1_START;

    /* Wait for SB */
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    currentState = I2C_STATE_BUSY_RX;

    /* Send device address for read */
    I2C1->DR = (address << 1) | 0x01;

    /* Wait for ADDR */
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* For single byte read, disable ACK before clearing ADDR */
    I2C1->CR1 &= ~I2C_CR1_ACK;

    /* Clear ADDR */
    (void)I2C1->SR2;

    /* Wait for RXNE */
    result = I2C_WaitForFlag(I2C_SR1_RXNE, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Read data */
    *value = (uint8_t)I2C1->DR;

    /* Generate STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Wait until BUSY is cleared */
    uint32_t timeout_start = systick_counter;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((systick_counter - timeout_start) > timeout_ms) {
            result = I2C_ERROR_TIMEOUT;
            goto cleanup;
        }
    }

cleanup:
    currentState = I2C_STATE_READY;

    /* Update statistics */
    uint32_t transactionTime = systick_counter - startTime;
    I2C_UpdateStats(result == I2C_OK, transactionTime);

    if (result != I2C_OK) {
        lastError = result;
    }

    return result;
}

/**
 * @brief Read multiple bytes from device registers
 */
I2C_Error_t I2C_ReadRegisters(uint8_t address, uint8_t reg, uint8_t* data, uint16_t size, uint32_t timeout_ms) {
    uint32_t startTime = systick_counter;
    I2C_Error_t result = I2C_OK;

    if (data == NULL || size == 0) {
        return I2C_ERROR_INVALID_PARAM;
    }

    /* Check if I2C is ready */
    if (!I2C_IsReady()) {
        lastError = I2C_ERROR_BUSY;
        return I2C_ERROR_BUSY;
    }

    currentState = I2C_STATE_BUSY_TX;

    /* Generate START */
    I2C1->CR1 |= I2C_CR1_START;
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Send device address for write */
    I2C1->DR = (address << 1);
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Clear ADDR */
    (void)I2C1->SR2;

    /* Send register address */
    I2C1->DR = reg;
    result = I2C_WaitForFlag(I2C_SR1_BTF, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Enable ACK for multi-byte read */
    I2C1->CR1 |= I2C_CR1_ACK;

    /* Generate repeated START */
    I2C1->CR1 |= I2C_CR1_START;
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    currentState = I2C_STATE_BUSY_RX;

    /* Send device address for read */
    I2C1->DR = (address << 1) | 0x01;
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Clear ADDR */
    (void)I2C1->SR2;

    /* Read multiple bytes */
    for (uint16_t i = 0; i < size; i++) {
        if (i == (size - 1)) {
            /* Last byte - disable ACK */
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }

        /* Wait for RXNE */
        result = I2C_WaitForFlag(I2C_SR1_RXNE, true, timeout_ms);
        if (result != I2C_OK) goto cleanup;

        /* Read data */
        data[i] = (uint8_t)I2C1->DR;
    }

    /* Generate STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Wait for BUSY to clear */
    uint32_t timeout_start = systick_counter;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((systick_counter - timeout_start) > timeout_ms) {
            result = I2C_ERROR_TIMEOUT;
            goto cleanup;
        }
    }

cleanup:
    currentState = I2C_STATE_READY;
    uint32_t transactionTime = systick_counter - startTime;
    I2C_UpdateStats(result == I2C_OK, transactionTime);

    if (result != I2C_OK) {
        lastError = result;
    }

    return result;
}

/**
 * @brief Check if device is present at given address
 */
bool I2C_IsDevicePresent(uint8_t address) {
    /* Check if I2C is ready */
    if (!I2C_IsReady()) {
        return false;
    }

    currentState = I2C_STATE_BUSY_TX;

    /* Generate START */
    I2C1->CR1 |= I2C_CR1_START;

    /* Wait for SB with timeout */
    uint32_t timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--) {
        if (timeout == 0) {
            currentState = I2C_STATE_READY;
            return false;
        }
    }

    /* Send address */
    I2C1->DR = (address << 1);

    /* Wait for ADDR or AF */
    timeout = 10000;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && timeout--);

    bool device_found = false;

    if (I2C1->SR1 & I2C_SR1_ADDR) {
        /* Device responded - MUST clear ADDR */
        (void)I2C1->SR2;  /* CRITICAL! */
        device_found = true;
    } else if (I2C1->SR1 & I2C_SR1_AF) {
        /* Clear AF flag */
        I2C1->SR1 &= ~I2C_SR1_AF;
        device_found = false;
    }

    /* Generate STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Wait for BUSY to clear */
    while (I2C1->SR2 & I2C_SR2_BUSY);

    currentState = I2C_STATE_READY;

    return device_found;
}

/**
 * @brief Scan I2C bus for devices
 */
I2C_Error_t I2C_ScanBus(I2C_Device_t* deviceList, uint8_t maxDevices, uint8_t* detectedCount) {
    if (deviceList == NULL || detectedCount == NULL) {
        return I2C_ERROR_INVALID_PARAM;
    }

    *detectedCount = 0;

    UART_SendString("\r\n[I2C] Scanning bus for devices...\r\n");
    UART_SendString("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    for (uint8_t row = 0; row < 8; row++) {
        char line[80];
        snprintf(line, sizeof(line), "%02X: ", row * 16);
        UART_SendString(line);

        for (uint8_t col = 0; col < 16; col++) {
            uint8_t address = (row * 16) + col;

            /* Skip reserved addresses */
            if (address < 0x08 || address > 0x77) {
                UART_SendString("   ");
                continue;
            }

            if (I2C_IsDevicePresent(address)) {
                snprintf(line, sizeof(line), "%02X ", address);
                UART_SendString(line);

                /* Add to device list if space available */
                if (*detectedCount < maxDevices) {
                    deviceList[*detectedCount].address = address;
                    snprintf(deviceList[*detectedCount].name, sizeof(deviceList[*detectedCount].name),
                             "Device_%02X", address);
                    deviceList[*detectedCount].isDetected = true;
                    deviceList[*detectedCount].lastCommunication = systick_counter;
                    deviceList[*detectedCount].errorCount = 0;
                    (*detectedCount)++;
                }
            } else {
                UART_SendString("-- ");
            }
        }
        UART_SendString("\r\n");
    }

    char summary[100];
    snprintf(summary, sizeof(summary), "[I2C] Scan complete. Found %d devices.\r\n", *detectedCount);
    UART_SendString(summary);

    return I2C_OK;
}

/**
 * @brief Write data to I2C device
 */
I2C_Error_t I2C_Write(uint8_t address, const uint8_t* data, uint16_t size, uint32_t timeout_ms) {
    if (data == NULL || size == 0) {
        return I2C_ERROR_INVALID_PARAM;
    }

    uint32_t startTime = systick_counter;
    I2C_Error_t result = I2C_OK;

    if (!I2C_IsReady()) {
        return I2C_ERROR_BUSY;
    }

    currentState = I2C_STATE_BUSY_TX;

    /* Generate START */
    I2C1->CR1 |= I2C_CR1_START;
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Send address */
    I2C1->DR = (address << 1);
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Clear ADDR */
    (void)I2C1->SR2;

    /* Send data */
    for (uint16_t i = 0; i < size; i++) {
        I2C1->DR = data[i];
        result = I2C_WaitForFlag(I2C_SR1_BTF, true, timeout_ms);
        if (result != I2C_OK) goto cleanup;
    }

    /* Generate STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Wait for BUSY to clear */
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((systick_counter - startTime) > timeout_ms) {
            result = I2C_ERROR_TIMEOUT;
            goto cleanup;
        }
    }

cleanup:
    currentState = I2C_STATE_READY;
    I2C_UpdateStats(result == I2C_OK, systick_counter - startTime);
    if (result != I2C_OK) lastError = result;
    return result;
}

/**
 * @brief Read data from I2C device
 */
I2C_Error_t I2C_Read(uint8_t address, uint8_t* data, uint16_t size, uint32_t timeout_ms) {
    if (data == NULL || size == 0) {
        return I2C_ERROR_INVALID_PARAM;
    }

    uint32_t startTime = systick_counter;
    I2C_Error_t result = I2C_OK;

    if (!I2C_IsReady()) {
        return I2C_ERROR_BUSY;
    }

    currentState = I2C_STATE_BUSY_RX;

    /* Generate START */
    I2C1->CR1 |= I2C_CR1_START;
    result = I2C_WaitForFlag(I2C_SR1_SB, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Send address with read bit */
    I2C1->DR = (address << 1) | 1;
    result = I2C_WaitForFlagWithError(I2C_SR1_ADDR, true, timeout_ms);
    if (result != I2C_OK) goto cleanup;

    /* Handle different size cases */
    if (size == 1) {
        /* Single byte - disable ACK before clearing ADDR */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2;

        result = I2C_WaitForFlag(I2C_SR1_RXNE, true, timeout_ms);
        if (result != I2C_OK) goto cleanup;

        data[0] = I2C1->DR;
    } else {
        /* Multi-byte - enable ACK */
        I2C1->CR1 |= I2C_CR1_ACK;
        (void)I2C1->SR2;

        for (uint16_t i = 0; i < size; i++) {
            if (i == size - 1) {
                /* Last byte - disable ACK */
                I2C1->CR1 &= ~I2C_CR1_ACK;
            }

            result = I2C_WaitForFlag(I2C_SR1_RXNE, true, timeout_ms);
            if (result != I2C_OK) goto cleanup;

            data[i] = I2C1->DR;
        }
    }

    /* Generate STOP */
    I2C1->CR1 |= I2C_CR1_STOP;

    /* Wait for BUSY to clear */
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((systick_counter - startTime) > timeout_ms) {
            result = I2C_ERROR_TIMEOUT;
            goto cleanup;
        }
    }

cleanup:
    currentState = I2C_STATE_READY;
    I2C_UpdateStats(result == I2C_OK, systick_counter - startTime);
    if (result != I2C_OK) lastError = result;
    return result;
}

/**
 * @brief Write then read from I2C device
 */
I2C_Error_t I2C_WriteRead(uint8_t address, const uint8_t* writeData, uint16_t writeSize,
                         uint8_t* readData, uint16_t readSize, uint32_t timeout_ms) {
    if (writeData == NULL || readData == NULL || writeSize == 0 || readSize == 0) {
        return I2C_ERROR_INVALID_PARAM;
    }

    /* First write the data */
    I2C_Error_t result = I2C_Write(address, writeData, writeSize, timeout_ms);
    if (result != I2C_OK) {
        return result;
    }

    /* Then read the response */
    return I2C_Read(address, readData, readSize, timeout_ms);
}

/**
 * @brief Register a device for monitoring
 */
I2C_Error_t I2C_RegisterDevice(I2C_Device_t* device) {
    if (device == NULL || deviceCount >= 8) {
        return I2C_ERROR_INVALID_PARAM;
    }

    memcpy(&registeredDevices[deviceCount], device, sizeof(I2C_Device_t));
    deviceCount++;

    return I2C_OK;
}

/**
 * @brief Get I2C statistics
 */
void I2C_GetStatistics(I2C_Stats_t* stats) {
    if (stats != NULL) {
        memcpy(stats, &i2cStats, sizeof(I2C_Stats_t));
    }
}

/**
 * @brief Reset I2C statistics
 */
void I2C_ResetStatistics(void) {
    memset(&i2cStats, 0, sizeof(I2C_Stats_t));
    UART_SendString("[I2C] Statistics reset\r\n");
}

/**
 * @brief Print I2C status and statistics
 */
void I2C_PrintStatus(void) {
    char status_msg[200];

    UART_SendString("\r\n=== I2C Status ===\r\n");

    const char* stateStr[] = {"READY", "BUSY_TX", "BUSY_RX", "ERROR"};
    snprintf(status_msg, sizeof(status_msg),
             "State: %s, Last Error: %d\r\n", stateStr[currentState], lastError);
    UART_SendString(status_msg);

    snprintf(status_msg, sizeof(status_msg),
             "Statistics:\r\n"
             "  Total Transactions: %lu\r\n"
             "  Successful: %lu\r\n"
             "  Timeout Errors: %lu\r\n"
             "  NACK Errors: %lu\r\n"
             "  Bus Errors: %lu\r\n"
             "  Max Transaction Time: %lu ms\r\n",
             i2cStats.totalTransactions, i2cStats.successfulTransactions,
             i2cStats.timeoutErrors, i2cStats.nackErrors, i2cStats.busErrors,
             i2cStats.maxTransactionTime);
    UART_SendString(status_msg);

    /* Show registered devices */
    if (deviceCount > 0) {
        UART_SendString("Registered Devices:\r\n");
        for (uint8_t i = 0; i < deviceCount; i++) {
            snprintf(status_msg, sizeof(status_msg),
                     "  [%d] %s (0x%02X) - %s, Errors: %lu\r\n",
                     i, registeredDevices[i].name, registeredDevices[i].address,
                     registeredDevices[i].isDetected ? "Present" : "Not Found",
                     registeredDevices[i].errorCount);
            UART_SendString(status_msg);
        }
    }

    UART_SendString("==================\r\n");
}

/**
 * @brief Clear I2C error flags and recover
 */
I2C_Error_t I2C_ClearErrors(void) {
    /* Clear all error flags */
    I2C1->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);

    /* Reset state */
    currentState = I2C_STATE_READY;
    lastError = I2C_OK;

    return I2C_OK;
}

/**
 * @brief Reset I2C peripheral
 */
I2C_Error_t I2C_Reset(void) {
    UART_SendString("[I2C] Performing hard reset...\r\n");

    /* Disable and reset I2C */
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    /* Reinitialize with saved configuration */
    return I2C_Init(&configBackup);
}

/**
 * @brief Get last error
 */
I2C_Error_t I2C_GetLastError(void) {
    return lastError;
}

/**
 * @brief Prepare I2C for low power mode
 */
I2C_Error_t I2C_PrepareForLowPower(void) {
    /* Disable I2C peripheral to save power */
    I2C1->CR1 &= ~I2C_CR1_PE;
    return I2C_OK;
}

/**
 * @brief Restore I2C after low power mode
 */
I2C_Error_t I2C_RestoreFromLowPower(void) {
    /* Re-enable I2C peripheral */
    I2C1->CR1 |= I2C_CR1_PE;
    currentState = I2C_STATE_READY;
    return I2C_OK;
}

/* ====================== Private Functions ====================== */

/**
 * @brief Wait for flag with timeout
 */
static I2C_Error_t I2C_WaitForFlag(uint32_t flag, bool state, uint32_t timeout_ms) {
    uint32_t startTime = systick_counter;

    while (((I2C1->SR1 & flag) != 0) != state) {
        if ((systick_counter - startTime) > timeout_ms) {
            return I2C_ERROR_TIMEOUT;
        }
    }

    return I2C_OK;
}

/**
 * @brief Wait for flag with error checking
 */
static I2C_Error_t I2C_WaitForFlagWithError(uint32_t flag, bool state, uint32_t timeout_ms) {
    uint32_t startTime = systick_counter;

    while (((I2C1->SR1 & flag) != 0) != state) {
        /* Check for timeout */
        if ((systick_counter - startTime) > timeout_ms) {
            i2cStats.timeoutErrors++;
            return I2C_ERROR_TIMEOUT;
        }

        /* Check for errors */
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;  /* Clear AF flag */
            i2cStats.nackErrors++;
            return I2C_ERROR_NACK;
        }

        if (I2C1->SR1 & I2C_SR1_ARLO) {
            I2C1->SR1 &= ~I2C_SR1_ARLO;
            i2cStats.busErrors++;
            return I2C_ERROR_ARBITRATION_LOST;
        }

        if (I2C1->SR1 & I2C_SR1_BERR) {
            I2C1->SR1 &= ~I2C_SR1_BERR;
            i2cStats.busErrors++;
            return I2C_ERROR_BUS_ERROR;
        }
    }

    return I2C_OK;
}

/**
 * @brief Update I2C statistics
 */
static void I2C_UpdateStats(bool success, uint32_t transactionTime) {
    i2cStats.totalTransactions++;

    if (success) {
        i2cStats.successfulTransactions++;
    }

    if (transactionTime > i2cStats.maxTransactionTime) {
        i2cStats.maxTransactionTime = transactionTime;
    }

    /* Update average transaction time */
    if (i2cStats.totalTransactions > 0) {
        i2cStats.averageTransactionTime =
            (i2cStats.averageTransactionTime * (i2cStats.totalTransactions - 1) + transactionTime) /
            i2cStats.totalTransactions;
    }
}



