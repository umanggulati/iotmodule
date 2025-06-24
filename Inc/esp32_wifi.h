/**
 * @file esp32_wifi.h
 * @brief ESP32 WiFi module interface - FIXED VERSION WITH PUBLIC FUNCTIONS
 * @desc WiFi connectivity with exposed communication functions for main.c
 */

#ifndef ESP32_WIFI_H
#define ESP32_WIFI_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ESP32 Response Codes */
typedef enum {
    ESP32_OK = 0,
    ESP32_ERROR,
    ESP32_TIMEOUT,
    ESP32_BUSY,
    ESP32_NO_RESPONSE,
    ESP32_INVALID_PARAM,
    ESP32_NOT_CONNECTED,
    ESP32_SEND_FAIL
} ESP32_Error_t;

/* WiFi Connection Status */
typedef enum {
    WIFI_STATUS_IDLE = 0,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_GOT_IP,
    WIFI_STATUS_DISCONNECTED,
    WIFI_STATUS_ERROR
} WiFi_Status_t;

/* Weather Data Structure */
typedef struct {
    int32_t temperature;    /* Temperature in hundredths of degree C */
    int32_t pressure;       /* Pressure in hPa */
    int32_t humidity;       /* Humidity in % (if available) */
    uint32_t timestamp;     /* Timestamp in seconds */
} WeatherData_t;

/* ====================== Core WiFi Functions ====================== */

/**
 * @brief Initialize ESP32 WiFi module
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_Init(void);

/**
 * @brief Reset ESP32 module
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_Reset(void);

/**
 * @brief Test ESP32 communication with AT command
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_Test(void);

/**
 * @brief Connect to WiFi network using credentials from wifi_config.h
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_Connect(void);

/**
 * @brief Disconnect from WiFi network
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_Disconnect(void);

/**
 * @brief Check if connected to WiFi with valid IP
 * @return bool: true if connected with valid IP
 */
bool ESP32_WiFi_IsConnected(void);

/**
 * @brief Check connection by verifying IP address
 * @return bool: true if connected with valid IP
 */
bool ESP32_WiFi_CheckConnection(void);

/**
 * @brief Get current WiFi connection status
 * @return WiFi_Status_t: Current status
 */
WiFi_Status_t ESP32_WiFi_GetStatus(void);

/**
 * @brief Get IP address as string
 * @param ip_buffer: Buffer to store IP string (min 16 bytes)
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_GetIP(char* ip_buffer);

/* ====================== Data Transmission Functions ====================== */

/**
 * @brief Send weather data to cloud server (STUB - not used)
 * @param data: Pointer to weather data structure
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_SendWeatherData(const WeatherData_t* data);

/* ====================== Status and Utility Functions ====================== */

/**
 * @brief Print WiFi status via UART
 * @return None
 */
void ESP32_WiFi_PrintStatus(void);

/**
 * @brief Get WiFi signal strength (RSSI)
 * @param rssi: Pointer to store RSSI value
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_GetRSSI(int32_t* rssi);

/**
 * @brief Configure ESP32 for low power mode
 * @param enable: true to enable low power
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_SetLowPower(bool enable);

/**
 * @brief Test TCP connectivity
 * @return ESP32_Error_t: Operation result
 */
ESP32_Error_t ESP32_WiFi_TestTCP(void);

/**
 * @brief Handle ESP32 background tasks (call periodically)
 * @return None
 */
void ESP32_WiFi_Process(void);

/* ====================== PUBLIC COMMUNICATION FUNCTIONS ====================== */
/* These are now public and can be called from main.c for inline communication */

/**
 * @brief Send string via USART2
 * @param str: String to send
 */
void ESP32_Send(const char* str);

/**
 * @brief Send command line with CRLF
 * @param cmd: Command to send
 */
void ESP32_SendLine(const char* cmd);

/**
 * @brief Check if data available in RX buffer
 * @return bool: true if data available
 */
bool ESP32_DataAvailable(void);

/**
 * @brief Read byte from RX buffer
 * @return uint8_t: Received byte
 */
uint8_t ESP32_ReadByte(void);

/**
 * @brief Clear RX buffer
 */
void ESP32_ClearBuffer(void);

/**
 * @brief Wait for specific response
 * @param expected: Expected response string
 * @param timeout_ms: Timeout in milliseconds
 * @return bool: true if response received
 */
bool ESP32_WaitForResponse(const char* expected, uint32_t timeout_ms);

/**
 * @brief Read all available data with timeout reset on each byte
 * @param buffer: Buffer to store data
 * @param max_size: Maximum buffer size
 * @param timeout_ms: Timeout in milliseconds
 * @note FIXED VERSION: Resets timeout on each received byte
 */
void ESP32_ReadAll(char* buffer, uint16_t max_size, uint32_t timeout_ms);

#endif /* ESP32_WIFI_H */
