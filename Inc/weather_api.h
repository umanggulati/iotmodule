/**
 * @file weather_api.h
 * @brief Simple Weather Station API - Easy-to-Use Interface
 * @desc User-friendly API that hides all the complexity of the weather station
 * @version 1.0
 * @date 2025
 *
 * USAGE EXAMPLE:
 *
 *   // Initialize everything
 *   if (!WeatherStation_Init()) {
 *       printf("Failed to initialize!\n");
 *       return;
 *   }
 *
 *   // Wait for WiFi connection
 *   while (!WeatherStation_IsConnected()) {
 *       WeatherStation_Delay(1000);
 *   }
 *
 *   // Get weather reading
 *   WeatherReading_t reading = WeatherStation_GetReading();
 *   if (reading.data_valid) {
 *       printf("Temperature: %.2f°C, Pressure: %d hPa\n",
 *              reading.temperature_celsius, reading.pressure_hpa);
 *
 *       // Send to cloud
 *       WeatherStation_SendToCloud(&reading);
 *   }
 */

#ifndef WEATHER_API_H
#define WEATHER_API_H

#include <stdint.h>
#include <stdbool.h>

/* ====================== Public Data Types ====================== */

/**
 * @brief Weather reading structure - simple and easy to use
 */
typedef struct {
    float temperature_celsius;      /* Temperature in Celsius (e.g., 25.6) */
    int pressure_hpa;              /* Pressure in hPa (e.g., 1013) */
    char timestamp[20];            /* Human readable timestamp "2025-01-15 14:30" */
    bool data_valid;               /* true if reading is valid */
    uint32_t reading_number;       /* Sequential reading count */
} WeatherReading_t;

/**
 * @brief System status information
 */
typedef struct {
    bool sensor_working;           /* BMP280 sensor status */
    bool wifi_connected;          /* WiFi connection status */
    char wifi_ip[16];             /* Current IP address */
    uint32_t total_readings;      /* Total readings taken */
    uint32_t successful_uploads;  /* Successful cloud uploads */
    uint32_t failed_uploads;      /* Failed cloud uploads */
    uint32_t uptime_seconds;      /* System uptime in seconds */
} WeatherStatus_t;

/**
 * @brief Configuration options for weather station
 */
typedef struct {
    uint32_t reading_interval_ms;     /* How often to read sensors (default: 30000) */
    uint32_t upload_interval_ms;      /* How often to upload to cloud (default: 300000) */
    bool auto_upload_enabled;         /* Enable automatic cloud uploads */
    bool debug_output_enabled;        /* Enable debug messages */
} WeatherConfig_t;

/* ====================== Core API Functions ====================== */

/**
 * @brief Initialize the entire weather station system
 * @return bool: true if initialization successful, false if failed
 * @note This function initializes all subsystems:
 *       - Hardware (LEDs, I2C, UART)
 *       - BMP280 sensor
 *       - ESP32 WiFi module
 *       - Task scheduler
 *       - Power management
 */
bool WeatherStation_Init(void);

/**
 * @brief Get a weather reading from sensors
 * @return WeatherReading_t: Structure containing weather data
 * @note This function blocks until reading is complete (usually <100ms)
 *       Check data_valid field before using the data
 */
WeatherReading_t WeatherStation_GetReading(void);

/**
 * @brief Send weather data to cloud service
 * @param reading: Pointer to weather reading structure
 * @return bool: true if upload successful, false if failed
 * @note This function blocks until upload is complete or timeout
 *       Requires WiFi connection to work
 */
bool WeatherStation_SendToCloud(WeatherReading_t* reading);

/**
 * @brief Check if WiFi is connected with valid IP
 * @return bool: true if connected, false if not connected
 */
bool WeatherStation_IsConnected(void);

/**
 * @brief Get comprehensive system status
 * @param status: Pointer to status structure to fill
 * @return bool: true if status retrieved successfully
 */
bool WeatherStation_GetStatus(WeatherStatus_t* status);

/* ====================== Configuration Functions ====================== */

/**
 * @brief Configure weather station behavior
 * @param config: Pointer to configuration structure
 * @return bool: true if configuration applied successfully
 */
bool WeatherStation_Configure(WeatherConfig_t* config);

/**
 * @brief Get current configuration
 * @param config: Pointer to configuration structure to fill
 * @return bool: true if configuration retrieved successfully
 */
bool WeatherStation_GetConfig(WeatherConfig_t* config);

/**
 * @brief Reset configuration to defaults
 * @return bool: true if reset successful
 */
bool WeatherStation_ResetConfig(void);

/* ====================== Automatic Operation Functions ====================== */

/**
 * @brief Start automatic weather monitoring
 * @return bool: true if started successfully
 * @note Starts task scheduler for automatic sensor reading and cloud uploads
 */
bool WeatherStation_StartAutoMode(void);

/**
 * @brief Stop automatic weather monitoring
 * @return bool: true if stopped successfully
 */
bool WeatherStation_StopAutoMode(void);

/**
 * @brief Check if automatic mode is running
 * @return bool: true if auto mode is active
 */
bool WeatherStation_IsAutoModeActive(void);

/* ====================== WiFi Management Functions ====================== */

/**
 * @brief Connect to WiFi network (blocking)
 * @param ssid: WiFi network name (if NULL, uses default from config)
 * @param password: WiFi password (if NULL, uses default from config)
 * @param timeout_seconds: Maximum time to wait for connection
 * @return bool: true if connected successfully
 */
bool WeatherStation_ConnectWiFi(const char* ssid, const char* password, uint32_t timeout_seconds);

/**
 * @brief Disconnect from WiFi network
 * @return bool: true if disconnected successfully
 */
bool WeatherStation_DisconnectWiFi(void);

/**
 * @brief Get WiFi signal strength
 * @return int: RSSI value in dBm (-100 to 0, higher is better)
 */
int WeatherStation_GetWiFiSignalStrength(void);

/* ====================== Utility Functions ====================== */

/**
 * @brief Simple delay function (milliseconds)
 * @param delay_ms: Delay time in milliseconds
 */
void WeatherStation_Delay(uint32_t delay_ms);

/**
 * @brief Print comprehensive system status to debug output
 * @return None
 */
void WeatherStation_PrintStatus(void);

/**
 * @brief Print weather reading in human-readable format
 * @param reading: Pointer to weather reading structure
 * @return None
 */
void WeatherStation_PrintReading(WeatherReading_t* reading);

/**
 * @brief Test all system components
 * @return bool: true if all tests pass, false if any fail
 * @note Useful for debugging and system validation
 */
bool WeatherStation_RunSystemTest(void);

/**
 * @brief Get last error message
 * @return const char*: Pointer to last error string
 */
const char* WeatherStation_GetLastError(void);

/* ====================== Advanced Functions ====================== */

/**
 * @brief Process weather station tasks manually
 * @return None
 * @note Call this in your main loop if not using auto mode
 *       Handles background WiFi management and sensor processing
 */
void WeatherStation_Process(void);

/**
 * @brief Set custom cloud endpoint
 * @param server: Server hostname or IP
 * @param port: Server port number
 * @param endpoint: API endpoint path
 * @return bool: true if configuration successful
 */
bool WeatherStation_SetCloudEndpoint(const char* server, uint16_t port, const char* endpoint);

/**
 * @brief Enable/disable specific features
 * @param feature: Feature to control (see WeatherFeature_t enum)
 * @param enable: true to enable, false to disable
 * @return bool: true if configuration successful
 */
typedef enum {
    WEATHER_FEATURE_AUTO_UPLOAD,
    WEATHER_FEATURE_DEBUG_OUTPUT,
    WEATHER_FEATURE_LED_INDICATORS,
    WEATHER_FEATURE_POWER_SAVING
} WeatherFeature_t;

bool WeatherStation_SetFeature(WeatherFeature_t feature, bool enable);

/* ====================== Default Configuration Values ====================== */

#define WEATHER_DEFAULT_READ_INTERVAL_MS        30000   /* 30 seconds */
#define WEATHER_DEFAULT_UPLOAD_INTERVAL_MS      300000  /* 5 minutes */
#define WEATHER_DEFAULT_WIFI_TIMEOUT_SEC        30      /* 30 seconds */
#define WEATHER_MAX_RETRIES                     3       /* Max retry attempts */

/* ====================== Error Codes ====================== */

typedef enum {
    WEATHER_ERROR_NONE = 0,
    WEATHER_ERROR_SENSOR_INIT,
    WEATHER_ERROR_WIFI_INIT,
    WEATHER_ERROR_SENSOR_READ,
    WEATHER_ERROR_WIFI_CONNECT,
    WEATHER_ERROR_CLOUD_UPLOAD,
    WEATHER_ERROR_INVALID_PARAM,
    WEATHER_ERROR_NOT_INITIALIZED,
    WEATHER_ERROR_TIMEOUT
} WeatherError_t;

#endif /* WEATHER_API_H */
