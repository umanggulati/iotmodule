/**
 * @file weather_api.c
 * @brief Simple Weather Station API Implementation
 * @desc User-friendly API implementation that wraps the complex underlying system
 */

#include "weather_api.h"
#include "uart.h"
#include "systick.h"
#include "system_config.h"
#include "i2c.h"
#include "esp32_wifi.h"
#include "task_scheduler.h"
#include "power_management.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <wifi_config_template.h>

/* ====================== External Variables ====================== */
/* These come from your existing main.c */
extern int32_t current_temperature;
extern int32_t current_pressure;
extern uint32_t reading_count;
extern uint32_t successful_cloud_updates;
extern uint32_t failed_cloud_updates;
extern bool task_scheduler_enabled;

/* External functions from your existing code */
extern bool BMP280_Init(void);
extern bool BMP280_Read(int32_t *temp_out, int32_t *press_out);
extern void Manual_Cloud_Send_WithResponse(void);
extern bool WiFi_IsConnected(void);
extern void LED_Init(void);
extern void System_Init(void);

/* ====================== Private Variables ====================== */
static bool weather_api_initialized = false;
static WeatherConfig_t current_config;
static char last_error_message[100] = "No error";
static uint32_t initialization_time = 0;

/* ====================== Private Function Prototypes ====================== */
static void WeatherAPI_SetError(const char* error_msg);
static void WeatherAPI_FormatTimestamp(char* buffer, uint32_t timestamp);
static void WeatherAPI_InitializeDefaultConfig(void);

/* ====================== Core API Functions ====================== */

/**
 * @brief Initialize the entire weather station system
 */
bool WeatherStation_Init(void) {
    UART_SendString("\r\n=== Weather Station API Initialization ===\r\n");

    /* Initialize default configuration first */
    WeatherAPI_InitializeDefaultConfig();

    /* Call your existing system initialization */
    System_Init();

    /* Additional API-specific initialization */
    initialization_time = systick_counter;

    /* Test critical components */

    /* Test 1: Check if BMP280 sensor is working */
    int32_t test_temp, test_pressure;
    if (!BMP280_Read(&test_temp, &test_pressure)) {
        WeatherAPI_SetError("BMP280 sensor initialization failed");
        UART_SendString("[WEATHER_API] ❌ Sensor test failed!\r\n");
        return false;
    }
    UART_SendString("[WEATHER_API] ✅ BMP280 sensor working\r\n");

    /* Test 2: Check ESP32 communication */
    if (ESP32_WiFi_Test() != ESP32_OK) {
        WeatherAPI_SetError("ESP32 WiFi module communication failed");
        UART_SendString("[WEATHER_API] ❌ ESP32 communication failed!\r\n");
        return false;
    }
    UART_SendString("[WEATHER_API] ✅ ESP32 communication working\r\n");

    /* Mark as initialized */
    weather_api_initialized = true;
    WeatherAPI_SetError("No error - system initialized successfully");

    UART_SendString("[WEATHER_API] 🎉 Weather Station API initialized successfully!\r\n");
    UART_SendString("===========================================\r\n");

    return true;
}

/**
 * @brief Get a weather reading from sensors
 */
WeatherReading_t WeatherStation_GetReading(void) {
    WeatherReading_t reading = {0};

    if (!weather_api_initialized) {
        WeatherAPI_SetError("Weather station not initialized");
        reading.data_valid = false;
        return reading;
    }

    /* Read from BMP280 sensor using your existing function */
    int32_t raw_temp, raw_pressure;
    if (BMP280_Read(&raw_temp, &raw_pressure)) {
        /* Convert your internal format to user-friendly format */
        reading.temperature_celsius = (float)raw_temp / 100.0f;  /* Convert from hundredths */
        reading.pressure_hpa = 1013 + ((raw_pressure - 393500) / 400);  /* Your pressure calculation */
        reading.reading_number = reading_count + 1;
        reading.data_valid = true;

        /* Generate human-readable timestamp */
        WeatherAPI_FormatTimestamp(reading.timestamp, systick_counter / 1000);

        /* Update global counters (for compatibility with existing code) */
        current_temperature = raw_temp;
        current_pressure = reading.pressure_hpa;
        reading_count++;

        WeatherAPI_SetError("Reading successful");

    } else {
        WeatherAPI_SetError("Failed to read sensor data");
        reading.data_valid = false;
    }

    return reading;
}

/**
 * @brief Send weather data to cloud service
 */
bool WeatherStation_SendToCloud(WeatherReading_t* reading) {
    if (!weather_api_initialized) {
        WeatherAPI_SetError("Weather station not initialized");
        return false;
    }

    if (reading == NULL) {
        WeatherAPI_SetError("Invalid reading data");
        return false;
    }

    if (!reading->data_valid) {
        WeatherAPI_SetError("Invalid sensor data - cannot upload");
        return false;
    }

    if (!WiFi_IsConnected()) {
        WeatherAPI_SetError("WiFi not connected");
        return false;
    }

    /* Update global variables to match the reading (for existing cloud function) */
    current_temperature = (int32_t)(reading->temperature_celsius * 100);
    current_pressure = reading->pressure_hpa;

    /* Use your existing cloud upload function */
    uint32_t initial_successful = successful_cloud_updates;
    Manual_Cloud_Send_WithResponse();

    /* Check if upload was successful by comparing counters */
    if (successful_cloud_updates > initial_successful) {
        WeatherAPI_SetError("Cloud upload successful");
        return true;
    } else {
        WeatherAPI_SetError("Cloud upload failed");
        return false;
    }
}

/**
 * @brief Check if WiFi is connected
 */
bool WeatherStation_IsConnected(void) {
    if (!weather_api_initialized) {
        return false;
    }

    return WiFi_IsConnected();  /* Use your existing function */
}

/**
 * @brief Get comprehensive system status
 */
bool WeatherStation_GetStatus(WeatherStatus_t* status) {
    if (!weather_api_initialized || status == NULL) {
        return false;
    }

    /* Test sensor by trying to read */
    int32_t test_temp, test_pressure;
    status->sensor_working = BMP280_Read(&test_temp, &test_pressure);

    /* Get WiFi status */
    status->wifi_connected = WiFi_IsConnected();

    /* Get IP address */
    if (status->wifi_connected) {
        ESP32_WiFi_GetIP(status->wifi_ip);
    } else {
        strcpy(status->wifi_ip, "0.0.0.0");
    }

    /* Get statistics from global variables */
    status->total_readings = reading_count;
    status->successful_uploads = successful_cloud_updates;
    status->failed_uploads = failed_cloud_updates;

    /* Calculate uptime */
    status->uptime_seconds = (systick_counter - initialization_time) / 1000;

    return true;
}

/* ====================== Configuration Functions ====================== */

/**
 * @brief Configure weather station behavior
 */
bool WeatherStation_Configure(WeatherConfig_t* config) {
    if (!weather_api_initialized || config == NULL) {
        WeatherAPI_SetError("Invalid configuration or not initialized");
        return false;
    }

    /* Validate configuration values */
    if (config->reading_interval_ms < 1000 || config->reading_interval_ms > 3600000) {
        WeatherAPI_SetError("Reading interval must be between 1 second and 1 hour");
        return false;
    }

    if (config->upload_interval_ms < 10000 || config->upload_interval_ms > 86400000) {
        WeatherAPI_SetError("Upload interval must be between 10 seconds and 24 hours");
        return false;
    }

    /* Apply configuration */
    memcpy(&current_config, config, sizeof(WeatherConfig_t));

    WeatherAPI_SetError("Configuration applied successfully");
    return true;
}

/**
 * @brief Get current configuration
 */
bool WeatherStation_GetConfig(WeatherConfig_t* config) {
    if (!weather_api_initialized || config == NULL) {
        return false;
    }

    memcpy(config, &current_config, sizeof(WeatherConfig_t));
    return true;
}

/**
 * @brief Reset configuration to defaults
 */
bool WeatherStation_ResetConfig(void) {
    if (!weather_api_initialized) {
        return false;
    }

    WeatherAPI_InitializeDefaultConfig();
    WeatherAPI_SetError("Configuration reset to defaults");
    return true;
}

/* ====================== Automatic Operation Functions ====================== */

/**
 * @brief Start automatic weather monitoring
 */
bool WeatherStation_StartAutoMode(void) {
    if (!weather_api_initialized) {
        WeatherAPI_SetError("Weather station not initialized");
        return false;
    }

    /* Start task scheduler using your existing system */
    if (!task_scheduler_enabled) {
        /* Start all registered tasks */
        for (uint8_t i = 0; i < TaskScheduler_GetTaskCount(); i++) {
            TaskScheduler_StartTask(i);
        }
        task_scheduler_enabled = true;

        UART_SendString("[WEATHER_API] 🚀 Automatic mode started\r\n");
        WeatherAPI_SetError("Automatic mode started successfully");
        return true;
    } else {
        WeatherAPI_SetError("Automatic mode already active");
        return false;
    }
}

/**
 * @brief Stop automatic weather monitoring
 */
bool WeatherStation_StopAutoMode(void) {
    if (!weather_api_initialized) {
        return false;
    }

    if (task_scheduler_enabled) {
        /* Stop all tasks */
        for (uint8_t i = 0; i < TaskScheduler_GetTaskCount(); i++) {
            TaskScheduler_StopTask(i);
        }
        task_scheduler_enabled = false;

        UART_SendString("[WEATHER_API] 🛑 Automatic mode stopped\r\n");
        WeatherAPI_SetError("Automatic mode stopped successfully");
        return true;
    } else {
        WeatherAPI_SetError("Automatic mode not active");
        return false;
    }
}

/**
 * @brief Check if automatic mode is running
 */
bool WeatherStation_IsAutoModeActive(void) {
    return weather_api_initialized && task_scheduler_enabled;
}

/* ====================== WiFi Management Functions ====================== */

/**
 * @brief Connect to WiFi network (blocking)
 */
bool WeatherStation_ConnectWiFi(const char* ssid, const char* password, uint32_t timeout_seconds) {
    if (!weather_api_initialized) {
        WeatherAPI_SetError("Weather station not initialized");
        return false;
    }

    /* Use your existing WiFi connection function */
    /* Note: For custom SSID/password, you'd need to modify ESP32_WiFi_Connect() */
    /* For now, using the configured credentials */

    UART_SendString("[WEATHER_API] Connecting to WiFi...\r\n");

    uint32_t start_time = systick_counter;
    ESP32_Error_t result = ESP32_WiFi_Connect();

    if (result == ESP32_OK) {
        /* Wait for actual connection with timeout */
        while ((systick_counter - start_time) < (timeout_seconds * 1000)) {
            if (WiFi_IsConnected()) {
                UART_SendString("[WEATHER_API] ✅ WiFi connected successfully!\r\n");
                WeatherAPI_SetError("WiFi connection successful");
                return true;
            }
            WeatherStation_Delay(1000);
        }
    }

    WeatherAPI_SetError("WiFi connection failed or timed out");
    UART_SendString("[WEATHER_API] ❌ WiFi connection failed!\r\n");
    return false;
}

/**
 * @brief Disconnect from WiFi network
 */
bool WeatherStation_DisconnectWiFi(void) {
    if (!weather_api_initialized) {
        return false;
    }

    ESP32_Error_t result = ESP32_WiFi_Disconnect();
    if (result == ESP32_OK) {
        WeatherAPI_SetError("WiFi disconnected successfully");
        return true;
    } else {
        WeatherAPI_SetError("WiFi disconnection failed");
        return false;
    }
}

/**
 * @brief Get WiFi signal strength
 */
int WeatherStation_GetWiFiSignalStrength(void) {
    if (!weather_api_initialized || !WiFi_IsConnected()) {
        return -100;  /* No signal */
    }

    int32_t rssi;
    if (ESP32_WiFi_GetRSSI(&rssi) == ESP32_OK) {
        return (int)rssi;
    }

    return -100;  /* Failed to get RSSI */
}

/* ====================== Utility Functions ====================== */

/**
 * @brief Simple delay function
 */
void WeatherStation_Delay(uint32_t delay_ms) {
    SysTick_Delay(delay_ms);
}

/**
 * @brief Print comprehensive system status
 */
void WeatherStation_PrintStatus(void) {
    if (!weather_api_initialized) {
        UART_SendString("[WEATHER_API] System not initialized!\r\n");
        return;
    }

    WeatherStatus_t status;
    if (WeatherStation_GetStatus(&status)) {
        UART_SendString("\r\n=== Weather Station API Status ===\r\n");

        char msg[150];
        snprintf(msg, sizeof(msg), "Sensor Status: %s\r\n",
                 status.sensor_working ? "✅ Working" : "❌ Failed");
        UART_SendString(msg);

        snprintf(msg, sizeof(msg), "WiFi Status: %s\r\n",
                 status.wifi_connected ? "✅ Connected" : "❌ Disconnected");
        UART_SendString(msg);

        if (status.wifi_connected) {
            snprintf(msg, sizeof(msg), "IP Address: %s\r\n", status.wifi_ip);
            UART_SendString(msg);

            int signal = WeatherStation_GetWiFiSignalStrength();
            snprintf(msg, sizeof(msg), "Signal Strength: %d dBm\r\n", signal);
            UART_SendString(msg);
        }

        snprintf(msg, sizeof(msg), "Total Readings: %lu\r\n", status.total_readings);
        UART_SendString(msg);

        snprintf(msg, sizeof(msg), "Cloud Uploads: %lu successful, %lu failed\r\n",
                 status.successful_uploads, status.failed_uploads);
        UART_SendString(msg);

        snprintf(msg, sizeof(msg), "Uptime: %lu seconds\r\n", status.uptime_seconds);
        UART_SendString(msg);

        snprintf(msg, sizeof(msg), "Auto Mode: %s\r\n",
                 WeatherStation_IsAutoModeActive() ? "✅ Active" : "❌ Inactive");
        UART_SendString(msg);

        UART_SendString("================================\r\n");
    }
}

/**
 * @brief Print weather reading in human-readable format
 */
void WeatherStation_PrintReading(WeatherReading_t* reading) {
    if (reading == NULL) {
        UART_SendString("[WEATHER_API] Invalid reading data\r\n");
        return;
    }

    if (!reading->data_valid) {
        UART_SendString("[WEATHER_API] ❌ Invalid sensor data\r\n");
        return;
    }

    char msg[200];
    snprintf(msg, sizeof(msg),
             "[WEATHER_API] 📊 Reading #%lu: %.2f°C, %d hPa at %s\r\n",
             reading->reading_number, reading->temperature_celsius,
             reading->pressure_hpa, reading->timestamp);
    UART_SendString(msg);
}

/**
 * @brief Test all system components
 */
bool WeatherStation_RunSystemTest(void) {
    UART_SendString("\r\n=== Weather Station System Test ===\r\n");

    bool all_tests_passed = true;

    /* Test 1: API Initialization */
    if (!weather_api_initialized) {
        UART_SendString("❌ Test 1: API not initialized\r\n");
        all_tests_passed = false;
    } else {
        UART_SendString("✅ Test 1: API initialized\r\n");
    }

    /* Test 2: Sensor Reading */
    WeatherReading_t test_reading = WeatherStation_GetReading();
    if (test_reading.data_valid) {
        UART_SendString("✅ Test 2: Sensor reading successful\r\n");
        WeatherStation_PrintReading(&test_reading);
    } else {
        UART_SendString("❌ Test 2: Sensor reading failed\r\n");
        all_tests_passed = false;
    }

    /* Test 3: WiFi Communication */
    if (ESP32_WiFi_Test() == ESP32_OK) {
        UART_SendString("✅ Test 3: ESP32 communication working\r\n");
    } else {
        UART_SendString("❌ Test 3: ESP32 communication failed\r\n");
        all_tests_passed = false;
    }

    /* Test 4: System Status */
    WeatherStatus_t status;
    if (WeatherStation_GetStatus(&status)) {
        UART_SendString("✅ Test 4: System status retrieval working\r\n");
    } else {
        UART_SendString("❌ Test 4: System status retrieval failed\r\n");
        all_tests_passed = false;
    }

    UART_SendString("================================\r\n");

    if (all_tests_passed) {
        UART_SendString("🎉 All tests PASSED! System is working correctly.\r\n");
    } else {
        UART_SendString("⚠️  Some tests FAILED! Check system configuration.\r\n");
    }

    return all_tests_passed;
}

/**
 * @brief Get last error message
 */
const char* WeatherStation_GetLastError(void) {
    return last_error_message;
}

/**
 * @brief Process weather station tasks manually
 */
void WeatherStation_Process(void) {
    if (!weather_api_initialized) {
        return;
    }

    /* Run task scheduler if enabled */
    if (task_scheduler_enabled) {
        TaskScheduler_RunDispatcher();
    }
}

/* ====================== Private Helper Functions ====================== */

/**
 * @brief Set error message for debugging
 */
static void WeatherAPI_SetError(const char* error_msg) {
    if (error_msg != NULL) {
        strncpy(last_error_message, error_msg, sizeof(last_error_message) - 1);
        last_error_message[sizeof(last_error_message) - 1] = '\0';
    }
}

/**
 * @brief Format timestamp into human-readable string
 */
static void WeatherAPI_FormatTimestamp(char* buffer, uint32_t timestamp) {
    if (buffer == NULL) return;

    /* Simple timestamp format: "Day HH:MM:SS" */
    uint32_t seconds = timestamp % 60;
    uint32_t minutes = (timestamp / 60) % 60;
    uint32_t hours = (timestamp / 3600) % 24;
    uint32_t days = timestamp / 86400;

    snprintf(buffer, 20, "Day%lu %02lu:%02lu:%02lu", days, hours, minutes, seconds);
}

/**
 * @brief Initialize default configuration
 */
static void WeatherAPI_InitializeDefaultConfig(void) {
    current_config.reading_interval_ms = WEATHER_DEFAULT_READ_INTERVAL_MS;
    current_config.upload_interval_ms = WEATHER_DEFAULT_UPLOAD_INTERVAL_MS;
    current_config.auto_upload_enabled = true;
    current_config.debug_output_enabled = true;
}
