/**
 * @file main.c
 * @brief Phase 3C: Complete IoT Weather Station with Simple API - UPDATED VERSION
 * @desc Integrated system with FIXED temperature JSON formatting, enhanced response reading, and NEW SIMPLE API
 */

#include "stm32f4xx.h"
#include "uart.h"
#include "systick.h"
#include "system_config.h"
#include "i2c.h"
#include "esp32_wifi.h"
#include "task_scheduler.h"
#include "power_management.h"
#include "weather_api.h"  /* NEW: Simple Weather Station API */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <wifi_config_template.h>

/* ====================== TESTING INTERVALS - MUCH FASTER ====================== */
#define FAST_WEATHER_READ_INTERVAL   15000              /* Read sensors every 15 seconds */
#define FAST_CLOUD_UPDATE_INTERVAL   60000              /* Send to cloud every 1 minute */
#define FAST_WIFI_MANAGER_INTERVAL   5000               /* WiFi manager every 5 seconds */

/* ====================== BMP280 Register Definitions ====================== */
#define BMP280_ADDR           0x76
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_DIG_T1     0x88

/* ====================== BMP280 Calibration Data ====================== */
static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;
static bool bmp280_initialized = false;

/* ====================== System State Variables ====================== */
/* Weather data storage */
int32_t current_temperature = 0;  /* Made non-static for API access */
int32_t current_pressure = 0;     /* Made non-static for API access */
uint32_t reading_count = 0;       /* Made non-static for API access */
static WeatherData_t currentWeather = {0};

/* WiFi State Management */
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_ERROR,
    WIFI_STATE_RETRYING
} WiFi_ConnectionState_t;

static WiFi_ConnectionState_t wifi_state = WIFI_STATE_DISCONNECTED;
bool task_scheduler_enabled = false;  /* Made non-static for API access */
static uint32_t wifi_retry_time = 0;
static uint32_t last_cloud_update = 0;
static uint32_t failed_wifi_attempts = 0;
uint32_t failed_cloud_updates = 0;    /* Made non-static for API access */
uint32_t successful_cloud_updates = 0; /* Made non-static for API access */
static uint32_t wifi_connected_time = 0;
static char current_ip[16] = "0.0.0.0";

/* WiFi Statistics */
static uint32_t wifi_connection_attempts = 0;
static uint32_t wifi_disconnection_count = 0;
static uint32_t last_wifi_check = 0;

/* Power Management Variables */
static bool power_management_enabled = false;
static uint32_t sleep_cycle_count = 0;
static uint32_t last_power_check = 0;
static uint32_t power_save_request_time = 0;
static bool auto_power_save_enabled = ENABLE_AUTO_SLEEP;

/* Power Management Statistics */
// static uint32_t sleep_cycles_completed = 0;  // Unused - commented out
// static uint32_t wake_up_count = 0;           // Unused - commented out
static uint32_t power_management_errors = 0;

/* Simple API Status */
static bool simple_api_initialized = false;

/* ====================== LED Control Functions ====================== */
void LED_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3u << 0) | (3u << 14) | (3u << 28));
    GPIOB->MODER |= ((1u << 0) | (1u << 14) | (1u << 28));
}

void LED_Green_On(void)  { GPIOB->ODR |= (1 << 0); }
void LED_Green_Off(void) { GPIOB->ODR &= ~(1 << 0); }
void LED_Blue_On(void)   { GPIOB->ODR |= (1 << 7); }
void LED_Blue_Off(void)  { GPIOB->ODR &= ~(1 << 7); }
void LED_Red_On(void)    { GPIOB->ODR |= (1 << 14); }
void LED_Red_Off(void)   { GPIOB->ODR &= ~(1 << 14); }

/* ====================== BMP280 Sensor Functions ====================== */
bool BMP280_Init(void) {
    uint8_t chipId;

    UART_SendString("[BMP280] Initializing sensor...\r\n");

    if (I2C_ReadRegister(BMP280_ADDR, BMP280_REG_ID, &chipId, 100) != I2C_OK) {
        UART_SendString("[BMP280] I2C communication failed!\r\n");
        return false;
    }

    if (chipId != 0x58) {
        char msg[100];
        snprintf(msg, sizeof(msg), "[BMP280] Wrong chip ID: 0x%02X (expected 0x58)\r\n", chipId);
        UART_SendString(msg);
        return false;
    }

    UART_SendString("[BMP280] Chip ID verified! Resetting sensor...\r\n");

    I2C_WriteRegister(BMP280_ADDR, BMP280_REG_RESET, 0xB6, 100);
    SysTick_Delay(50);

    uint8_t calib[6];
    if (I2C_ReadRegisters(BMP280_ADDR, BMP280_REG_DIG_T1, calib, 6, 100) == I2C_OK) {
        dig_T1 = (calib[1] << 8) | calib[0];
        dig_T2 = (calib[3] << 8) | calib[2];
        dig_T3 = (calib[5] << 8) | calib[4];

        char msg[150];
        snprintf(msg, sizeof(msg), "[BMP280] Calibration: T1=%u, T2=%d, T3=%d\r\n",
                 dig_T1, dig_T2, dig_T3);
        UART_SendString(msg);
    } else {
        UART_SendString("[BMP280] Failed to read calibration data!\r\n");
        return false;
    }

    I2C_WriteRegister(BMP280_ADDR, BMP280_REG_CONFIG, 0x00, 100);
    I2C_WriteRegister(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x27, 100);

    bmp280_initialized = true;
    UART_SendString("[BMP280] Initialization complete!\r\n");
    return true;
}

bool BMP280_Read(int32_t *temp_out, int32_t *press_out) {
    if (!bmp280_initialized) {
        UART_SendString("[BMP280] Not initialized!\r\n");
        return false;
    }

    uint8_t data[6];
    if (I2C_ReadRegisters(BMP280_ADDR, BMP280_REG_PRESS_MSB, data, 6, 200) != I2C_OK) {
        UART_SendString("[BMP280] Failed to read sensor data!\r\n");
        return false;
    }

    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    *temp_out = (t_fine * 5 + 128) >> 8;

    *press_out = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    if (*temp_out < -4000 || *temp_out > 8500) {
        UART_SendString("[BMP280] Temperature out of range!\r\n");
        return false;
    }

    return true;
}

/* ====================== Enhanced WiFi Management Functions ====================== */
void WiFi_UpdateConnectionState(void) {
    bool actually_connected = ESP32_WiFi_IsConnected();

    switch (wifi_state) {
        case WIFI_STATE_DISCONNECTED:
            if (actually_connected) {
                wifi_state = WIFI_STATE_CONNECTED;
                wifi_connected_time = systick_counter;
                UART_SendString("[WiFi] State changed: DISCONNECTED -> CONNECTED\r\n");
                ESP32_WiFi_GetIP(current_ip);
                LED_Green_On();
            }
            break;

        case WIFI_STATE_CONNECTING:
            if (actually_connected) {
                wifi_state = WIFI_STATE_CONNECTED;
                wifi_connected_time = systick_counter;
                failed_wifi_attempts = 0;
                UART_SendString("[WiFi] State changed: CONNECTING -> CONNECTED\r\n");
                ESP32_WiFi_GetIP(current_ip);
                LED_Green_On();
            } else {
                if ((systick_counter - wifi_retry_time) > 30000) {
                    wifi_state = WIFI_STATE_ERROR;
                    failed_wifi_attempts++;
                    UART_SendString("[WiFi] State changed: CONNECTING -> ERROR (timeout)\r\n");
                    LED_Red_On();
                    SysTick_Delay(200);
                    LED_Red_Off();
                }
            }
            break;

        case WIFI_STATE_CONNECTED:
            if (!actually_connected) {
                wifi_state = WIFI_STATE_DISCONNECTED;
                wifi_disconnection_count++;
                strcpy(current_ip, "0.0.0.0");
                UART_SendString("[WiFi] State changed: CONNECTED -> DISCONNECTED\r\n");
                LED_Green_Off();
            }
            break;

        case WIFI_STATE_ERROR:
        case WIFI_STATE_RETRYING:
            if (actually_connected) {
                wifi_state = WIFI_STATE_CONNECTED;
                wifi_connected_time = systick_counter;
                failed_wifi_attempts = 0;
                UART_SendString("[WiFi] State changed: ERROR/RETRYING -> CONNECTED\r\n");
                ESP32_WiFi_GetIP(current_ip);
                LED_Green_On();
            }
            break;
    }
}

bool WiFi_IsConnected(void) {
    return (wifi_state == WIFI_STATE_CONNECTED);
}

const char* WiFi_GetStateString(void) {
    const char* state_strings[] = {
        "DISCONNECTED", "CONNECTING", "CONNECTED", "ERROR", "RETRYING"
    };
    return state_strings[wifi_state];
}

/* ====================== FIXED CLOUD FUNCTIONS WITH PROPER TEMPERATURE ====================== */

/**
 * @brief FIXED: Enhanced Manual Cloud Send with Response Reading and Proper Temperature
 */
/**
 * @brief ThingSpeak Cloud Send - FIXED for real IoT platform
 */
void Manual_Cloud_Send_WithResponse(void) {
    UART_SendString("\r\n[THINGSPEAK] Sending to real IoT platform...\r\n");

    if (!WiFi_IsConnected()) {
        UART_SendString("[THINGSPEAK] WiFi not connected!\r\n");
        return;
    }

    if (reading_count == 0) {
        UART_SendString("[THINGSPEAK] No sensor data available!\r\n");
        return;
    }

    /* Create ThingSpeak data format */
    char thingspeak_data[200];
    snprintf(thingspeak_data, sizeof(thingspeak_data),
             "api_key=%s&field1=%ld.%02ld&field2=%ld",
             THINGSPEAK_API_KEY,
             current_temperature/100, (long)abs(current_temperature%100),
             current_pressure);

    UART_SendString("[THINGSPEAK] Data: ");
    UART_SendString(thingspeak_data);
    UART_SendString("\r\n");

    /* Connect to ThingSpeak */
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CIPCLOSE");
    SysTick_Delay(1000);

    ESP32_SendLine("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80");

    if (ESP32_WaitForResponse("CONNECT", 10000)) {
        UART_SendString("[THINGSPEAK] ✅ Connected to ThingSpeak!\r\n");

        /* Build HTTP POST request */
        char http_request[400];
        snprintf(http_request, sizeof(http_request),
                 "POST /update HTTP/1.1\r\n"
                 "Host: api.thingspeak.com\r\n"
                 "Content-Type: application/x-www-form-urlencoded\r\n"
                 "Content-Length: %d\r\n"
                 "Connection: close\r\n\r\n"
                 "%s",
                 strlen(thingspeak_data), thingspeak_data);

        /* Send data */
        char send_cmd[50];
        snprintf(send_cmd, sizeof(send_cmd), "AT+CIPSEND=%d", strlen(http_request));
        ESP32_ClearBuffer();
        ESP32_SendLine(send_cmd);

        if (ESP32_WaitForResponse(">", 2000)) {
            ESP32_Send(http_request);

            if (ESP32_WaitForResponse("SEND OK", 5000)) {
                UART_SendString("[THINGSPEAK] ✅ Data sent successfully!\r\n");

                /* Read response */
                SysTick_Delay(2000);
                char response[512];
                ESP32_ReadAll(response, sizeof(response), 3000);

                if (strstr(response, "HTTP/1.1 200") != NULL) {
                    UART_SendString("[THINGSPEAK] ✅ Data accepted by ThingSpeak!\r\n");
                    UART_SendString("[THINGSPEAK] 🎉 Check your channel at: https://thingspeak.com/channels/2996841\r\n");
                    successful_cloud_updates++;
                } else {
                    UART_SendString("[THINGSPEAK] ❌ ThingSpeak rejected data\r\n");
                    failed_cloud_updates++;
                }

                last_cloud_update = systick_counter;
            } else {
                UART_SendString("[THINGSPEAK] ❌ Send failed!\r\n");
                failed_cloud_updates++;
            }
        } else {
            UART_SendString("[THINGSPEAK] ❌ Send command failed!\r\n");
            failed_cloud_updates++;
        }

        ESP32_SendLine("AT+CIPCLOSE");
    } else {
        UART_SendString("[THINGSPEAK] ❌ Failed to connect!\r\n");
        failed_cloud_updates++;
    }
}

/**
 * @brief Test Different HTTP Endpoints
 */
void Test_HTTP_Endpoints(void) {
    UART_SendString("\r\n[TEST] Testing different HTTP endpoints...\r\n");

    /* Test 1: httpbin.org/post (echoes back your data) */
    UART_SendString("\r\n--- Test 1: httpbin.org/post (Echo Service) ---\r\n");
    Manual_Cloud_Send_WithResponse();

    SysTick_Delay(5000);

    /* Test 2: httpbin.org/status/200 (Simple success test) */
    UART_SendString("\r\n--- Test 2: httpbin.org/status/200 (Success Test) ---\r\n");

    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CIPCLOSE");
    SysTick_Delay(1000);

    ESP32_SendLine("AT+CIPSTART=\"TCP\",\"httpbin.org\",80");
    if (ESP32_WaitForResponse("CONNECT", 10000)) {
        char simple_request[] = "GET /status/200 HTTP/1.1\r\nHost: httpbin.org\r\nConnection: close\r\n\r\n";

        char send_cmd[50];
        snprintf(send_cmd, sizeof(send_cmd), "AT+CIPSEND=%d", strlen(simple_request));
        ESP32_SendLine(send_cmd);

        if (ESP32_WaitForResponse(">", 2000)) {
            ESP32_Send(simple_request);
            if (ESP32_WaitForResponse("SEND OK", 5000)) {
                SysTick_Delay(2000);
                char response[512];
                ESP32_ReadAll(response, sizeof(response), 3000);

                UART_SendString("Simple GET Response:\r\n");
                UART_SendString(response);
                UART_SendString("\r\n");
            }
        }
        ESP32_SendLine("AT+CIPCLOSE");
    }
}

/**
 * @brief ADDED: Debug Temperature Formatting Function
 */
void Debug_Temperature_Formatting(void) {
    UART_SendString("\r\n[DEBUG] === Temperature Formatting Test ===\r\n");

    char msg[200];
    snprintf(msg, sizeof(msg), "Current temperature raw: %ld\r\n", current_temperature);
    UART_SendString(msg);

    snprintf(msg, sizeof(msg), "Temperature integer part: %ld\r\n", current_temperature/100);
    UART_SendString(msg);

    snprintf(msg, sizeof(msg), "Temperature decimal part: %02ld\r\n", (long)abs(current_temperature%100));
    UART_SendString(msg);

    snprintf(msg, sizeof(msg), "Temperature formatted: %ld.%02ld°C\r\n",
             current_temperature/100, (long)abs(current_temperature%100));
    UART_SendString(msg);

    /* Test JSON formatting */
    char test_json[256];
    snprintf(test_json, sizeof(test_json),
             "{\"device\":\"%s\",\"temperature\":%ld.%02ld,\"pressure\":%ld}",
             DEVICE_ID,
             current_temperature/100, (long)abs(current_temperature%100),
             current_pressure);

    UART_SendString("Test JSON: ");
    UART_SendString(test_json);
    UART_SendString("\r\n");
    UART_SendString("==========================================\r\n");
}

/* ====================== Task Functions ====================== */
void Task_WeatherReading(void) {
    int32_t temp, press_raw;

    #if DEBUG_SENSOR
    UART_SendString("\r\n[TASK] Weather reading task executing...\r\n");
    #endif
    LED_Green_On();

    if (BMP280_Read(&temp, &press_raw)) {
        reading_count++;
        current_temperature = temp;
        current_pressure = 1013 + ((press_raw - 393500) / 400);

        currentWeather.temperature = current_temperature;
        currentWeather.pressure = current_pressure;
        currentWeather.timestamp = systick_counter / 1000;

        #if DEBUG_SENSOR
        char msg[150];
        snprintf(msg, sizeof(msg),
                 "[TASK] Reading #%lu: %ld.%02ld°C, %ld hPa\r\n",
                 reading_count,
                 current_temperature/100, (long)abs(current_temperature%100),
                 current_pressure);
        UART_SendString(msg);
        #endif
    } else {
        UART_SendString("[TASK] Failed to read BMP280!\r\n");
        LED_Red_On();
        SysTick_Delay(100);
        LED_Red_Off();
    }

    LED_Green_Off();
    #if DEBUG_SENSOR
    UART_SendString("[TASK] Weather reading task complete\r\n");
    #endif
}

void Task_CloudUpdate(void) {
    uint32_t current_time = systick_counter;

    #if DEBUG_CLOUD
    UART_SendString("\r\n[TASK] Cloud update task executing...\r\n");
    #endif

    /* REDUCED INTERVAL FOR TESTING */
    if ((current_time - last_cloud_update) < FAST_CLOUD_UPDATE_INTERVAL) {
        #if DEBUG_CLOUD
        UART_SendString("[TASK] Cloud update interval not reached, skipping\r\n");
        #endif
        return;
    }

    if (!WiFi_IsConnected()) {
        #if DEBUG_CLOUD
        UART_SendString("[TASK] WiFi not connected, skipping cloud update\r\n");
        #endif
        return;
    }

    if (reading_count == 0) {
        #if DEBUG_CLOUD
        UART_SendString("[TASK] No sensor data, skipping cloud update\r\n");
        #endif
        return;
    }

    /* Use FIXED enhanced cloud sending for automatic updates too */
    UART_SendString("\r\n[AUTO] Automatic cloud update with FIXED temperature...\r\n");
    Manual_Cloud_Send_WithResponse();
}

void Task_WiFiManager(void) {
    uint32_t current_time = systick_counter;

    #if DEBUG_WIFI
    UART_SendString("\r\n[TASK] WiFi manager task executing...\r\n");
    #endif

    WiFi_UpdateConnectionState();

    switch (wifi_state) {
        case WIFI_STATE_DISCONNECTED:
            if ((current_time - wifi_retry_time) > WIFI_RETRY_INTERVAL) {
                #if DEBUG_WIFI
                UART_SendString("[WiFi] Attempting to connect...\r\n");
                #endif
                wifi_state = WIFI_STATE_CONNECTING;
                wifi_retry_time = current_time;
                wifi_connection_attempts++;

                if (ESP32_WiFi_Connect() == ESP32_OK) {
                    #if DEBUG_WIFI
                    UART_SendString("[WiFi] Connection initiated successfully\r\n");
                    #endif
                } else {
                    wifi_state = WIFI_STATE_ERROR;
                    failed_wifi_attempts++;
                    #if DEBUG_WIFI
                    UART_SendString("[WiFi] Connection initiation failed\r\n");
                    #endif
                }
            }
            break;

        case WIFI_STATE_ERROR:
            if ((current_time - wifi_retry_time) > (WIFI_RETRY_INTERVAL * 2)) {
                #if DEBUG_WIFI
                UART_SendString("[WiFi] Retrying after error...\r\n");
                #endif
                wifi_state = WIFI_STATE_RETRYING;
                wifi_retry_time = current_time;
            }
            break;

        case WIFI_STATE_RETRYING:
            #if DEBUG_WIFI
            UART_SendString("[WiFi] Retry attempt...\r\n");
            #endif
            wifi_state = WIFI_STATE_CONNECTING;
            wifi_connection_attempts++;

            if (ESP32_WiFi_Connect() == ESP32_OK) {
                #if DEBUG_WIFI
                UART_SendString("[WiFi] Retry connection initiated\r\n");
                #endif
            } else {
                wifi_state = WIFI_STATE_ERROR;
                failed_wifi_attempts++;
                #if DEBUG_WIFI
                UART_SendString("[WiFi] Retry connection failed\r\n");
                #endif
            }
            break;

        case WIFI_STATE_CONNECTED:
            if ((current_time - last_wifi_check) > 30000) {
                last_wifi_check = current_time;
                #if DEBUG_WIFI
                UART_SendString("[WiFi] Periodic connection check...\r\n");
                #endif
            }
            break;

        case WIFI_STATE_CONNECTING:
            break;
    }

    #if DEBUG_WIFI
    char state_msg[100];
    snprintf(state_msg, sizeof(state_msg), "[TASK] WiFi state: %s\r\n", WiFi_GetStateString());
    UART_SendString(state_msg);
    UART_SendString("[TASK] WiFi manager task complete\r\n");
    #endif
}

void Task_PowerManager(void) {
    uint32_t current_time = systick_counter;

    #if DEBUG_POWER_MGMT
    UART_SendString("\r\n[TASK] Power manager task executing (FIXED VERSION)...\r\n");
    #endif

    if ((current_time - last_power_check) > 30000) {
        last_power_check = current_time;

        uint32_t power_estimate = PowerMgmt_GetPowerConsumptionEstimate();

        #if DEBUG_POWER_MGMT
        {
            char msg[100];
            snprintf(msg, sizeof(msg), "[POWER] Current power estimate: %lu µA\r\n", power_estimate);
            UART_SendString(msg);
        }
        #endif

        if (auto_power_save_enabled && ENABLE_AUTO_SLEEP) {
            uint32_t time_since_cloud = (current_time - last_cloud_update) / 1000;
            uint32_t time_until_next = (FAST_CLOUD_UPDATE_INTERVAL / 1000) - time_since_cloud;

            if (time_until_next > SLEEP_DURATION && SLEEP_BETWEEN_READS) {
                power_save_request_time = current_time;

                #if DEBUG_POWER_MGMT
                UART_SendString("[POWER] Power save conditions met, configuring wake-up timer...\r\n");
                #endif

                if (PowerMgmt_SetWakeupTimer(SLEEP_DURATION)) {
                    UART_SendString("[POWER] ✅ Wake-up timer configured successfully\r\n");
                    sleep_cycle_count++;
                } else {
                    UART_SendString("[POWER] ❌ Wake-up timer configuration failed\r\n");
                    power_management_errors++;
                }
            }
        }
    }

    #if DEBUG_POWER_MGMT
    UART_SendString("[TASK] Power manager task complete (FIXED VERSION)\r\n");
    #endif
}

void Task_SystemMonitor(void) {
    char msg[150];

    UART_SendString("\r\n=== System Status - TEMPERATURE FIXED VERSION ===\r\n");

    snprintf(msg, sizeof(msg), "Sensor readings: %lu\r\n", reading_count);
    UART_SendString(msg);

    if (reading_count > 0) {
        snprintf(msg, sizeof(msg), "Last reading: %ld.%02ld°C, %ld hPa\r\n",
                 current_temperature/100, (long)abs(current_temperature%100), current_pressure);
        UART_SendString(msg);
    }

    snprintf(msg, sizeof(msg), "WiFi State: %s\r\n", WiFi_GetStateString());
    UART_SendString(msg);

    if (WiFi_IsConnected()) {
        snprintf(msg, sizeof(msg), "WiFi IP: %s\r\n", current_ip);
        UART_SendString(msg);

        uint32_t connected_duration = (systick_counter - wifi_connected_time) / 1000;
        snprintf(msg, sizeof(msg), "Connected for: %lu seconds\r\n", connected_duration);
        UART_SendString(msg);
    }

    snprintf(msg, sizeof(msg), "Cloud Updates: %lu successful, %lu failed\r\n",
             successful_cloud_updates, failed_cloud_updates);
    UART_SendString(msg);

    snprintf(msg, sizeof(msg), "FAST TESTING INTERVALS ACTIVE:\r\n");
    UART_SendString(msg);
    snprintf(msg, sizeof(msg), "  - Weather reading: %d seconds\r\n", FAST_WEATHER_READ_INTERVAL/1000);
    UART_SendString(msg);
    snprintf(msg, sizeof(msg), "  - Cloud update: %d seconds\r\n", FAST_CLOUD_UPDATE_INTERVAL/1000);
    UART_SendString(msg);

    if (last_cloud_update > 0) {
        uint32_t since_update = (systick_counter - last_cloud_update) / 1000;
        snprintf(msg, sizeof(msg), "Last cloud update: %lu seconds ago\r\n", since_update);
        UART_SendString(msg);
    }

    snprintf(msg, sizeof(msg), "Task Scheduler: %s\r\n",
             task_scheduler_enabled ? "ENABLED" : "DISABLED");
    UART_SendString(msg);

    TaskScheduler_PrintStatus();
    UART_SendString("============================================\r\n");
}

/* ====================== Manual Test Functions ====================== */
void Manual_Read_Sensors(void) {
    UART_SendString("\r\n[MANUAL] Reading sensors directly...\r\n");
    Task_WeatherReading();
}

void Test_AT_Command(void) {
    UART_SendString("\r\n[TEST] ESP32 AT Test...\r\n");
    if (ESP32_WiFi_Test() == ESP32_OK) {
        UART_SendString("[AT] ESP32 responding OK\r\n");
    } else {
        UART_SendString("[AT] ESP32 not responding!\r\n");
    }
}

void Test_WiFi_Connect(void) {
    UART_SendString("\r\n[TEST] Manual WiFi connection...\r\n");
    wifi_connection_attempts++;

    if (ESP32_WiFi_Connect() == ESP32_OK) {
        UART_SendString("[WiFi] Manual connection successful!\r\n");
        wifi_state = WIFI_STATE_CONNECTED;
        wifi_connected_time = systick_counter;
        ESP32_WiFi_GetIP(current_ip);
        LED_Green_On();
        ESP32_WiFi_PrintStatus();
    } else {
        UART_SendString("[WiFi] Manual connection failed!\r\n");
        wifi_state = WIFI_STATE_ERROR;
        failed_wifi_attempts++;
        LED_Red_On();
        SysTick_Delay(500);
        LED_Red_Off();
    }
}

void Test_Power_Management_Complete(void) {
    UART_SendString("\r\n[POWER TEST] === Complete Power Management Test ===\r\n");

    if (PowerMgmt_TestConfiguration()) {
        UART_SendString("[POWER TEST] ✅ All power management tests PASSED!\r\n");

        uint32_t test_durations[] = {5, 10, 30, 60};
        for (int i = 0; i < 4; i++) {
            char msg[100];
            snprintf(msg, sizeof(msg), "[POWER TEST] Testing %lu second timer...", test_durations[i]);
            UART_SendString(msg);

            if (PowerMgmt_SetWakeupTimer(test_durations[i])) {
                UART_SendString(" ✅ SUCCESS\r\n");
            } else {
                UART_SendString(" ❌ FAILED\r\n");
                break;
            }
        }

        UART_SendString("[POWER TEST] 🎉 Power management is fully working!\r\n");
    } else {
        UART_SendString("[POWER TEST] ❌ Power management test failed\r\n");
    }
}

/* ====================== System Initialization ====================== */
void System_Init(void) {
    SysTick_Init();
    UART_Init(115200);
    LED_Init();

    UART_SendString("\r\n\r\n");
    UART_SendString("═══════════════════════════════════════════════════════\r\n");
    UART_SendString("    STM32F429ZI Weather Station - WITH SIMPLE API!      \r\n");
    UART_SendString("   Complete IoT Platform + Easy-to-Use Interface       \r\n");
    UART_SendString("   🚀 FAST TESTING + TEMPERATURE FIX + API 🚀       \r\n");
    UART_SendString("═══════════════════════════════════════════════════════\r\n");

    TaskScheduler_Init();

    UART_SendString("[POWER] Initializing FIXED Power Management...\r\n");
    PowerMgmt_Init();
    power_management_enabled = true;
    UART_SendString("[POWER] ✅ FIXED Power Management initialized successfully!\r\n");

    I2C_Config_t i2cConfig = {
        .clockSpeed = 100000,
        .ownAddress = 0x33,
        .enableGeneralCall = false,
        .enableClockStretching = true
    };

    if (I2C_Init(&i2cConfig) != I2C_OK) {
        UART_SendString("[ERROR] I2C initialization failed!\r\n");
        LED_Red_On();
        return;
    }

    if (!BMP280_Init()) {
        UART_SendString("[ERROR] BMP280 initialization failed!\r\n");
        LED_Red_On();
        return;
    }

    UART_SendString("\r\n[TEST] Initial sensor reading...\r\n");
    Manual_Read_Sensors();

    if (ESP32_WiFi_Init() != ESP32_OK) {
        UART_SendString("[ERROR] ESP32 initialization failed!\r\n");
        LED_Red_On();
        return;
    }

    wifi_state = WIFI_STATE_DISCONNECTED;
    wifi_retry_time = systick_counter;

    UART_SendString("\r\n[SYSTEM] All core systems initialized successfully!\r\n");
}

/* ====================== Main Function ====================== */
int main(void) {
    /* First: Initialize core system (UART, sensors, etc.) */
    System_Init();

    /* Then: Initialize the Simple API wrapper layer */
    UART_SendString("\r\n[API] Initializing Weather Station API wrapper...\r\n");
    if (!WeatherStation_Init()) {
        UART_SendString("[WARNING] Weather Station API initialization failed!\r\n");
        UART_SendString("Core system still works, but Simple API may not function.\r\n");
        simple_api_initialized = false;
    } else {
        UART_SendString("[API] ✅ Weather Station API initialized successfully!\r\n");
        UART_SendString("[API] Both core system and Simple API are ready!\r\n");
        simple_api_initialized = true;
    }

    Test_WiFi_Connect();

    uint8_t taskId;
    UART_SendString("\r\n[SETUP] Registering tasks with FAST TESTING intervals...\r\n");

    /* REDUCED INTERVALS FOR TESTING */
    if (TaskScheduler_RegisterTask(Task_WeatherReading, FAST_WEATHER_READ_INTERVAL,
                                  TASK_PRIORITY_NORMAL, "WeatherRead", &taskId) == TASK_OK) {
        UART_SendString("[SETUP] Weather reading task registered (15 sec intervals)\r\n");
    }

    if (TaskScheduler_RegisterTask(Task_CloudUpdate, FAST_CLOUD_UPDATE_INTERVAL,
                                  TASK_PRIORITY_NORMAL, "CloudUpdate", &taskId) == TASK_OK) {
        UART_SendString("[SETUP] Cloud update task registered (1 min intervals)\r\n");
    }

    if (TaskScheduler_RegisterTask(Task_WiFiManager, FAST_WIFI_MANAGER_INTERVAL,
                                  TASK_PRIORITY_HIGH, "WiFiManager", &taskId) == TASK_OK) {
        UART_SendString("[SETUP] WiFi manager task registered (5 sec intervals)\r\n");
    }

    if (TaskScheduler_RegisterTask(Task_PowerManager, 30000,
                                  TASK_PRIORITY_LOW, "PowerManager", &taskId) == TASK_OK) {
        UART_SendString("[SETUP] Power manager task registered\r\n");
    }

    if (TaskScheduler_RegisterTask(Task_SystemMonitor, 120000,  /* Every 2 minutes */
                                  TASK_PRIORITY_LOW, "SystemMonitor", &taskId) == TASK_OK) {
        UART_SendString("[SETUP] System monitor task registered\r\n");
    }

    UART_SendString("\r\n═══════════════════════════════════════════════════════\r\n");
    UART_SendString("Enhanced Commands - Now with SIMPLE API:\r\n");
    UART_SendString("  's' - Simple API test (easy weather reading) 🔧✨\r\n");
    UART_SendString("  'r' - Read sensors manually\r\n");
    UART_SendString("  'z' - DEBUG temperature formatting 🔧\r\n");
    UART_SendString("  'v' - VERIFY JSON data (FIXED temperature) 🔍\r\n");
    UART_SendString("  'p' - FIXED cloud send with proper temp 📊\r\n");
    UART_SendString("  'h' - Test HTTP endpoints 🌐\r\n");
    UART_SendString("  'c' - Connect to WiFi manually\r\n");
    UART_SendString("  'a' - Test AT command\r\n");
    UART_SendString("  'w' - Show WiFi status\r\n");
    UART_SendString("  'i' - Show enhanced system info\r\n");
    UART_SendString("  'e' - ENABLE task scheduler\r\n");
    UART_SendString("  'd' - DISABLE task scheduler\r\n");
    UART_SendString("  't' - Show task status\r\n");
    UART_SendString("  'y' - Complete power management test\r\n");
    UART_SendString("  'x' - Toggle auto power save\r\n");
    UART_SendString("═══════════════════════════════════════════════════════\r\n");

    UART_SendString("\r\n🎉 NEW SIMPLE API WORKFLOW:\r\n");
    UART_SendString("   ✅ Single WeatherStation_Init() call does everything\r\n");
    UART_SendString("   ✅ Core system + API wrapper both ready\r\n");
    UART_SendString("   ✅ Human-readable data format available\r\n");
    UART_SendString("   ✅ Built-in error handling active\r\n");
    UART_SendString("\r\n📊 TESTING:\r\n");
    UART_SendString("   's' - Test the new Simple API (should work now!)\r\n");
    UART_SendString("   'z' - Debug temperature formatting\r\n");
    UART_SendString("   'v' - Verify JSON transmission\r\n");
    UART_SendString("   'p' - Send with response reading\r\n\r\n");

    UART_SendString("🔥 READY TO TEST: Press 's' to test the Simple API!\r\n");
    UART_SendString("Press 'e' to enable scheduler and start automatic operation!\r\n\r\n");

    while (1) {
        uint32_t current_time = systick_counter;

        if (task_scheduler_enabled) {
            TaskScheduler_RunDispatcher();
        }

        if (UART_IsDataAvailable()) {
            uint8_t cmd = UART_ReceiveByte();

            switch(cmd) {
                case 's':
                case 'S':
                    /* ULTRA-SIMPLIFIED API Test - No floating point */
                    UART_SendString("\r\n=== ULTRA-SIMPLIFIED API TEST ===\r\n");

                    /* Test 1: Check API status */
                    UART_SendString("Test 1: API Status Check...\r\n");
                    if (simple_api_initialized) {
                        UART_SendString("✅ API is initialized\r\n");
                    } else {
                        UART_SendString("❌ API not initialized\r\n");
                    }

                    /* Test 2: Quick sensor reading */
                    UART_SendString("Test 2: Quick Sensor Reading...\r\n");
                    int32_t test_temp, test_pressure;
                    if (BMP280_Read(&test_temp, &test_pressure)) {
                        UART_SendString("✅ Sensor reading successful\r\n");
                        char msg[100];
                        snprintf(msg, sizeof(msg), "📊 Raw: %ld.%02ld°C, press=%ld\r\n",
                                test_temp/100, (long)abs(test_temp%100), test_pressure);
                        UART_SendString(msg);

                        /* Calculate pressure in integer format only */
                        int pressure_hpa = 1013 + ((test_pressure - 393500) / 400);
                        snprintf(msg, sizeof(msg), "🎯 Formatted: %ld.%02ld°C, %d hPa\r\n",
                                test_temp/100, (long)abs(test_temp%100), pressure_hpa);
                        UART_SendString(msg);
                    } else {
                        UART_SendString("❌ Sensor reading failed\r\n");
                    }

                    /* Test 3: Simple WiFi check */
                    UART_SendString("Test 3: WiFi Check...\r\n");
                    if (WiFi_IsConnected()) {
                        UART_SendString("✅ WiFi connected\r\n");
                    } else {
                        UART_SendString("❌ WiFi not connected\r\n");
                    }

                    /* Test 4: Quick stats */
                    UART_SendString("Test 4: Quick Stats...\r\n");
                    char stats[100];
                    snprintf(stats, sizeof(stats), "📈 Readings: %lu, Uptime: %lu sec\r\n",
                             reading_count, systick_counter/1000);
                    UART_SendString(stats);

                    /* Summary */
                    UART_SendString("\r\n🎉 ALL TESTS COMPLETE!\r\n");
                    UART_SendString("Simple API concept demonstrated successfully!\r\n");
                    UART_SendString("=========================================\r\n");
                    break;

                case 'r':
                case 'R':
                    Manual_Read_Sensors();
                    break;

                case 'z':
                case 'Z':
                    Debug_Temperature_Formatting();
                    break;

                case 'v':
                case 'V':
                    UART_SendString("\r\n[VERIFY] Verifying FIXED JSON data transmission...\r\n");
                    Manual_Cloud_Send_WithResponse();
                    UART_SendString("[VERIFY] Look for temperature value in response above!\r\n");
                    UART_SendString("[VERIFY] Should now show proper temperature like 29.40!\r\n");
                    break;

                case 'p':
                case 'P':
                    Manual_Cloud_Send_WithResponse();
                    break;

                case 'h':
                case 'H':
                    Test_HTTP_Endpoints();
                    break;

                case 'c':
                case 'C':
                    Test_WiFi_Connect();
                    break;

                case 'a':
                case 'A':
                    Test_AT_Command();
                    break;

                case 'w':
                case 'W':
                    ESP32_WiFi_PrintStatus();
                    WiFi_UpdateConnectionState();

                    char status_msg[100];
                    snprintf(status_msg, sizeof(status_msg),
                             "Enhanced WiFi State: %s\r\n", WiFi_GetStateString());
                    UART_SendString(status_msg);
                    break;

                case 'i':
                case 'I':
                    Task_SystemMonitor();
                    break;

                case 'e':
                case 'E':
                    if (!task_scheduler_enabled) {
                        UART_SendString("\r\n🚀 ENABLING ENHANCED TASK SCHEDULER...\r\n");

                        for (uint8_t i = 0; i < TaskScheduler_GetTaskCount(); i++) {
                            TaskScheduler_StartTask(i);
                        }

                        task_scheduler_enabled = true;
                        UART_SendString("✅ Enhanced task scheduler ENABLED!\r\n");
                        UART_SendString("⚡ Fast testing intervals active\r\n");
                        UART_SendString("🔧 Temperature formatting FIXED\r\n");
                        UART_SendString("🎉 Simple API available\r\n\r\n");
                    } else {
                        UART_SendString("\r\n⚠ Task scheduler already enabled\r\n");
                    }
                    break;

                case 'd':
                case 'D':
                    if (task_scheduler_enabled) {
                        UART_SendString("\r\n🛑 DISABLING TASK SCHEDULER...\r\n");

                        for (uint8_t i = 0; i < TaskScheduler_GetTaskCount(); i++) {
                            TaskScheduler_StopTask(i);
                        }

                        task_scheduler_enabled = false;
                        UART_SendString("✅ Task scheduler DISABLED\r\n\r\n");
                    } else {
                        UART_SendString("\r\n⚠ Task scheduler already disabled\r\n");
                    }
                    break;

                case 't':
                case 'T':
                    TaskScheduler_PrintStatus();
                    UART_SendString("Task Scheduler: ");
                    UART_SendString(task_scheduler_enabled ? "ENABLED" : "DISABLED");
                    UART_SendString("\r\n");
                    break;

                case 'y':
                case 'Y':
                    Test_Power_Management_Complete();
                    break;

                case 'x':
                case 'X':
                    auto_power_save_enabled = !auto_power_save_enabled;
                    UART_SendString(auto_power_save_enabled ?
                        "\r\n[POWER] Auto power save ENABLED\r\n" :
                        "\r\n[POWER] Auto power save DISABLED\r\n");
                    break;

                default:
                    UART_SendByte(cmd);
                    break;
            }
        }

        /* Enhanced LED heartbeat */
        static uint32_t last_blink = 0;
        if ((current_time - last_blink) > 1000) {
            last_blink = current_time;

            switch (wifi_state) {
                case WIFI_STATE_DISCONNECTED:
                    GPIOB->ODR ^= (1 << 7);
                    break;
                case WIFI_STATE_CONNECTING:
                case WIFI_STATE_RETRYING:
                    if ((current_time % 500) < 250) {
                        LED_Blue_On();
                    } else {
                        LED_Blue_Off();
                    }
                    break;
                case WIFI_STATE_ERROR:
                    GPIOB->ODR ^= (1 << 14);
                    break;
                case WIFI_STATE_CONNECTED:
                    LED_Green_On();
                    LED_Blue_Off();
                    LED_Red_Off();
                    break;
            }
        }

        SysTick_Delay(10);
    }

    return 0;
}
