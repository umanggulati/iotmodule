/**
 * @file wifi_config.h
 * @brief Enhanced WiFi and System Configuration - Phase 3B with Power Management
 * @desc Complete configuration for Phase 3B including power management settings
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

/* ====================== WiFi Settings ====================== */
/* IMPORTANT: Update these with your network credentials! */
#define WIFI_SSID               "YOUR_WIFI_SSID_HERE"
#define WIFI_PASSWORD           "YOUR_WIFI_PASSWORD_HERE"

/* ====================== Cloud Server Settings ====================== */
#define CLOUD_SERVER            "api.thingspeak.com"
#define CLOUD_PORT              80
#define CLOUD_UPDATE_URL        "/update"
#define THINGSPEAK_API_KEY      "YOUR_THINGSPEAK_API_KEY_HERE"

/* Option 2: Your own server / Testing server */
#define CLOUD_SERVER            "httpbin.org"       /* Change to your server */
#define CLOUD_PORT              80
#define CLOUD_UPDATE_URL        "/post"             /* Your endpoint */

/* Option 3: MQTT (if you want to use MQTT instead) */
// #define USE_MQTT
// #define MQTT_BROKER          "mqtt.broker.com"
// #define MQTT_PORT            1883
// #define MQTT_TOPIC           "weather/data"

/* ====================== Task Timing Intervals ====================== */
#define WEATHER_READ_INTERVAL   30000               /* Read sensors every 30 seconds */
#define CLOUD_UPDATE_INTERVAL   300000              /* Send to cloud every 5 minutes */
#define WIFI_MANAGER_INTERVAL   10000               /* WiFi manager every 10 seconds */
#define WIFI_RETRY_INTERVAL     60000               /* Retry WiFi every 1 minute if disconnected */
#define SYSTEM_MONITOR_INTERVAL 60000               /* System monitor every 1 minute */

/* ====================== Task Priorities ====================== */
#define TASK_PRIORITY_HIGH      0                   /* High priority tasks */
#define TASK_PRIORITY_NORMAL    1                   /* Normal priority tasks */
#define TASK_PRIORITY_LOW       2                   /* Low priority tasks */
#define TASK_PRIORITY_IDLE      3                   /* Background tasks */

/* ====================== Device Settings ====================== */
#define DEVICE_ID               "STM32_Weather_01"  /* Unique device identifier */
#define DEVICE_LOCATION         "Home"              /* Device location */

/* ====================== Power Management Configuration ====================== */
#define ENABLE_AUTO_SLEEP       0                   /* Set to 1 to enable auto sleep */
#define SLEEP_DURATION          60                  /* Sleep duration in seconds */
#define SLEEP_BETWEEN_READS     0                   /* Set to 1 to sleep between readings */

/* Note: Power mode constants are defined in power_management.h */

/* ====================== WiFi Advanced Settings ====================== */
#define WIFI_CONNECTION_TIMEOUT 30000               /* WiFi connection timeout */
#define WIFI_PERIODIC_CHECK     30000               /* Periodic connection check */
#define MAX_WIFI_RETRY_ATTEMPTS 5                   /* Max retry attempts */

/* ====================== Error Handling ====================== */
#define MAX_SENSOR_ERRORS       5                   /* Max consecutive sensor errors */
#define MAX_CLOUD_ERRORS        3                   /* Max consecutive cloud errors */
#define MAX_WIFI_ERRORS         10                  /* Max WiFi errors before reset */

/* ====================== Debug Settings ====================== */
#define DEBUG_WIFI              1                   /* Enable WiFi debug messages */
#define DEBUG_SENSOR            1                   /* Enable sensor debug messages */
#define DEBUG_CLOUD             1                   /* Enable cloud communication debug */
#define DEBUG_TASK_SCHEDULER    0                   /* Enable task scheduler debug */
#define DEBUG_POWER_MGMT        1                   /* Enable power management debug */

#endif /* WIFI_CONFIG_H */
