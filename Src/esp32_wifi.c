/**
 * @file esp32_wifi.c
 * @brief ESP32 WiFi module implementation - FIXED VERSION
 * @desc WiFi connectivity with proven working patterns from test code
 */

#include "esp32_wifi.h"
#include "uart.h"
#include "systick.h"
#include "system_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wifi_config_template.h>

/* ====================== USART2 Configuration for ESP32 ====================== */

/* USART2 pins for ESP32 communication */
#define ESP32_UART_PORT        GPIOD
#define ESP32_UART_TX_PIN      5    /* PD5 */
#define ESP32_UART_RX_PIN      6    /* PD6 */
#define ESP32_UART_AF          7    /* AF7 for USART2 */

/* ESP32 AT command settings - INCREASED BUFFER SIZE */
#define ESP32_UART_BAUDRATE    115200
#define ESP32_CMD_TIMEOUT      5000   /* 5 seconds default timeout */
#define ESP32_RX_BUFFER_SIZE   2048   /* INCREASED from 512 to 2048 */

/* ====================== Private Variables ====================== */

/* Interrupt-based RX buffer - LARGER SIZE */
volatile uint8_t esp32_rx_buffer[ESP32_RX_BUFFER_SIZE];
volatile uint16_t esp32_rx_write_idx = 0;
volatile uint16_t esp32_rx_read_idx = 0;

/* WiFi status tracking */
static WiFi_Status_t wifiStatus = WIFI_STATUS_IDLE;
static bool esp32Initialized = false;
static char currentIP[16] = "0.0.0.0";
static int32_t currentRSSI = -100;

/* ====================== USART2 Interrupt Handler ====================== */

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t data = (uint8_t)(USART2->DR & 0xFF);
        uint16_t next_write = (esp32_rx_write_idx + 1) % ESP32_RX_BUFFER_SIZE;

        if (next_write != esp32_rx_read_idx) {
            esp32_rx_buffer[esp32_rx_write_idx] = data;
            esp32_rx_write_idx = next_write;
        }
    }

    /* Clear all error flags */
    if (USART2->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        volatile uint32_t dummy = USART2->DR;
        (void)dummy;
    }
}

/* ====================== Private Function Prototypes ====================== */

static void ESP32_UART_Init(void);

/* ====================== Private Functions ====================== */

/**
 * @brief Initialize USART2 for ESP32 communication
 */
static void ESP32_UART_Init(void) {
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Configure PD5 (TX) and PD6 (RX) */
    GPIOD->MODER &= ~((3u << 10) | (3u << 12));
    GPIOD->MODER |= ((2u << 10) | (2u << 12));  /* AF mode */

    /* Set alternate function */
    GPIOD->AFR[0] &= ~((0xF << 20) | (0xF << 24));
    GPIOD->AFR[0] |= ((7 << 20) | (7 << 24));

    /* Configure USART2 */
    USART2->CR1 = 0;
    USART2->BRR = SystemCoreClock_Get() / ESP32_UART_BAUDRATE;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

    /* Enable USART2 interrupt */
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* ====================== PUBLIC COMMUNICATION FUNCTIONS ====================== */

/**
 * @brief Send string via USART2 - PUBLIC
 */
void ESP32_Send(const char* str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *str++;
    }
}

/**
 * @brief Send command line with CRLF - PUBLIC
 */
void ESP32_SendLine(const char* cmd) {
    ESP32_Send(cmd);
    ESP32_Send("\r\n");
}

/**
 * @brief Check if data available in RX buffer - PUBLIC
 */
bool ESP32_DataAvailable(void) {
    return (esp32_rx_read_idx != esp32_rx_write_idx);
}

/**
 * @brief Read byte from RX buffer - PUBLIC
 */
uint8_t ESP32_ReadByte(void) {
    while (!ESP32_DataAvailable());
    uint8_t data = esp32_rx_buffer[esp32_rx_read_idx];
    esp32_rx_read_idx = (esp32_rx_read_idx + 1) % ESP32_RX_BUFFER_SIZE;
    return data;
}

/**
 * @brief Clear RX buffer - PUBLIC
 */
void ESP32_ClearBuffer(void) {
    NVIC_DisableIRQ(USART2_IRQn);
    esp32_rx_read_idx = 0;
    esp32_rx_write_idx = 0;
    while (USART2->SR & USART_SR_RXNE) {
        (void)USART2->DR;
    }
    NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief Wait for specific response - PUBLIC
 */
bool ESP32_WaitForResponse(const char* expected, uint32_t timeout_ms) {
    uint32_t start = systick_counter;
    char buffer[256] = {0};  /* Reasonable buffer size */
    uint16_t buf_idx = 0;

    while ((systick_counter - start) < timeout_ms) {
        if (ESP32_DataAvailable()) {
            if (buf_idx < sizeof(buffer) - 1) {
                buffer[buf_idx++] = ESP32_ReadByte();
                buffer[buf_idx] = '\0';

                if (strstr(buffer, expected) != NULL) {
                    return true;
                }
            } else {
                /* Buffer full, reset */
                buf_idx = 0;
                memset(buffer, 0, sizeof(buffer));
            }
        }
    }
    return false;
}

/**
 * @brief Read all available data - PUBLIC - FIXED VERSION FROM TEST CODE
 */
void ESP32_ReadAll(char* buffer, uint16_t max_size, uint32_t timeout_ms) {
    uint32_t start = systick_counter;
    uint16_t index = 0;

    while ((systick_counter - start) < timeout_ms && index < max_size - 1) {
        if (ESP32_DataAvailable()) {
            buffer[index++] = ESP32_ReadByte();
            start = systick_counter; /* CRITICAL FIX: Reset timeout on each byte */
        }
    }
    buffer[index] = '\0';
}

/**
 * @brief Check if connected by checking IP - PUBLIC
 */
bool ESP32_WiFi_CheckConnection(void) {
    char response[256];

    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CIFSR");
    SysTick_Delay(500);

    ESP32_ReadAll(response, sizeof(response), 1000);

    /* Check if we have a valid IP (not 0.0.0.0) */
    if (strstr(response, "STAIP,\"0.0.0.0\"") != NULL) {
        return false;
    }

    char* ip_start = strstr(response, "STAIP,\"");
    if (ip_start) {
        ip_start += 7;
        char* ip_end = strchr(ip_start, '"');
        if (ip_end) {
            int len = ip_end - ip_start;
            if (len < 16) {
                strncpy(currentIP, ip_start, len);
                currentIP[len] = '\0';
                return true;
            }
        }
    }

    return false;
}

/* ====================== Public Functions ====================== */

/**
 * @brief Initialize ESP32 WiFi module
 */
ESP32_Error_t ESP32_WiFi_Init(void) {
    /* Initialize UART for ESP32 */
    ESP32_UART_Init();

    #if DEBUG_WIFI
    UART_SendString("[ESP32] Initializing WiFi module...\r\n");
    #endif

    /* Wait for ESP32 to be ready */
    SysTick_Delay(1000);

    /* Test communication */
    ESP32_Error_t result = ESP32_WiFi_Test();
    if (result != ESP32_OK) {
        #if DEBUG_WIFI
        UART_SendString("[ESP32] Communication test failed!\r\n");
        #endif
        return result;
    }

    /* Disable echo */
    ESP32_ClearBuffer();
    ESP32_SendLine("ATE0");
    ESP32_WaitForResponse("OK", 1000);

    /* Set WiFi mode to Station */
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CWMODE=1");
    if (!ESP32_WaitForResponse("OK", 2000)) {
        #if DEBUG_WIFI
        UART_SendString("[ESP32] Failed to set WiFi mode!\r\n");
        #endif
        return ESP32_ERROR;
    }

    /* Enable auto-connect */
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CWAUTOCONN=1");
    ESP32_WaitForResponse("OK", 1000);

    /* Set single connection mode */
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CIPMUX=0");
    ESP32_WaitForResponse("OK", 1000);

    esp32Initialized = true;
    wifiStatus = WIFI_STATUS_IDLE;

    #if DEBUG_WIFI
    UART_SendString("[ESP32] WiFi module initialized successfully!\r\n");
    #endif

    return ESP32_OK;
}

/**
 * @brief Reset ESP32 module
 */
ESP32_Error_t ESP32_WiFi_Reset(void) {
    #if DEBUG_WIFI
    UART_SendString("[ESP32] Resetting module...\r\n");
    #endif

    ESP32_ClearBuffer();
    ESP32_SendLine("AT+RST");

    if (ESP32_WaitForResponse("ready", 10000)) {
        wifiStatus = WIFI_STATUS_IDLE;
        SysTick_Delay(1000);
        #if DEBUG_WIFI
        UART_SendString("[ESP32] Reset successful!\r\n");
        #endif
        return ESP32_OK;
    }

    return ESP32_TIMEOUT;
}

/**
 * @brief Test ESP32 communication
 */
ESP32_Error_t ESP32_WiFi_Test(void) {
    ESP32_ClearBuffer();
    ESP32_SendLine("AT");

    if (ESP32_WaitForResponse("OK", 1000)) {
        return ESP32_OK;
    }

    return ESP32_NO_RESPONSE;
}

/**
 * @brief Connect to WiFi network - IMPROVED VERSION
 */
ESP32_Error_t ESP32_WiFi_Connect(void) {
    char cmd[200];

    #if DEBUG_WIFI
    char msg[100];
    snprintf(msg, sizeof(msg), "[ESP32] Connecting to WiFi: %s\r\n", WIFI_SSID);
    UART_SendString(msg);
    #endif

    /* Check if already connected */
    if (ESP32_WiFi_CheckConnection()) {
        wifiStatus = WIFI_STATUS_GOT_IP;
        #if DEBUG_WIFI
        UART_SendString("[ESP32] Already connected!\r\n");
        #endif

        /* Try to get RSSI */
        ESP32_WiFi_GetRSSI(&currentRSSI);
        return ESP32_OK;
    }

    /* Build connection command */
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);

    wifiStatus = WIFI_STATUS_CONNECTING;
    ESP32_ClearBuffer();
    ESP32_SendLine(cmd);

    /* Wait for connection - IMPROVED handling like test code */
    uint32_t start = systick_counter;
    char response[512];
    bool success = false;

    #if DEBUG_WIFI
    UART_SendString("Connecting");
    #endif

    while ((systick_counter - start) < 30000) {  /* 30 second timeout */
        /* Collect response */
        ESP32_ReadAll(response, sizeof(response), 500);

        /* Check various success indicators - like test code */
        if (strstr(response, "WIFI CONNECTED") ||
            strstr(response, "WIFI GOT IP") ||
            strstr(response, "\r\nOK\r\n")) {
            success = true;
            break;
        }

        /* Check for failure */
        if (strstr(response, "FAIL") || strstr(response, "ERROR")) {
            wifiStatus = WIFI_STATUS_ERROR;
            #if DEBUG_WIFI
            UART_SendString("\r\n[ESP32] Connection failed!\r\n");
            #endif
            return ESP32_ERROR;
        }

        #if DEBUG_WIFI
        UART_SendString(".");
        #endif
        ESP32_ClearBuffer();
    }

    if (success) {
        #if DEBUG_WIFI
        UART_SendString("\r\n[ESP32] Connection command accepted!\r\n");
        #endif

        /* Verify by checking IP - like test code */
        SysTick_Delay(2000);  /* Give time for DHCP */
        if (ESP32_WiFi_CheckConnection()) {
            wifiStatus = WIFI_STATUS_GOT_IP;
            ESP32_WiFi_GetRSSI(&currentRSSI);
            #if DEBUG_WIFI
            UART_SendString("[ESP32] WiFi connected successfully!\r\n");
            #endif
            return ESP32_OK;
        }
    }

    #if DEBUG_WIFI
    UART_SendString("\r\n[ESP32] Connection timeout or failed!\r\n");
    #endif
    wifiStatus = WIFI_STATUS_DISCONNECTED;
    return ESP32_TIMEOUT;
}

/**
 * @brief Disconnect from WiFi
 */
ESP32_Error_t ESP32_WiFi_Disconnect(void) {
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CWQAP");

    if (ESP32_WaitForResponse("OK", 2000)) {
        wifiStatus = WIFI_STATUS_DISCONNECTED;
        strcpy(currentIP, "0.0.0.0");
        currentRSSI = -100;
        #if DEBUG_WIFI
        UART_SendString("[ESP32] Disconnected from WiFi\r\n");
        #endif
        return ESP32_OK;
    }

    return ESP32_ERROR;
}

/**
 * @brief Check if connected to WiFi
 */
bool ESP32_WiFi_IsConnected(void) {
    return ESP32_WiFi_CheckConnection();
}

/**
 * @brief Get WiFi status
 */
WiFi_Status_t ESP32_WiFi_GetStatus(void) {
    return wifiStatus;
}

/**
 * @brief Get IP address
 */
ESP32_Error_t ESP32_WiFi_GetIP(char* ip_buffer) {
    if (ip_buffer == NULL) {
        return ESP32_INVALID_PARAM;
    }

    strcpy(ip_buffer, currentIP);
    return ESP32_OK;
}

/**
 * @brief Send weather data to cloud - STUB FUNCTION (not used in main.c)
 */
ESP32_Error_t ESP32_WiFi_SendWeatherData(const WeatherData_t* data) {
    /* This function is not used anymore - cloud sending is done inline in main.c */
    UART_SendString("[ESP32] SendWeatherData called - using inline version instead\r\n");
    return ESP32_OK;
}

/**
 * @brief Print WiFi status
 */
void ESP32_WiFi_PrintStatus(void) {
    char msg[100];
    const char* status_str[] = {
        "IDLE", "CONNECTING", "CONNECTED", "GOT_IP", "DISCONNECTED", "ERROR"
    };

    UART_SendString("\r\n=== ESP32 WiFi Status ===\r\n");

    snprintf(msg, sizeof(msg), "Initialized: %s\r\n",
             esp32Initialized ? "Yes" : "No");
    UART_SendString(msg);

    snprintf(msg, sizeof(msg), "Status: %s\r\n",
             status_str[wifiStatus]);
    UART_SendString(msg);

    snprintf(msg, sizeof(msg), "IP Address: %s\r\n", currentIP);
    UART_SendString(msg);

    if (currentRSSI > -100) {
        snprintf(msg, sizeof(msg), "Signal Strength: %ld dBm\r\n", currentRSSI);
        UART_SendString(msg);
    }

    UART_SendString("========================\r\n");
}

/**
 * @brief Get WiFi signal strength
 */
ESP32_Error_t ESP32_WiFi_GetRSSI(int32_t* rssi) {
    if (rssi == NULL) {
        return ESP32_INVALID_PARAM;
    }

    char response[200];

    /* Query current AP info */
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CWJAP?");
    ESP32_ReadAll(response, sizeof(response), 1000);

    /* Parse RSSI from response */
    /* Response format: +CWJAP:"ssid","mac",ch,rssi */
    char* rssi_start = strrchr(response, ',');
    if (rssi_start) {
        currentRSSI = atoi(rssi_start + 1);
        *rssi = currentRSSI;
        return ESP32_OK;
    }

    /* If parsing fails, return default */
    *rssi = currentRSSI;
    return ESP32_OK;
}

/**
 * @brief Set low power mode
 */
ESP32_Error_t ESP32_WiFi_SetLowPower(bool enable) {
    ESP32_ClearBuffer();

    if (enable) {
        ESP32_SendLine("AT+SLEEP=1");  /* Light sleep */
    } else {
        ESP32_SendLine("AT+SLEEP=0");  /* Disable sleep */
    }

    if (ESP32_WaitForResponse("OK", 2000)) {
        return ESP32_OK;
    }

    return ESP32_ERROR;
}

/**
 * @brief Process background tasks
 */
void ESP32_WiFi_Process(void) {
    /* This can be used for handling unsolicited messages */
    /* or periodic connection checks if needed */
}

/**
 * @brief Test TCP connectivity - WORKING VERSION FROM TEST CODE
 */
ESP32_Error_t ESP32_WiFi_TestTCP(void) {
    UART_SendString("\r\n[TEST] Starting TCP connectivity test...\r\n");

    /* Test 1: Basic AT command */
    UART_SendString("[TEST] 1. Testing AT command...\r\n");
    ESP32_ClearBuffer();
    ESP32_SendLine("AT");
    if (ESP32_WaitForResponse("OK", 1000)) {
        UART_SendString("[TEST] ✓ AT command OK\r\n");
    } else {
        UART_SendString("[TEST] ✗ AT command failed!\r\n");
        return ESP32_ERROR;
    }

    /* Test 2: Check connection status */
    UART_SendString("[TEST] 2. Checking WiFi connection...\r\n");
    if (ESP32_WiFi_CheckConnection()) {
        char msg[50];
        snprintf(msg, sizeof(msg), "[TEST] ✓ WiFi connected (IP: %s)\r\n", currentIP);
        UART_SendString(msg);
    } else {
        UART_SendString("[TEST] ✗ WiFi not connected!\r\n");
        return ESP32_ERROR;
    }

    /* Test 3: Try TCP to Google DNS */
    UART_SendString("[TEST] 3. Testing TCP to 8.8.8.8:80...\r\n");
    ESP32_ClearBuffer();
    ESP32_SendLine("AT+CIPCLOSE");
    SysTick_Delay(1000);

    ESP32_SendLine("AT+CIPSTART=\"TCP\",\"8.8.8.8\",80");

    if (ESP32_WaitForResponse("CONNECT", 10000)) {
        UART_SendString("[TEST] ✓ TCP connection successful!\r\n");
        ESP32_SendLine("AT+CIPCLOSE");
        return ESP32_OK;
    } else {
        UART_SendString("[TEST] ✗ TCP connection failed!\r\n");
        return ESP32_ERROR;
    }
}
