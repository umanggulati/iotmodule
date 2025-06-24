# STM32F429ZI IoT Weather Station

A complete IoT weather monitoring system built on STM32F429ZI microcontroller with ESP32 WiFi connectivity and BMP280 environmental sensor. Features real-time data collection, cloud integration via ThingSpeak, and a user-friendly API.

## 🌟 Features

- **Real-time Weather Monitoring**: Temperature and atmospheric pressure sensing
- **WiFi Connectivity**: ESP32-based internet connection with AT command interface
- **Cloud Integration**: Automatic data upload to ThingSpeak IoT platform
- **Power Management**: Advanced power saving modes and wake-up timers
- **Task Scheduler**: Cooperative multitasking for sensor reading, WiFi management, and cloud updates
- **Simple API**: Easy-to-use weather station API for rapid development
- **Baremetal Implementation**: No RTOS dependency, efficient resource usage
- **Professional Logging**: Comprehensive debug output and system monitoring

## 🔧 Hardware Requirements

### Main Components
- **STM32F429ZI Development Board** (Nucleo-F429ZI recommended)
- **ESP32-WROOM-32 Module** with AT Firmware
- **BMP280 Temperature/Pressure Sensor**
- **Breadboard and Jumper Wires**
- **USB Cables** for programming and power



## 📋 Hardware Connections

### ESP32-WROOM-32 to STM32F429ZI Connection

The ESP32 module communicates with STM32 via USART2 using AT commands:

| STM32F429ZI Pin | ESP32 Pin | Function |
|----------------|-----------|----------|
| PD5 (USART2_TX) | RX2 | STM32 → ESP32 Data |
| PD6 (USART2_RX) | TX2 | ESP32 → STM32 Data |
| GND | GND | Ground |
| 3.3V | 3.3V | Power |

**ESP32 AT Command Configuration:**
- **Interface**: UART2 for AT commands (RX2/TX2)
- **Baudrate**: 115200
- **Data Format**: 8 bits, No parity, 1 stop bit
- **Flow Control**: None (CTS/RTS not used)

### BMP280 Sensor to STM32F429ZI Connection

The BMP280 sensor connects via I2C1 interface:

| STM32F429ZI Pin | BMP280 Pin | Function |
|----------------|------------|----------|
| PB8 (I2C1_SCL) | SCL | I2C Clock |
| PB9 (I2C1_SDA) | SDA | I2C Data |
| GND | GND | Ground |
| 3.3V | VCC | Power (3.3V) |

**I2C Configuration:**
- **Address**: 0x76 (default, can be 0x77 if SD0 pulled high)
- **Clock Speed**: 100kHz (Standard Mode)
- **Pull-up Resistors**: 4.7kΩ (usually integrated on sensor board)

### Debug UART Connection

For development and monitoring:

| STM32F429ZI Pin | USB-Serial Converter |
|----------------|---------------------|
| PD8 (USART3_TX) | RX |
| PD9 (USART3_RX) | TX |
| GND | GND |

### LED Indicators

| LED Color | STM32 Pin | Function |
|-----------|-----------|----------|
| Green | PB0 | WiFi Connected / System OK |
| Blue | PB7 | WiFi Connecting / Activity |
| Red | PB14 | Error / WiFi Disconnected |

## 🔌 Wiring Diagrams

### STM32F429ZI ↔ ESP32-WROOM-32 Connection

```
STM32F429ZI Nucleo                    ESP32-WROOM-32 Module
┌─────────────────────────┐          ┌──────────────────────┐
│                         │          │                      │
│  PD5 (USART2_TX) ──────────────────→ RX2                  │
│                         │          │                      │
│  PD6 (USART2_RX) ←────────────────── TX2                  │
│                         │          │                      │
│  3.3V ─────────────────────────────→ 3.3V                 │
│                         │          │                      │
│  GND ──────────────────────────────→ GND                  │
│                         │          │                      │
└─────────────────────────┘          └──────────────────────┘
```

### STM32F429ZI ↔ BMP280 Sensor Connection

```
STM32F429ZI Nucleo                    BMP280 Sensor
┌─────────────────────────┐          ┌──────────────────────┐
│                         │          │                      │
│  PB8 (I2C1_SCL) ──────────────────→ SCL                  │
│                         │          │                      │
│  PB9 (I2C1_SDA) ←────────────────→ SDA                  │
│                         │          │                      │
│  3.3V ─────────────────────────────→ VCC                  │
│                         │          │                      │
│  GND ──────────────────────────────→ GND                  │
│                         │          │                      │
└─────────────────────────┘          └──────────────────────┘
```

### LED Connections (On STM32F429ZI)

```
STM32F429ZI Pins:
├── PB0  → Green LED (WiFi Connected)
├── PB7  → Blue LED (WiFi Activity) 
└── PB14 → Red LED (Error Status)
```

## ⚙️ ESP32 AT Firmware Setup

### Prerequisites

1. **Download ESP32 Flash Download Tool** from [Espressif](https://www.espressif.com/en/support/download/other-tools)
2. **Get AT Firmware** for ESP32-WROOM-32 from [ESP-AT releases](https://github.com/espressif/esp-at/releases)

### Flashing AT Firmware

#### Windows Method:
1. **Open ESP32 Flash Download Tool**
2. **Select Chip Type**: ESP32
3. **Select Work Mode**: Developer Mode
4. **Configure Files**: Add the following files with their addresses:
   ```
   0x1000    bootloader/bootloader.bin
   0x8000    partition_table/partition-table.bin  
   0x10000   ota_data_initial.bin
   0x20000   at_customize.bin
   0x21000   customized_partitions/mfg_nvs.bin
   0x100000  esp-at.bin
   ```
5. **Flash Settings**: 
   - Flash Mode: DIO
   - Flash Frequency: 40MHz  
   - Flash Size: 4MB
6. **Select COM Port** and click **START**

#### Linux/macOS Method:
```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 \
  --before default_reset --after hard_reset write_flash -z \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 bootloader/bootloader.bin \
  0x8000 partition_table/partition-table.bin \
  0x10000 ota_data_initial.bin \
  0x20000 at_customize.bin \
  0x21000 customized_partitions/mfg_nvs.bin \
  0x100000 esp-at.bin
```

### Verify AT Firmware

Connect to ESP32 via serial terminal (115200 baud) and test:
```
AT

Response: OK

AT+GMR

Response: 
AT version:3.2.0.0(s-ec2dec2 - ESP32 - Jul 28 2023 07:05:28)
SDK version:v5.0.2-376-g24b9d38a24-dirty
compile time(6118fc22):Jul 28 2023 09:47:28
Bin version:v3.2.0.0(WROOM-32)
OK
```

## 🌐 ThingSpeak Integration

### Setting up ThingSpeak Account

1. **Create Account**: Visit [ThingSpeak.com](https://thingspeak.com) and create a free account
2. **Create New Channel**: 
   - Navigate to "My Channels" → "New Channel"
   - **Channel Name**: "STM32 Weather Station"
   - **Field 1**: "Temperature" 
   - **Field 2**: "Pressure"
   - Click "Save Channel"
3. **Get API Key**: 
   - Go to "API Keys" tab
   - Copy the "Write API Key"

### Configuration

Edit `wifi_config.h` file with your credentials:

```c
// WiFi Network Settings
#define WIFI_SSID               "Your_WiFi_Network_Name"
#define WIFI_PASSWORD           "Your_WiFi_Password"

// ThingSpeak Cloud Settings  
#define THINGSPEAK_API_KEY      "YOUR_WRITE_API_KEY_HERE"
#define CLOUD_SERVER            "api.thingspeak.com"
#define CLOUD_PORT              80
#define CLOUD_UPDATE_URL        "/update"
```

### Data Format Sent to ThingSpeak

The system automatically formats and sends data as:
```
POST /update HTTP/1.1
Host: api.thingspeak.com
Content-Type: application/x-www-form-urlencoded

api_key=YOUR_API_KEY&field1=25.67&field2=1013
```

Where:
- **field1**: Temperature in Celsius (e.g., 25.67°C)
- **field2**: Atmospheric pressure in hPa (e.g., 1013 hPa)

### Viewing Your Data

1. **Live Data**: Visit your ThingSpeak channel page
2. **Charts**: Data automatically appears in Field 1 and Field 2 charts
3. **Export**: Download data as CSV, JSON, or XML
4. **Widgets**: Create custom dashboards and widgets

## 📁 Project File Structure

```
stm32f429-iot-module/
├── README.md                    # This documentation file
├── .gitignore                   # Git ignore rules
├── .gitattributes              # Git attributes
├── .cproject                   # Eclipse CDT project file
├── .project                    # Eclipse project file
├── STM32F429ZITX_FLASH.ld      # Linker script for Flash
├── STM32F429ZITX_RAM.ld        # Linker script for RAM
├── stm32f429-iot-module.launch # Debug configuration
├── embeddedC_gpio1234 Debug.launch # Debug launch config
├── 
├── Inc/                        # Header files
│   ├── cmsis_armcc.h          # CMSIS ARM compiler header
│   ├── cmsis_armclang.h       # CMSIS ARM Clang header
│   ├── cmsis_compiler.h       # CMSIS compiler definitions
│   ├── cmsis_gcc.h           # CMSIS GCC compiler header
│   ├── cmsis_iccarm.h        # CMSIS IAR compiler header
│   ├── cmsis_version.h       # CMSIS version definitions
│   ├── core_cm4.h            # Cortex-M4 core definitions
│   ├── esp32_wifi.h          # ESP32 WiFi AT command driver
│   ├── i2c.h                 # I2C driver for BMP280
│   ├── mpu_armv7.h           # MPU (Memory Protection Unit) definitions
│   ├── mpu_armv8.h           # MPU ARMv8 definitions
│   ├── power_management.h    # Power management system
│   ├── stm32f4xx.h          # STM32F4 register definitions
│   ├── system_config.h       # System configuration
│   ├── system_stm32f4xx.h   # STM32F4 system header
│   ├── systick.h            # SysTick timer driver
│   ├── task_scheduler.h     # Cooperative task scheduler
│   ├── tz_context.h         # TrustZone context definitions
│   ├── uart.h               # UART driver
│   ├── weather_api.h        # Simple weather station API
│   └── wifi_config_template.h # WiFi configuration template
│
├── Src/                       # Source files
│   ├── esp32_wifi.c          # ESP32 WiFi AT command implementation
│   ├── i2c.c                 # I2C driver implementation
│   ├── main.c                # Main application file
│   ├── power_management.c    # Power management implementation
│   ├── syscalls.c            # System call implementations
│   ├── sysmem.c              # System memory management
│   ├── system_config.c       # System configuration implementation
│   ├── systick.c             # SysTick timer implementation
│   ├── task_scheduler.c      # Task scheduler implementation
│   ├── uart.c                # UART driver implementation
│   └── weather_api.c         # Weather station API implementation
│
├── Startup/                   # Startup and system files
│   └── startup_stm32f429zitx.s # Assembly startup file
│
├── Debug/                     # Debug build output
│   ├── (compiled object files)
│   └── (debug information)
│
├── Binaries/                  # Compiled binary files
│   └── (generated .bin/.hex files)
│
└── settings/                  # IDE settings and configurations
    └── (IDE-specific configuration files)
```

## 💻 Software Setup

### Prerequisites

1. **STM32CubeIDE** (recommended) or **ARM Keil** for compilation
2. **Git** for cloning the repository
3. **Serial Terminal** (PuTTY, Tera Term, or built-in IDE terminal)

### Building the Project

1. **Clone Repository**:
```bash
git clone https://github.com/yourusername/stm32-weather-station.git
cd stm32-weather-station
```

2. **Configure WiFi and ThingSpeak**:
   - Open `wifi_config.h`
   - Update `WIFI_SSID` and `WIFI_PASSWORD`
   - Update `THINGSPEAK_API_KEY` with your Write API Key

3. **Import to STM32CubeIDE**:
   - File → Import → Existing Projects into Workspace
   - Select the project folder
   - Build the project

4. **Flash to STM32**:
   - Connect STM32F429ZI via USB
   - Run → Debug As → STM32 MCU C/C++ Application
   - Program will be flashed and debugger will start

## 🚀 Quick Start Guide

### 1. Hardware Assembly
- Connect all components according to the wiring diagram
- Ensure ESP32 has AT firmware properly flashed
- Power up the system via USB

### 2. Initial Configuration
```c
// In wifi_config.h, update these critical settings:
#define WIFI_SSID               "YourWiFiNetwork"
#define WIFI_PASSWORD           "YourPassword"  
#define THINGSPEAK_API_KEY      "YourThingSpeakWriteAPIKey"
```

### 3. First Run Experience
1. **Connect Serial Terminal**: 
   - Baud: 115200, Data: 8N1
   - Connect to STM32 debug UART (PD8/PD9)
2. **Power On**: System initialization messages will appear
3. **Verify Components**:
   ```
   [BMP280] Chip ID verified! 
   [ESP32] WiFi module initialized successfully!
   [SYSTEM] All core systems initialized successfully!
   ```

### 4. Interactive Command Interface

The system provides a comprehensive command interface:

| Command | Function | Description |
|---------|----------|-------------|
| `s` | Simple API Test | Test user-friendly weather API |
| `r` | Read Sensors | Manual sensor reading |
| `c` | Connect WiFi | Manual WiFi connection |
| `p` | Send to Cloud | Manual ThingSpeak upload |
| `e` | Enable Auto Mode | Start automatic operation |
| `d` | Disable Auto Mode | Stop automatic operation |
| `i` | System Info | Comprehensive status display |
| `w` | WiFi Status | Network connection details |
| `a` | AT Test | Test ESP32 communication |
| `h` | HTTP Test | Test cloud connectivity |
| `z` | Debug Temperature | Temperature formatting test |
| `v` | Verify JSON | Validate data transmission |
| `y` | Power Test | Power management validation |

### 5. Automatic Operation

**Enable Automatic Mode:**
```
Press 'e' in terminal

Response:
🚀 ENABLING ENHANCED TASK SCHEDULER...
✅ Enhanced task scheduler ENABLED!
⚡ Fast testing intervals active
🔧 Temperature formatting FIXED
🎉 Simple API available
```

**Monitor Operation:**
```
Press 'i' for system status

=== System Status ===
Sensor readings: 23
Last reading: 25.67°C, 1013 hPa
WiFi State: CONNECTED
WiFi IP: 192.168.1.105
Cloud Updates: 8 successful, 0 failed
Task Scheduler: ENABLED
```

## 📊 Simple Weather API Usage

The project includes an intuitive API for easy integration:

### Basic Weather Reading
```c
#include "weather_api.h"

int main(void) {
    // Initialize everything with one function call
    if (!WeatherStation_Init()) {
        printf("Initialization failed!\n");
        return -1;
    }
    
    // Connect to WiFi (blocks until connected or timeout)
    if (!WeatherStation_ConnectWiFi(NULL, NULL, 30)) {
        printf("WiFi connection failed!\n");
        return -1;
    }
    
    while (1) {
        // Get weather reading
        WeatherReading_t reading = WeatherStation_GetReading();
        if (reading.data_valid) {
            printf("📊 Temperature: %.2f°C\n", reading.temperature_celsius);
            printf("📊 Pressure: %d hPa\n", reading.pressure_hpa);
            printf("📊 Time: %s\n", reading.timestamp);
            
            // Send to ThingSpeak
            if (WeatherStation_SendToCloud(&reading)) {
                printf("✅ Data uploaded successfully!\n");
            } else {
                printf("❌ Upload failed: %s\n", WeatherStation_GetLastError());
            }
        }
        
        WeatherStation_Delay(60000); // Wait 1 minute
    }
}
```

### Automatic Operation Mode
```c
// Start fully automatic weather station
if (WeatherStation_StartAutoMode()) {
    printf("🚀 Weather station running automatically!\n");
    
    // System now handles everything:
    // - Regular sensor readings (every 30 seconds)
    // - Automatic cloud uploads (every 5 minutes)  
    // - WiFi connection management
    // - Error recovery
    
    while (1) {
        // Just process background tasks
        WeatherStation_Process();
        WeatherStation_Delay(100);
    }
}
```

### System Monitoring
```c
// Get comprehensive system status
WeatherStatus_t status;
if (WeatherStation_GetStatus(&status)) {
    printf("Sensor: %s\n", status.sensor_working ? "✅ OK" : "❌ Failed");
    printf("WiFi: %s (%s)\n", status.wifi_connected ? "✅ Connected" : "❌ Disconnected", status.wifi_ip);
    printf("Readings: %lu total\n", status.total_readings);
    printf("Uploads: %lu successful, %lu failed\n", status.successful_uploads, status.failed_uploads);
    printf("Uptime: %lu seconds\n", status.uptime_seconds);
}

// Print formatted status to terminal
WeatherStation_PrintStatus();
```

## 🔧 Configuration Options

### Timing Configuration

```c
// In wifi_config.h - Adjust measurement intervals

// Fast testing mode (for development)
#define FAST_WEATHER_READ_INTERVAL   15000   // 15 seconds
#define FAST_CLOUD_UPDATE_INTERVAL   60000   // 1 minute

// Production mode (for deployment)  
#define WEATHER_READ_INTERVAL        30000   // 30 seconds
#define CLOUD_UPDATE_INTERVAL        300000  // 5 minutes
#define WIFI_MANAGER_INTERVAL        10000   // 10 seconds
```

### Power Management

```c
// Power saving options
#define ENABLE_AUTO_SLEEP       0    // Set to 1 to enable auto sleep
#define SLEEP_DURATION         60    // Sleep duration in seconds
#define SLEEP_BETWEEN_READS    0     // Sleep between sensor readings
```

### Debug Output Control

```c
// Enable/disable debug messages for different subsystems
#define DEBUG_WIFI              1    // WiFi connection debug
#define DEBUG_SENSOR            1    // Sensor reading debug  
#define DEBUG_CLOUD             1    // Cloud communication debug
#define DEBUG_TASK_SCHEDULER    0    // Task scheduler debug
#define DEBUG_POWER_MGMT        1    // Power management debug
```

### Error Handling Limits

```c
// Maximum retry attempts before giving up
#define MAX_SENSOR_ERRORS       5    // Sensor communication errors
#define MAX_CLOUD_ERRORS        3    // Cloud upload errors  
#define MAX_WIFI_ERRORS         10   // WiFi connection errors
#define WIFI_CONNECTION_TIMEOUT 30000 // WiFi timeout (milliseconds)
```

## 📈 Monitoring and Debugging

### Serial Output Messages

The system provides comprehensive logging:

```
═══════════════════════════════════════════════════════
    STM32F429ZI Weather Station - WITH SIMPLE API!      
   Complete IoT Platform + Easy-to-Use Interface       
═══════════════════════════════════════════════════════

[BMP280] Initializing sensor...
[BMP280] Chip ID verified! Resetting sensor...
[BMP280] Calibration: T1=27504, T2=26435, T3=-1000
[BMP280] Initialization complete!

[ESP32] Initializing WiFi module...
[ESP32] WiFi module initialized successfully!

[WiFi] Connecting to WiFi: YourNetwork
Connecting.....
[WiFi] Connection command accepted!
[ESP32] WiFi connected successfully!

[TASK] Reading #1: 25.67°C, 1013 hPa
[THINGSPEAK] Sending to real IoT platform...
[THINGSPEAK] ✅ Data accepted by ThingSpeak!
[THINGSPEAK] 🎉 Check your channel at: https://thingspeak.com/channels/YOUR_CHANNEL_ID
```

### System Status Display

Use the `i` command for detailed system information:

```
=== System Status - TEMPERATURE FIXED VERSION ===
Sensor readings: 47
Last reading: 25.67°C, 1013 hPa
WiFi State: CONNECTED  
WiFi IP: 192.168.1.105
Connected for: 1847 seconds
Cloud Updates: 15 successful, 0 failed
FAST TESTING INTERVALS ACTIVE:
  - Weather reading: 15 seconds
  - Cloud update: 60 seconds
Last cloud update: 23 seconds ago
Task Scheduler: ENABLED
============================================
```

### LED Status Indicators

Visual feedback through onboard LEDs:

- **🟢 Green LED (PB0)**:
  - **Solid ON**: WiFi connected, system operating normally
  - **Slow Blink**: Sensor reading in progress
  - **OFF**: System disconnected or in power-save mode

- **🔵 Blue LED (PB7)**:
  - **Fast Blink**: WiFi connecting or attempting connection
  - **Slow Blink**: Network activity (data transmission)
  - **OFF**: WiFi idle or connected

- **🔴 Red LED (PB14)**:
  - **Flash**: Error condition detected
  - **Solid ON**: Critical system error
  - **OFF**: Normal operation

## 🐛 Troubleshooting Guide

### WiFi Connection Issues

**Symptoms**: `[WiFi] Connection failed!` or `WiFi State: DISCONNECTED`

**Solutions**:
1. **Verify Credentials**:
   ```c
   // Check wifi_config.h
   #define WIFI_SSID     "ExactNetworkName"  // Case sensitive!
   #define WIFI_PASSWORD "ExactPassword"     // Check special characters
   ```

2. **Test ESP32 Communication**:
   ```
   Press 'a' in terminal → Should show "ESP32 responding OK"
   If failed: Check PD5↔RX2, PD6↔TX2 connections
   Verify ESP32 power (3.3V) and ground connections
   ```

3. **Manual WiFi Test**:
   ```
   Press 'c' → Manual connection attempt
   Monitor debug output for specific error messages
   ```

4. **ESP32 AT Firmware**:
   ```
   Verify AT firmware is properly flashed
   Test with direct serial connection to ESP32:
   AT → Should respond "OK"
   AT+GMR → Should show firmware version
   Ensure RX2/TX2 pins are configured for AT commands
   ```

### Sensor Communication Problems

**Symptoms**: `[BMP280] I2C communication failed!` or invalid readings

**Solutions**:
1. **Check I2C Connections**:
   ```
   STM32 PB8 → BMP280 SCL
   STM32 PB9 → BMP280 SDA  
   3.3V → BMP280 VCC
   GND → BMP280 GND
   ```

2. **Test I2C Bus**:
   ```
   Press 'r' → Manual sensor reading
   Should show: "[BMP280] Chip ID verified!"
   ```

3. **Verify Power Supply**:
   ```
   Measure 3.3V on BMP280 VCC pin
   Ensure sufficient current capability
   Check for loose connections
   ```

4. **I2C Address**:
   ```c
   // Default address is 0x76
   // If SD0 pin is pulled high, address becomes 0x77
   // Modify BMP280_ADDR in code if needed
   ```

### Cloud Upload Failures

**Symptoms**: `[THINGSPEAK] ❌ Data rejected` or upload timeouts

**Solutions**:
1. **Verify API Key**:
   ```c
   // Ensure Write API Key is correct in wifi_config.h
   #define THINGSPEAK_API_KEY "YOUR_16_CHARACTER_KEY"
   ```

2. **Test HTTP Connectivity**:
   ```
   Press 'h' → Test HTTP endpoints
   Should connect to httpbin.org successfully
   ```

3. **Check Internet Access**:
   ```
   Press 'w' → Should show valid IP address
   Verify router allows outbound HTTP on port 80
   ```

4. **Manual Upload Test**:
   ```
   Press 'p' → Manual cloud send with response
   Monitor debug output for HTTP response codes
   ```

### System Performance Issues

**Symptoms**: System hangs, crashes, or erratic behavior

**Solutions**:
1. **Power Supply Check**:
   ```
   Verify stable 5V USB supply
   Check for voltage drops during WiFi transmission
   Use powered USB hub if needed
   ```

2. **Memory Usage**:
   ```
   Press 't' → Task scheduler status
   Monitor for stack overflow warnings in debug output
   ```

3. **Reset and Recovery**:
   ```
   Power cycle the entire system
   Check all connections are secure
   Verify breadboard connections with multimeter
   ```

### Debug Commands for Diagnostics

| Command | Purpose | Expected Output |
|---------|---------|----------------|
| `a` | ESP32 Communication | `ESP32 responding OK` |
| `r` | Sensor Test | `Reading #X: XX.XX°C, XXXX hPa` |
| `c` | WiFi Connection | `Manual connection successful!` |  
| `w` | Network Status | `WiFi connected (IP: X.X.X.X)` |
| `h` | HTTP Test | `TCP connection successful!` |
| `p` | Cloud Upload | `Data accepted by ThingSpeak!` |
| `i` | System Status | Complete system information |
| `y` | Power Test | `All power management tests PASSED!` |

## 📚 Complete API Reference

### Core Initialization
```c
bool WeatherStation_Init(void);
// Returns: true if all systems initialized successfully
// Call this first before any other API functions
```

### Data Acquisition
```c
WeatherReading_t WeatherStation_GetReading(void);
// Returns: Weather data structure with validity flag
// Check data_valid field before using readings

typedef struct {
    float temperature_celsius;    // Temperature in °C (-40.0 to +85.0)
    int pressure_hpa;            // Pressure in hPa (300 to 1100)  
    char timestamp[20];          // Human readable "DayX HH:MM:SS"
    bool data_valid;             // true if reading is valid
    uint32_t reading_number;     // Sequential reading count
} WeatherReading_t;
```

### Network Connectivity
```c
bool WeatherStation_ConnectWiFi(const char* ssid, const char* password, uint32_t timeout_sec);
// Parameters: WiFi credentials (NULL uses config file), timeout in seconds
// Returns: true if connected successfully

bool WeatherStation_IsConnected(void);
// Returns: true if WiFi connected with valid IP address

bool WeatherStation_DisconnectWiFi(void);
// Returns: true if disconnected successfully

int WeatherStation_GetWiFiSignalStrength(void);
// Returns: RSSI in dBm (-100 to 0, higher is better)
```

### Cloud Integration
```c
bool WeatherStation_SendToCloud(WeatherReading_t* reading);
// Parameters: Pointer to valid weather reading
// Returns: true if uploaded to ThingSpeak successfully

bool WeatherStation_SetCloudEndpoint(const char* server, uint16_t port, const char* endpoint);
// Parameters: Custom server configuration  
// Returns: true if configuration accepted
```

### Automatic Operation
```c
bool WeatherStation_StartAutoMode(void);
// Starts automatic sensor reading and cloud uploading
// Returns: true if auto mode started successfully

bool WeatherStation_StopAutoMode(void);  
// Stops automatic operation
// Returns: true if auto mode stopped successfully

bool WeatherStation_IsAutoModeActive(void);
// Returns: true if automatic mode is currently running
```

### System Status and Configuration
```c
bool WeatherStation_GetStatus(WeatherStatus_t* status);
// Parameters: Pointer to status structure to fill
// Returns: true if status retrieved successfully

typedef struct {
    bool sensor_working;         // BMP280 sensor status
    bool wifi_connected;        // Network connection status  
    char wifi_ip[16];           // Current IP address string
    uint32_t total_readings;    // Total sensor readings taken
    uint32_t successful_uploads; // Successful cloud uploads
    uint32_t failed_uploads;    // Failed cloud uploads
    uint32_t uptime_seconds;    // System uptime in seconds
} WeatherStatus_t;

bool WeatherStation_Configure(WeatherConfig_t* config);
bool WeatherStation_GetConfig(WeatherConfig_t* config);  
bool WeatherStation_ResetConfig(void);

typedef struct {
    uint32_t reading_interval_ms;    // Sensor reading interval
    uint32_t upload_interval_ms;     // Cloud upload interval
    bool auto_upload_enabled;        // Enable automatic uploads
    bool debug_output_enabled;       // Enable debug messages
} WeatherConfig_t;
```

### Utility Functions
```c
void WeatherStation_Delay(uint32_t delay_ms);
// Simple blocking delay function

void WeatherStation_PrintStatus(void);
// Print comprehensive status to debug UART

void WeatherStation_PrintReading(WeatherReading_t* reading);
// Print weather reading in human-readable format

bool WeatherStation_RunSystemTest(void);
// Run comprehensive system self-test
// Returns: true if all tests pass

const char* WeatherStation_GetLastError(void);
// Returns: Pointer to last error message string

void WeatherStation_Process(void);
// Process background tasks (call in main loop if not using auto mode)
```

### Feature Control
```c
typedef enum {
    WEATHER_FEATURE_AUTO_UPLOAD,     // Automatic cloud uploading
    WEATHER_FEATURE_DEBUG_OUTPUT,    // Debug message output
    WEATHER_FEATURE_LED_INDICATORS,  // Status LED indicators  
    WEATHER_FEATURE_POWER_SAVING     // Power management features
} WeatherFeature_t;

bool WeatherStation_SetFeature(WeatherFeature_t feature, bool enable);
// Enable or disable specific system features
// Returns: true if feature setting applied successfully
```

## 🔒 Production Deployment

### Security Considerations
- **WiFi Credentials**: Store securely, consider using encrypted configuration
- **API Keys**: Rotate ThingSpeak API keys periodically
- **Physical Security**: Ensure device cannot be easily tampered with
- **Network Security**: Use WPA3 encryption on WiFi network

### Performance Optimization
- **Power Consumption**: Enable auto-sleep modes for battery operation
- **Data Efficiency**: Adjust upload intervals based on application needs
- **Memory Usage**: Monitor stack usage with debug output
- **Network Traffic**: Optimize data transmission frequency

### Reliability Features
- **Watchdog Timer**: Automatic system reset on hangs
- **Error Recovery**: Automatic retry mechanisms for all operations
- **Data Validation**: Sensor data range checking and outlier detection
- **Connection Monitoring**: Automatic WiFi reconnection on failures

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

### Development Guidelines
- Follow existing code style and formatting
- Add comprehensive comments for new features
- Test thoroughly on actual hardware  
- Update documentation for any API changes
- Include example usage for new functions

## 🏆 Acknowledgments

- **STMicroelectronics**: STM32F429ZI microcontroller and development ecosystem
- **Espressif Systems**: ESP32 WiFi module and comprehensive AT command firmware
- **Bosch Sensortec**: BMP280 environmental sensor and detailed documentation
- **ThingSpeak/MathWorks**: Robust IoT cloud platform and analytics tools
- **Open Source Community**: STM32 development libraries and example code

---

**Project Status**: ✅ Production Ready | 🚀 Fully Functional | 📊 Cloud Connected | 🔧 Simple API

**Compatible Hardware**: STM32F429ZI + ESP32-WROOM-32 + BMP280 | **Last Updated**: 2025

![image](https://github.com/user-attachments/assets/e4b958ed-9d54-49d5-82a2-e2c28b51b783)

![IMG_20250624_234223](https://github.com/user-attachments/assets/ef38859f-92da-432f-a689-3cd2520ade87)
