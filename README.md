# STM32F429ZI IoT Module - Baremetal Framework

A professional-grade, baremetal IoT framework for STM32F429ZI microcontroller, built from scratch without using STM32 HAL libraries.

## 🎯 Project Overview

This project demonstrates how to build a complete IoT system using direct register manipulation, featuring:
- Custom task scheduler with priority-based execution
- Advanced power management with multiple sleep modes
- Modular sensor integration framework
- UART communication with command interface
- Professional error handling and debugging

## 📁 Project Structure

```
stm32f429-iot-module/
├── Inc/                    # Header files (interfaces)
│   ├── power_management.h  # Power modes and wake-up control
│   ├── task_scheduler.h    # Cooperative multitasking system
│   ├── systick.h          # 1ms timing services
│   ├── uart.h             # Serial communication
│   └── system_config.h    # Clock configuration
├── Src/                   # Implementation files
│   ├── main.c            # Application entry and demo
│   ├── power_management.c # Low-power implementation
│   ├── task_scheduler.c  # Task execution engine
│   ├── systick.c        # Timer implementation
│   └── uart.c           # UART driver
├── Startup/              # Boot sequence
│   └── startup_stm32f429zitx.s  # Assembly startup code
├── STM32F429ZITX_FLASH.ld       # Flash memory linker script
└── STM32F429ZITX_RAM.ld        # RAM linker script
```

## 🚀 Features

### Task Scheduler
- Priority-based cooperative multitasking
- Dynamic task registration and management
- Performance monitoring (execution time, jitter)
- Task suspension/resumption
- System-wide task control

### Power Management
- Multiple power modes: Run, Sleep, Stop, Standby
- Wake-up source configuration (RTC, GPIO)
- Voltage scaling for optimization
- Power consumption estimation
- Automatic power-saving tasks

### Communication
- Robust UART implementation
- Command-line interface
- Error handling with timeout
- Debug output system

## 🛠️ Hardware Requirements

- **Board**: STM32F429ZI Discovery or Nucleo-144
- **Debugger**: ST-Link V2/V3
- **Optional**: USB-to-Serial adapter for UART communication

## 💻 Software Requirements

- **IDE**: STM32CubeIDE (recommended) or any ARM GCC toolchain
- **Compiler**: arm-none-eabi-gcc
- **Debugger**: OpenOCD or STM32CubeProgrammer
- **Terminal**: PuTTY, Tera Term, or similar (115200 baud, 8N1)

## 📦 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/umanggulati/iotmodule.git
cd iotmodule
```

### 2. Import to STM32CubeIDE
1. Open STM32CubeIDE
2. File → Import → Existing Projects into Workspace
3. Select the cloned directory
4. Build project (Ctrl+B)

### 3. Flash and Run
1. Connect your STM32F429ZI board via ST-Link
2. Click "Debug" or "Run" in STM32CubeIDE
3. Open serial terminal at 115200 baud

## 📖 Code Examples

### Registering a Task
```c
void Task_LED_Blink(void) {
    static bool led_state = false;
    GPIOB->BSRR = led_state ? (1 << 0) : (1 << 16);
    led_state = !led_state;
}

// In main():
uint8_t taskId;
TaskScheduler_RegisterTask(Task_LED_Blink, 500, TASK_PRIORITY_HIGH, "LED", &taskId);
TaskScheduler_StartTask(taskId);
```

### Entering Low-Power Mode
```c
// Configure wake-up after 5 seconds
PowerMgmt_ConfigureWakeupSources(WAKEUP_SOURCE_RTC_WAKEUP);
PowerMgmt_SetWakeupTimer(5);

// Enter Stop mode
PowerMgmt_EnterLowPowerMode(POWER_MODE_STOP_LP);
```

## 🎮 Command Interface

When running, the system provides these commands via UART:
- `s` - Show scheduler status
- `r` - Reset performance statistics
- `p` - Test task suspension/resumption
- `1-5` - Test various power modes
- `q` - Exit demo

## 📚 Documentation

For detailed explanations of the code and architecture, check out the blog series:
1. [Part 1: Project Architecture](https://yourblog.com/stm32-part-1)
2. [Part 2: Boot Process and Startup](https://yourblog.com/stm32-part-2)
3. [Part 3: SysTick Timer Deep Dive](https://yourblog.com/stm32-part-3)
4. More coming soon...

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

## 🔗 Resources

- [STM32F429 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [ARM Cortex-M4 Programming Manual](https://developer.arm.com/documentation/ddi0439/b/)
- [Blog Series](https://yourblog.com/category/stm32-iot-development)

## 📧 Contact

- **Author**: Umang Gulati
- **Blog**: [Your Blog URL]
- **Email**: [Your Email]

---
⭐ If you find this project helpful, please consider giving it a star!

