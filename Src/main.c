/* main.c - Complete Task Scheduler Demo with Power Management for STM32F429ZI */

#include "stm32f4xx.h"
#include "uart.h"
#include "systick.h"
#include "task_scheduler.h"
#include "power_management.h"  // Add the new header
#include <stdio.h>

/* ====================== Demo Tasks ====================== */

/* LED1 Task - Blink Green LED (PB0) every 500ms (High Priority) */
void Task_LED1_Blink(void) {
    static bool led_state = false;

    /* Configure PB0 (LD1 - Green LED) if not already configured */
    static bool initialized = false;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable GPIOB clock
        GPIOB->MODER |= (1 << 0);             // Output mode for PB0 (bits 1:0 = 01)
        initialized = true;
    }

    /* Toggle LED */
    if (led_state) {
        GPIOB->BSRR = (1 << 0);     // Set PB0 high (LED on)
    } else {
        GPIOB->BSRR = (1 << 16);    // Set PB0 low (LED off)
    }

    led_state = !led_state;

    /* Debug output */
    UART_SendString("[TASK] LED1 (Green) toggled\r\n");
}

/* LED2 Task - Blink Blue LED (PB7) every 1000ms (Normal Priority) */
void Task_LED2_Blink(void) {
    static bool led_state = false;

    /* Configure PB7 (LD2 - Blue LED) if not already configured */
    static bool initialized = false;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable GPIOB clock
        GPIOB->MODER |= (1 << 14);            // Output mode for PB7 (bits 15:14 = 01)
        initialized = true;
    }

    /* Toggle LED */
    if (led_state) {
        GPIOB->BSRR = (1 << 7);     // Set PB7 high (LED on)
    } else {
        GPIOB->BSRR = (1 << 23);    // Set PB7 low (LED off)
    }

    led_state = !led_state;

    /* Debug output */
    UART_SendString("[TASK] LED2 (Blue) toggled\r\n");
}

/* LED3 Task - Blink Red LED (PB14) every 2000ms (Normal Priority) */
void Task_LED3_Blink(void) {
    static bool led_state = false;

    /* Configure PB14 (LD3 - Red LED) if not already configured */
    static bool initialized = false;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable GPIOB clock
        GPIOB->MODER |= (1 << 28);            // Output mode for PB14 (bits 29:28 = 01)
        initialized = true;
    }

    /* Toggle LED */
    if (led_state) {
        GPIOB->BSRR = (1 << 14);    // Set PB14 high (LED on)
    } else {
        GPIOB->BSRR = (1 << 30);    // Set PB14 low (LED off)
    }

    led_state = !led_state;

    /* Debug output */
    UART_SendString("[TASK] LED3 (Red) toggled\r\n");
}

/* UART Status Task - Print status every 3000ms (Low Priority) */
void Task_UARTStatus(void) {
    char message[100];

    snprintf(message, sizeof(message),
             "[TASK] Status Update - Time: %lums\r\n",
             systick_counter);
    UART_SendString(message);
}

/* System Monitor Task - Print system info every 5000ms (Idle Priority) */
void Task_SystemMonitor(void) {
    char message[150];

    snprintf(message, sizeof(message),
             "[TASK] System Monitor - Tasks: %d, SysTick: %lums\r\n",
             TaskScheduler_GetTaskCount(), systick_counter);
    UART_SendString(message);

    /* Print scheduler status */
    TaskScheduler_PrintStatus();

    /* Print power status */
    PowerMode_t currentMode = PowerMgmt_GetCurrentMode();
    uint32_t powerEstimate = PowerMgmt_GetPowerConsumptionEstimate();

    snprintf(message, sizeof(message),
             "[POWER] Current mode: %d, Est. consumption: %lu uA\r\n",
             (int)currentMode, powerEstimate);
    UART_SendString(message);
}

/* New Task - Periodic power saving task (Idle Priority) */
/* New Task - Periodic power saving task (Idle Priority) */
void Task_PowerSaving(void) {
    static uint32_t counter = 0;
    char message[100];

    counter++;

    /* Every 10 executions (50 seconds with a 5s period), enter Stop mode briefly to demonstrate power saving */
    if (counter % 10 == 0) {
        snprintf(message, sizeof(message),
                 "[POWER] Automatic power saving - Entering Stop mode for 2s\r\n");
        UART_SendString(message);

        /* Let UART finish sending */
        SysTick_Delay(10);

        /* Configure wake-up source - RTC wake-up timer */
        PowerMgmt_ConfigureWakeupSources(WAKEUP_SOURCE_RTC_WAKEUP);

        /* Set wake-up timer to 2 seconds */
        PowerMgmt_SetWakeupTimer(2);

        /* Enter Stop mode */
        PowerMgmt_EnterLowPowerMode(POWER_MODE_STOP_LP);

        /* Report wake-up source after returning */
        WakeupSource_t source = PowerMgmt_GetLastWakeupSource();
        snprintf(message, sizeof(message),
                 "[POWER] Woke up from Stop mode, source: 0x%02X\r\n",
                 (unsigned int)source);
        UART_SendString(message);
    }
}
/* ====================== Main Demo Function ====================== */

int main(void)
{

	/* Verify interrupt vector table */
	{
	    /* Get the RTC_WKUP_IRQHandler address from the vector table */
	    uint32_t *vector_table = (uint32_t *)SCB->VTOR;
	    uint32_t rtc_wkup_handler_addr = vector_table[RTC_WKUP_IRQn + 16]; // 16 system exceptions before IRQs

	    /* We'll print this in hex to see if it's a valid address */
	    char debug_msg[100];
	    snprintf(debug_msg, sizeof(debug_msg),
	             "\r\n[STARTUP] RTC_WKUP_IRQHandler address: 0x%08lX\r\n",
	             rtc_wkup_handler_addr);
	    UART_Init(115200); // Early UART init for debugging
	    UART_SendString(debug_msg);

	    /* Check if the vector appears valid (non-zero and within code memory region) */
	    if (rtc_wkup_handler_addr == 0 || rtc_wkup_handler_addr < 0x08000000 || rtc_wkup_handler_addr > 0x08100000) {
	        UART_SendString("[STARTUP] ERROR: Invalid RTC_WKUP_IRQHandler in vector table!\r\n");
	        /* Consider adding some emergency reset or recovery code here */
	    } else {
	        UART_SendString("[STARTUP] RTC_WKUP_IRQHandler address looks valid\r\n");
	    }
	}
    /* Initialize SysTick and UART */
    SysTick_Init();
    UART_Init(115200);

    UART_SendString("\r\n=== Task Scheduler and Power Management Demo ===\r\n");
    UART_SendString("Using correct LED pins:\r\n");
    UART_SendString("  LD1 (Green) - PB0\r\n");
    UART_SendString("  LD2 (Blue)  - PB7\r\n");
    UART_SendString("  LD3 (Red)   - PB14\r\n\r\n");

    /* Initialize the power management subsystem */
    PowerMgmt_Init();

    /* Configure power management */
    PowerConfig_t powerConfig;
    powerConfig.mode = POWER_MODE_RUN;
    powerConfig.voltageScale = VOLTAGE_SCALE_1;
    powerConfig.flashPowerDown = true;
    powerConfig.enableWakeupPin = true;
    powerConfig.enableBackupRegulator = false;
    powerConfig.enablePVD = false;
    powerConfig.pvdLevel = PVD_LEVEL_2V7;
    powerConfig.wakeupSources = WAKEUP_SOURCE_RTC_WAKEUP | WAKEUP_SOURCE_PIN;

    PowerMgmt_Configure(&powerConfig);

    /* Initialize the task scheduler */
    TaskScheduler_Init();

    /* Register demo tasks with different priorities and periods */
    uint8_t taskId;

    /* High priority LED task - Green LED */
    if (TaskScheduler_RegisterTask(Task_LED1_Blink, 500, TASK_PRIORITY_HIGH, "LED1_Green", &taskId) == TASK_OK) {
        TaskScheduler_StartTask(taskId);
    }

    /* Normal priority LED task - Blue LED */
    if (TaskScheduler_RegisterTask(Task_LED2_Blink, 1000, TASK_PRIORITY_NORMAL, "LED2_Blue", &taskId) == TASK_OK) {
        TaskScheduler_StartTask(taskId);
    }

    /* Normal priority LED task - Red LED */
    if (TaskScheduler_RegisterTask(Task_LED3_Blink, 2000, TASK_PRIORITY_NORMAL, "LED3_Red", &taskId) == TASK_OK) {
        TaskScheduler_StartTask(taskId);
    }

    /* Low priority UART status task */
    if (TaskScheduler_RegisterTask(Task_UARTStatus, 3000, TASK_PRIORITY_LOW, "UART_Status", &taskId) == TASK_OK) {
        TaskScheduler_StartTask(taskId);
    }

    /* Idle priority system monitor task */
    if (TaskScheduler_RegisterTask(Task_SystemMonitor, 5000, TASK_PRIORITY_IDLE, "System_Monitor", &taskId) == TASK_OK) {
        TaskScheduler_StartTask(taskId);
    }

    /* Idle priority power saving task */
    if (TaskScheduler_RegisterTask(Task_PowerSaving, 5000, TASK_PRIORITY_IDLE, "Power_Saving", &taskId) == TASK_OK) {
        TaskScheduler_StartTask(taskId);
    }

    UART_SendString("\r\nAll tasks registered and started!\r\n");

    /* Simple command interface */
    UART_SendString("\r\nTask Scheduler and Power Management Commands:\r\n");
    UART_SendString("  's' - Show scheduler status\r\n");
    UART_SendString("  'r' - Reset performance statistics\r\n");
    UART_SendString("  'p' - Test suspend/resume LED1\r\n");
    UART_SendString("  't' - Change LED2 period to 2000ms\r\n");
    UART_SendString("  'a' - Suspend/resume all tasks\r\n");
    UART_SendString("  'i' - Change LED1 priority to LOW\r\n");
    UART_SendString("  'j' - Get LED1 execution time\r\n");

    /* New power management commands */
    UART_SendString("  '1' - Enter Sleep mode (wake with any interrupt)\r\n");
    UART_SendString("  '2' - Enter Stop mode (wake after 5s)\r\n");
    UART_SendString("  '3' - Enter Stop mode with low-power regulator\r\n");
    UART_SendString("  '4' - Toggle voltage scaling (Scale 1 <-> Scale 3)\r\n");
    UART_SendString("  '5' - Show power status\r\n");

    UART_SendString("  'q' - Exit demo\r\n");
    UART_SendString("Enter command: ");

    /* Main loop with task scheduler */
    while(1) {
        /* Run the task dispatcher */
        TaskScheduler_RunDispatcher();

        /* Handle user input */
        if (UART_IsDataAvailable()) {
            uint8_t cmd = UART_ReceiveByte();

            switch (cmd) {
                case 's':
                case 'S':
                    TaskScheduler_PrintStatus();
                    break;

                case 'r':
                case 'R':
                    // Reset performance stats for all tasks
                    UART_SendString("\r\nResetting performance statistics for all tasks...\r\n");
                    for (uint8_t i = 0; i < 6; i++) {  // Now we have 6 tasks
                        TaskScheduler_ResetStatistics(i);
                    }
                    UART_SendString("Performance statistics reset complete!\r\n");
                    break;

                case 'p':
                case 'P':
                    // Test suspend/resume functionality
                    UART_SendString("\r\nDemo: Suspending LED1 (Green) for 3 seconds...\r\n");
                    TaskScheduler_SuspendTask(0);  // Suspend LED1 (Green)
                    SysTick_Delay(3000);
                    TaskScheduler_ResumeTask(0);   // Resume LED1
                    UART_SendString("LED1 (Green) resumed - watch it blink!\r\n");
                    break;

                case 't':
                case 'T':
                    // Test period change
                    UART_SendString("\r\nChanging LED2 (Blue) period to 2000ms...\r\n");
                    TaskScheduler_SetPeriod(1, 2000);
                    UART_SendString("LED2 will now blink every 2 seconds\r\n");
                    break;

                case 'a':
                case 'A':
                    // Suspend all tasks
                    UART_SendString("\r\nDemo: Suspending ALL tasks for 4 seconds...\r\n");
                    UART_SendString("Watch - all LEDs should stop blinking!\r\n");
                    TaskScheduler_SuspendAll();
                    SysTick_Delay(4000);
                    TaskScheduler_ResumeAll();
                    UART_SendString("All tasks resumed - LEDs should resume blinking!\r\n");
                    break;

                case 'i':
                case 'I':
                    // Change LED1 priority to LOW
                    UART_SendString("\r\nChanging LED1 (Green) priority from HIGH to LOW...\r\n");
                    TaskScheduler_SetPriority(0, TASK_PRIORITY_LOW);
                    UART_SendString("LED1 now has low priority - it may be less responsive\r\n");
                    break;

                case 'j':
                case 'J':
                    // Get LED1 execution time
                    {
                        uint32_t execTime = TaskScheduler_GetTaskExecutionTime(0);
                        uint32_t jitter = TaskScheduler_GetMaxJitter(0);

                        char perfMsg[200];
                        snprintf(perfMsg, sizeof(perfMsg),
                                "\r\nLED1 (Green) Performance:\r\n"
                                "  Max Execution Time: %lums\r\n"
                                "  Max Jitter: %lums\r\n",
                                execTime, jitter);
                        UART_SendString(perfMsg);
                    }
                    break;

                /* New power management commands */
                case '1':
                    // Enter Sleep mode
                    UART_SendString("\r\nEntering Sleep mode - press any key to wake up...\r\n");
                    SysTick_Delay(10);  // Allow UART to finish sending
                    PowerMgmt_EnterLowPowerMode(POWER_MODE_SLEEP);
                    UART_SendString("Woke up from Sleep mode!\r\n");
                    break;

                case '2':
                    // Enter Stop mode with main regulator
                    UART_SendString("\r\nEntering Stop mode (Main Regulator) - will wake up after 5s...\r\n");
                    SysTick_Delay(10);  // Allow UART to finish sending

                    // Configure RTC wake-up timer for 5s
                    PowerMgmt_ConfigureWakeupSources(WAKEUP_SOURCE_RTC_WAKEUP);
                    PowerMgmt_SetWakeupTimer(5);

                    // Enter Stop mode
                    PowerMgmt_EnterLowPowerMode(POWER_MODE_STOP_MR);

                    UART_SendString("Woke up from Stop mode!\r\n");
                    break;

                case '3':
                    // Enter Stop mode with low-power regulator and flash powerdown
                    UART_SendString("\r\nEntering Stop mode (Low-Power Regulator) - will wake up after 5s...\r\n");
                    SysTick_Delay(10);  // Allow UART to finish sending

                    // Configure RTC wake-up timer for 5s
                    PowerMgmt_ConfigureWakeupSources(WAKEUP_SOURCE_RTC_WAKEUP);
                    PowerMgmt_SetWakeupTimer(5);

                    // Enter Stop mode
                    PowerMgmt_EnterLowPowerMode(POWER_MODE_STOP_LP_FPD);

                    UART_SendString("Woke up from Stop mode (Low-Power)!\r\n");
                    break;

                case '4':
                    // Toggle voltage scaling
                    {
                        static bool useScale1 = true;

                        if (useScale1) {
                            UART_SendString("\r\nSwitching to Voltage Scale 3 (low power)...\r\n");
                            PowerMgmt_SetVoltageScale(VOLTAGE_SCALE_3);
                        } else {
                            UART_SendString("\r\nSwitching to Voltage Scale 1 (high performance)...\r\n");
                            PowerMgmt_SetVoltageScale(VOLTAGE_SCALE_1);
                        }

                        useScale1 = !useScale1;
                        UART_SendString("Voltage scaling updated.\r\n");
                    }
                    break;

                case '5':
                    // Show power status
                    {
                        PowerMode_t mode = PowerMgmt_GetCurrentMode();
                        uint32_t powerConsumption = PowerMgmt_GetPowerConsumptionEstimate();

                        char powerMsg[200];
                        snprintf(powerMsg, sizeof(powerMsg),
                                "\r\nPower Management Status:\r\n"
                                "  Current Mode: %d\r\n"
                                "  Estimated Power Consumption: %lu uA\r\n"
                                "  Last Wake-up Source: 0x%02X\r\n",
                                (int)mode, powerConsumption, (unsigned int)PowerMgmt_GetLastWakeupSource());
                        UART_SendString(powerMsg);
                    }
                    break;

                case 'q':
                case 'Q':
                    UART_SendString("\r\nExiting demo...\r\n");
                    goto exit_demo;

                case 'd':
                case 'D':
                    // Debug RTC wake-up configuration
                    UART_SendString("\r\nDebug: Reinitializing RTC wake-up with extra debug...\r\n");

                    // Manually force-configure EXTI and NVIC for RTC wake-up
                    EXTI->IMR |= (1 << 22);   // Unmask interrupt on line 22
                    EXTI->RTSR |= (1 << 22);  // Rising edge trigger
                    EXTI->FTSR &= ~(1 << 22); // Disable falling edge
                    EXTI->PR = (1 << 22);     // Clear any pending interrupt

                    // Enable RTC wake-up interrupt in NVIC
                    NVIC_SetPriority(RTC_WKUP_IRQn, 0);
                    NVIC_EnableIRQ(RTC_WKUP_IRQn);

                    // Configure wake-up timer for 5 seconds
                    PowerMgmt_ConfigureWakeupSources(WAKEUP_SOURCE_RTC_WAKEUP);
                    PowerMgmt_SetWakeupTimer(5);

                    // Display register values for debugging
                    char dbgMsg[150];
                    snprintf(dbgMsg, sizeof(dbgMsg),
                            "RTC_CR=0x%08lX\r\nRTC_ISR=0x%08lX\r\nEXTI_IMR=0x%08lX\r\nEXTI_RTSR=0x%08lX\r\nEXTI_PR=0x%08lX\r\n",
                            RTC->CR, RTC->ISR, EXTI->IMR, EXTI->RTSR, EXTI->PR);
                    UART_SendString(dbgMsg);

                    // Now enter Stop mode
                    UART_SendString("Entering Stop mode with debug - should wake up after 5s...\r\n");
                    SysTick_Delay(10);
                    PowerMgmt_EnterLowPowerMode(POWER_MODE_STOP_MR);

                    /* Special bypass test - Generate a software RTC wake-up */
                    {
                        UART_SendString("\r\nTesting direct RTC wake-up trigger\r\n");

                        /* Clear any pending EXTI line 22 interrupt */
                        EXTI->PR = (1 << 22);

                        /* Set up RTC wake-up flags */
                        /* Disable RTC write protection */
                        RTC->WPR = 0xCA;
                        RTC->WPR = 0x53;

                        /* Set wake-up flag explicitly */
                        RTC->ISR |= RTC_ISR_WUTF;

                        /* Re-enable RTC write protection */
                        RTC->WPR = 0xFF;

                        /* Generate interrupt on EXTI line 22 */
                        EXTI->SWIER |= (1 << 22);

                        /* Give time for interrupt to process */
                        SysTick_Delay(10);

                        UART_SendString("Direct RTC wake-up trigger test completed\r\n");
                    }

                    // After wake-up, display registers again
                    snprintf(dbgMsg, sizeof(dbgMsg),
                            "Wake-up Debug:\r\nRTC_ISR=0x%08lX\r\nEXTI_PR=0x%08lX\r\n",
                            RTC->ISR, EXTI->PR);
                    UART_SendString(dbgMsg);

                    UART_SendString("Debug wake-up test complete\r\n");
                    break;

                default:
                    UART_SendString("Unknown command. Try: s, r, p, t, a, i, j, 1-5, or q\r\n");
                    break;
            }

            UART_SendString("Enter command: ");
        }

        /* Small delay to prevent CPU overload */
        SysTick_Delay(1);
    }

exit_demo:
    /* Stop all tasks for clean exit */
    for (uint8_t i = 0; i < 6; i++) {  // We now have 6 tasks
        TaskScheduler_StopTask(i);
    }

    UART_SendString("\r\n=== Task Scheduler and Power Management Demo Complete ===\r\n");
    UART_SendString("Demonstration finished. All tasks stopped.\r\n");
    UART_SendString("\r\nFeatures demonstrated:\r\n");
    UART_SendString("  ✓ Multi-task scheduling with priorities\r\n");
    UART_SendString("  ✓ Task suspension and resumption\r\n");
    UART_SendString("  ✓ Dynamic period adjustment\r\n");
    UART_SendString("  ✓ Priority changes\r\n");
    UART_SendString("  ✓ Performance monitoring\r\n");
    UART_SendString("  ✓ System-wide task control\r\n");
    UART_SendString("  ✓ Power management with multiple low-power modes\r\n");
    UART_SendString("  ✓ Voltage scaling for power optimization\r\n");
    UART_SendString("  ✓ Wake-up source configuration and detection\r\n");
    UART_SendString("\r\nReady for next phase development!\r\n");

    while(1) {
        SysTick_Delay(100);
    }

    return 0;
}
