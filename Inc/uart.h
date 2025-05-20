/**

* @file uart.h

* @brief UART driver implementation for STM32F429ZI

*/

#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Error codes */
typedef enum {
    UART_OK = 0,           // <- We're here
	UART_ERROR_TIMEOUT,  // Added this line
	UART_ERROR_BUSY,
	UART_ERROR_OVERFLOW,
	UART_ERROR_PARITY,
	UART_ERROR_FRAMING,
	UART_ERROR_NOISE,
	UART_ERROR_OVERRUN

} UART_Error;

/* Configuration structure */
typedef struct {
	uint32_t baudRate;
	uint8_t wordLength;
	uint8_t stopBits;
	uint8_t parity;
	uint8_t mode;           // TX, RX, or TX_RX
	uint8_t oversampling;   // 16x or 8x oversampling
	uint8_t hwFlowControl;  // RTS/CTS control
}UART_Config;

/* Configuration constants */

#define UART_WORDLENGTH_8B  0
#define UART_WORDLENGTH_9B  1
#define UART_STOPBITS_1     0
#define UART_STOPBITS_0_5   1
#define UART_STOPBITS_2     2
#define UART_STOPBITS_1_5   3

#define UART_PARITY_NONE    0
#define UART_PARITY_EVEN    1
#define UART_PARITY_ODD     2

#define UART_MODE_TX        1
#define UART_MODE_RX        2
#define UART_MODE_TX_RX     3

#define UART_OVERSAMPLING_16  0
#define UART_OVERSAMPLING_8   1

#define UART_HWCONTROL_NONE     0
#define UART_HWCONTROL_RTS      1
#define UART_HWCONTROL_CTS      2
#define UART_HWCONTROL_RTS_CTS  3

/* Function prototypes */

UART_Error UART_InitConfig(UART_Config* config);
UART_Error UART_Transmit(const char* data, uint16_t size, uint32_t timeout);

UART_Error UART_Receive(char* buffer, uint16_t size, uint32_t timeout);
bool UART_IsDataAvailable(void);
UART_Error UART_SendByte(uint8_t byte);
uint8_t UART_ReceiveByte(void);
UART_Error UART_ClearErrors(void);
void UART_EnableInterrupts(uint32_t interrupts);
void UART_DisableInterrupts(uint32_t interrupts);
UART_Error UART_UpdateBaudRate(uint32_t newBaudRate);


void UART_Init(uint32_t baud);

/* Send a null-terminated string via UART */
void UART_SendString(const char* str);

#endif

