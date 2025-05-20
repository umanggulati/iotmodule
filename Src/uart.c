#include "uart.h"
#include "systick.h"
#include "system_config.h"
#include <stddef.h>

/* Global variable for SysTick counter */


/**
 * @file uart.c
 * @brief UART driver implementation for STM32F429ZI
 */

void UART_Init(uint32_t baud)
{
    /* Enable clocks and configure GPIO pins as before */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    GPIOD->MODER &= ~(GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
    GPIOD->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOD->AFR[1] &= ~(0xF << 0); /* Clear PD8 AF bits */
    GPIOD->AFR[1] &= ~(0xF << 4); /* Clear PD9 AF bits */
    GPIOD->AFR[1] |= (7 << 0); /* PD8 = AF7 (USART3_TX) */
    GPIOD->AFR[1] |= (7 << 4); /* PD9 = AF7 (USART3_RX) */

    /* Disable USART before configuration */
    USART3->CR1 &= ~USART_CR1_UE;

    /* Calculate BRR value using integer math only (more reliable than float) */
    uint32_t divider = SystemCoreClock_Get()/ baud;
    uint32_t mantissa = divider / 16;
    uint32_t fraction = divider % 16;

    /* Apply rounding if needed */
    /* The (divider * 16) % 16 calculation gives us the remaining fractional part */
    /* If it's ≥ 0.5, we round up */
    uint32_t remainder = (SystemCoreClock_Get()  % baud) * 16 / baud;
    if (remainder >= 8) {
        if (fraction < 15) {
            fraction++;
        } else {
            fraction = 0;
            mantissa++;
        }
    }

    /* Set BRR register value */
    USART3->BRR = (mantissa << 4) | fraction;

    /* Configure word length (8 bits) and no parity */
    USART3->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);

    /* Configure stop bits (1 stop bit) */
    USART3->CR2 &= ~USART_CR2_STOP;

    /* Enable transmitter and receiver */
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* Enable USART */
    USART3->CR1 |= USART_CR1_UE;
}
void UART_SendString(const char* str) {
    /* Send characters until null terminator is reached */
    while (*str) {
        /* Wait until transmit data register is empty */
        while (!(USART3->SR & USART_SR_TXE));

        /* Send the character */
        USART3->DR = *str++;
    }

    /* Wait until transmission is complete */
    while (!(USART3->SR & USART_SR_TC));
}

UART_Error UART_InitConfig(UART_Config* config) {
    /* Check if config pointer is valid */
    if (config == NULL) {
        return UART_ERROR_BUSY;
    }

    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* Configure GPIO pins */
    GPIOD->MODER &= ~(GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
    GPIOD->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOD->AFR[1] &= ~(0xF << 0); /* Clear PD8 AF bits */
    GPIOD->AFR[1] &= ~(0xF << 4); /* Clear PD9 AF bits */
    GPIOD->AFR[1] |= (7 << 0); /* PD8 = AF7 (USART3_TX) */
    GPIOD->AFR[1] |= (7 << 4); /* PD9 = AF7 (USART3_RX) */

    /* Disable USART before configuration */
    USART3->CR1 &= ~USART_CR1_UE;

    /* Configure oversampling */
    if (config->oversampling == UART_OVERSAMPLING_8) {
        USART3->CR1 |= USART_CR1_OVER8;
    } else {
        USART3->CR1 &= ~USART_CR1_OVER8;
    }

    /* Calculate BRR value */
    uint32_t divider = SystemCoreClock_Get() / config->baudRate;
    uint32_t mantissa, fraction;

    if (config->oversampling == UART_OVERSAMPLING_8) {
        mantissa = divider / 8;
        fraction = (divider % 8);
    } else {
        mantissa = divider / 16;
        fraction = (divider % 16);
    }

    USART3->BRR = (mantissa << 4) | fraction;

    /* Configure word length */
    if (config->wordLength == UART_WORDLENGTH_9B) {
        USART3->CR1 |= USART_CR1_M;
    } else {
        USART3->CR1 &= ~USART_CR1_M;
    }

    /* Configure parity */
    USART3->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
    if (config->parity == UART_PARITY_EVEN) {
        USART3->CR1 |= USART_CR1_PCE;
    } else if (config->parity == UART_PARITY_ODD) {
        USART3->CR1 |= (USART_CR1_PCE | USART_CR1_PS);
    }

    /* Configure stop bits */
    USART3->CR2 &= ~USART_CR2_STOP;
    USART3->CR2 |= (config->stopBits << 12);

    /* Configure hardware flow control */
    USART3->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
    if (config->hwFlowControl == UART_HWCONTROL_RTS) {
        USART3->CR3 |= USART_CR3_RTSE;
    } else if (config->hwFlowControl == UART_HWCONTROL_CTS) {
        USART3->CR3 |= USART_CR3_CTSE;
    } else if (config->hwFlowControl == UART_HWCONTROL_RTS_CTS) {
        USART3->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
    }

    /* Enable transmitter and/or receiver based on mode */
    USART3->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
    if (config->mode & UART_MODE_TX) {
        USART3->CR1 |= USART_CR1_TE;
    }
    if (config->mode & UART_MODE_RX) {
        USART3->CR1 |= USART_CR1_RE;
    }

    /* Enable USART */
    USART3->CR1 |= USART_CR1_UE;

    return UART_OK;
}

UART_Error UART_Transmit(const char* data, uint16_t size, uint32_t timeout) {
    /* Check if data pointer is valid */
    if (data == NULL || size == 0) {
        return UART_ERROR_BUSY;
    }

    /* Check if transmitter is enabled */
    if (!(USART3->CR1 & USART_CR1_TE)) {
        return UART_ERROR_BUSY;
    }

    /* Record start time for timeout */
    uint32_t startTime = systick_counter;  /* Assumes systick_counter is global */

    /* Transmit data byte by byte */
    for (uint16_t i = 0; i < size; i++) {
        /* Wait for TXE flag with timeout */
        while (!(USART3->SR & USART_SR_TXE)) {
            if ((systick_counter - startTime) > timeout) {
                return UART_ERROR_TIMEOUT;
            }
        }

        /* Send byte */
        USART3->DR = (uint8_t)(data[i] & 0xFF);
    }

    /* Wait for transmission complete with timeout */
    while (!(USART3->SR & USART_SR_TC)) {
        if ((systick_counter - startTime) > timeout) {
            return UART_ERROR_TIMEOUT;
        }
    }

    return UART_OK;
}
UART_Error UART_Receive(char* buffer, uint16_t size, uint32_t timeout) {
    /* Check if buffer pointer is valid */
    if (buffer == NULL || size == 0) {
        return UART_ERROR_BUSY;
    }

    /* Check if receiver is enabled */
    if (!(USART3->CR1 & USART_CR1_RE)) {
        return UART_ERROR_BUSY;
    }

    /* Record start time for timeout */
    uint32_t startTime = systick_counter;

    /* Receive data byte by byte */
    for (uint16_t i = 0; i < size; i++) {
        /* Wait for RXNE flag with timeout */
        while (!(USART3->SR & USART_SR_RXNE)) {
            if ((systick_counter - startTime) > timeout) {
                return UART_ERROR_TIMEOUT;
            }
        }

        /* Check for receive errors */
        if (USART3->SR & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
            /* Read DR to clear error flags */
            volatile uint32_t dummy = USART3->DR;
            (void)dummy;  /* Prevent compiler warning */

            /* Return appropriate error */
            if (USART3->SR & USART_SR_PE) return UART_ERROR_PARITY;
            if (USART3->SR & USART_SR_FE) return UART_ERROR_FRAMING;
            if (USART3->SR & USART_SR_NE) return UART_ERROR_NOISE;
            if (USART3->SR & USART_SR_ORE) return UART_ERROR_OVERRUN;
        }

        /* Read received byte */
        buffer[i] = (char)(USART3->DR & 0xFF);
    }

    return UART_OK;
}
bool UART_IsDataAvailable(void) {
    return (USART3->SR & USART_SR_RXNE) ? true : false;
}

UART_Error UART_SendByte(uint8_t byte) {
    /* Check if transmitter is enabled */
    if (!(USART3->CR1 & USART_CR1_TE)) {
        return UART_ERROR_BUSY;
    }

    /* Record start time for timeout */
    uint32_t startTime = systick_counter;
    uint32_t timeout = 1000;  /* 1 second timeout */

    /* Wait for TXE flag with timeout */
    while (!(USART3->SR & USART_SR_TXE)) {
        if ((systick_counter - startTime) > timeout) {
            return UART_ERROR_TIMEOUT;
        }
    }

    /* Send the byte */
    USART3->DR = byte;

    return UART_OK;
}

/* Returns 0 on timeout - check UART_IsDataAvailable() first for safety */
uint8_t UART_ReceiveByte(void) {
    uint32_t startTime = systick_counter;
    while (!(USART3->SR & USART_SR_RXNE)) {
        if ((systick_counter - startTime) > 1000) {
            return 0;  /* Timeout - could be valid data or error */
        }
    }
    return (uint8_t)(USART3->DR & 0xFF);
}

UART_Error UART_ClearErrors(void) {
    /* Check if any errors are present */
    if (USART3->SR & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
        /* Clear error flags by reading SR followed by DR */
        volatile uint32_t dummy = USART3->SR;
        dummy = USART3->DR;
        (void)dummy;  /* Prevent compiler warning */
    }

    return UART_OK;
}
void UART_EnableInterrupts(uint32_t interrupts) {
    /* Enable RXNE interrupt if requested */
    if (interrupts & USART_CR1_RXNEIE) {
        USART3->CR1 |= USART_CR1_RXNEIE;
    }

    /* Enable TXE interrupt if requested */
    if (interrupts & USART_CR1_TXEIE) {
        USART3->CR1 |= USART_CR1_TXEIE;
    }

    /* Enable TC interrupt if requested */
    if (interrupts & USART_CR1_TCIE) {
        USART3->CR1 |= USART_CR1_TCIE;
    }

    /* Enable error interrupts if requested */
    if (interrupts & USART_CR3_EIE) {
        USART3->CR3 |= USART_CR3_EIE;
    }

    /* Enable NVIC interrupt line for USART3 */
    NVIC_EnableIRQ(USART3_IRQn);
}
void UART_DisableInterrupts(uint32_t interrupts) {
    /* Disable RXNE interrupt if requested */
    if (interrupts & USART_CR1_RXNEIE) {
        USART3->CR1 &= ~USART_CR1_RXNEIE;
    }

    /* Disable TXE interrupt if requested */
    if (interrupts & USART_CR1_TXEIE) {
        USART3->CR1 &= ~USART_CR1_TXEIE;
    }

    /* Disable TC interrupt if requested */
    if (interrupts & USART_CR1_TCIE) {
        USART3->CR1 &= ~USART_CR1_TCIE;
    }

    /* Disable error interrupts if requested */
    if (interrupts & USART_CR3_EIE) {
        USART3->CR3 &= ~USART_CR3_EIE;
    }
}
UART_Error UART_UpdateBaudRate(uint32_t newBaudRate) {
    /* Check if baudrate is valid */
    if (newBaudRate == 0) {
        return UART_ERROR_BUSY;
    }

    /* Store current USART enable state */
    uint32_t ue_state = USART3->CR1 & USART_CR1_UE;

    /* Disable USART */
    USART3->CR1 &= ~USART_CR1_UE;

    /* Calculate new BRR value */
    uint32_t divider = SystemCoreClock_Get() / newBaudRate;
    uint32_t mantissa, fraction;

    /* Check if oversampling by 8 is enabled */
    if (USART3->CR1 & USART_CR1_OVER8) {
        mantissa = divider / 8;
        fraction = (divider % 8);
    } else {
        mantissa = divider / 16;
        fraction = (divider % 16);
    }

    /* Set new BRR register value */
    USART3->BRR = (mantissa << 4) | fraction;

    /* Restore USART enable state */
    if (ue_state) {
        USART3->CR1 |= USART_CR1_UE;
    }

    return UART_OK;
}

