/*
 * uart.h
 *
 *  Created on: May 23, 2025
 *      Author: Szymon Sikorski
 *      UART header for STM32C031C6T6
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

// Initialise system clock (HSIDIV = /1)
void SYSCLK_INIT(void);

// Initialise USART1 for TX/RX on PB6/PB7
void USART1_RX_TX_INIT(void);

// Write a character over UART
void UART_WRITE(int ch);

// Read a character from UART
int UART_READ(void);

// Calculate USART BRR value from system clock and desired baud rate
uint32_t usart_calc_brr(uint32_t clk_hz, uint32_t baud);

#endif /* UART_H_ */
