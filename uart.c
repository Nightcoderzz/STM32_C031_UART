/*
 * uart.c
 *
 *  Created on: May 23, 2025
 *      Author: Szymon Sikorski
   	   UART driver for STM32 C031C6T6
   */


#include <stdio.h>
#include <stdint.h>
#include "stm32c0xx.h"
#include "uart.h"

#define GPIOBEN        (1U << 1)
#define USART1EN       (1U << 14)

#define CR1_TE         (1U << 3) // TX ENABLE
#define CR1_RE		   (1U << 2) // RX ENABLE
#define CR1_UE         (1U << 0)

#define USART_ISR_TXE  (1U << 7)
#define USART_ISR_RXNE (1U << 5) // P.822

#define SYSCLK_DIV1    (0b111<<11)
#define USART_SYSCLK   (0x01<<0)
#define BAUD_RATE	    9600
#define SYS_CLK	        48000000



// STSTEM CLK
void SYSCLK_INIT(void) {
    RCC->CR &= ~(SYSCLK_DIV1); // System clk Divide by 1 HSIDIV RM p134
}
void USART1_RX_TX_INIT(void) {
    // Enable GPIOB and USART1 clocks
    RCC->IOPENR  |= GPIOBEN;
    RCC->APBENR2 |= USART1EN;
    RCC->CCIPR	 |= USART_SYSCLK; // USART clk source from SYSCLK RM0490 Rev 5 p159

    // Set PB6 (TX) to Alternate Function Mode (MODER6 = 0b10)
    GPIOB->MODER &= ~(3U << (2 * 6));       // Clear MODER6[1:0]
    GPIOB->MODER |=  (2U << (2 * 6));       // Set MODER6 = 0b10

    // Set PB7 (RX) to Alternate Function Mode (MODER7 = 0b10)
    GPIOB->MODER &= ~(3U << (2*7));             // Clear MODER6[1:0]
    GPIOB->MODER |=  (2U << (2*7));             // Set MODER7 = 0b10

    // Set AF0 for PB6 (AFRL6[3:0] = 0000)
    GPIOB->AFR[0] &= ~(0xF << (4 * 6));     // Clear AFRL6

    // Set AF0 for PB7 (AFRL6[3:0] = 0000)
    GPIOB->AFR[0] &= ~(0xF << 28);          // Clear AFRL7

    // Set baud rate (9600 at 48 MHz)
    USART1->BRR = usart_calc_brr(SYS_CLK, BAUD_RATE );

//---------------------------USART RX TX ENABLE---------------------------
    // USART enable
    USART1->CR1 |= (CR1_UE);

   // Enable RX TX
    USART1->CR1 |= (CR1_RE | CR1_TE);
}
//-----------------UART WRITE------------------------------
void UART_WRITE(int ch) {
    while (!(USART1->ISR & USART_ISR_TXE)){}
    USART1->TDR = ( ch & 0xFF);
}
//----------------UART RECIVE------------------------------
int UART_READ(void) {
    while (!(USART1->ISR & USART_ISR_RXNE)){} //while RXNE Read data register not empty
     return USART1->RDR;

}
//--------------BAUD RATE--------------------------------------
uint32_t usart_calc_brr(uint32_t clk_hz, uint32_t baud) {
    uint32_t usartdiv;


        usartdiv = (clk_hz + baud / 4) / baud;
        uint32_t mantissa = usartdiv / 16;
        uint32_t fraction = usartdiv % 16;
        return (mantissa << 4) | (fraction & 0x0F);
    }

//EXTRA
//----------------------PRINTF------------------------------
int __io_putchar(int ch) { // integral part o printf
	UART_WRITE(ch);
	return (ch);
}

