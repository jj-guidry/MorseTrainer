
#include <stdio.h>
#include "serial_port.h"
#include "main.h"


void uart_send_char(char ch){
	while (!(USART5->ISR & (1 << 7)));
	USART5->TDR = ch;
}

void uart_send_string(const char* str) {
    while (*str) {  // Continue until the null terminator is encountered
        uart_send_char(*str);
        str++;  // Move to the next character in the string
    }
}

void uart_send_int(int value) {
    char buffer[12];  // Buffer to hold the string representation of the integer
    snprintf(buffer, sizeof(buffer), "%d", value);  // Convert the integer to a string
    uart_send_string(buffer);  // Send the string over UART
}

void uart_send_hex(int value) {
    char buffer[12];  // Buffer to hold the string representation of the integer
    snprintf(buffer, sizeof(buffer), "%x", value);  // Convert the integer to a string
    uart_send_string(buffer);  // Send the string over UART
    uart_send_char(' ');
}

void init_serial_port(){

	// ------------------ serial port UART setup ----------------

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN; // enable GPIOD and GPIOC clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
	// set MODER's to alternate function
	GPIOC->MODER |= GPIO_MODER_MODER12_1;
	GPIOC->MODER &= ~(GPIO_MODER_MODER12_0);

	GPIOD->MODER |= GPIO_MODER_MODER2_1;
	GPIOD->MODER &= ~GPIO_MODER_MODER2_0;

	// configure pc12 for UART5_TX (AF2)
	GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL12;
	GPIOC->AFR[1] |= (2 << 16);

	// configure pd2 for UART5_RX (AF2)
	GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL2;
	GPIOD->AFR[0] |= (2 << 8);

	// first, turn off the UE bit
	USART5->CR1 &= ~USART_CR1_UE;

	// set word length to 8 bits
	USART5->CR1 &= ~USART_CR1_M0;
	USART5->CR1 &= ~USART_CR1_M1;

	// set for 1 stop bit
	USART5->CR2 &= ~(USART_CR2_STOP);

	// no parity
	USART5->CR1 &= ~(USART_CR1_PCE);

	// 16x over-sampling
	USART5->CR1 &= ~(USART_CR1_OVER8);

	// baud rate 115200
	USART5->BRR = 0x1a1;

	// enable receiver and transmitter
	USART5->CR1 |= (USART_CR1_TE | USART_CR1_RE);

	// enable UART
	USART5->CR1 |= USART_CR1_UE;

	// wait for things to work?
	while(!(USART5->ISR & USART_ISR_REACK) || !(USART5->ISR & USART_ISR_TEACK));

}
