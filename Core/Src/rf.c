
#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "rf.h"

// RF COMMAND: 001AAAAA 01010101
// -------------w reg-----data----

void rf_setup_commands(){
	// power up
	// init in tx PRIM_RX stays low
	// air data rate
	// channel frequency
	rf_send_command(0x0b20); // write to config register
}

void init_rf(){
	// spi2
	// pb12 -> NSS
	// pb13 -> SCK
	// pb14 -> MISO
	// pb15 -> MOSI

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set pb 12, 13, 14, 15 to AF
	GPIOB->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER12_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER12_0);

	GPIOB->AFR[1] &= ~(0xFFFF0000); // all of 12, 13, 14, 15 to AF1

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPI2->CR1 &= ~SPI_CR1_SPE;
	while(SPI2->CR1 & SPI_CR1_SPE); // wait for it to be off
	SPI2->CR1 |= SPI_CR1_BR; // set to all 1's, highest divisor, lowest baud rate (48MHz/256 = 187.5KHz)
	SPI2->CR1 |= SPI_CR1_MSTR; // stm to master mode
	SPI2->CR2 |= SPI_CR2_SSOE;
	SPI2->CR2 |= SPI_CR2_NSSP;
	SPI2->CR1 |= SPI_CR1_SPE; // enable channel

}

void rf_send_command(uint16_t cmd){
	while(!(SPI2->SR & SPI_SR_TXE));
	SPI2->DR = cmd;
}
