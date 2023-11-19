
#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "rf.h"

// RF COMMAND: 001AAAAA 01010101
// -------------w reg-----data----

uint8_t* SPI_DR_8BIT = (uint8_t*) &(SPI2->DR); // used for sending only 8 bits to SPI (otherwise generates more than 8 pulses)

void rf_setup_commands(){
	// power up
	// init in tx PRIM_RX stays low
	// air data rate
	// channel frequency

	GPIOB->BRR = 1<<12; // drive NSS low
	rf_send_command(0x00); // read at 0x00
	rf_send_command(0xFF); // nop, just to generate clock pulses so that slave can give data at 0x00

	//while(!(SPI2->SR & SPI_SR_TXE));
	GPIOB->BSRR = 1<<12; // drive high after transfers
}

void init_rf(){
	// spi2
	// pb12 -> NSS
	// pb13 -> SCK
	// pb14 -> MISO
	// pb15 -> MOSI

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set pb 13, 14, 15 to AF
	GPIOB->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER13_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER13_0);

	GPIOB->MODER |= GPIO_MODER_MODER12_0; // manually toggle pb12 for NSS, cause SPI NSS hardware is being too freaky

	GPIOB->BSRR = 1<<12; // drive NSS high preliminarily

	GPIOB->AFR[1] &= ~(0xFFF00000); // all of 13, 14, 15 to AF1

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPI2->CR1 &= ~SPI_CR1_SPE;
	while(SPI2->CR1 & SPI_CR1_SPE); // wait for it to be off
	SPI2->CR1 |= SPI_CR1_BR; // set to all 1's, highest divisor, lowest baud rate (48MHz/256 = 187.5KHz)
	SPI2->CR1 |= SPI_CR1_MSTR; // stm to master mode
	SPI2->CR2 |= SPI_CR2_SSOE;
	SPI2->CR2 |= SPI_CR2_NSSP;
	SPI2->CR1 |= SPI_CR1_SPE; // enable channel
	HAL_Delay(1);

}

void rf_send_command(uint16_t cmd){
	while(!(SPI2->SR & SPI_SR_TXE)); // wait for buffer to be empty
	*SPI_DR_8BIT = cmd;
}
