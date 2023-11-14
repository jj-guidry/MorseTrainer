
#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "rf.h"

void init_rf(){
	// spi2
	// pb12 -> NSS
	// pb13 -> SCK
	// pb14 -> MISO
	// pb15 -> MOSI

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set pb 12, 13, 14, 15 to AF
	GPIOB->MODER |= GPIOB_MODER_MODER_15_1 | GPIOB_MODER_MODER_14_1 | GPIOB_MODER_MODER_13_1 | GPIOB_MODER_MODER_12_1;
	GPIOB->MODER &= ~(GPIOB_MODER_MODER_15_0 | GPIOB_MODER_MODER_14_0 | GPIOB_MODER_MODER_13_0 | GPIOB_MODER_MODER_12_0);

	GPIOB->AFR[1] &= ~(0xFFFF0000); // all of 12, 13, 14, 15 to AF1

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	/*
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 &= ~SPI_CR1_SPE; // turn off channel before config
	while(SPI1->CR1 & SPI_CR1_SPE); // wait for it to be off
	SPI1->CR1 |= SPI_CR1_BR; // set baud rate to all 1's
	SPI1->CR2 |= SPI_CR2_DS_3; // setting the DS is really wierd, see documentation, will be 0xf after this line
	SPI1->CR2 &= ~(SPI_CR2_DS_2 | SPI_CR2_DS_1); // clear middle two bits for 1001 -> 10 bit word size
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR2 |= SPI_CR2_NSSP;
	SPI1->CR1 |= SPI_CR1_SPE;*/

	SPI2->CR1 &= ~SPI_CR1_SPE;
	SPI2->CR1 &= ~SPI_CR1_BR; // set to all 0's, lowest divisor, highest baud rate
	//SPI2->CR2->


}
