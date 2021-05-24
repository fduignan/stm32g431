// spi.h for stm32f411
// Author: Frank Duignan.  
// Updates posted on http://ioprog.com 

#include <stdint.h>
#include "../include/STM32G431xx.h"
#include "spi.h"
void spi::begin(void)
{
	int drain_count,drain;
	
	RCC->APB2ENR |= (1 << 12);		// turn on SPI1 	
	RCC->APB2RSTR |= (1 << 12);		// Reset the SPI interface
	RCC->APB2RSTR &= ~(1 << 12);		// Deassert Reset for the SPI interface
/*
 * PA4  SPI1 NSS : AF5
 * PA5  SPI1 SCK : AF5
 * PA7  SPI1 MOSI : AF5
 * PA6  SPI1 MISO : AF5
 */
	
	// GPIOA bits 5 and 7 are used for SPI1 (Alternative functions 0)	
	RCC->AHB2ENR |= (1 << 0); 	// enable port A
    GPIOA->MODER &= ~( (1 << 14) + (1 << 12) + (1 << 10)+(1<<8)); // select Alternative function
    GPIOA->MODER |= ((1 << 15) + (1 << 13) + (1 << 11)+(1 << 9));  // for bits 4,5,6,7 
	GPIOA->AFRL &= 0x0000ffff;
    GPIOA->AFRL |= 0x55550000; // select Alt. Function 5	
	// set port bits up as high speed outputs
	GPIOA->OSPEEDR |= (1 << 15) + (1 << 14) + (1 << 13) + (1 << 12) +(1 << 11) + (1 << 10) + (1 << 9) + (1 << 8);
	
	
	// Now configure the SPI interface
	drain = SPI1->SR;				// dummy read of SR to clear MODF	
	// enable SSM, set SSI, enable SPI, PCLK/2, MSB First Master, Clock = 1 when idle
	// Will switch to software slave management
	SPI1->CR1 =+ (1 << 5) + (1 << 4) + (1 << 2) + (1 << 1) + (1 << 0); // Master mode, about 5MHz.  CPHA=CPOL=1
	
	SPI1->CR2 = (1 << 12) + (1 << 10) + (1 << 9) + (1 << 8) + (1 << 2); 	// SS output enabled, 8 bit mode

	// If this is not done then there are all sorts of alignment problems with the SPI FIFOs.
    startTransaction();
    for (drain_count = 0; drain_count < 32; drain_count++)
		drain = transfer((uint8_t)0x00);
	stopTransaction();
}

uint8_t spi::transfer(uint8_t data)
{	

    *((uint8_t*)&SPI1->DR) = data;        		
	while (((SPI1->SR & (1 << 7))!=0));// Wait for Busy			
	return *((uint8_t*)&SPI1->DR);
}

uint16_t spi::transfer(uint16_t data)
{    
    SPI1->DR = data;        
	while (  (SPI1->SR & (1 << 7)) );
    return SPI1->DR;	
}
void spi::startTransaction(void)
{
	SPI1->CR1 |= (1 << 6); // Enable SPI (SPE = 1)

}
void spi::stopTransaction(void)
{	
	volatile unsigned Timeout = 1000;    
	while (SPI1->SR & ((1 << 12) + (1 << 11)) );     // wait for fifo to empty
	while (((SPI1->SR & (1 << 0))!=0)&&(Timeout--)); // Wait for RXNE
	Timeout = 1000;    
	while (((SPI1->SR & (1 << 1))==0)&&(Timeout--)); // Wait for TXE
	Timeout = 1000;    
	while (((SPI1->SR & (1 << 7))!=0)&&(Timeout--)); // Wait for Busy		
	SPI1->CR1 &= ~(1 << 6); // Disable SPI (SPE = 0)
		
	while((GPIOA->IDR & (1 << 4))==0); // wait for NSS to go high
	
}
