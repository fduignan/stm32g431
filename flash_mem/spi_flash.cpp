#include <stdint.h>
#include "spi_flash.h"
#include "spi.h"
// spi flash device support.  Tested with an ST Micro M25P16
void serial_flash::begin(spi & SPI)
{
	// Assuming that the SPI object has been initialized already
	this->SPI = &SPI;
}
void serial_flash::read(uint32_t address, uint8_t *buffer, uint32_t len)
{
	// address is a 24 bit value
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x03);	// read command
	SPI->transfer((uint8_t)((address>>16) & 0xff));	// MSB of address
	SPI->transfer((uint8_t)((address>>8) & 0xff));		
	SPI->transfer((uint8_t)((address) & 0xff));	// LSB of address
	for (uint32_t i=0; i < len; i++)
	{
		buffer[i] = SPI->transfer((uint8_t)0xff);
	}	
	SPI->stopTransaction();
}
void serial_flash::program_page(uint32_t address, uint8_t *buffer, uint32_t len)
{
	write_enable();
	// address is a 24 bit value
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x02);	// program page command
	SPI->transfer((uint8_t)((address>>16) & 0xff));	// MSB of address
	SPI->transfer((uint8_t)((address>>8) & 0xff));		
	SPI->transfer((uint8_t)((address) & 0xff));	// LSB of address
	for (uint32_t i=0; i < len; i++)
	{
		SPI->transfer((uint8_t)buffer[i]);
	}	
	SPI->stopTransaction();
	
}
void serial_flash::erase_sector(uint32_t address)
{	
	write_enable();
	SPI->startTransaction();
	SPI->transfer((uint8_t)0xd8);	
	SPI->transfer((uint8_t)((address>>16) & 0xff));	// MSB of address
	SPI->transfer((uint8_t)((address>>8) & 0xff));		
	SPI->transfer((uint8_t)((address) & 0xff));	// LSB of address	
	SPI->stopTransaction();
	// need to wait until erase is complete
	while (read_status1() & 1 ); // Wait for WIP to clear
}
void serial_flash::bulk_erase()
{
	write_enable();
	SPI->startTransaction();
	SPI->transfer((uint8_t)0xc7);	
	SPI->stopTransaction();
	while(read_status1() & 1); // wait until erase has completed
}
void serial_flash::deep_power_down()
{
	SPI->startTransaction();
	SPI->transfer((uint8_t)0xb9);	
	SPI->stopTransaction();
}
void serial_flash::deep_power_release()
{
	SPI->startTransaction();
	SPI->transfer((uint8_t)0xab);	
	SPI->stopTransaction();
}

void serial_flash::write_enable(void)
{
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x06);	
	SPI->stopTransaction();		
}	
void serial_flash::write_disable(void)
{
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x04);	
	SPI->stopTransaction();		
}
uint32_t serial_flash::read_id(void)
{
	// Have to be mindful of the FIFO in the STM32G431.  There 
	// doen't seem to be a way of flushing it short of doing a full
	// reset of the SPI subsystem
	uint16_t id_byte[4];
	uint32_t returnvalue=0;
	SPI->startTransaction();	
	SPI->transfer((uint8_t) 0x9f);
	id_byte[0]=SPI->transfer((uint8_t)0xff);		
	id_byte[1]=SPI->transfer((uint8_t)0xff);
	id_byte[2]=SPI->transfer((uint8_t)0xff);		
	returnvalue = id_byte[0];
	returnvalue = returnvalue << 8;
	returnvalue += id_byte[1];	
	SPI->stopTransaction();
	return returnvalue;
}
uint8_t serial_flash::read_status1(void)
{
	uint8_t rvalue;
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x05);	
	rvalue = SPI->transfer((uint8_t)0xff);
	SPI->stopTransaction();
	return rvalue;
}
uint8_t serial_flash::read_status2(void)
{
	uint8_t rvalue;
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x35);	
	rvalue = SPI->transfer((uint8_t)0xff);
	SPI->stopTransaction();
	return rvalue;
}
uint8_t serial_flash::read_status3(void)
{
	uint8_t rvalue;
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x15);	
	rvalue = SPI->transfer((uint8_t)0xff);
	SPI->stopTransaction();
	return rvalue;
}
void serial_flash::write_status(uint8_t stat)
{
	write_enable();
	// address is a 24 bit value
	SPI->startTransaction();
	SPI->transfer((uint8_t)0x01);	
	SPI->transfer(stat);
	SPI->stopTransaction();
}
