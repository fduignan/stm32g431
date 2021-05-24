// spi flash device support.  Tested with an ST Micro M25P16
#ifndef __spi_flash_h
#define __spi_flash_h
#include "spi.h"
class serial_flash {
public:
	serial_flash() {};
	void begin(spi & SPI);
	void read(uint32_t address, uint8_t *buffer, uint32_t len);
	void program_page(uint32_t address, uint8_t *buffer, uint32_t len);
	void erase_sector(uint32_t address);
	void bulk_erase();
	void deep_power_down();
	void deep_power_release();	
	
//private:
	spi * SPI;
	void write_enable(void);
	void write_disable(void);
	uint32_t read_id(void);
	uint8_t read_status1(void);
	uint8_t read_status2(void);
	uint8_t read_status3(void);
	void write_status(uint8_t stat);
};
#endif
