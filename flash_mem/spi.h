// spi.h for stm32f411
#ifndef __spi_h
#define __spi_h
class spi {
public:
	spi(){}
	void begin();
	void startTransaction(void);
	void stopTransaction(void);
	uint8_t transfer(uint8_t data);
	uint16_t transfer(uint16_t data);
private:
};
#endif
