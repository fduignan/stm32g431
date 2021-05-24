#include <stdint.h>
#include "../include/STM32G431xx.h"

void delay(uint32_t dly)
{
    while(dly--);
}

void pinMode(GPIOB_Type *Port, uint32_t BitNumber, uint32_t Mode)
{
	// This function writes the given Mode bits to the appropriate location for
	// the given BitNumber in the Port specified.  It leaves other bits unchanged
	// Mode values:
	// 0 : digital input
	// 1 : digital output
	// 2 : Alternative function
	// 3 : Analog input
	uint32_t mode_value = Port->MODER; // read current value of Mode register 
	Mode = Mode << (2 * BitNumber);    // There are two Mode bits per port bit so need to shift
																	   // the mask for Mode up to the proper location
	mode_value = mode_value & ~(3u << (BitNumber * 2)); // Clear out old mode bits
	mode_value = mode_value | Mode; // set new bits
	Port->MODER = mode_value; // write back to port mode register
}
int main()
{    
	uint32_t Count = 0;
    RCC->AHB2ENR |= (1 << 1); // enable Port B    
	pinMode(GPIOB,0,1); // make bit 0 an output
	pinMode(GPIOB,1,1); // make bit 0 an output
	pinMode(GPIOB,2,1); // make bit 0 an output
	pinMode(GPIOB,3,1); // make bit 0 an output
	pinMode(GPIOB,4,1); // make bit 0 an output
	pinMode(GPIOB,5,1); // make bit 0 an output
	pinMode(GPIOB,6,1); // make bit 0 an output
	pinMode(GPIOB,7,1); // make bit 0 an output
	pinMode(GPIOB,8,1); // make bit 0 an output
	pinMode(GPIOB,9,1); // make bit 0 an output
    while(1)
    {
        GPIOB->ODR = Count++;
        delay(1000000);        
    }
}
