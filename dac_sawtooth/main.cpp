#include <stdint.h>
#include "../include/STM32G431xx.h"

void delay(uint32_t dly)
{
    while(dly--);
}
void initClock();

void initDACS()
{
    // DACs are on PINS 9 (PA4) and 10 (PA5) (LQFP32)
    // These are DAC1_OUT1 and DAC1_OUT2 resp.
    RCC->AHB2ENR |= (1 << 16); // enable DAC1
    RCC->AHB2ENR |= (1 << 0); // enable Port A    
    DAC1->DAC_CR |= (1 << 16) + (1 << 0); // enable both outputs;
    DAC1->DAC_DHR12R1 = 0;
    DAC1->DAC_DHR12R2 = 0;
    // Now configure PA4 and PA5 as analog ouputs
    GPIOA->MODER |= ((1 << 8) + (1 << 9) + (1 << 10) + (1 << 11));
}

int main()
{    
    
    uint16_t output = 0;
    initDACS();
    GPIOA->MODER &= ~(1 << 1); // Make bit 0 an output
    GPIOA->MODER |= (1 << 0);
    while(1)
    {
        
        DAC1->DAC_DHR12R1 = output;
        DAC1->DAC_DHR12R2 = output;
        output++;
        if (output > 4096) 
            output = 0;
        GPIOA->ODR |= 1;
        delay(10);
        GPIOA->ODR &= ~1;
        delay(10);
    }
}
