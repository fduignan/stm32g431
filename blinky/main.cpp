#include <stdint.h>
#include "../include/STM32G431xx.h"

void delay(uint32_t dly)
{
    while(dly--);
}
void initClock();

int main()
{    
    RCC->AHB2ENR |= (1 << 0); // enable Port A    
    GPIOA->MODER &= ~(1 << 1); // Make bit 0 an output
    GPIOA->MODER |= (1 << 0);
    while(1)
    {
        GPIOA->ODR |= 1;
        delay(1000000);
        GPIOA->ODR &= ~1;
        delay(1000000);
    }
}
