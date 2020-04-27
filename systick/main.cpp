/* Toggle an LED on GPIO0 in response to a systick interrupt.
*/
#include <stdint.h>
#include "../include/STM32G431xx.h"

// These constants are used together to define the interrupt (sampling) rate
// Some values produce more accurate results than others. The interrupt period is CPU_CLOCK_SPEED / FS
// If this works out to be an integer then the achieved sample rate will be closer to the expected value (no truncation)

#define CPU_CLOCK_SPEED 170000000
#define FS 50000
void delay(uint32_t dly)
{
    while(dly--);
}
void initPorts()
{
    RCC->AHB2ENR |= (1 << 0); // enable Port A    
    GPIOA->MODER &= ~(1 << 1); // Make bit 0 an output
    GPIOA->MODER |= (1 << 0);
}

void Systick_Handler()
{ 
    static uint32_t milliseconds = 0;
    milliseconds++;
    if (milliseconds == 1000)
    {
        milliseconds = 0;
        
    }
    GPIOA->ODR ^= 1; // toggle a port bit so sampling rate can be measured with an oscilloscope
    
}
void initSystick()
{
    
    STK->CTRL |= ( 7 ); // enable systick, source = cpu clock, enable interrupt
// SysTick clock source = 170MHz.  Divide this down to create 1 millisecond period
    STK->LOAD = 170000-1;   
    STK->VAL = 10; // don't want long wait for counter to count down from initial high unknown value
}
int main()
{    
    initPorts();
    initSystick();
    enable_interrupts();
        
    while(1)
    {
     
        

    }
}
