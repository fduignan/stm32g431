#include <stdint.h>
#include "serial.h"
#include "../include/STM32G431xx.h"

void delay(uint32_t dly)
{
    while(dly--);
}
void setup()
{
    GPIOA->MODER &= ~(1 << 1); // Make bit 0 an output
    GPIOA->MODER |= (1 << 0);
}
serial Serial;
int main()
{
    char c;
    delay(1000000); // long delay in case I screw up the clocks.
    setup();  
    Serial.begin();
    enable_interrupts();
    Serial.print("Type something\r\n");
    while(1)
    {
        if (c=Serial.egetc())
        {
            Serial.print("You typed: ");  
            Serial.print(c);
            Serial.print("\r\n");        
        }
    }
}
