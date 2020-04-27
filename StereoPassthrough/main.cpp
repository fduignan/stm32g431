/* Stereo audio example - does not use CMSIS library 
 * As much as possible code will be contained in a single file with 
 * the exception of interrupt vectors etc.
*/


/* Architecture : Timer 2 will be used to trigger ADC conversions using the EXTI system within the MCU
 * An ADC interrupt service routine is triggered at the end of each conversion.  
 * Dual ADC mode (Master/Slave) is used as this application is targeting stereo sound processing
*/
#include <stdint.h>
#include "../include/STM32G431xx.h"

// These constants are used together to define the interrupt (sampling) rate
// Some values produce more accurate results than others. The interrupt period is CPU_CLOCK_SPEED / FS
// If this works out to be an integer then the achieved sample rate will be closer to the expected value (no truncation)

#define CPU_CLOCK_SPEED 170000000
#define FS 100000
void delay(uint32_t dly)
{
    while(dly--);
}
void initPorts()
{
    RCC->AHB2ENR |= (1 << 5); // enable Port F
    GPIOF->MODER &= ~(1 << 1); // Make bit 0 an output
    GPIOF->MODER |= (1 << 0);
}
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

void initADC_old()
{
    RCC->CCIPR1 |= (1 << 29); // select system clock as ADC clock    
    // Fast ADC channels are on ADC12_INP1 to ADC12_INP4.
    // INP1 and INP2 are on pins : PA0 (5), PA1 (6)
    RCC->AHB2ENR |= (1 << 0); // enable Port 0
    GPIOA->MODER |= (1 << 1) | (1 << 0); // Set PA0 mode to analogue (ADC) 
    GPIOA->MODER |= (1 << 3) | (1 << 2); // Set PA1 mode to analogue (ADC) 
    RCC->AHB2ENR |= (1 << 13); // enable clock for ADC12
    RCC->AHB2RSTR &= ~(1 << 13);  // Take ADC out of reset
    ADC1->CR = 0;             // ensure  ADEN = 0 to allow configuration
    ADC12_Common->CCR = (1 << 22);        // enable the voltage reference
    ADC1->CR = (1 << 28);         // Turn on the voltage reference

    // Start calibration sequence
    ADC1->CR |= (1 << 31);
    while(ADC1->CR & (1 << 31)); // wait for calibration to finish
    RCC->APB1ENR1  |= (1 << 0);   // Enable Timer 2
    RCC->APB1RSTR1 &= ~(1 << 0);  // Take Timer 2 out of reset
    // Counter will be run in down-counting mode:  Counting up to TIM2_ARR and reset back to zero
    TIM2->CR1 = 0;      // Ensure CEN = 0 so other register bits can be set
    TIM2->SR = 0x00;    // Start with a clear Timer 2 status register
    TIM2->PSC = 0;      // select prescaler value of 1
    TIM2->ARR = CPU_CLOCK_SPEED/FS;    
    TIM2->CR2 = (1 << 5);   // Select Master mode for Timer 2
    TIM2->EGR |= (1 << 0);  // Set UG bit
    TIM2->CR1 |= (1 << 4);  // Count down
    TIM2->CR1 |= (1 << 0);  // Enable timer 2
    // According to page 585 of RM0440 Rev 2 TIM2_TRGO event is ADC Trigger source EXT11 and
    // the bits 1011 need to be written to EXTSEL in ADC1_CFGR
    ADC1->CFGR = (1 << 31)+(1 << 12)+(1 << 10)+(1 << 8)+(1 << 6)+(1 << 5); // Disable injection, enable external trigger, TIM2_TRG0 = trigger, allow overrun
    ADC1->SQR1 = 1; // two channels in the sequence
    ADC1->SQR1 |= (1 << 6) | (2 << 12);  // Channel 1 and Channel 2 are in the conversion sequence.
    ADC1->IER = (1 << 3);       // enable interrupt when conversion is complete
    NVIC->ISER0 |= (1 << 18); // enable ADC interrupts in NVIC
    ADC1->CR |= (1 << 0);     // enable ADC
    ADC1->CR |= (1 << 2);     // start sampling
    
}
void initADC()
{
    RCC->CCIPR1 |= (1 << 29); // select system clock as ADC clock    
    // Fast ADC channels are on ADC12_INP1 to ADC12_INP4.
    // INP1 and INP2 are on pins : PA0 (5), PA1 (6)
    RCC->AHB2ENR |= (1 << 0); // enable Port 0
    GPIOA->MODER |= (1 << 1) | (1 << 0); // Set PA0 mode to analogue (ADC) 
    GPIOA->MODER |= (1 << 3) | (1 << 2); // Set PA1 mode to analogue (ADC) 
    RCC->AHB2ENR |= (1 << 13); // enable clock for ADC12
    RCC->AHB2RSTR &= ~(1 << 13);  // Take ADC out of reset
    ADC1->CR = 0;   // ensure  ADEN = 0 to allow configuration
    ADC2->CR = 0;   // ensure  ADEN = 0 to allow configuration
    ADC12_Common->CCR = (1 << 22);        // enable the voltage reference
    ADC1->CR = (1 << 28);         // Turn on the voltage reference
    ADC2->CR = (1 << 28);         // Turn on the voltage reference

    // Start calibration sequence
    ADC1->CR |= (1 << 31);
    while(ADC1->CR & (1 << 31)); // wait for calibration to finish
    
    ADC2->CR |= (1 << 31);
    while(ADC2->CR & (1 << 31)); // wait for calibration to finish
    
    RCC->APB1ENR1  |= (1 << 0);   // Enable Timer 2
    RCC->APB1RSTR1 &= ~(1 << 0);  // Take Timer 2 out of reset
    // Counter will be run in down-counting mode:  Counting up to TIM2_ARR and reset back to zero
    TIM2->CR1 = 0;      // Ensure CEN = 0 so other register bits can be set
    TIM2->SR = 0x00;    // Start with a clear Timer 2 status register
    TIM2->PSC = 0;      // select prescaler value of 1
    TIM2->ARR = CPU_CLOCK_SPEED/FS;    
    TIM2->CR2 = (1 << 5);   // Select Master mode for Timer 2
    TIM2->EGR |= (1 << 0);  // Set UG bit
    TIM2->CR1 |= (1 << 4);  // Count down
    TIM2->CR1 |= (1 << 0);  // Enable timer 2
    // According to page 585 of RM0440 Rev 2 TIM2_TRGO event is ADC Trigger source EXT11 and
    // the bits 1011 need to be written to EXTSEL in ADC1_CFGR
    ADC1->CFGR = (1 << 31)+(1 << 12)+(1 << 10)+(1 << 8)+(1 << 6)+(1 << 5); // Disable injection, enable external trigger, TIM2_TRG0 = trigger, allow overrun
    ADC1->SQR1 = 0; // one channel in the sequence
    ADC1->SQR1 |= (1 << 6);  // Channel 1 in the conversion sequence.
    ADC1->IER = (1 << 3);       // enable interrupt when conversion is complete
    
    ADC2->CFGR = (1 << 31)+(1 << 12)+(1 << 10)+(1 << 8)+(1 << 6)+(1 << 5); // Disable injection, enable external trigger, TIM2_TRG0 = trigger, allow overrun
    ADC2->SQR1 = 0; // one channel in the sequence
    ADC2->SQR1 |= (2 << 6);  // Channel 2  in the conversion sequence.    
    
    ADC12_Common->CCR |= 1; // Dual mode 
    
    NVIC->ISER0 |= (1 << 18); // enable ADC interrupts in NVIC
    ADC1->CR |= (1 << 0);     // enable ADC
    ADC2->CR |= (1 << 2);     // start sampling    
}

void ADC_Handler()
{    
    
    static int count = 0;
    GPIOF->ODR |= 1; // drive port pin high         
    
    DAC1->DAC_DHR12R1 = ADC1->DR;
    DAC1->DAC_DHR12R2 = count++;//ADC1->DR;//ADC2->DR;
    // The green LED output is used to measure the execution time of the ISR
        
    ADC1->ISR = (1 << 3) + (1 << 2);     // clear ADC interrupt flags
    GPIOF->ODR &= ~1; // drive port pin low
    
}
void ADC_Handler_old()
{
    if (ADC1->ISR & (1 << 3) )
    {
    GPIOF->ODR |= 1; // drive port pin high 
    ADC1->ISR = (1 << 3);
    }
    uint16_t ADCResult = ADC1->DR;  
    DAC1->DAC_DHR12R1 = ADCResult;    
    DAC1->DAC_DHR12R2 = ADCResult;
    // The green LED output is used to measure the execution time of the ISR
    
    
    ADC1->ISR = (1 << 2);     // clear ADC interrupt flag
    GPIOF->ODR &= ~1; // drive port pin low
    
}
int main()
{    
    
    uint16_t output = 0;
    initPorts();    
    initDACS();
    initADC();
    enable_interrupts();
        
    while(1)
    {             

    }
}
