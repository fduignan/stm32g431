// Issues : it is important to use int16_t for i/o data in the ADC interrupt ADC_Handler routine
// For some reason a gain is creeping in when the cut-off frequency is lowered.  This leads to numeric/DAC/ADC overflows

/* FIR Low pass filter making use of the FMAC's within the STM32G431 MCU
*/


/* Architecture : The plan: Set up the ADC, connect it directly to the FMAC which in turn will directly drive a DAC
 * Single channel in and out.
 * The filter coefficients were produced using a jupyter notebook file (see current directory)
 * The filter length is 64
 * The FMAC memory has 256x16 bit entries
 * The first 64 will be X1 and will hold input samples in a circular buffer
 * The second 64 will be X2 and will hold the filter coefficients (b)
*/
#include <stdint.h>
#include "../include/STM32G431xx.h"
#include "coffs.h"
// These constants are used together to define the interrupt (sampling) rate
// Some values produce more accurate results than others. The interrupt period is CPU_CLOCK_SPEED / FS
// If this works out to be an integer then the achieved sample rate will be closer to the expected value (no truncation)

#define CPU_CLOCK_SPEED 170000000
#define FS 48000
void delay(uint32_t dly)
{
    while(dly--);
}
void initFMAC()
{

	RCC->AHB1ENR |= (1 << 4); // enable the FMAC clock
	
	uint32_t d1 = 8;
	uint32_t X1_Full_WM = 0;
	uint32_t X1_Buf_Size = FILTER_LENGTH+d1;
	uint32_t X1_Base = 0;
	
	// Turn on output clipping
	FMAC->CR |= (1 << 15);
	
	FMAC->X1BUFCFG = (X1_Full_WM << 16) + (X1_Buf_Size << 8) + X1_Base;		
	uint32_t X2_Buf_Size = FILTER_LENGTH;	
	uint32_t X2_Base = FILTER_LENGTH+d1; // start X2 just after X1+d1
	FMAC->X2BUFCFG = (X2_Buf_Size << 8) + X2_Base;	
		
	uint32_t Y_Base = FILTER_LENGTH*2+d1;
	uint32_t Y_Buf_Size = 1; 
	FMAC->YBUFCFG = (Y_Buf_Size << 8) + Y_Base;

	// Pre-Load the X1 buffer (not strictly necessary)
	FMAC->PARAM = 0;
	FMAC->PARAM = (1 << 24) + FILTER_LENGTH;
	FMAC->PARAM |= (1 << 31);
	for (int i=0; i < FILTER_LENGTH; i++)
	{
		FMAC->WDATA = (int16_t)0;		
	}
	
	// Load the X2 buffer
	FMAC->PARAM = 0;
	FMAC->PARAM = (1 << 25) + FILTER_LENGTH;
	FMAC->PARAM |= (1 << 31);
	for (int i=0; i < FILTER_LENGTH; i++)
	{
		FMAC->WDATA = (int16_t)b[i];		
	}
	
	// Set the FMAC function to FIR/Convolution			
	FMAC->PARAM = 0;
	FMAC->PARAM = (1 << 27) + FILTER_LENGTH;
	FMAC->PARAM |= (1 << 31);
	
}
void initPorts()
{
    RCC->AHB2ENR |= (1 << 2); // enable Port C
    GPIOC->MODER &= ~(1 << 7); // Make bit 3 an output
    GPIOC->MODER |= (1 << 6);
	GPIOC->OSPEEDR |= (3);
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

void initADC()
{
    
        
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
    RCC->CCIPR1 |= (1 << 29); // select system clock as ADC clock    
    
    // Fast ADC channels are on ADC12_INP1 to ADC12_INP4.
    // INP1 and INP2 are on pins : PA0 (5), PA1 (6)
    RCC->AHB2ENR |= (1 << 0); // enable Port 0
    GPIOA->MODER |= (1 << 1) | (1 << 0); // Set PA0 mode to analogue (ADC)     
    RCC->AHB2ENR |= (1 << 13); // enable clock for ADC12
    RCC->AHB2RSTR &= ~(1 << 13);  // Take ADC out of reset
    ADC1->CR = 0;             // ensure  ADEN = 0 to allow configuration    
    ADC12_Common->CCR |= (8 << 18) + 6 + (3 << 16);  // Prescale clock by factor of 8 (must be greater than 4) , Dual mode, adc_hclk/4           
    ADC1->CR = (1 << 28);         // Turn on the voltage reference    
    // Start calibration sequence
    ADC1->CR |= (1 << 31);
    while(ADC1->CR & (1 << 31)); // wait for calibration to finish    
    // According to page 585 of RM0440 Rev 2 TIM2_TRGO event is ADC Trigger source EXT11 and
    // the bits 1011 need to be written to EXTSEL in ADC1_CFGR    
    ADC1->CFGR = (1 << 31)+(1 << 12)+(1 << 10)+(1 << 8)+(1 << 6)+(1 << 5); // Disable injection, enable external trigger, TIM2_TRG0 = trigger, allow overrun    
    ADC1->SQR1 = 0; // once channels in the sequence per ADC
    ADC1->SQR1 |= (1 << 6);   // Channel 1 on ADC1        
    ADC1->IER = (1 << 3);     // enable interrupt when conversion is complete
    NVIC->ISER0 |= (1 << 18); // enable ADC interrupts in NVIC
    ADC1->CR |= (1 << 0);     // enable ADC1    
    ADC1->CR |= (1 << 2);     // start sampling    
    
}
void ADC_Handler()
{    
    static int16_t Chan1=0;
    static int16_t Chan2=0;
    // The LED output is used to measure the execution time of the ISR           
    GPIOC->ODR |= (1 << 3); // drive port pin high                   
    if (ADC1->ISR & (1 << 3) )
    {
        int16_t temp;		
        Chan1 = (ADC12_Common->CDR & 0xffff)-2048;  		
		FMAC->WDATA = Chan1;// Chan1 - (int16_t)2048; // subtract away the DC offset in the input signal	
		while(FMAC->SR & 1); // wait for output data to be available
		temp = FMAC->RDATA;    
        DAC1->DAC_DHR12R1 = temp+(int16_t)2048; // add back in the dc offset required by the DAC
        ADC1->ISR = (1 << 3);     // clear ADC interrupt flag        
    }    
    GPIOC->ODR &= ~(1 << 3); // drive port pin high                   
}

int main()
{    
    
    uint16_t output = 0;
	//b[0]=1;
    initPorts();    
	initFMAC();
    initDACS();
    initADC();
    enable_interrupts();
        
    while(1)
    {             
		
		delay(1000);

    }
}
