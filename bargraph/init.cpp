#include <stdint.h>
#include "../include/STM32G431xx.h"

void init(void);
void Default_Handler(void);
void Systick_Handler(void);
void USART1_Handler(void);
void Line_State_Handler(void);
int main(void);
// The following are 'declared' in the linker script
extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;


extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));

typedef void (*fptr)(void);
// the section "vectors" is placed at the beginning of flash 
// by the linker script
#pragma GCC push_options
#pragma GCC optimize ("O0")

const fptr Vectors[] __attribute__((section(".vectors"))) ={
	(fptr)0x20008000, 	/* Top of stack (32k) */ 
	init,   		/* Reset Handler */
	Default_Handler,	/* NMI */
	Default_Handler,	/* Hard Fault */
	Default_Handler,	/* MemManage */
	Default_Handler,	/* Reserved  */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* Reserved */ 
	Default_Handler,	/* Reserved */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* SVCall */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* Reserved */
	Default_Handler,	/* PendSV */
	Default_Handler,	/* SysTick */	
/* External interrupt handlers follow */
	Default_Handler, 	/* 0: WWDG */
	Default_Handler, 	/* 1: PVD_PVM */
	Default_Handler, 	/* 2: RTC/TAMP/CSS_LSE */
	Default_Handler, 	/* 3: RTC_WKUP */
	Default_Handler, 	/* 4: FLASH */
	Default_Handler, 	/* 5: RCC */
	Default_Handler, 	/* 6: EXTI0 */
	Default_Handler,    /* 7: EXTI1 */
	Default_Handler, 	/* 8: EXTI2 */
	Default_Handler, 	/* 9: EXTI3 */
	Default_Handler, 	/* 10: EXTI4 */
	Default_Handler, 	/* 11: DMA1_CH1 */
	Default_Handler, 	/* 12: DMA1_CH2 */
	Default_Handler, 	/* 13: DMA1_CH3 */
	Default_Handler, 	/* 14: DMA1_CH4 */
	Default_Handler, 	/* 15: DMA1_CH5 */
	Default_Handler, 	/* 16: DMA1_CH6 */
	Default_Handler, 	/* 17: DMA1_CH7 */
	Default_Handler, 	/* 18: ADC1_2 */
	Default_Handler, 	/* 19: USB_HP */
	Default_Handler, 	/* 20: USB_LP */
	Default_Handler, 	/* 21: fdcan1_intr1_it */
	Default_Handler, 	/* 22: fdcan1_intr0_it */
	Default_Handler, 	/* 23: EXTI9_5 */
	Default_Handler, 	/* 24: TIM1_BRK/TIM15 */
	Default_Handler, 	/* 25: TIM1_UP/TIM16 */
	Default_Handler, 	/* 26: TIM1_TRG_COM/TIM17/TIM1_DIR/TIM1_IDX */
	Default_Handler, 	/* 27: TIM1_CC */
	Default_Handler, 	/* 28: TIM2 */
	Default_Handler, 	/* 29: TIM3 */
    Default_Handler, 	/* 30  TIM4 */
	Default_Handler, 	/* 31: I2C1_EV */
	Default_Handler, 	/* 32: I2C1_ER */
	Default_Handler, 	/* 33: I2C2_EV */
	Default_Handler, 	/* 34: I2C2_ER */
	Default_Handler, 	/* 35: SPI1 */
	Default_Handler, 	/* 36: SPI2 */
	Default_Handler, 	/* 37: USART1 */
	Default_Handler, 	/* 38: USART2 */
	Default_Handler, 	/* 39: USART3 */
	Default_Handler, 	/* 40: EXTI15_10 */
	Default_Handler, 	/* 41: RTC_ALARM */
	Default_Handler, 	/* 42: USBWakeUP */
	Default_Handler, 	/* 43: TIM8_BRK/TIM8_TERR/TIM8_IERR */
	Default_Handler, 	/* 44: TIM8_UP */
	Default_Handler, 	/* 45: TIM8_TRG_COM/TIM8_DIR/TIM8_IDX */
	Default_Handler, 	/* 46: TIM1_CC */
	Default_Handler, 	/* 47: ADC3 */
	Default_Handler, 	/* 48: FSMC */
	Default_Handler, 	/* 49: LPTIM1 */
	Default_Handler, 	/* 50: TIM5 */
	Default_Handler, 	/* 51: SPI3 */
	Default_Handler, 	/* 52: UART4 */
	Default_Handler, 	/* 53: UART5 */
	Default_Handler, 	/* 54: TIM6_DACUNDER */	
	Default_Handler, 	/* 55: TIM7_DACUNDER */
	Default_Handler, 	/* 56: DMA2_CH1 */
	Default_Handler, 	/* 57: DMA2_CH2 */
	Default_Handler, 	/* 58: DMA2_CH3 */
	Default_Handler, 	/* 59: DMA2_CH4 */
	Default_Handler, 	/* 60: DMA2_CH5 */
	Default_Handler, 	/* 61: ADC4 global interrupt */
	Default_Handler, 	/* 62: ADC5 global interrupt */
	Default_Handler, 	/* 63: UCPD1 global interrupt */
	Default_Handler, 	/* 64: COMP1_2_3 */
	Default_Handler, 	/* 65: COMP4_5_6 */
	Default_Handler, 	/* 66: COMP7 */
	Default_Handler, 	/* 67: HRTIM1_Master_IRQn */
	Default_Handler, 	/* 68: HRTIM1_TIMA_IRQn */
	Default_Handler, 	/* 69: HRTIM1_TIMB_IRQn */
	Default_Handler, 	/* 70: HRTIM1_TIMC_IRQn */
	Default_Handler, 	/* 71: HRTIM1_TIMD_IRQn */
	Default_Handler, 	/* 72: HRTIM1_TIME_IRQn */
	Default_Handler, 	/* 73: HRTIM1_TIM_FLT_IRQn */
	Default_Handler, 	/* 74: HRTIM1_TIMF_IRQn */
	Default_Handler, 	/* 75: CRS */
	Default_Handler, 	/* 76: SAI */
	Default_Handler, 	/* 77: TIM20_BRK/TIM20_TERR/TIM20_TERR/TIM20_IERR */
	Default_Handler, 	/* 78: TIM20_UP */
	Default_Handler, 	/* 79: TIM20_TRG_COM/TIM20_DIR/TIM20_IDX */
	Default_Handler, 	/* 80: TIM20_CC */
	Default_Handler, 	/* 81: FPU */
	Default_Handler, 	/* 82: I2C4_EV */
	Default_Handler, 	/* 83: I2C4_ER */
	Default_Handler, 	/* 84: SPI4 */
	Default_Handler, 	/* 85: AES */
	Default_Handler, 	/* 86: FDCAN2_intr0 */
	Default_Handler, 	/* 87: FDCAN2_intr1 */
	Default_Handler, 	/* 88: FDCAN3_intr0 */
	Default_Handler, 	/* 89: FDCAN3_intr1 */
	Default_Handler, 	/* 90: RNG */
	Default_Handler, 	/* 91: LPUART */
	Default_Handler, 	/* 92: I2C3_EV */
    Default_Handler, 	/* 93: I2C3_ER */
    Default_Handler, 	/* 94: DMAMUX_OVR */
    Default_Handler, 	/* 95: QUADSPI */
    Default_Handler, 	/* 96: DMA1_CH8 */
    Default_Handler, 	/* 97: DMA2_CH6*/
    Default_Handler, 	/* 98: DMA2_CH7 */
    Default_Handler, 	/* 99: DMA2_CH8*/
    Default_Handler, 	/* 100: Cordic */
    Default_Handler 	/* 101: FMAC */    
};
void initClock()
{
    // After reset, CPU clock is set to HSISYS = 16MHz
    
// This is potentially a dangerous function as it could
// result in a system with an invalid clock signal - result: a stuck system
        // Set the PLL up
        // First ensure PLL is disabled
        RCC->CR &= ~(1<<24);
        while( (RCC->CR & (1 <<25))); // wait for PLL ready to be cleared
        
        // Warning here: if system clock is greater than 24MHz then wait-state(s) need to be
        // inserted into Flash memory interface
        // If the chip is run at 170MHz then 7 wait states are required.
        // SysClock is taken from output R of the PLL.  It is divided by 2 by default so
        // should aim for 340MHz output from PLL
        // 340 = 16 * 85 / 4 so N = 85; M = 3 (note divisor = M+1)
        FLASH->ACR &= 0xfffffff0;
        FLASH->ACR |= 7;        
        // Turn on FLASH prefetch buffer
        FLASH->ACR |= (1 << 8);
        // Set PLL input clock to 16MHz HSI clock
        RCC->PLLSYSCFGR |= (1<<1);        
        RCC->PLLSYSCFGR &= 0xffff80ff; // clear N bits
        RCC->PLLSYSCFGR |= (85 << 8);  // set N = 85;
        RCC->PLLSYSCFGR &= 0xffffff0f; // clear M bits
        RCC->PLLSYSCFGR |= (3 << 4);  // set M = 4;
        RCC->PLLSYSCFGR |= (1 << 24); // enable R output
        
        // and turn the PLL back on again
        RCC->CR |= (1<<24);        
        // set PLL as system clock source 
        RCC->CFGR |= (1<<1)+(1<<0);
}
void init_array()
{
    // This function calls constructors for global and static objects
    uint32_t count;
    uint32_t i;
    
    count = __preinit_array_end - __preinit_array_start;
    for (i = 0; i < count; i++)
        __preinit_array_start[i] ();
    count = __init_array_end - __init_array_start;
    for (i = 0; i < count; i++)
        __init_array_start[i] (); 
}
void init()
{
// do global/static data initialization
	unsigned char *src;
	unsigned char *dest;
	unsigned len;
    initClock();
	src= &INIT_DATA_VALUES;
	dest= &INIT_DATA_START;
	len= &INIT_DATA_END-&INIT_DATA_START;
	while (len--)
		*dest++ = *src++;
// zero out the uninitialized global/static variables
	dest = &BSS_START;
	len = &BSS_END - &BSS_START;
	while (len--)
		*dest++=0;
    init_array();
	main();
}

void Default_Handler()
{
	while(1);
}
#pragma GCC pop_options
