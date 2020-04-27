
/****************************************************************************************************//**
 * @file     STM32G431xx.h
 *
 * @brief    CMSIS Cortex-M4 Peripheral Access Layer Header File for
 *           STM32G431xx from .
 *
 * @version  V1.0
 * @date     9. February 2020
 *
 * @note     Generated with SVDConv V2.87l 
 *           from CMSIS SVD File 'STM32G431xx.svd' Version 1.0,
 *******************************************************************************************************/



/** @addtogroup (null)
  * @{
  */

/** @addtogroup STM32G431xx
  * @{
  */

#ifndef STM32G431XX_H
#define STM32G431XX_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* -------------------  STM32G431xx Specific Interrupt Numbers  ------------------- */
  WWDG_IRQn                     =   0,              /*!<   0  Window Watchdog interrupt                                        */
  PVD_PVM_IRQn                  =   1,              /*!<   1  PVD through EXTI line detection                                  */
  RTC_TAMP_CSS_LSE_IRQn         =   2,              /*!<   2  RTC_TAMP_CSS_LSE                                                 */
  RTC_WKUP_IRQn                 =   3,              /*!<   3  RTC Wakeup timer                                                 */
  FLASH_IRQn                    =   4,              /*!<   4  FLASH                                                            */
  RCC_IRQn                      =   5,              /*!<   5  RCC                                                              */
  EXTI0_IRQn                    =   6,              /*!<   6  EXTI Line0 interrupt                                             */
  EXTI1_IRQn                    =   7,              /*!<   7  EXTI Line1 interrupt                                             */
  EXTI2_IRQn                    =   8,              /*!<   8  EXTI Line2 interrupt                                             */
  EXTI3_IRQn                    =   9,              /*!<   9  EXTI Line3 interrupt                                             */
  EXTI4_IRQn                    =  10,              /*!<  10  EXTI Line4 interrupt                                             */
  DMA1_CH1_IRQn                 =  11,              /*!<  11  DMA1 channel 1 interrupt                                         */
  DMA1_CH2_IRQn                 =  12,              /*!<  12  DMA1 channel 2 interrupt                                         */
  DMA1_CH3_IRQn                 =  13,              /*!<  13  DMA1 channel 3 interrupt                                         */
  DMA1_CH4_IRQn                 =  14,              /*!<  14  DMA1 channel 4 interrupt                                         */
  DMA1_CH5_IRQn                 =  15,              /*!<  15  DMA1 channel 5 interrupt                                         */
  DMA1_CH6_IRQn                 =  16,              /*!<  16  DMA1 channel 6 interrupt                                         */
  ADC1_2_IRQn                   =  18,              /*!<  18  ADC1 and ADC2 global interrupt                                   */
  USB_HP_IRQn                   =  19,              /*!<  19  USB_HP                                                           */
  USB_LP_IRQn                   =  20,              /*!<  20  USB_LP                                                           */
  fdcan1_intr1_it_IRQn          =  21,              /*!<  21  fdcan1_intr1_it                                                  */
  fdcan1_intr0_it_IRQn          =  22,              /*!<  22  fdcan1_intr0_it                                                  */
  EXTI9_5_IRQn                  =  23,              /*!<  23  EXTI9_5                                                          */
  TIM1_BRK_TIM15_IRQn           =  24,              /*!<  24  TIM1_BRK_TIM15                                                   */
  TIM1_UP_TIM16_IRQn            =  25,              /*!<  25  TIM1_UP_TIM16                                                    */
  TIM1_TRG_COM_IRQn             =  26,              /*!<  26  TIM1_TRG_COM/                                                    */
  TIM1_CC_IRQn                  =  27,              /*!<  27  TIM1 capture compare interrupt                                   */
  TIM2_IRQn                     =  28,              /*!<  28  TIM2                                                             */
  TIM3_IRQn                     =  29,              /*!<  29  TIM3                                                             */
  TIM4_IRQn                     =  30,              /*!<  30  TIM4                                                             */
  I2C1_EV_IRQn                  =  31,              /*!<  31  I2C1_EV                                                          */
  I2C1_ER_IRQn                  =  32,              /*!<  32  I2C1_ER                                                          */
  I2C2_EV_IRQn                  =  33,              /*!<  33  I2C2_EV                                                          */
  I2C2_ER_IRQn                  =  34,              /*!<  34  I2C2_ER                                                          */
  SPI1_IRQn                     =  35,              /*!<  35  SPI1                                                             */
  SPI2_IRQn                     =  36,              /*!<  36  SPI2                                                             */
  USART1_IRQn                   =  37,              /*!<  37  USART1                                                           */
  USART2_IRQn                   =  38,              /*!<  38  USART2                                                           */
  USART3_IRQn                   =  39,              /*!<  39  USART3                                                           */
  EXTI15_10_IRQn                =  40,              /*!<  40  EXTI15_10                                                        */
  RTC_ALARM_IRQn                =  41,              /*!<  41  RTC_ALARM                                                        */
  USBWakeUP_IRQn                =  42,              /*!<  42  USBWakeUP                                                        */
  TIM8_BRK_IRQn                 =  43,              /*!<  43  TIM8_BRK                                                         */
  TIM8_UP_IRQn                  =  44,              /*!<  44  TIM8_UP                                                          */
  TIM8_TRG_COM_IRQn             =  45,              /*!<  45  TIM8_TRG_COM                                                     */
  TIM8_CC_IRQn                  =  46,              /*!<  46  TIM8_CC                                                          */
  LPTIM1_IRQn                   =  49,              /*!<  49  LPTIM1                                                           */
  SPI3_IRQn                     =  51,              /*!<  51  SPI3                                                             */
  UART4_IRQn                    =  52,              /*!<  52  UART4                                                            */
  TIM6_DACUNDER_IRQn            =  54,              /*!<  54  TIM6_DACUNDER                                                    */
  TIM7_IRQn                     =  55,              /*!<  55  TIM7                                                             */
  DMA2_CH1_IRQn                 =  56,              /*!<  56  DMA2_CH1                                                         */
  DMA2_CH2_IRQn                 =  57,              /*!<  57  DMA2_CH2                                                         */
  DMA2_CH3_IRQn                 =  58,              /*!<  58  DMA2_CH3                                                         */
  DMA2_CH4_IRQn                 =  59,              /*!<  59  DMA2_CH4                                                         */
  DMA2_CH5_IRQn                 =  60,              /*!<  60  DMA2_CH5                                                         */
  UCPD1_IRQn                    =  63,              /*!<  63  UCPD1                                                            */
  COMP1_2_3_IRQn                =  64,              /*!<  64  COMP1_2_3                                                        */
  COMP4_IRQn                    =  65,              /*!<  65  COMP4_5_6                                                        */
  CRS_IRQn                      =  75,              /*!<  75  CRS                                                              */
  SAI_IRQn                      =  76,              /*!<  76  SAI                                                              */
  FPU_IRQn                      =  81,              /*!<  81  Floating point unit interrupt                                    */
  AES_IRQn                      =  85,              /*!<  85  AES                                                              */
  RNG_IRQn                      =  90,              /*!<  90  RNG                                                              */
  LPUART_IRQn                   =  91,              /*!<  91  LPUART                                                           */
  I2C3_EV_IRQn                  =  92,              /*!<  92  I2C3_EV                                                          */
  I2C3_ER_IRQn                  =  93,              /*!<  93  I2C3_ER                                                          */
  DMAMUX_OVR_IRQn               =  94,              /*!<  94  DMAMUX_OVR                                                       */
  DMA2_CH6_IRQn                 =  97,              /*!<  97  DMA2_CH6                                                         */
  Cordic_IRQn                   = 100,              /*!< 100  Cordic                                                           */
  FMAC_IRQn                     = 101               /*!< 101  FMAC                                                             */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M4 Processor and Core Peripherals---------------- */
#define __CM4_REV                 0x0001            /*!< Cortex-M4 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               4            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  1            /*!< FPU present or not                                                    */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm4.h"                               /*!< Cortex-M4 processor and core peripherals                              */
#include "system_STM32G431xx.h"                     /*!< STM32G431xx System                                                    */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                       CRC                      ================ */
/* ================================================================================ */


/**
  * @brief Cyclic redundancy check calculation unit (CRC)
  */

typedef struct {                                    /*!< CRC Structure                                                         */
  __IO uint32_t  DR;                                /*!< Data register                                                         */
  __IO uint32_t  IDR;                               /*!< Independent data register                                             */
  __IO uint32_t  CR;                                /*!< Control register                                                      */
  __I  uint32_t  RESERVED;
  __IO uint32_t  INIT;                              /*!< Initial CRC value                                                     */
  __IO uint32_t  POL;                               /*!< polynomial                                                            */
} CRC_Type;


/* ================================================================================ */
/* ================                      WWDG                      ================ */
/* ================================================================================ */


/**
  * @brief WinWATCHDOG (WWDG)
  */

typedef struct {                                    /*!< WWDG Structure                                                        */
  __O  uint32_t  KR;                                /*!< Key register                                                          */
  __IO uint32_t  PR;                                /*!< Prescaler register                                                    */
  __IO uint32_t  RLR;                               /*!< Reload register                                                       */
  __I  uint32_t  SR;                                /*!< Status register                                                       */
  __IO uint32_t  WINR;                              /*!< Window register                                                       */
} WWDG_Type;


/* ================================================================================ */
/* ================                      IWDG                      ================ */
/* ================================================================================ */


/**
  * @brief System window watchdog (IWDG)
  */

typedef struct {                                    /*!< IWDG Structure                                                        */
  __IO uint32_t  CR;                                /*!< Control register                                                      */
  __IO uint32_t  CFR;                               /*!< Configuration register                                                */
  __IO uint32_t  SR;                                /*!< Status register                                                       */
} IWDG_Type;


/* ================================================================================ */
/* ================                      I2C1                      ================ */
/* ================================================================================ */


/**
  * @brief Inter-integrated circuit (I2C1)
  */

typedef struct {                                    /*!< I2C1 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< Control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control register 2                                                    */
  __IO uint32_t  OAR1;                              /*!< Own address register 1                                                */
  __IO uint32_t  OAR2;                              /*!< Own address register 2                                                */
  __IO uint32_t  TIMINGR;                           /*!< Timing register                                                       */
  __IO uint32_t  TIMEOUTR;                          /*!< Status register 1                                                     */
  __IO uint32_t  ISR;                               /*!< Interrupt and Status register                                         */
  __O  uint32_t  ICR;                               /*!< Interrupt clear register                                              */
  __I  uint32_t  PECR;                              /*!< PEC register                                                          */
  __I  uint32_t  RXDR;                              /*!< Receive data register                                                 */
  __IO uint32_t  TXDR;                              /*!< Transmit data register                                                */
} I2C1_Type;


/* ================================================================================ */
/* ================                      FLASH                     ================ */
/* ================================================================================ */


/**
  * @brief Flash (FLASH)
  */

typedef struct {                                    /*!< FLASH Structure                                                       */
  __IO uint32_t  ACR;                               /*!< Access control register                                               */
  __O  uint32_t  PDKEYR;                            /*!< Power down key register                                               */
  __O  uint32_t  KEYR;                              /*!< Flash key register                                                    */
  __O  uint32_t  OPTKEYR;                           /*!< Option byte key register                                              */
  __IO uint32_t  SR;                                /*!< Status register                                                       */
  __IO uint32_t  CR;                                /*!< Flash control register                                                */
  __IO uint32_t  ECCR;                              /*!< Flash ECC register                                                    */
  __I  uint32_t  RESERVED;
  __IO uint32_t  OPTR;                              /*!< Flash option register                                                 */
  __IO uint32_t  PCROP1SR;                          /*!< Flash Bank 1 PCROP Start address register                             */
  __IO uint32_t  PCROP1ER;                          /*!< Flash Bank 1 PCROP End address register                               */
  __IO uint32_t  WRP1AR;                            /*!< Flash Bank 1 WRP area A address register                              */
  __IO uint32_t  WRP1BR;                            /*!< Flash Bank 1 WRP area B address register                              */
  __I  uint32_t  RESERVED1[15];
  __IO uint32_t  SEC1R;                             /*!< securable area bank1 register                                         */
} FLASH_Type;


/* ================================================================================ */
/* ================                     DBGMCU                     ================ */
/* ================================================================================ */


/**
  * @brief Debug support (DBGMCU)
  */

typedef struct {                                    /*!< DBGMCU Structure                                                      */
  __I  uint32_t  IDCODE;                            /*!< MCU Device ID Code Register                                           */
  __IO uint32_t  CR;                                /*!< Debug MCU Configuration Register                                      */
  __IO uint32_t  APB1L_FZ;                          /*!< APB Low Freeze Register 1                                             */
  __IO uint32_t  APB1H_FZ;                          /*!< APB Low Freeze Register 2                                             */
  __IO uint32_t  APB2_FZ;                           /*!< APB High Freeze Register                                              */
} DBGMCU_Type;


/* ================================================================================ */
/* ================                       RCC                      ================ */
/* ================================================================================ */


/**
  * @brief Reset and clock control (RCC)
  */

typedef struct {                                    /*!< RCC Structure                                                         */
  __IO uint32_t  CR;                                /*!< Clock control register                                                */
  __IO uint32_t  ICSCR;                             /*!< Internal clock sources calibration register                           */
  __IO uint32_t  CFGR;                              /*!< Clock configuration register                                          */
  __IO uint32_t  PLLSYSCFGR;                        /*!< PLL configuration register                                            */
  __I  uint32_t  RESERVED[2];
  __IO uint32_t  CIER;                              /*!< Clock interrupt enable register                                       */
  __I  uint32_t  CIFR;                              /*!< Clock interrupt flag register                                         */
  __O  uint32_t  CICR;                              /*!< Clock interrupt clear register                                        */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  AHB1RSTR;                          /*!< AHB1 peripheral reset register                                        */
  __IO uint32_t  AHB2RSTR;                          /*!< AHB2 peripheral reset register                                        */
  __IO uint32_t  AHB3RSTR;                          /*!< AHB3 peripheral reset register                                        */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  APB1RSTR1;                         /*!< APB1 peripheral reset register 1                                      */
  __IO uint32_t  APB1RSTR2;                         /*!< APB1 peripheral reset register 2                                      */
  __IO uint32_t  APB2RSTR;                          /*!< APB2 peripheral reset register                                        */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  AHB1ENR;                           /*!< AHB1 peripheral clock enable register                                 */
  __IO uint32_t  AHB2ENR;                           /*!< AHB2 peripheral clock enable register                                 */
  __IO uint32_t  AHB3ENR;                           /*!< AHB3 peripheral clock enable register                                 */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  APB1ENR1;                          /*!< APB1ENR1                                                              */
  __IO uint32_t  APB1ENR2;                          /*!< APB1 peripheral clock enable register 2                               */
  __IO uint32_t  APB2ENR;                           /*!< APB2ENR                                                               */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  AHB1SMENR;                         /*!< AHB1 peripheral clocks enable in Sleep and Stop modes register        */
  __IO uint32_t  AHB2SMENR;                         /*!< AHB2 peripheral clocks enable in Sleep and Stop modes register        */
  __IO uint32_t  AHB3SMENR;                         /*!< AHB3 peripheral clocks enable in Sleep and Stop modes register        */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  APB1SMENR1;                        /*!< APB1SMENR1                                                            */
  __IO uint32_t  APB1SMENR2;                        /*!< APB1 peripheral clocks enable in Sleep and Stop modes register
                                                         2                                                                     */
  __IO uint32_t  APB2SMENR;                         /*!< APB2SMENR                                                             */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  CCIPR1;                            /*!< CCIPR                                                                 */
  __I  uint32_t  RESERVED8;
  __IO uint32_t  BDCR;                              /*!< BDCR                                                                  */
  __IO uint32_t  CSR;                               /*!< CSR                                                                   */
  __IO uint32_t  CRRCR;                             /*!< Clock recovery RC register                                            */
  __IO uint32_t  CCIPR2;                            /*!< Peripherals independent clock configuration register                  */
} RCC_Type;


/* ================================================================================ */
/* ================                       PWR                      ================ */
/* ================================================================================ */


/**
  * @brief Power control (PWR)
  */

typedef struct {                                    /*!< PWR Structure                                                         */
  __IO uint32_t  CR1;                               /*!< Power control register 1                                              */
  __IO uint32_t  CR2;                               /*!< Power control register 2                                              */
  __IO uint32_t  CR3;                               /*!< Power control register 3                                              */
  __IO uint32_t  CR4;                               /*!< Power control register 4                                              */
  __I  uint32_t  SR1;                               /*!< Power status register 1                                               */
  __I  uint32_t  SR2;                               /*!< Power status register 2                                               */
  __O  uint32_t  SCR;                               /*!< Power status clear register                                           */
  __I  uint32_t  RESERVED;
  __IO uint32_t  PUCRA;                             /*!< Power Port A pull-up control register                                 */
  __IO uint32_t  PDCRA;                             /*!< Power Port A pull-down control register                               */
  __IO uint32_t  PUCRB;                             /*!< Power Port B pull-up control register                                 */
  __IO uint32_t  PDCRB;                             /*!< Power Port B pull-down control register                               */
  __IO uint32_t  PUCRC;                             /*!< Power Port C pull-up control register                                 */
  __IO uint32_t  PDCRC;                             /*!< Power Port C pull-down control register                               */
  __IO uint32_t  PUCRD;                             /*!< Power Port D pull-up control register                                 */
  __IO uint32_t  PDCRD;                             /*!< Power Port D pull-down control register                               */
  __IO uint32_t  PUCRE;                             /*!< Power Port E pull-up control register                                 */
  __IO uint32_t  PDCRE;                             /*!< Power Port E pull-down control register                               */
  __IO uint32_t  PUCRF;                             /*!< Power Port F pull-up control register                                 */
  __IO uint32_t  PDCRF;                             /*!< Power Port F pull-down control register                               */
  __IO uint32_t  PUCRG;                             /*!< Power Port G pull-up control register                                 */
  __IO uint32_t  PDCRG;                             /*!< Power Port G pull-down control register                               */
  __I  uint32_t  RESERVED1[10];
  __IO uint32_t  CR5;                               /*!< Power control register 5                                              */
} PWR_Type;


/* ================================================================================ */
/* ================                       RNG                      ================ */
/* ================================================================================ */


/**
  * @brief Random number generator (RNG)
  */

typedef struct {                                    /*!< RNG Structure                                                         */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __I  uint32_t  DR;                                /*!< data register                                                         */
} RNG_Type;


/* ================================================================================ */
/* ================                       AES                      ================ */
/* ================================================================================ */


/**
  * @brief Advanced encryption standard hardware accelerator (AES)
  */

typedef struct {                                    /*!< AES Structure                                                         */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __I  uint32_t  SR;                                /*!< status register                                                       */
  __IO uint32_t  DINR;                              /*!< data input register                                                   */
  __I  uint32_t  DOUTR;                             /*!< data output register                                                  */
  __IO uint32_t  KEYR0;                             /*!< key register 0                                                        */
  __IO uint32_t  KEYR1;                             /*!< key register 1                                                        */
  __IO uint32_t  KEYR2;                             /*!< key register 2                                                        */
  __IO uint32_t  KEYR3;                             /*!< key register 3                                                        */
  __IO uint32_t  IVR0;                              /*!< initialization vector register 0                                      */
  __IO uint32_t  IVR1;                              /*!< initialization vector register 1                                      */
  __IO uint32_t  IVR2;                              /*!< initialization vector register 2                                      */
  __IO uint32_t  IVR3;                              /*!< initialization vector register 3                                      */
  __IO uint32_t  KEYR4;                             /*!< key register 4                                                        */
  __IO uint32_t  KEYR5;                             /*!< key register 5                                                        */
  __IO uint32_t  KEYR6;                             /*!< key register 6                                                        */
  __IO uint32_t  KEYR7;                             /*!< key register 7                                                        */
  __IO uint32_t  SUSP0R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP1R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP2R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP3R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP4R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP5R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP6R;                            /*!< suspend registers                                                     */
  __IO uint32_t  SUSP7R;                            /*!< suspend registers                                                     */
} AES_Type;


/* ================================================================================ */
/* ================                      GPIOA                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose I/Os (GPIOA)
  */

typedef struct {                                    /*!< GPIOA Structure                                                       */
  __IO uint32_t  MODER;                             /*!< GPIO port mode register                                               */
  __IO uint32_t  OTYPER;                            /*!< GPIO port output type register                                        */
  __IO uint32_t  OSPEEDR;                           /*!< GPIO port output speed register                                       */
  __IO uint32_t  PUPDR;                             /*!< GPIO port pull-up/pull-down register                                  */
  __I  uint32_t  IDR;                               /*!< GPIO port input data register                                         */
  __IO uint32_t  ODR;                               /*!< GPIO port output data register                                        */
  __O  uint32_t  BSRR;                              /*!< GPIO port bit set/reset register                                      */
  __IO uint32_t  LCKR;                              /*!< GPIO port configuration lock register                                 */
  __IO uint32_t  AFRL;                              /*!< GPIO alternate function low register                                  */
  __IO uint32_t  AFRH;                              /*!< GPIO alternate function high register                                 */
  __O  uint32_t  BRR;                               /*!< GPIO port bit reset register                                          */
} GPIOA_Type;


/* ================================================================================ */
/* ================                      GPIOB                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose I/Os (GPIOB)
  */

typedef struct {                                    /*!< GPIOB Structure                                                       */
  __IO uint32_t  MODER;                             /*!< GPIO port mode register                                               */
  __IO uint32_t  OTYPER;                            /*!< GPIO port output type register                                        */
  __IO uint32_t  OSPEEDR;                           /*!< GPIO port output speed register                                       */
  __IO uint32_t  PUPDR;                             /*!< GPIO port pull-up/pull-down register                                  */
  __I  uint32_t  IDR;                               /*!< GPIO port input data register                                         */
  __IO uint32_t  ODR;                               /*!< GPIO port output data register                                        */
  __O  uint32_t  BSRR;                              /*!< GPIO port bit set/reset register                                      */
  __IO uint32_t  LCKR;                              /*!< GPIO port configuration lock register                                 */
  __IO uint32_t  AFRL;                              /*!< GPIO alternate function low register                                  */
  __IO uint32_t  AFRH;                              /*!< GPIO alternate function high register                                 */
  __O  uint32_t  BRR;                               /*!< GPIO port bit reset register                                          */
} GPIOB_Type;


/* ================================================================================ */
/* ================                      GPIOC                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose I/Os (GPIOC)
  */

typedef struct {                                    /*!< GPIOC Structure                                                       */
  __IO uint32_t  MODER;                             /*!< GPIO port mode register                                               */
  __IO uint32_t  OTYPER;                            /*!< GPIO port output type register                                        */
  __IO uint32_t  OSPEEDR;                           /*!< GPIO port output speed register                                       */
  __IO uint32_t  PUPDR;                             /*!< GPIO port pull-up/pull-down register                                  */
  __I  uint32_t  IDR;                               /*!< GPIO port input data register                                         */
  __IO uint32_t  ODR;                               /*!< GPIO port output data register                                        */
  __O  uint32_t  BSRR;                              /*!< GPIO port bit set/reset register                                      */
  __IO uint32_t  LCKR;                              /*!< GPIO port configuration lock register                                 */
  __IO uint32_t  AFRL;                              /*!< GPIO alternate function low register                                  */
  __IO uint32_t  AFRH;                              /*!< GPIO alternate function high register                                 */
  __O  uint32_t  BRR;                               /*!< GPIO port bit reset register                                          */
} GPIOC_Type;


/* ================================================================================ */
/* ================                      TIM15                     ================ */
/* ================================================================================ */


/**
  * @brief General purpose timers (TIM15)
  */

typedef struct {                                    /*!< TIM15 Structure                                                       */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register (output mode)                           */
  };
  __I  uint32_t  RESERVED;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  DTR2;                              /*!< timer Deadtime Register 2                                             */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  TISEL;                             /*!< TIM timer input selection register                                    */
  __IO uint32_t  AF1;                               /*!< TIM alternate function option register 1                              */
  __IO uint32_t  AF2;                               /*!< TIM alternate function option register 2                              */
  __I  uint32_t  RESERVED4[221];
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM15_Type;


/* ================================================================================ */
/* ================                      TIM16                     ================ */
/* ================================================================================ */


/**
  * @brief General purpose timers (TIM16)
  */

typedef struct {                                    /*!< TIM16 Structure                                                       */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __I  uint32_t  RESERVED;
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register (output mode)                           */
  };
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  DTR2;                              /*!< timer Deadtime Register 2                                             */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  TISEL;                             /*!< TIM timer input selection register                                    */
  __IO uint32_t  AF1;                               /*!< TIM alternate function option register 1                              */
  __IO uint32_t  AF2;                               /*!< TIM alternate function option register 2                              */
  __I  uint32_t  RESERVED5[221];
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM16_Type;


/* ================================================================================ */
/* ================                      TIM1                      ================ */
/* ================================================================================ */


/**
  * @brief Advanced-timers (TIM1)
  */

typedef struct {                                    /*!< TIM1 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register 1 (output mode)                         */
  };
  
  union {
    __IO uint32_t  CCMR2_Input;                     /*!< capture/compare mode register 2 (input mode)                          */
    __IO uint32_t  CCMR2_Output;                    /*!< capture/compare mode register 2 (output mode)                         */
  };
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __IO uint32_t  CCR3;                              /*!< capture/compare register 3                                            */
  __IO uint32_t  CCR4;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __IO uint32_t  CCR5;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  CCR6;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  CCMR3_Output;                      /*!< capture/compare mode register 2 (output mode)                         */
  __IO uint32_t  DTR2;                              /*!< timer Deadtime Register 2                                             */
  __IO uint32_t  ECR;                               /*!< DMA control register                                                  */
  __IO uint32_t  TISEL;                             /*!< TIM timer input selection register                                    */
  __IO uint32_t  AF1;                               /*!< TIM alternate function option register 1                              */
  __IO uint32_t  AF2;                               /*!< TIM alternate function option register 2                              */
  __I  uint32_t  RESERVED[221];
  __IO uint32_t  DCR;                               /*!< control register                                                      */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM1_Type;


/* ================================================================================ */
/* ================                      TIM2                      ================ */
/* ================================================================================ */


/**
  * @brief Advanced-timers (TIM2)
  */

typedef struct {                                    /*!< TIM2 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register 1 (output mode)                         */
  };
  
  union {
    __IO uint32_t  CCMR2_Input;                     /*!< capture/compare mode register 2 (input mode)                          */
    __IO uint32_t  CCMR2_Output;                    /*!< capture/compare mode register 2 (output mode)                         */
  };
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __IO uint32_t  CCR3;                              /*!< capture/compare register 3                                            */
  __IO uint32_t  CCR4;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __IO uint32_t  CCR5;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  CCR6;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  CCMR3_Output;                      /*!< capture/compare mode register 2 (output mode)                         */
  __IO uint32_t  DTR2;                              /*!< timer Deadtime Register 2                                             */
  __IO uint32_t  ECR;                               /*!< DMA control register                                                  */
  __IO uint32_t  TISEL;                             /*!< TIM timer input selection register                                    */
  __IO uint32_t  AF1;                               /*!< TIM alternate function option register 1                              */
  __IO uint32_t  AF2;                               /*!< TIM alternate function option register 2                              */
  __I  uint32_t  RESERVED[221];
  __IO uint32_t  DCR;                               /*!< control register                                                      */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM2_Type;


/* ================================================================================ */
/* ================                      TIM6                      ================ */
/* ================================================================================ */


/**
  * @brief Basic-timers (TIM6)
  */

typedef struct {                                    /*!< TIM6 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __I  uint32_t  RESERVED;
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
} TIM6_Type;


/* ================================================================================ */
/* ================                    LPTIMER1                    ================ */
/* ================================================================================ */


/**
  * @brief Low power timer (LPTIMER1)
  */

typedef struct {                                    /*!< LPTIMER1 Structure                                                    */
  __I  uint32_t  ISR;                               /*!< Interrupt and Status Register                                         */
  __O  uint32_t  ICR;                               /*!< Interrupt Clear Register                                              */
  __IO uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __IO uint32_t  CFGR;                              /*!< Configuration Register                                                */
  __IO uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  CMP;                               /*!< Compare Register                                                      */
  __IO uint32_t  ARR;                               /*!< Autoreload Register                                                   */
  __I  uint32_t  CNT;                               /*!< Counter Register                                                      */
  __IO uint32_t  OR;                                /*!< option register                                                       */
} LPTIMER1_Type;


/* ================================================================================ */
/* ================                     USART1                     ================ */
/* ================================================================================ */


/**
  * @brief Universal synchronous asynchronous receiver transmitter (USART1)
  */

typedef struct {                                    /*!< USART1 Structure                                                      */
  __IO uint32_t  CR1;                               /*!< Control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control register 2                                                    */
  __IO uint32_t  CR3;                               /*!< Control register 3                                                    */
  __IO uint32_t  BRR;                               /*!< Baud rate register                                                    */
  __IO uint32_t  GTPR;                              /*!< Guard time and prescaler register                                     */
  __IO uint32_t  RTOR;                              /*!< Receiver timeout register                                             */
  __O  uint32_t  RQR;                               /*!< Request register                                                      */
  __I  uint32_t  ISR;                               /*!< Interrupt & status register                                           */
  __O  uint32_t  ICR;                               /*!< Interrupt flag clear register                                         */
  __I  uint32_t  RDR;                               /*!< Receive data register                                                 */
  __IO uint32_t  TDR;                               /*!< Transmit data register                                                */
  __IO uint32_t  PRESC;                             /*!< USART prescaler register                                              */
} USART1_Type;


/* ================================================================================ */
/* ================                      UART4                     ================ */
/* ================================================================================ */


/**
  * @brief Universal synchronous asynchronous receiver transmitter (UART4)
  */

typedef struct {                                    /*!< UART4 Structure                                                       */
  __IO uint32_t  CR1;                               /*!< Control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control register 2                                                    */
  __IO uint32_t  CR3;                               /*!< Control register 3                                                    */
  __IO uint32_t  BRR;                               /*!< Baud rate register                                                    */
  __IO uint32_t  GTPR;                              /*!< Guard time and prescaler register                                     */
  __IO uint32_t  RTOR;                              /*!< Receiver timeout register                                             */
  __O  uint32_t  RQR;                               /*!< Request register                                                      */
  __I  uint32_t  ISR;                               /*!< Interrupt & status register                                           */
  __O  uint32_t  ICR;                               /*!< Interrupt flag clear register                                         */
  __I  uint32_t  RDR;                               /*!< Receive data register                                                 */
  __IO uint32_t  TDR;                               /*!< Transmit data register                                                */
  __IO uint32_t  PRESC;                             /*!< USART prescaler register                                              */
} UART4_Type;


/* ================================================================================ */
/* ================                     LPUART1                    ================ */
/* ================================================================================ */


/**
  * @brief Universal synchronous asynchronous receiver transmitter (LPUART1)
  */

typedef struct {                                    /*!< LPUART1 Structure                                                     */
  __IO uint32_t  CR1;                               /*!< Control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control register 2                                                    */
  __IO uint32_t  CR3;                               /*!< Control register 3                                                    */
  __IO uint32_t  BRR;                               /*!< Baud rate register                                                    */
  __I  uint32_t  RESERVED[2];
  __O  uint32_t  RQR;                               /*!< Request register                                                      */
  __I  uint32_t  ISR;                               /*!< Interrupt & status register                                           */
  __O  uint32_t  ICR;                               /*!< Interrupt flag clear register                                         */
  __I  uint32_t  RDR;                               /*!< Receive data register                                                 */
  __IO uint32_t  TDR;                               /*!< Transmit data register                                                */
  __IO uint32_t  PRESC;                             /*!< Prescaler register                                                    */
} LPUART1_Type;


/* ================================================================================ */
/* ================                      SPI1                      ================ */
/* ================================================================================ */


/**
  * @brief Serial peripheral interface/Inter-IC sound (SPI1)
  */

typedef struct {                                    /*!< SPI1 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __IO uint32_t  DR;                                /*!< data register                                                         */
  __IO uint32_t  CRCPR;                             /*!< CRC polynomial register                                               */
  __I  uint32_t  RXCRCR;                            /*!< RX CRC register                                                       */
  __I  uint32_t  TXCRCR;                            /*!< TX CRC register                                                       */
  __IO uint32_t  I2SCFGR;                           /*!< configuration register                                                */
  __IO uint32_t  I2SPR;                             /*!< prescaler register                                                    */
} SPI1_Type;


/* ================================================================================ */
/* ================                      EXTI                      ================ */
/* ================================================================================ */


/**
  * @brief External interrupt/event controller (EXTI)
  */

typedef struct {                                    /*!< EXTI Structure                                                        */
  __IO uint32_t  IMR1;                              /*!< Interrupt mask register                                               */
  __IO uint32_t  EMR1;                              /*!< Event mask register                                                   */
  __IO uint32_t  RTSR1;                             /*!< Rising Trigger selection register                                     */
  __IO uint32_t  FTSR1;                             /*!< Falling Trigger selection register                                    */
  __IO uint32_t  SWIER1;                            /*!< Software interrupt event register                                     */
  __IO uint32_t  PR1;                               /*!< Pending register                                                      */
  __I  uint32_t  RESERVED[2];
  __IO uint32_t  IMR2;                              /*!< Interrupt mask register                                               */
  __IO uint32_t  EMR2;                              /*!< Event mask register                                                   */
  __IO uint32_t  RTSR2;                             /*!< Rising Trigger selection register                                     */
  __IO uint32_t  FTSR2;                             /*!< Falling Trigger selection register                                    */
  __IO uint32_t  SWIER2;                            /*!< Software interrupt event register                                     */
  __IO uint32_t  PR2;                               /*!< Pending register                                                      */
} EXTI_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief Real-time clock (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  __IO uint32_t  TR;                                /*!< time register                                                         */
  __IO uint32_t  DR;                                /*!< date register                                                         */
  __I  uint32_t  SSR;                               /*!< sub second register                                                   */
  __IO uint32_t  ICSR;                              /*!< initialization and status register                                    */
  __IO uint32_t  PRER;                              /*!< prescaler register                                                    */
  __IO uint32_t  WUTR;                              /*!< wakeup timer register                                                 */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __I  uint32_t  RESERVED[2];
  __O  uint32_t  WPR;                               /*!< write protection register                                             */
  __IO uint32_t  CALR;                              /*!< calibration register                                                  */
  __O  uint32_t  SHIFTR;                            /*!< shift control register                                                */
  __I  uint32_t  TSTR;                              /*!< time stamp time register                                              */
  __I  uint32_t  TSDR;                              /*!< time stamp date register                                              */
  __I  uint32_t  TSSSR;                             /*!< timestamp sub second register                                         */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  ALRMAR;                            /*!< alarm A register                                                      */
  __IO uint32_t  ALRMASSR;                          /*!< alarm A sub second register                                           */
  __IO uint32_t  ALRMBR;                            /*!< alarm B register                                                      */
  __IO uint32_t  ALRMBSSR;                          /*!< alarm B sub second register                                           */
  __I  uint32_t  SR;                                /*!< status register                                                       */
  __I  uint32_t  MISR;                              /*!< status register                                                       */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  SCR;                               /*!< status register                                                       */
} RTC_Type;


/* ================================================================================ */
/* ================                      DMA1                      ================ */
/* ================================================================================ */


/**
  * @brief DMA controller (DMA1)
  */

typedef struct {                                    /*!< DMA1 Structure                                                        */
  __I  uint32_t  ISR;                               /*!< interrupt status register                                             */
  __O  uint32_t  IFCR;                              /*!< DMA interrupt flag clear register                                     */
  __IO uint32_t  CCR1;                              /*!< DMA channel 1 configuration register                                  */
  __IO uint32_t  CNDTR1;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR1;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR1;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED;
  __IO uint32_t  CCR2;                              /*!< DMA channel 2 configuration register                                  */
  __IO uint32_t  CNDTR2;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR2;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR2;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CCR3;                              /*!< DMA channel 3 configuration register                                  */
  __IO uint32_t  CNDTR3;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR3;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR3;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  CCR4;                              /*!< DMA channel 3 configuration register                                  */
  __IO uint32_t  CNDTR4;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR4;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR4;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  CCR5;                              /*!< DMA channel 4 configuration register                                  */
  __IO uint32_t  CNDTR5;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR5;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR5;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  CCR6;                              /*!< DMA channel 5 configuration register                                  */
  __IO uint32_t  CNDTR6;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR6;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR6;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  CCR7;                              /*!< DMA channel 6 configuration register                                  */
  __IO uint32_t  CNDTR7;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR7;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR7;                             /*!< DMA channel x memory address register                                 */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  CCR8;                              /*!< DMA channel 7 configuration register                                  */
  __IO uint32_t  CNDTR8;                            /*!< channel x number of data to transfer register                         */
  __IO uint32_t  CPAR8;                             /*!< DMA channel x peripheral address register                             */
  __IO uint32_t  CMAR8;                             /*!< DMA channel x memory address register                                 */
} DMA1_Type;


/* ================================================================================ */
/* ================                     DMAMUX                     ================ */
/* ================================================================================ */


/**
  * @brief DMAMUX (DMAMUX)
  */

typedef struct {                                    /*!< DMAMUX Structure                                                      */
  __IO uint32_t  C0CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C1CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C2CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C3CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C4CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C5CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C6CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C7CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C8CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C9CR;                              /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C10CR;                             /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C11CR;                             /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C12CR;                             /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C13CR;                             /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C14CR;                             /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __IO uint32_t  C15CR;                             /*!< DMAMux - DMA request line multiplexer channel x control register      */
  __I  uint32_t  RESERVED[16];
  __I  uint32_t  CSR;                               /*!< DMAMUX request line multiplexer interrupt channel status register     */
  __O  uint32_t  CFR;                               /*!< DMAMUX request line multiplexer interrupt clear flag register         */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  RG0CR;                             /*!< DMAMux - DMA request generator channel x control register             */
  __IO uint32_t  RG1CR;                             /*!< DMAMux - DMA request generator channel x control register             */
  __IO uint32_t  RG2CR;                             /*!< DMAMux - DMA request generator channel x control register             */
  __IO uint32_t  RG3CR;                             /*!< DMAMux - DMA request generator channel x control register             */
  __I  uint32_t  RESERVED2[12];
  __I  uint32_t  RGSR;                              /*!< DMAMux - DMA request generator status register                        */
  __O  uint32_t  RGCFR;                             /*!< DMAMux - DMA request generator clear flag register                    */
} DMAMUX_Type;


/* ================================================================================ */
/* ================                     SYSCFG                     ================ */
/* ================================================================================ */


/**
  * @brief System configuration controller (SYSCFG)
  */

typedef struct {                                    /*!< SYSCFG Structure                                                      */
  __IO uint32_t  MEMRMP;                            /*!< Remap Memory register                                                 */
  __IO uint32_t  CFGR1;                             /*!< peripheral mode configuration register                                */
  __IO uint32_t  EXTICR1;                           /*!< external interrupt configuration register 1                           */
  __IO uint32_t  EXTICR2;                           /*!< external interrupt configuration register 2                           */
  __IO uint32_t  EXTICR3;                           /*!< external interrupt configuration register 3                           */
  __IO uint32_t  EXTICR4;                           /*!< external interrupt configuration register 4                           */
  __IO uint32_t  SCSR;                              /*!< CCM SRAM control and status register                                  */
  __IO uint32_t  CFGR2;                             /*!< configuration register 2                                              */
  __IO uint32_t  SWPR;                              /*!< SRAM Write protection register 1                                      */
  __O  uint32_t  SKR;                               /*!< SRAM2 Key Register                                                    */
} SYSCFG_Type;


/* ================================================================================ */
/* ================                     VREFBUF                    ================ */
/* ================================================================================ */


/**
  * @brief Voltage reference buffer (VREFBUF)
  */

typedef struct {                                    /*!< VREFBUF Structure                                                     */
  __IO uint32_t  VREFBUF_CSR;                       /*!< VREF_BUF Control and Status Register                                  */
  __IO uint32_t  VREFBUF_CCR;                       /*!< VREF_BUF Calibration Control Register                                 */
} VREFBUF_Type;


/* ================================================================================ */
/* ================                      COMP                      ================ */
/* ================================================================================ */


/**
  * @brief Comparator control and status register (COMP)
  */

typedef struct {                                    /*!< COMP Structure                                                        */
  __IO uint32_t  COMP_C1CSR;                        /*!< Comparator control/status register                                    */
  __IO uint32_t  COMP_C2CSR;                        /*!< Comparator control/status register                                    */
  __IO uint32_t  COMP_C3CSR;                        /*!< Comparator control/status register                                    */
  __I  uint16_t  RESERVED[3];
  __IO uint32_t  COMP_C4CSR;                        /*!< Comparator control/status register                                    */
} COMP_Type;


/* ================================================================================ */
/* ================                      OPAMP                     ================ */
/* ================================================================================ */


/**
  * @brief Operational amplifiers (OPAMP)
  */

typedef struct {                                    /*!< OPAMP Structure                                                       */
  __IO uint32_t  OPAMP1_CSR;                        /*!< OPAMP1 control/status register                                        */
  __IO uint32_t  OPAMP2_CSR;                        /*!< OPAMP2 control/status register                                        */
  __IO uint32_t  OPAMP3_CSR;                        /*!< OPAMP3 control/status register                                        */
  __I  uint32_t  RESERVED[3];
  __IO uint32_t  OPAMP1_TCMR;                       /*!< OPAMP1 control/status register                                        */
  __IO uint32_t  OPAMP2_TCMR;                       /*!< OPAMP2 control/status register                                        */
  __IO uint32_t  OPAMP3_TCMR;                       /*!< OPAMP3 control/status register                                        */
} OPAMP_Type;


/* ================================================================================ */
/* ================                      DAC1                      ================ */
/* ================================================================================ */


/**
  * @brief Digital-to-analog converter (DAC1)
  */

typedef struct {                                    /*!< DAC1 Structure                                                        */
  __IO uint32_t  DAC_CR;                            /*!< DAC control register                                                  */
  __O  uint32_t  DAC_SWTRGR;                        /*!< DAC software trigger register                                         */
  __IO uint32_t  DAC_DHR12R1;                       /*!< DAC channel1 12-bit right-aligned data holding register               */
  __IO uint32_t  DAC_DHR12L1;                       /*!< DAC channel1 12-bit left aligned data holding register                */
  __IO uint32_t  DAC_DHR8R1;                        /*!< DAC channel1 8-bit right aligned data holding register                */
  __IO uint32_t  DAC_DHR12R2;                       /*!< DAC channel2 12-bit right aligned data holding register               */
  __IO uint32_t  DAC_DHR12L2;                       /*!< DAC channel2 12-bit left aligned data holding register                */
  __IO uint32_t  DAC_DHR8R2;                        /*!< DAC channel2 8-bit right-aligned data holding register                */
  __IO uint32_t  DAC_DHR12RD;                       /*!< Dual DAC 12-bit right-aligned data holding register                   */
  __IO uint32_t  DAC_DHR12LD;                       /*!< DUAL DAC 12-bit left aligned data holding register                    */
  __IO uint32_t  DAC_DHR8RD;                        /*!< DUAL DAC 8-bit right aligned data holding register                    */
  __I  uint32_t  DAC_DOR1;                          /*!< DAC channel1 data output register                                     */
  __I  uint32_t  DAC_DOR2;                          /*!< DAC channel2 data output register                                     */
  __IO uint32_t  DAC_SR;                            /*!< DAC status register                                                   */
  __IO uint32_t  DAC_CCR;                           /*!< DAC calibration control register                                      */
  __IO uint32_t  DAC_MCR;                           /*!< DAC mode control register                                             */
  __IO uint32_t  DAC_SHSR1;                         /*!< DAC Sample and Hold sample time register 1                            */
  __IO uint32_t  DAC_SHSR2;                         /*!< DAC Sample and Hold sample time register 2                            */
  __IO uint32_t  DAC_SHHR;                          /*!< DAC Sample and Hold hold time register                                */
  __IO uint32_t  DAC_SHRR;                          /*!< DAC Sample and Hold refresh time register                             */
  __I  uint32_t  RESERVED[2];
  __IO uint32_t  DAC_STR1;                          /*!< Sawtooth register                                                     */
  __IO uint32_t  DAC_STR2;                          /*!< Sawtooth register                                                     */
  __IO uint32_t  DAC_STMODR;                        /*!< Sawtooth Mode register                                                */
} DAC1_Type;


/* ================================================================================ */
/* ================                      ADC1                      ================ */
/* ================================================================================ */


/**
  * @brief Analog-to-Digital Converter (ADC1)
  */

typedef struct {                                    /*!< ADC1 Structure                                                        */
  __IO uint32_t  ISR;                               /*!< interrupt and status register                                         */
  __IO uint32_t  IER;                               /*!< interrupt enable register                                             */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __IO uint32_t  CFGR;                              /*!< configuration register                                                */
  __IO uint32_t  CFGR2;                             /*!< configuration register                                                */
  __IO uint32_t  SMPR1;                             /*!< sample time register 1                                                */
  __IO uint32_t  SMPR2;                             /*!< sample time register 2                                                */
  __I  uint32_t  RESERVED;
  __IO uint32_t  TR1;                               /*!< watchdog threshold register 1                                         */
  __IO uint32_t  TR2;                               /*!< watchdog threshold register                                           */
  __IO uint32_t  TR3;                               /*!< watchdog threshold register 3                                         */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  SQR1;                              /*!< regular sequence register 1                                           */
  __IO uint32_t  SQR2;                              /*!< regular sequence register 2                                           */
  __IO uint32_t  SQR3;                              /*!< regular sequence register 3                                           */
  __IO uint32_t  SQR4;                              /*!< regular sequence register 4                                           */
  __I  uint32_t  DR;                                /*!< regular Data Register                                                 */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  JSQR;                              /*!< injected sequence register                                            */
  __I  uint32_t  RESERVED3[4];
  __IO uint32_t  OFR1;                              /*!< offset register 1                                                     */
  __IO uint32_t  OFR2;                              /*!< offset register 2                                                     */
  __IO uint32_t  OFR3;                              /*!< offset register 3                                                     */
  __IO uint32_t  OFR4;                              /*!< offset register 4                                                     */
  __I  uint32_t  RESERVED4[4];
  __I  uint32_t  JDR1;                              /*!< injected data register 1                                              */
  __I  uint32_t  JDR2;                              /*!< injected data register 2                                              */
  __I  uint32_t  JDR3;                              /*!< injected data register 3                                              */
  __I  uint32_t  JDR4;                              /*!< injected data register 4                                              */
  __I  uint32_t  RESERVED5[4];
  __IO uint32_t  AWD2CR;                            /*!< Analog Watchdog 2 Configuration Register                              */
  __IO uint32_t  AWD3CR;                            /*!< Analog Watchdog 3 Configuration Register                              */
  __I  uint32_t  RESERVED6[2];
  __IO uint32_t  DIFSEL;                            /*!< Differential Mode Selection Register 2                                */
  __IO uint32_t  CALFACT;                           /*!< Calibration Factors                                                   */
  __I  uint32_t  RESERVED7[2];
  __IO uint32_t  GCOMP;                             /*!< Gain compensation Register                                            */
} ADC1_Type;


/* ================================================================================ */
/* ================                  ADC12_Common                  ================ */
/* ================================================================================ */


/**
  * @brief Analog-to-Digital Converter (ADC12_Common)
  */

typedef struct {                                    /*!< ADC12_Common Structure                                                */
  __I  uint32_t  CSR;                               /*!< ADC Common status register                                            */
  __I  uint32_t  RESERVED;
  __IO uint32_t  CCR;                               /*!< ADC common control register                                           */
  __I  uint32_t  CDR;                               /*!< ADC common regular data register for dual and triple modes            */
} ADC12_Common_Type;


/* ================================================================================ */
/* ================                      FMAC                      ================ */
/* ================================================================================ */


/**
  * @brief Filter Math Accelerator (FMAC)
  */

typedef struct {                                    /*!< FMAC Structure                                                        */
  __IO uint32_t  X1BUFCFG;                          /*!< FMAC X1 Buffer Configuration register                                 */
  __IO uint32_t  X2BUFCFG;                          /*!< FMAC X2 Buffer Configuration register                                 */
  __IO uint32_t  YBUFCFG;                           /*!< FMAC Y Buffer Configuration register                                  */
  __IO uint32_t  PARAM;                             /*!< FMAC Parameter register                                               */
  __IO uint32_t  CR;                                /*!< FMAC Control register                                                 */
  __I  uint32_t  SR;                                /*!< FMAC Status register                                                  */
  __O  uint32_t  WDATA;                             /*!< FMAC Write Data register                                              */
  __I  uint32_t  RDATA;                             /*!< FMAC Read Data register                                               */
} FMAC_Type;


/* ================================================================================ */
/* ================                     CORDIC                     ================ */
/* ================================================================================ */


/**
  * @brief CORDIC Co-processor (CORDIC)
  */

typedef struct {                                    /*!< CORDIC Structure                                                      */
  __IO uint32_t  CSR;                               /*!< CORDIC Control Status register                                        */
  __IO uint32_t  WDATA;                             /*!< FMAC Write Data register                                              */
  __I  uint32_t  RDATA;                             /*!< FMAC Read Data register                                               */
} CORDIC_Type;


/* ================================================================================ */
/* ================                       SAI                      ================ */
/* ================================================================================ */


/**
  * @brief Serial audio interface (SAI)
  */

typedef struct {                                    /*!< SAI Structure                                                         */
  __I  uint32_t  RESERVED;
  __IO uint32_t  ACR1;                              /*!< AConfiguration register 1                                             */
  __IO uint32_t  ACR2;                              /*!< AConfiguration register 2                                             */
  __IO uint32_t  AFRCR;                             /*!< AFRCR                                                                 */
  __IO uint32_t  ASLOTR;                            /*!< ASlot register                                                        */
  __IO uint32_t  AIM;                               /*!< AInterrupt mask register2                                             */
  __IO uint32_t  ASR;                               /*!< AStatus register                                                      */
  __IO uint32_t  ACLRFR;                            /*!< AClear flag register                                                  */
  __IO uint32_t  ADR;                               /*!< AData register                                                        */
  __IO uint32_t  BCR1;                              /*!< BConfiguration register 1                                             */
  __IO uint32_t  BCR2;                              /*!< BConfiguration register 2                                             */
  __IO uint32_t  BFRCR;                             /*!< BFRCR                                                                 */
  __IO uint32_t  BSLOTR;                            /*!< BSlot register                                                        */
  __IO uint32_t  BIM;                               /*!< BInterrupt mask register2                                             */
  __I  uint32_t  BSR;                               /*!< BStatus register                                                      */
  __O  uint32_t  BCLRFR;                            /*!< BClear flag register                                                  */
  __IO uint32_t  BDR;                               /*!< BData register                                                        */
  __IO uint32_t  PDMCR;                             /*!< PDM control register                                                  */
  __IO uint32_t  PDMDLY;                            /*!< PDM delay register                                                    */
} SAI_Type;


/* ================================================================================ */
/* ================                      TAMP                      ================ */
/* ================================================================================ */


/**
  * @brief Tamper and backup registers (TAMP)
  */

typedef struct {                                    /*!< TAMP Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __I  uint32_t  RESERVED;
  __IO uint32_t  FLTCR;                             /*!< TAMP filter control register                                          */
  __I  uint32_t  RESERVED1[7];
  __IO uint32_t  IER;                               /*!< TAMP interrupt enable register                                        */
  __I  uint32_t  SR;                                /*!< TAMP status register                                                  */
  __I  uint32_t  MISR;                              /*!< TAMP masked interrupt status register                                 */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  SCR;                               /*!< TAMP status clear register                                            */
  __I  uint32_t  RESERVED3[48];
  __IO uint32_t  BKP0R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP1R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP2R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP3R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP4R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP5R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP6R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP7R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP8R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP9R;                             /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP10R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP11R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP12R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP13R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP14R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP15R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP16R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP17R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP18R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP19R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP20R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP21R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP22R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP23R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP24R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP25R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP26R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP27R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP28R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP29R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP30R;                            /*!< TAMP backup register                                                  */
  __IO uint32_t  BKP31R;                            /*!< TAMP backup register                                                  */
} TAMP_Type;


/* ================================================================================ */
/* ================                       FPU                      ================ */
/* ================================================================================ */


/**
  * @brief Floting point unit (FPU)
  */

typedef struct {                                    /*!< FPU Structure                                                         */
  __IO uint32_t  FPCCR;                             /*!< Floating-point context control register                               */
  __IO uint32_t  FPCAR;                             /*!< Floating-point context address register                               */
  __IO uint32_t  FPSCR;                             /*!< Floating-point status control register                                */
} FPU_Type;


/* ================================================================================ */
/* ================                       MPU                      ================ */
/* ================================================================================ */


/**
  * @brief Memory protection unit (MPU)
  */

typedef struct {                                    /*!< MPU Structure                                                         */
  __I  uint32_t  TYPER;                             /*!< MPU type register                                                     */
  __I  uint32_t  CTRL;                              /*!< MPU control register                                                  */
  __IO uint32_t  RNR;                               /*!< MPU region number register                                            */
  __IO uint32_t  RBAR;                              /*!< MPU region base address register                                      */
  __IO uint32_t  RASR;                              /*!< MPU region attribute and size register                                */
} MPU_Type;


/* ================================================================================ */
/* ================                       STK                      ================ */
/* ================================================================================ */


/**
  * @brief SysTick timer (STK)
  */

typedef struct {                                    /*!< STK Structure                                                         */
  __IO uint32_t  CTRL;                              /*!< SysTick control and status register                                   */
  __IO uint32_t  LOAD;                              /*!< SysTick reload value register                                         */
  __IO uint32_t  VAL;                               /*!< SysTick current value register                                        */
  __IO uint32_t  CALIB;                             /*!< SysTick calibration value register                                    */
} STK_Type;


/* ================================================================================ */
/* ================                       SCB                      ================ */
/* ================================================================================ */


/**
  * @brief System control block (SCB)
  */

typedef struct {                                    /*!< SCB Structure                                                         */
  __I  uint32_t  CPUID;                             /*!< CPUID base register                                                   */
  __IO uint32_t  ICSR;                              /*!< Interrupt control and state register                                  */
  __IO uint32_t  VTOR;                              /*!< Vector table offset register                                          */
  __IO uint32_t  AIRCR;                             /*!< Application interrupt and reset control register                      */
  __IO uint32_t  SCR;                               /*!< System control register                                               */
  __IO uint32_t  CCR;                               /*!< Configuration and control register                                    */
  __IO uint32_t  SHPR1;                             /*!< System handler priority registers                                     */
  __IO uint32_t  SHPR2;                             /*!< System handler priority registers                                     */
  __IO uint32_t  SHPR3;                             /*!< System handler priority registers                                     */
  __IO uint32_t  SHCRS;                             /*!< System handler control and state register                             */
  __IO uint32_t  CFSR_UFSR_BFSR_MMFSR;              /*!< Configurable fault status register                                    */
  __IO uint32_t  HFSR;                              /*!< Hard fault status register                                            */
  __I  uint32_t  RESERVED;
  __IO uint32_t  MMFAR;                             /*!< Memory management fault address register                              */
  __IO uint32_t  BFAR;                              /*!< Bus fault address register                                            */
  __IO uint32_t  AFSR;                              /*!< Auxiliary fault status register                                       */
} SCB_Type;


/* ================================================================================ */
/* ================                      NVIC                      ================ */
/* ================================================================================ */


/**
  * @brief Nested Vectored Interrupt Controller (NVIC)
  */

typedef struct {                                    /*!< NVIC Structure                                                        */
  __IO uint32_t  ISER0;                             /*!< Interrupt Set-Enable Register                                         */
  __IO uint32_t  ISER1;                             /*!< Interrupt Set-Enable Register                                         */
  __IO uint32_t  ISER2;                             /*!< Interrupt Set-Enable Register                                         */
  __IO uint32_t  ISER3;                             /*!< Interrupt Set-Enable Register                                         */
  __I  uint32_t  RESERVED[28];
  __IO uint32_t  ICER0;                             /*!< Interrupt Clear-Enable Register                                       */
  __IO uint32_t  ICER1;                             /*!< Interrupt Clear-Enable Register                                       */
  __IO uint32_t  ICER2;                             /*!< Interrupt Clear-Enable Register                                       */
  __IO uint32_t  ICER3;                             /*!< Interrupt Clear-Enable Register                                       */
  __I  uint32_t  RESERVED1[28];
  __IO uint32_t  ISPR0;                             /*!< Interrupt Set-Pending Register                                        */
  __IO uint32_t  ISPR1;                             /*!< Interrupt Set-Pending Register                                        */
  __IO uint32_t  ISPR2;                             /*!< Interrupt Set-Pending Register                                        */
  __IO uint32_t  ISPR3;                             /*!< Interrupt Set-Pending Register                                        */
  __I  uint32_t  RESERVED2[28];
  __IO uint32_t  ICPR0;                             /*!< Interrupt Clear-Pending Register                                      */
  __IO uint32_t  ICPR1;                             /*!< Interrupt Clear-Pending Register                                      */
  __IO uint32_t  ICPR2;                             /*!< Interrupt Clear-Pending Register                                      */
  __IO uint32_t  ICPR3;                             /*!< Interrupt Clear-Pending Register                                      */
  __I  uint32_t  RESERVED3[28];
  __I  uint32_t  IABR0;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  IABR1;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  IABR2;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  IABR3;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  RESERVED4[60];
  __IO uint32_t  IPR0;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR1;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR2;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR3;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR4;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR5;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR6;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR7;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR8;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR9;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR10;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR11;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR12;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR13;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR14;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR15;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR16;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR17;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR18;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR19;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR20;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR21;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR22;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR23;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR24;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR25;                             /*!< Interrupt Priority Register                                           */
  __I  uint32_t  RESERVED5[678];
  __IO uint32_t  STIR;                              /*!< Software trigger interrupt register                                   */
} NVIC_Type;


/* ================================================================================ */
/* ================                    FPU_CPACR                   ================ */
/* ================================================================================ */


/**
  * @brief Floating point unit CPACR (FPU_CPACR)
  */

typedef struct {                                    /*!< FPU_CPACR Structure                                                   */
  __IO uint32_t  CPACR;                             /*!< Coprocessor access control register                                   */
} FPU_CPACR_Type;


/* ================================================================================ */
/* ================                    SCB_ACTRL                   ================ */
/* ================================================================================ */


/**
  * @brief System control block ACTLR (SCB_ACTRL)
  */

typedef struct {                                    /*!< SCB_ACTRL Structure                                                   */
  __IO uint32_t  ACTRL;                             /*!< Auxiliary control register                                            */
} SCB_ACTRL_Type;


/* ================================================================================ */
/* ================                      FDCAN                     ================ */
/* ================================================================================ */


/**
  * @brief FDCAN (FDCAN)
  */

typedef struct {                                    /*!< FDCAN Structure                                                       */
  __I  uint32_t  CREL;                              /*!< FDCAN Core Release Register                                           */
  __I  uint32_t  ENDN;                              /*!< FDCAN Core Release Register                                           */
  __I  uint32_t  RESERVED;
  __IO uint32_t  DBTP;                              /*!< This register is only writable if bits CCCR.CCE and CCCR.INIT
                                                         are set. The CAN bit time may be programed in the range of 4
                                                          to 25 time quanta. The CAN time quantum may be programmed in
                                                          the range of 1 to 1024 FDCAN clock periods. tq = (DBRP + 1)
                                                          FDCAN clock period. DTSEG1 is the sum of Prop_Seg and Phase_Seg1.
                                                          DTSEG2 is Phase_Seg2. Therefore the length of the bit time is
                                                          (programmed values) [DTSEG1 + DTSEG2 + 3] tq or (functional
                                                          values) [Sync_Seg + Prop_Seg + Phase_Seg1 + Phase_Seg2] tq.
                                                          The Information Pro                                                  */
  __IO uint32_t  TEST;                              /*!< Write access to the Test Register has to be enabled by setting
                                                         bit CCCR[TEST] to 1 . All Test Register functions are set to
                                                          their reset values when bit CCCR[TEST] is reset. Loop Back mode
                                                          and software control of Tx pin FDCANx_TX are hardware test modes.
                                                          Programming TX differently from 00 may disturb the message transfer
                                                          on the CAN bus.                                                      */
  __IO uint32_t  RWD;                               /*!< The RAM Watchdog monitors the READY output of the Message RAM.
                                                         A Message RAM access starts the Message RAM Watchdog Counter
                                                          with the value configured by the RWD[WDC] bits. The counter
                                                          is reloaded with RWD[WDC] bits when the Message RAM signals
                                                          successful completion by activating its READY output. In case
                                                          there is no response from the Message RAM until the counter
                                                          has counted down to 0, the counter stops and interrupt flag
                                                          IR[WDI] bit is set. The RAM Watchdog Counter is clocked by the
                                                          fdcan_pclk clock.                                                    */
  __IO uint32_t  CCCR;                              /*!< For details about setting and resetting of single bits see Software
                                                         initialization.                                                       */
  __IO uint32_t  NBTP;                              /*!< FDCAN_NBTP                                                            */
  __IO uint32_t  TSCC;                              /*!< FDCAN Timestamp Counter Configuration Register                        */
  __I  uint32_t  TSCV;                              /*!< FDCAN Timestamp Counter Value Register                                */
  __IO uint32_t  TOCC;                              /*!< FDCAN Timeout Counter Configuration Register                          */
  __I  uint32_t  TOCV;                              /*!< FDCAN Timeout Counter Value Register                                  */
  __I  uint32_t  RESERVED1[4];
  __I  uint32_t  ECR;                               /*!< FDCAN Error Counter Register                                          */
  __IO uint32_t  PSR;                               /*!< FDCAN Protocol Status Register                                        */
  __IO uint32_t  TDCR;                              /*!< FDCAN Transmitter Delay Compensation Register                         */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  IR;                                /*!< The flags are set when one of the listed conditions is detected
                                                         (edge-sensitive). The flags remain set until the Host clears
                                                          them. A flag is cleared by writing a 1 to the corresponding
                                                          bit position. Writing a 0 has no effect. A hard reset will clear
                                                          the register. The configuration of IE controls whether an interrupt
                                                          is generated. The configuration of ILS controls on which interrupt
                                                          line an interrupt is signaled.                                       */
  __IO uint32_t  IE;                                /*!< The settings in the Interrupt Enable register determine which
                                                         status changes in the Interrupt Register will be signaled on
                                                          an interrupt line.                                                   */
  __IO uint32_t  ILS;                               /*!< The Interrupt Line Select register assigns an interrupt generated
                                                         by a specific interrupt flag from the Interrupt Register to
                                                          one of the two module interrupt lines. For interrupt generation
                                                          the respective interrupt line has to be enabled via ILE[EINT0]
                                                          and ILE[EINT1].                                                      */
  __IO uint32_t  ILE;                               /*!< Each of the two interrupt lines to the CPU can be enabled/disabled
                                                         separately by programming bits EINT0 and EINT1.                       */
  __I  uint32_t  RESERVED3[8];
  __IO uint32_t  RXGFC;                             /*!< Global settings for Message ID filtering. The Global Filter
                                                         Configuration controls the filter path for standard and extended
                                                          messages as described in Figure706: Standard Message ID filter
                                                          path and Figure707: Extended Message ID filter path.                 */
  __IO uint32_t  XIDAM;                             /*!< FDCAN Extended ID and Mask Register                                   */
  __I  uint32_t  HPMS;                              /*!< This register is updated every time a Message ID filter element
                                                         configured to generate a priority event match. This can be used
                                                          to monitor the status of incoming high priority messages and
                                                          to enable fast access to these messages.                             */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  RXF0S;                             /*!< FDCAN Rx FIFO 0 Status Register                                       */
  __IO uint32_t  RXF0A;                             /*!< CAN Rx FIFO 0 Acknowledge Register                                    */
  __I  uint32_t  RXF1S;                             /*!< FDCAN Rx FIFO 1 Status Register                                       */
  __IO uint32_t  RXF1A;                             /*!< FDCAN Rx FIFO 1 Acknowledge Register                                  */
  __I  uint32_t  RESERVED5[8];
  __IO uint32_t  TXBC;                              /*!< FDCAN Tx Buffer Configuration Register                                */
  __I  uint32_t  TXFQS;                             /*!< The Tx FIFO/Queue status is related to the pending Tx requests
                                                         listed in register TXBRP. Therefore the effect of Add/Cancellation
                                                          requests may be delayed due to a running Tx scan (TXBRP not
                                                          yet updated).                                                        */
  __I  uint32_t  TXBRP;                             /*!< FDCAN Tx Buffer Request Pending Register                              */
  __IO uint32_t  TXBAR;                             /*!< FDCAN Tx Buffer Add Request Register                                  */
  __IO uint32_t  TXBCR;                             /*!< FDCAN Tx Buffer Cancellation Request Register                         */
  __I  uint32_t  TXBTO;                             /*!< FDCAN Tx Buffer Transmission Occurred Register                        */
  __I  uint32_t  TXBCF;                             /*!< FDCAN Tx Buffer Cancellation Finished Register                        */
  __IO uint32_t  TXBTIE;                            /*!< FDCAN Tx Buffer Transmission Interrupt Enable Register                */
  __IO uint32_t  TXBCIE;                            /*!< FDCAN Tx Buffer Cancellation Finished Interrupt Enable Register       */
  __I  uint32_t  TXEFS;                             /*!< FDCAN Tx Event FIFO Status Register                                   */
  __IO uint32_t  TXEFA;                             /*!< FDCAN Tx Event FIFO Acknowledge Register                              */
  __I  uint32_t  RESERVED6[5];
  __IO uint32_t  CKDIV;                             /*!< FDCAN CFG clock divider register                                      */
} FDCAN_Type;


/* ================================================================================ */
/* ================                      UCPD1                     ================ */
/* ================================================================================ */


/**
  * @brief UCPD1 (UCPD1)
  */

typedef struct {                                    /*!< UCPD1 Structure                                                       */
  __IO uint32_t  CFG1;                              /*!< UCPD configuration register 1                                         */
  __IO uint32_t  CFG2;                              /*!< UCPD configuration register 2                                         */
  __I  uint32_t  RESERVED;
  __IO uint32_t  CR;                                /*!< UCPD configuration register 2                                         */
  __IO uint32_t  IMR;                               /*!< UCPD Interrupt Mask Register                                          */
  __IO uint32_t  SR;                                /*!< UCPD Status Register                                                  */
  __IO uint32_t  ICR;                               /*!< UCPD Interrupt Clear Register                                         */
  __IO uint32_t  TX_ORDSET;                         /*!< UCPD Tx Ordered Set Type Register                                     */
  __IO uint32_t  TX_PAYSZ;                          /*!< UCPD Tx Paysize Register                                              */
  __IO uint32_t  TXDR;                              /*!< UCPD Tx Data Register                                                 */
  __I  uint32_t  RX_ORDSET;                         /*!< UCPD Rx Ordered Set Register                                          */
  __I  uint32_t  RX_PAYSZ;                          /*!< UCPD Rx Paysize Register                                              */
  __I  uint32_t  RXDR;                              /*!< UCPD Rx Data Register                                                 */
  __IO uint32_t  RX_ORDEXT1;                        /*!< UCPD Rx Ordered Set Extension Register 1                              */
  __IO uint32_t  RX_ORDEXT2;                        /*!< UCPD Rx Ordered Set Extension Register 2                              */
} UCPD1_Type;


/* ================================================================================ */
/* ================                  USB_FS_device                 ================ */
/* ================================================================================ */


/**
  * @brief USB_FS_device (USB_FS_device)
  */

typedef struct {                                    /*!< USB_FS_device Structure                                               */
  __IO uint32_t  EP0R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP1R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP2R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP3R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP4R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP5R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP6R;                              /*!< USB endpoint n register                                               */
  __IO uint32_t  EP7R;                              /*!< USB endpoint n register                                               */
  __I  uint32_t  RESERVED[8];
  __IO uint32_t  CNTR;                              /*!< USB control register                                                  */
  __IO uint32_t  ISTR;                              /*!< USB interrupt status register                                         */
  __I  uint32_t  FNR;                               /*!< USB frame number register                                             */
  __IO uint32_t  DADDR;                             /*!< USB device address                                                    */
  __IO uint32_t  BTABLE;                            /*!< Buffer table address                                                  */
} USB_FS_device_Type;


/* ================================================================================ */
/* ================                       CRS                      ================ */
/* ================================================================================ */


/**
  * @brief CRS (CRS)
  */

typedef struct {                                    /*!< CRS Structure                                                         */
  __IO uint32_t  CR;                                /*!< CRS control register                                                  */
  __IO uint32_t  CFGR;                              /*!< This register can be written only when the frequency error counter
                                                         is disabled (CEN bit is cleared in CRS_CR). When the counter
                                                          is enabled, this register is write-protected.                        */
  __I  uint32_t  ISR;                               /*!< CRS interrupt and status register                                     */
  __IO uint32_t  ICR;                               /*!< CRS interrupt flag clear register                                     */
} CRS_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define CRC_BASE                        0x40023000UL
#define WWDG_BASE                       0x40002C00UL
#define IWDG_BASE                       0x40003000UL
#define I2C1_BASE                       0x40005400UL
#define I2C2_BASE                       0x40005800UL
#define I2C3_BASE                       0x40007800UL
#define FLASH_BASE                      0x40022000UL
#define DBGMCU_BASE                     0xE0042000UL
#define RCC_BASE                        0x40021000UL
#define PWR_BASE                        0x40007000UL
#define RNG_BASE                        0x50060800UL
#define AES_BASE                        0x50060000UL
#define GPIOA_BASE                      0x48000000UL
#define GPIOB_BASE                      0x48000400UL
#define GPIOC_BASE                      0x48000800UL
#define GPIOD_BASE                      0x48000C00UL
#define GPIOE_BASE                      0x48001000UL
#define GPIOF_BASE                      0x48001400UL
#define GPIOG_BASE                      0x48001800UL
#define TIM15_BASE                      0x40014000UL
#define TIM16_BASE                      0x40014400UL
#define TIM17_BASE                      0x40014800UL
#define TIM1_BASE                       0x40012C00UL
#define TIM8_BASE                       0x40013400UL
#define TIM2_BASE                       0x40000000UL
#define TIM3_BASE                       0x40000400UL
#define TIM4_BASE                       0x40000800UL
#define TIM6_BASE                       0x40001000UL
#define TIM7_BASE                       0x40001400UL
#define LPTIMER1_BASE                   0x40007C00UL
#define USART1_BASE                     0x40013800UL
#define USART2_BASE                     0x40004400UL
#define USART3_BASE                     0x40004800UL
#define UART4_BASE                      0x40004C00UL
#define LPUART1_BASE                    0x40008000UL
#define SPI1_BASE                       0x40013000UL
#define SPI3_BASE                       0x40003C00UL
#define SPI2_BASE                       0x40003800UL
#define EXTI_BASE                       0x40010400UL
#define RTC_BASE                        0x40002800UL
#define DMA1_BASE                       0x40020000UL
#define DMA2_BASE                       0x40020400UL
#define DMAMUX_BASE                     0x40020800UL
#define SYSCFG_BASE                     0x40010000UL
#define VREFBUF_BASE                    0x40010030UL
#define COMP_BASE                       0x40010200UL
#define OPAMP_BASE                      0x40010300UL
#define DAC1_BASE                       0x50000800UL
#define DAC2_BASE                       0x50000C00UL
#define DAC3_BASE                       0x50001000UL
#define DAC4_BASE                       0x50001400UL
#define ADC1_BASE                       0x50000000UL
#define ADC2_BASE                       0x50000100UL
#define ADC12_Common_BASE               0x50000300UL // Mistake in SVD File corrected here.
#define ADC345_Common_BASE              0x50000700UL
#define FMAC_BASE                       0x40021400UL
#define CORDIC_BASE                     0x40020C00UL
#define SAI_BASE                        0x40015400UL
#define TAMP_BASE                       0x40002400UL
#define FPU_BASE                        0xE000EF34UL
#define MPU_BASE                        0xE000E084UL
#define STK_BASE                        0xE000E010UL
#define SCB_BASE                        0xE000E040UL
#define NVIC_BASE                       0xE000E100UL
#define FPU_CPACR_BASE                  0xE000EF08UL
#define SCB_ACTRL_BASE                  0xE000E008UL
#define FDCAN_BASE                      0x4000A400UL
#define FDCAN1_BASE                     0x40006400UL
#define UCPD1_BASE                      0x4000A000UL
#define USB_FS_device_BASE              0x40005C00UL
#define CRS_BASE                        0x40002000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define CRC                             ((CRC_Type                *) CRC_BASE)
#define WWDG                            ((WWDG_Type               *) WWDG_BASE)
#define IWDG                            ((IWDG_Type               *) IWDG_BASE)
#define I2C1                            ((I2C1_Type               *) I2C1_BASE)
#define I2C2                            ((I2C1_Type               *) I2C2_BASE)
#define I2C3                            ((I2C1_Type               *) I2C3_BASE)
#define FLASH                           ((FLASH_Type              *) FLASH_BASE)
#define DBGMCU                          ((DBGMCU_Type             *) DBGMCU_BASE)
#define RCC                             ((RCC_Type                *) RCC_BASE)
#define PWR                             ((PWR_Type                *) PWR_BASE)
#define RNG                             ((RNG_Type                *) RNG_BASE)
#define AES                             ((AES_Type                *) AES_BASE)
#define GPIOA                           ((GPIOA_Type              *) GPIOA_BASE)
#define GPIOB                           ((GPIOB_Type              *) GPIOB_BASE)
#define GPIOC                           ((GPIOC_Type              *) GPIOC_BASE)
#define GPIOD                           ((GPIOC_Type              *) GPIOD_BASE)
#define GPIOE                           ((GPIOC_Type              *) GPIOE_BASE)
#define GPIOF                           ((GPIOC_Type              *) GPIOF_BASE)
#define GPIOG                           ((GPIOC_Type              *) GPIOG_BASE)
#define TIM15                           ((TIM15_Type              *) TIM15_BASE)
#define TIM16                           ((TIM16_Type              *) TIM16_BASE)
#define TIM17                           ((TIM16_Type              *) TIM17_BASE)
#define TIM1                            ((TIM1_Type               *) TIM1_BASE)
#define TIM8                            ((TIM1_Type               *) TIM8_BASE)
#define TIM2                            ((TIM2_Type               *) TIM2_BASE)
#define TIM3                            ((TIM2_Type               *) TIM3_BASE)
#define TIM4                            ((TIM2_Type               *) TIM4_BASE)
#define TIM6                            ((TIM6_Type               *) TIM6_BASE)
#define TIM7                            ((TIM6_Type               *) TIM7_BASE)
#define LPTIMER1                        ((LPTIMER1_Type           *) LPTIMER1_BASE)
#define USART1                          ((USART1_Type             *) USART1_BASE)
#define USART2                          ((USART1_Type             *) USART2_BASE)
#define USART3                          ((USART1_Type             *) USART3_BASE)
#define UART4                           ((UART4_Type              *) UART4_BASE)
#define LPUART1                         ((LPUART1_Type            *) LPUART1_BASE)
#define SPI1                            ((SPI1_Type               *) SPI1_BASE)
#define SPI3                            ((SPI1_Type               *) SPI3_BASE)
#define SPI2                            ((SPI1_Type               *) SPI2_BASE)
#define EXTI                            ((EXTI_Type               *) EXTI_BASE)
#define RTC                             ((RTC_Type                *) RTC_BASE)
#define DMA1                            ((DMA1_Type               *) DMA1_BASE)
#define DMA2                            ((DMA1_Type               *) DMA2_BASE)
#define DMAMUX                          ((DMAMUX_Type             *) DMAMUX_BASE)
#define SYSCFG                          ((SYSCFG_Type             *) SYSCFG_BASE)
#define VREFBUF                         ((VREFBUF_Type            *) VREFBUF_BASE)
#define COMP                            ((COMP_Type               *) COMP_BASE)
#define OPAMP                           ((OPAMP_Type              *) OPAMP_BASE)
#define DAC1                            ((DAC1_Type               *) DAC1_BASE)
#define DAC2                            ((DAC1_Type               *) DAC2_BASE)
#define DAC3                            ((DAC1_Type               *) DAC3_BASE)
#define DAC4                            ((DAC1_Type               *) DAC4_BASE)
#define ADC1                            ((ADC1_Type               *) ADC1_BASE)
#define ADC2                            ((ADC1_Type               *) ADC2_BASE)
#define ADC12_Common                    ((ADC12_Common_Type       *) ADC12_Common_BASE)
#define ADC345_Common                   ((ADC12_Common_Type       *) ADC345_Common_BASE)
#define FMAC                            ((FMAC_Type               *) FMAC_BASE)
#define CORDIC                          ((CORDIC_Type             *) CORDIC_BASE)
#define SAI                             ((SAI_Type                *) SAI_BASE)
#define TAMP                            ((TAMP_Type               *) TAMP_BASE)
#define FPU                             ((FPU_Type                *) FPU_BASE)
#define MPU                             ((MPU_Type                *) MPU_BASE)
#define STK                             ((STK_Type                *) STK_BASE)
#define SCB                             ((SCB_Type                *) SCB_BASE)
#define NVIC                            ((NVIC_Type               *) NVIC_BASE)
#define FPU_CPACR                       ((FPU_CPACR_Type          *) FPU_CPACR_BASE)
#define SCB_ACTRL                       ((SCB_ACTRL_Type          *) SCB_ACTRL_BASE)
#define FDCAN                           ((FDCAN_Type              *) FDCAN_BASE)
#define FDCAN1                          ((FDCAN_Type              *) FDCAN1_BASE)
#define UCPD1                           ((UCPD1_Type              *) UCPD1_BASE)
#define USB_FS_device                   ((USB_FS_device_Type      *) USB_FS_device_BASE)
#define CRS                             ((CRS_Type                *) CRS_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group STM32G431xx */
/** @} */ /* End of group (null) */

#ifdef __cplusplus
}
#endif


#endif  /* STM32G431xx_H */

