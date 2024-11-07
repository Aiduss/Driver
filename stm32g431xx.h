/* ----------------------------------------------------------------------------
 * STM32G431RB Device Driver
 * ----------------------------------------------------------------------------
 * Version: 0.0.1
 * Release Date: September 2024
 * Current Date: October   2024
 * Author: Eximus Studio
 * ----------------------------------------------------------------------------
 * Description for category 2 device |rm 1.4:
 *     - FLASH 		: 128/64/32K single bank
 *     - SRAM1 		: 16K, parity check on the whole SRAM1
 *     - SRAM2 		: 6K, no parity check
 *     - CCM SRAM 	: 10, parity check on the whole CCM SRAM
 *     - CRS		: Yes
 *     - DMA		: 2 DMA controllers / both 6 channels
 *
 * To Do List:
 *     todo
 *
 * Notes:
 *     - pGPIOA->MODER = 25; is the same as  *(0x40020000 + 0x00) = 25
 *     - GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*)0x40020000
 *
 * ----------------------------------------------------------------------------
 */

#ifndef INC_STM32G431XX_H_
#define INC_STM32G431XX_H_

#include <stdint.h>


/* ----------------------------------------------------------------------------- */
/* ---[ STM32-G431RB MEMORY BASE ADDRESSES & BOUNDARIES ]--- */
/* ----------------------------------------------------------------------------- */
#define FLASH_BASEADDR			0x08000000U		// |rm 2.2.2
#define SRAM1_BASEADDR			0x20000000U		// |rm 2.2.2
#define SRAM2_BASEADDR			0x20014000U		// |rm 2.2.2
#define ROM_BASEADDR			0x1FFF0000U		// |rm 2.2.2
#define SRAM 					SRAM1_BASEADDR
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
/* ---[ STM32-G431RB PERIPHERALS BASE ADDRESSES from MEMORY MAP  ]--- */
/* ----------------------------------------------------------------------------- */
#define PERIPHERAL_BASEADDR			0x4000000UL			// |rm 2.2.2
#define APB1PERIPH_BASEADDR			PERIPHERAL_BASEADDR	// |rm 2.2.2
#define APB2PERIPH_BASEADDR			0x40010000UL			// |rm 2.2.2
#define AHB1PERIPH_BASEADDR			0x40020000UL			// |rm 2.2.2
#define AHB2PERIPH_BASEADDR			0x48000000UL			// |rm 2.2.2
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
/* ---[ AHB1 BUS PERIPHERAL BASE ADDRESSES |rm 2.2.2 ]--- */
/* ----------------------------------------------------------------------------- */
#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)	// Direct memory access controller 1
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)	// Direct memory access controller 2
#define DMAMUX_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)	// DMA request multiplexer
#define CORDIC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)	// CORDIC coprocessor for trigonometric functions
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)	// Reset and clock control + 0x1000
#define FMAC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)	// Filter math accelerator
#define FINT_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)	// Flash Interface
#define RES1_AHB1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)	// Reserved
#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3000)	// Cyclic redundancy check calculation unit
#define RES2_AHB1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3400)	// Reserved
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
/* ---[ AHB2 BUS PERIPHERAL BASE ADDRESSES ]--- */
/* ----------------------------------------------------------------------------- */
#define GPIOA_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1800)
#define ADC1_2_BASEADDR             (AHB2PERIPH_BASEADDR + 0x0000)  // ADC1 and ADC2
#define ADC3_5_BASEADDR             (AHB2PERIPH_BASEADDR + 0x0400)  // ADC3, ADC4, and ADC5
#define DAC1_BASEADDR               (AHB2PERIPH_BASEADDR + 0x0800)  // Digital-to-Analog Converter 1
#define DAC2_BASEADDR               (AHB2PERIPH_BASEADDR + 0x0C00)  // Digital-to-Analog Converter 2
#define DAC3_BASEADDR               (AHB2PERIPH_BASEADDR + 0x1000)  // Digital-to-Analog Converter 3
#define DAC4_BASEADDR               (AHB2PERIPH_BASEADDR + 0x1400)  // Digital-to-Analog Converter 4
#define AES_BASEADDR                (AHB2PERIPH_BASEADDR + 0x5000)  // Advanced Encryption Standard
#define RNG_BASEADDR                (AHB2PERIPH_BASEADDR + 0x5006)  // Random Number Generator
#define FMC_BASEADDR                (AHB2PERIPH_BASEADDR + 0x5006)  // Flexible Memory Controller
#define AHB2_RESERVED_BASEADDR1     (AHB2PERIPH_BASEADDR + 0x4800)  // Reserved (127 MB)
#define AHB2_RESERVED_BASEADDR2     (AHB2PERIPH_BASEADDR + 0x5000)  // Reserved (377 KB)
#define AHB2_RESERVED_BASEADDR3     (AHB2PERIPH_BASEADDR + 0x5006)  // Reserved (256 MB)
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
/* ---[ APB1 BUS PERIPHERAL BASE ADDRESSES ]--- */
/* ----------------------------------------------------------------------------- */
#define TIM2_BASEADDR               (APB1PERIPH_BASEADDR + 0x0000)  // Timer 2
#define TIM3_BASEADDR               (APB1PERIPH_BASEADDR + 0x0400)  // Timer 3
#define TIM4_BASEADDR               (APB1PERIPH_BASEADDR + 0x0800)  // Timer 4
#define TIM5_BASEADDR               (APB1PERIPH_BASEADDR + 0x0C00)  // Timer 5
#define TIM6_BASEADDR               (APB1PERIPH_BASEADDR + 0x1000)  // Timer 6
#define TIM7_BASEADDR               (APB1PERIPH_BASEADDR + 0x1400)  // Timer 7
#define LPTIM1_BASEADDR             (APB1PERIPH_BASEADDR + 0x7C00)  // Low-Power Timer 1
#define TAMP_BASEADDR               (APB1PERIPH_BASEADDR + 0x2000)  // Tamper
#define CRS_BASEADDR                (APB1PERIPH_BASEADDR + 0x2400)  // Clock Recovery System
#define RTC_BASEADDR                (APB1PERIPH_BASEADDR + 0x2800)  // Real-Time Clock and Backup registers
#define WWDG_BASEADDR               (APB1PERIPH_BASEADDR + 0x2C00)  // Window Watchdog
#define IWDG_BASEADDR               (APB1PERIPH_BASEADDR + 0x3000)  // Independent Watchdog
#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0x3800)  // SPI/I2S 2
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)  // SPI/I2S 3
#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x4400)  // USART 2
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 0x4800)  // USART 3
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x4C00)  // UART 4
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x5000)  // UART 5
#define LPUART1_BASEADDR            (APB1PERIPH_BASEADDR + 0x8000)  // Low-Power UART 1
#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)  // I2C 1
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)  // I2C 2
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x7800)  // I2C 3
#define I2C4_BASEADDR               (APB1PERIPH_BASEADDR + 0x8400)  // I2C 4
#define UCPD1_BASEADDR              (APB1PERIPH_BASEADDR + 0xA000)  // USB Type-C / USB Power Delivery
#define USB_DEV_FS_BASEADDR         (APB1PERIPH_BASEADDR + 0x5C00)  // USB Full Speed Device
#define USB_SRAM_BASEADDR           (APB1PERIPH_BASEADDR + 0x6000)  // USB SRAM 1 KB
#define FDCAN1_BASEADDR             (APB1PERIPH_BASEADDR + 0x6400)  // FDCAN 1
#define FDCAN2_BASEADDR             (APB1PERIPH_BASEADDR + 0x6800)  // FDCAN 2
#define FDCAN3_BASEADDR             (APB1PERIPH_BASEADDR + 0x6C00)  // FDCAN 3
#define PWR_BASEADDR                (APB1PERIPH_BASEADDR + 0x7000)  // Power Control
#define FDCAN_RAM_BASEADDR          (APB1PERIPH_BASEADDR + 0xA400)  // FDCAN Message RAM
#define APB1_RESERVED_BASEADDR1     (APB1PERIPH_BASEADDR + 0x8800)  // Reserved (6 KB)
#define APB1_RESERVED_BASEADDR2     (APB1PERIPH_BASEADDR + 0xA800)  // Reserved (1 KB)
#define APB1_RESERVED_BASEADDR3     (APB1PERIPH_BASEADDR + 0xAC00)  // Reserved (1 KB)
#define APB1_RESERVED_BASEADDR4     (APB1PERIPH_BASEADDR + 0xAFFE)  // Reserved (23 KB)
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
/* ---[ APB2 BUS PERIPHERAL BASE ADDRESSES ]--- */
/* ----------------------------------------------------------------------------- */
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x0000)  // System Configuration Controller
#define VREFBUF_BASEADDR            (APB2PERIPH_BASEADDR + 0x0030)  // Voltage Reference Buffer
#define COMP_BASEADDR               (APB2PERIPH_BASEADDR + 0x0200)  // Comparator
#define OPAMP_BASEADDR              (APB2PERIPH_BASEADDR + 0x0300)  // Operational Amplifier
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x0400)  // External Interrupts
#define APB2_RESERVED_BASEADDR1     (APB2PERIPH_BASEADDR + 0x0800)  // Reserved (9 KB)
#define TIM1_BASEADDR               (APB2PERIPH_BASEADDR + 0x2C00)  // Timer 1
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)  // SPI/I2S 1
#define TIM8_BASEADDR               (APB2PERIPH_BASEADDR + 0x3400)  // Timer 8
#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)  // USART 1
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)  // SPI/I2S 4
#define TIM15_BASEADDR              (APB2PERIPH_BASEADDR + 0x4000)  // Timer 15
#define TIM16_BASEADDR              (APB2PERIPH_BASEADDR + 0x4400)  // Timer 16
#define TIM17_BASEADDR              (APB2PERIPH_BASEADDR + 0x4800)  // Timer 17
#define APB2_RESERVED_BASEADDR2     (APB2PERIPH_BASEADDR + 0x4C00)  // Reserved (1 KB)
#define TIM20_BASEADDR              (APB2PERIPH_BASEADDR + 0x5000)  // Timer 20
#define SAI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x5400)  // Serial Audio Interface 1
#define APB2_RESERVED_BASEADDR3     (APB2PERIPH_BASEADDR + 0x5800)  // Reserved (4 KB)
#define HRTIM_BASEADDR              (APB2PERIPH_BASEADDR + 0x6800)  // High-Resolution Timer (HRTIM)
#define APB2_RESERVED_BASEADDR4     (APB2PERIPH_BASEADDR + 0x7800)  // Reserved (2 KB)
/* ----------------------------------------------------------------------------- */


/* ---[ GPIO PERIPHERAL REGISTRY DEFINITION STRUCTURE ]--- */
/* ----------------------------------------------------------------------------- */
typedef struct
{
	volatile uint32_t MODER;		// GPIO port mode				: 0x00
	volatile uint32_t OTYPER;		// GPIO port output type		: 0x04
	volatile uint32_t OSPEEDR;		// GPIO port output speed		: 0x08
	volatile uint32_t PUPDR;		// GPIO port pull up & pull down: 0x0C
	volatile uint32_t IDR;			// GPIO port input data			: 0x10
	volatile uint32_t ODR;			// GPIO port output data		: 0x14
	volatile uint32_t BSRR;			// GPIO port bit set & reset	: 0x18
	volatile uint32_t LCKR;			// GPIO port configuration lock	: 0x1C
	volatile uint32_t AFR[2];		// AFRL & AFRH					: 0x20 & 0x24
	volatile uint32_t BRR;			// GPIO bit reset				: 0x28
} GPIO_RegDef_t;

/* ---[ GPIO PERIPHERAL INITIALIZATION WITH GPIO_REGDEF POINTER ]--- */
/* ----------------------------------------------------------------------------- */
#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASEADDR)		// peripheral base address typecasted to GPIOx_RegDef_t
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASEADDR)		// analog to : GPIO_RegDef_t *pGPIOB = (GPIO_RegDef_t*) 0x48000000 + 0x0400
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASEADDR)		// analog to : GPIO_RegDef_t *pGPIOC = (GPIO_RegDef_t*) 0x48000000 + 0x0800
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASEADDR)		// analog to : GPIO_RegDef_t *pGPIOD = (GPIO_RegDef_t*) 0x48000000 + 0x0C00
#define GPIOE			((GPIO_RegDef_t*) GPIOE_BASEADDR)		// analog to : GPIO_RegDef_t *pGPIOE = (GPIO_RegDef_t*) 0x48000000 + 0x1000
#define GPIOF			((GPIO_RegDef_t*) GPIOF_BASEADDR)		// analog to : GPIO_RegDef_t *pGPIOF = (GPIO_RegDef_t*) 0x48000000 + 0x1400
#define GPIOG			((GPIO_RegDef_t*) GPIOG_BASEADDR)		// analog to : GPIO_RegDef_t *pGPIOG = (GPIO_RegDef_t*) 0x48000000 + 0x1800
/* ----------------------------------------------------------------------------- */


/* ---[ RCC PERIPHERAL REGISTRY DEFINITION STRUCTURE ]--- */
/* ----------------------------------------------------------------------------- */
typedef struct
{
	volatile uint32_t RCC_CR;			/* Clock control register				: 0x00 */
	volatile uint32_t RCC_ICSCR;		/* Internal clock sources calibration	: 0x04 */
	volatile uint32_t RCC_CFGR;			/* Clock configuration register			: 0x08 */
	volatile uint32_t RCC_PLLCFGR;		/* PLL configuration register			: 0x0C */
	volatile uint32_t RCC_CIER;			/* Clock interrupt enable register		: 0x18 */
	volatile uint32_t RCC_CIFR;			/* Clock interrupt flag register		: 0x1C */
	volatile uint32_t RCC_CICR;			/* Clock interrupt clear register		: 0x20 */
	volatile uint32_t RCC_AHB1RSTR;		/* AHB1 peripheral reset register		: 0x28 */
	volatile uint32_t RCC_AHB2RSTR;		/* AHB2 peripheral reset register		: 0x2C */
	volatile uint32_t RCC_AHB3RSTR;		/* AHB3 peripheral reset register		: 0x30 */
	volatile uint32_t RCC_APB1RSTR1;	/* APB1 peripheral reset register 1		: 0x38 */
	volatile uint32_t RCC_APB1RSTR2;	/* APB1 peripheral reset register 2		: 0x3C */
	volatile uint32_t RCC_APB2RSTR;		/* APB2 peripheral reset register		: 0x40 */
	volatile uint32_t RCC_AHB1ENR;		/* AHB1 peripheral clock enable			: 0x48 */
	volatile uint32_t RCC_AHB2ENR;		/* AHB2 peripheral clock enable			: 0x4C */
	volatile uint32_t RCC_AHB3ENR;		/* AHB3 peripheral clock enable			: 0x50 */
	volatile uint32_t RCC_APB1ENR1;		/* APB1 peripheral clock enable reg 1	: 0x58 */
	volatile uint32_t RCC_APB1ENR2;		/* APB1 peripheral clock enable reg 2	: 0x5C */
	volatile uint32_t RCC_APB2ENR;		/* APB2 peripheral clock enable			: 0x60 */
	volatile uint32_t RCC_AHB1SMENR;	/* AHB1 Sleep & Stop modes				: 0x68 */
	volatile uint32_t RCC_AHB2SMENR;	/* AHB2 Sleep & Stop modes				: 0x6C */
	volatile uint32_t RCC_AHB3SMENR;	/* AHB3 Sleep & Stop modes				: 0x70 */
	volatile uint32_t RCC_APB1SMENR1;	/* APB1 Sleep & Stop modes 1			: 0x78 */
	volatile uint32_t RCC_APB1SMENR2;	/* APB1 Sleep & Stop modes 2			: 0x7C */
	volatile uint32_t RCC_APB2SMENR;	/* APB2 Sleep & Stop modes				: 0x80 */
	volatile uint32_t RCC_CCIPR;		/* Peripherals independent clock config	: 0x88 */
	volatile uint32_t RCC_BDCR;			/* RTC domain control register			: 0x90 */
	volatile uint32_t RCC_CSR;			/* Control / status register			: 0x94 */
	volatile uint32_t RCC_CRRCR;		/* Clock recovery RC register			: 0x98 */
	volatile uint32_t RCC_CCIPR2;		/* Peripherals independent clock config	: 0x9C */
} RCC_RegDef_t;
/* ----------------------------------------------------------------------------- */

/* ---[ RCC PERIPHERAL INITIALIZATION WITH RCC_REGDEF POINTER ]--- */
/* ----------------------------------------------------------------------------- */
#define RCC				((RCC_RegDef_t*) RCC_BASEADDR)
/* ----------------------------------------------------------------------------- */



/*---[ CLOCK ENABLE & DISABLE MACROS FOR GPIO PERIPHERALS ]---*/
/* ----------------------------------------------------------------------------- */
#define GPIOA_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 0))	/* Bitwise OR : |= : to set bits */
#define GPIOB_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->RCC_AHB2ENR |= (1 << 6))

#define GPIOA_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 0))	/* Bitwise CLEAR : &= : to clear bits */
#define GPIOB_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->RCC_AHB2ENR &= ~(1 << 6))

//#define I2C1_PLCK_EN()			(RCC->RCC_APB1ENR1 |= (1 << 21))
/* ----------------------------------------------------------------------------- */


/*---[ REET GPIOx PERIPHERALS ]---*/
/* ----------------------------------------------------------------------------- */
#define GPIOA_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 0));	(RCC->RCC_AHB2RSTR &= ~( 1 << 0)); } while(0)	// set and reset(clear) in single do while line
#define GPIOB_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 1));	(RCC->RCC_AHB2RSTR &= ~( 1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 2));	(RCC->RCC_AHB2RSTR &= ~( 1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 3));	(RCC->RCC_AHB2RSTR &= ~( 1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 4));	(RCC->RCC_AHB2RSTR &= ~( 1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 5));	(RCC->RCC_AHB2RSTR &= ~( 1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do { (RCC->RCC_AHB2RSTR |= ( 1 << 6));	(RCC->RCC_AHB2RSTR &= ~( 1 << 6)); } while(0)
/* ----------------------------------------------------------------------------- */


/* ---[ GENERIC DEVICE MACROS ]--- */
/* ----------------------------------------------------------------------------- */
#define ENABLE  		1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
/* ----------------------------------------------------------------------------- */


#endif /* INC_STM32G431XX_H_ */
