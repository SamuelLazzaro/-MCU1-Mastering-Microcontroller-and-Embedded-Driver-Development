/*
 * stm32f407xx.h
 *
 *  Created on: Nov 3, 2024
 *      Author: Samuel.Lazzaro
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/************		START: Processor Specific Details		********* */
/*
 * ARM Cortex Mx Processor NVIC ISERx register address
 */
#define NVIC_ISER0				((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3				((volatile uint32_t *)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register address
 */
#define NVIC_ICER0				((volatile uint32_t *)0xE000E180)
#define NVIC_ICER1				((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2				((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3				((volatile uint32_t *)0xE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PRIORITY_BASE_ADDRESS		((volatile uint32_t *)0xE000E400)

#define NUM_PRIORITY_BITS_IMPLEMENTED		4


// Base addresses of Flash and SRAM memories
#define FLASH_BASE_ADDRESS		0x08000000U
#define SRAM1_BASE_ADDRESS		0x20000000U		// 112kB
#define SRAM2_BASE_ADDRESS		0x20001C00U		// (SRAM1_BASE_ADDRESS + 0x1C00) where 0x1C00 = 112kB
#define ROM_BASE_ADDRESS		0x1FFF0000U
#define SRAM					SRAM1_BASE_ADDRESS

// AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASE				0x40000000U
#define APB1_PERIPH_BASE		PERIPH_BASE
#define APB2_PERIPH_BASE		0x40010000U
#define AHB1_PERIPH_BASE		0x40020000U
#define AHB2_PERIPH_BASE		0x50000000U

// Base addresses of peripherals which are hanging on AHB1 bus
#define GPIOA_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x1C00)
#define GPIOI_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x2000)
#define GPIOJ_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x2400)
#define GPIOK_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x2800)

#define RCC_BASE_ADDRESS		(AHB1_PERIPH_BASE + 0x3800)

// Base addresses of peripherals which are hanging on APB1 bus
#define I2C1_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x5800)
#define I2C3_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x5C00)
#define SPI2_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x3C00)
#define USART2_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x4400)
#define USART3_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x4800)
#define UART4_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASE_ADDRESS		(APB1_PERIPH_BASE + 0x5000)

// Base addresses of peripherals which are hanging on APB2 bus
#define SPI1_BASE_ADDRESS		(APB2_PERIPH_BASE + 0x3000)
#define SPI4_BASE_ADDRESS		(APB2_PERIPH_BASE + 0x3400)
#define USART1_BASE_ADDRESS		(APB2_PERIPH_BASE + 0x1000)
#define USART6_BASE_ADDRESS		(APB2_PERIPH_BASE + 0x1400)
#define EXTI_BASE_ADDRESS		(APB2_PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE_ADDRESS		(APB2_PERIPH_BASE + 0x3800)



// -------------------------------------------------------- //
// 		Peripheral register definition structures			//
// -------------------------------------------------------- //

// Note: Registers of a peripheral are specific to MCU
// e.g. : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different
// (more or less) compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of
// MCUs. Please check your Device Reference Manual.

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct {
	volatile uint32_t MODER;		// GPIO port mode register,					Address offset: 0x00
	volatile uint32_t OTYPER;		// GPIO port output type register,			Address offset: 0x04
	volatile uint32_t OSPEEDR;		// GPIO port output speed register,			Address offset: 0x08
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register,	Address offset: 0x0C
	volatile uint32_t IDR;			// GPIO port input data register,			Address offset: 0x10
	volatile uint32_t ODR;			// GPIO port output data register,			Address offset: 0x14
	volatile uint32_t BSRR;			// GPIO port bit set/reset register,		Address offset: 0x18
	volatile uint32_t LCKR;			// GPIO port configuration lock register,	Address offset: 0x1C
	volatile uint32_t AFR[2];		// AFR[0] : GPIO alternate function low register, Address offset: 0x20;	AFR[1] : GPIO alternate function high register, Address offset: 0x24
} GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct {
	volatile uint32_t CR;			// RCC clock control register,				Address offset: 0x00
	volatile uint32_t PLLCFGR;		// RCC PLL configuration register,			Address offset: 0x04
	volatile uint32_t CFGR;			// RCC clock configuration register,		Address offset: 0x08
	volatile uint32_t CIR;			// RCC clock interrupt register,			Address offset: 0x0C
	volatile uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register,		Address offset: 0x10
	volatile uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register,		Address offset: 0x14
	volatile uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register,		Address offset: 0x18
	uint32_t RESERVED0;				// Reserved 0x1C
	volatile uint32_t APB1RSTR;		// RCC APB1 peripheral reset register,		Address offset: 0x20
	volatile uint32_t APB2RSTR;		// RCC APB2 peripheral reset register,		Address offset: 0x24
	uint32_t RESERVED1[2];			// Reserver 0x28 & 0x2C
	volatile uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register,		Address offset: 0x30
	volatile uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register, 		Address offset: 0x34
	volatile uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register,		Address offset: 0x38
	volatile uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register,		Address offset: 0x40
	volatile uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register,		Address offset: 0x44
	uint32_t RESERVED2[2];			// Reserved 0x48 & 0x4C
	volatile uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register,		Address offset: 0x50
	volatile uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register,		Address offset: 0x54
	volatile uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable in low power mode register,		Address offset: 0x58
	uint32_t RESERVED3;				// Reserved 0x5C
	volatile uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register,		Address offset: 0x60
	volatile uint32_t APB2LPENR;	// RCC APB2 peripheral clock enable in low power mode register,		Address offset: 0x64
	uint32_t RESERVED4[2];			// Reserved 0x68 & 0x6C
	volatile uint32_t BDCR;			// RCC Backup domain control register,		Address offset: 0x70
	volatile uint32_t CSR;			// RCC clock control & status register,		Address offset: 0x74
	uint32_t RESERVED5[2];			// Reserved 0x78 & 0x7C
	volatile uint32_t SSCGR;		// RCC spread spectrum clock generation register,	Address offset: 0x80
	volatile uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register,		Address offset: 0x84
	volatile uint32_t PLLSAICFGR;	// RCC PLL configuration register,			Address offset: 0x88
	volatile uint32_t DCKCFGR;		// RCC Dedicated Clock Configuration Register,		Address offset: 0x8C
} RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct {
	volatile uint32_t MEMRMP;		// SYSCFG memory remap register,							Address offset: 0x00
	volatile uint32_t PMC;			// SYSCFG peripheral mode configuration register,			Address offset: 0x04
	volatile uint32_t EXTICR[4];	// SYSCFG external interrupt configuration registers,		Address offset: 0x08, 0x0C, 0x10, 0x14
	uint32_t RESERVED0[2];			// Reserved 0x18 and 0x1C
	volatile uint32_t CMPCR;		// Compensation cell control register,						Address offset: 0x20
} SYSCFG_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */
typedef struct {
	volatile uint32_t CR1;			// SPI control register 1,				Address offset: 0x00
	volatile uint32_t CR2;			// SPI control register 2,				Address offset: 0x04
	volatile uint32_t SR;			// SPI status register,					Address offset: 0x08
	volatile uint32_t DR;			// SPI data register,					Address offset: 0x0C
	volatile uint32_t CRCPR;		// SPI CRC polynomial register,			Address offset: 0x10
	volatile uint32_t RXCRCR;		// SPI RX CRC register,					Address offset: 0x14
	volatile uint32_t TXCRCR;		// SPI TX CRC register,					Address offset: 0x18
	volatile uint32_t I2SCFGR;		// SPI I2S configuration register,		Address offset: 0x1C
	volatile uint32_t I2SPR;		// SPI I2S prescaler register,			Address offset: 0x20
} SPI_RegDef_t;


// ------------------------------------------------------------------------------------ //
// 		Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t ) //
// ------------------------------------------------------------------------------------ //
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASE_ADDRESS)
#define GPIOJ		((GPIO_RegDef_t*)GPIOJ_BASE_ADDRESS)
#define GPIOK		((GPIO_RegDef_t*)GPIOK_BASE_ADDRESS)

#define RCC			((RCC_RegDef_t*)RCC_BASE_ADDRESS)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)

#define SPI1		((SPI_RegDef_t*)SPI1_BASE_ADDRESS)
#define SPI2		((SPI_RegDef_t*)SPI2_BASE_ADDRESS)
#define SPI3		((SPI_RegDef_t*)SPI3_BASE_ADDRESS)
#define SPI4		((SPI_RegDef_t*)SPI4_BASE_ADDRESS)


// ------------------------------------------------------------------------------------ //
// 						Clock Enable Macros for GPIOx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()		( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()		( RCC->AHB1ENR |= (1 << 10) )


// ------------------------------------------------------------------------------------ //
// 						Clock Enable Macros for I2Cx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )


// ------------------------------------------------------------------------------------ //
// 						Clock Enable Macros for SPIx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= (1 << 13) )

#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )


// ------------------------------------------------------------------------------------ //
// 						Clock Enable Macros for USARTx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5) )

#define USART2_PLCK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART3_PLCK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define USART4_PLCK_EN()	( RCC->APB1ENR |= (1 << 19) )
#define USART5_PLCK_EN()	( RCC->APB1ENR |= (1 << 20) )


// ------------------------------------------------------------------------------------ //
// 						Clock Enable Macros for SYSCFG peripherals 						//
// ------------------------------------------------------------------------------------ //
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )


// ------------------------------------------------------------------------------------ //
// 						Clock Disable Macros for GPIOx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 10) )


// ------------------------------------------------------------------------------------ //
// 						Clock Disable Macros for I2Cx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )


// ------------------------------------------------------------------------------------ //
// 						Clock Disable Macros for SPIx peripherals 						//
// ------------------------------------------------------------------------------------ //
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 13) )

#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )


// ------------------------------------------------------------------------------------ //
// 						Clock Disable Macros for USARTx peripherals 					//
// ------------------------------------------------------------------------------------ //
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )

#define USART2_PLCK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PLCK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define USART4_PLCK_DI()	( RCC->APB1ENR &= ~(1 << 19) )
#define USART5_PLCK_DI()	( RCC->APB1ENR &= ~(1 << 20) )


// ------------------------------------------------------------------------------------ //
// 						Clock Disable Macros for SYSCFG peripherals 					//
// ------------------------------------------------------------------------------------ //
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 0) );	( RCC->AHB1RSTR &= ~(1 << 0) );}while(0)
#define GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 1) );	( RCC->AHB1RSTR &= ~(1 << 1) );}while(0)
#define GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 2) );	( RCC->AHB1RSTR &= ~(1 << 2) );}while(0)
#define GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 3) );	( RCC->AHB1RSTR &= ~(1 << 3) );}while(0)
#define GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 4) );	( RCC->AHB1RSTR &= ~(1 << 4) );}while(0)
#define GPIOF_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 5) );	( RCC->AHB1RSTR &= ~(1 << 5) );}while(0)
#define GPIOG_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 6) );	( RCC->AHB1RSTR &= ~(1 << 6) );}while(0)
#define GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 7) );	( RCC->AHB1RSTR &= ~(1 << 7) );}while(0)
#define GPIOI_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 8) );	( RCC->AHB1RSTR &= ~(1 << 8) );}while(0)
#define GPIOJ_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 9) );	( RCC->AHB1RSTR &= ~(1 << 9) );}while(0)
#define GPIOK_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 10) );	( RCC->AHB1RSTR &= ~(1 << 10) );}while(0)


/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{( RCC->APB2RSTR |= (1 << 12) );  ( RCC->APB2RSTR &= ~(1 << 12) );}while(0)
#define SPI2_REG_RESET()	do{( RCC->APB1RSTR |= (1 << 14) );	( RCC->APB1RSTR &= ~(1 << 14) );}while(0)
#define SPI3_REG_RESET()	do{( RCC->APB1RSTR |= (1 << 15) );	( RCC->APB1RSTR &= ~(1 << 15) );}while(0)
#define SPI4_REG_RESET()	do{( RCC->APB2RSTR |= (1 << 13) );  ( RCC->APB2RSTR &= ~(1 << 13) );}while(0)

/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASE_ADDRESS_TO_CODE(x)	(	(x == GPIOA) ? 0 : \
											(x == GPIOB) ? 1 : \
											(x == GPIOC) ? 2 : \
											(x == GPIOD) ? 3 : \
											(x == GPIOE) ? 4 : \
											(x == GPIOF) ? 5 : \
											(x == GPIOG) ? 6 : \
											(x == GPIOH) ? 7 : \
											(x == GPIOI) ? 8 : \
											(x == GPIOJ) ? 9 : \
											(x == GPIOK) ? 10 : 0)


/*
 * IRQ (Interrupt Request) Number of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: you may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * IRQ Priority
 */
#define NVIC_IRQ_PRIORITY0		0
#define NVIC_IRQ_PRIORITY1		1
#define NVIC_IRQ_PRIORITY2		2
#define NVIC_IRQ_PRIORITY3		3
#define NVIC_IRQ_PRIORITY4		4
#define NVIC_IRQ_PRIORITY5		5
#define NVIC_IRQ_PRIORITY6		6
#define NVIC_IRQ_PRIORITY7		7
#define NVIC_IRQ_PRIORITY8		8
#define NVIC_IRQ_PRIORITY9		9
#define NVIC_IRQ_PRIORITY10		10
#define NVIC_IRQ_PRIORITY11		11
#define NVIC_IRQ_PRIORITY12		12
#define NVIC_IRQ_PRIORITY13		13
#define NVIC_IRQ_PRIORITY14		14
#define NVIC_IRQ_PRIORITY15		15

/*
 * Bit position definition of SPI peripheral
 */
// Bit position definition SPI_CR1 (SPI control register 1)
#define SPI_CR1_CPHA		0		// Clock phase
#define SPI_CR1_CPOL		1		// Clock polarity
#define SPI_CR1_MSTR		2		// Master selection
#define SPI_CR1_BR			3		// Baud rate control
#define SPI_CR1_SPE			6		// SPI enable
#define SPI_CR1_LSBFIRST	7		// Frame format
#define SPI_CR1_SSI			8		// Internal slave select
#define SPI_CR1_SSM			9		// Software slave management
#define SPI_CR1_RXONLY		10		// Receive only
#define SPI_CR1_DFF			11		// Data frame format
#define SPI_CR1_CRCNEXT		12		// CRC transfer next
#define SPI_CR1_CRCEN		13		// Hardware CRC calculation enable
#define SPI_CR1_BIDIOE		14		// Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE	15		// Bidirectional data mode enable

// Bit position definition SPI_CR2 (SPI control register 2)
#define SPI_CR2_RXDMAEN		0		// RX buffer DMA enable
#define SPI_CR2_TXDMAEN		1		// TX buffer DMA enable
#define SPI_CR2_SSOE		2		// SS output enable
#define SPI_CR2_FRF			4		// Frame format
#define SPI_CR2_ERRIE		5		// Error interrupt enable
#define SPI_CR2_RXNEIE		6		// RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE		7		// TX buffer empty interrupt enable

// Bit position definition SPI_SR (SPI status register)
#define SPI_SR_RXNE			0		// Receive buffer not empty
#define SPI_SR_TXE			1		// Transmit buffer empty
#define SPI_SR_CHSIDE		2		// Channel side
#define SPI_SR_UDR			3		// Underrun flag
#define SPI_SR_CRCERR		4		// CRC error flag
#define SPI_SR_MODF			5		// Mode fault
#define SPI_SR_OVR			6		// Overrun flag
#define SPI_SR_BSY			7		// Busy flag
#define SPI_SR_FRE			8		// Frame format error



/*
 * Some generic Macros
 */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

#endif /* INC_STM32F407XX_H_ */
