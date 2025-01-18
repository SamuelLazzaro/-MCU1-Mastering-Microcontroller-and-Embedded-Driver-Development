/*
 * stm32nucleo_h563zi.h
 *
 *  Created on: Dec 13, 2024
 *      Author: Samuel.Lazzaro
 */

#ifndef INC_STM32NUCLEO_H563ZI_H_
#define INC_STM32NUCLEO_H563ZI_H_

#include <stdint.h>

/**********		START: Processor Specific Details	*********/
/*
 * ARM Cortex M33 Processor NVIC ISERx register address
 */
#define NVIC_ISER0		((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t *)0xE000E10C)
#define NVIC_ISER4		((volatile uint32_t *)0xE000E110)

#define NVIC_ISER0_NS	((volatile uint32_t *)0xE002E100)
#define NVIC_ISER1_NS	((volatile uint32_t *)0xE002E104)
#define NVIC_ISER2_NS	((volatile uint32_t *)0xE002E108)
#define NVIC_ISER3_NS	((volatile uint32_t *)0xE002E10C)
#define NVIC_ISER4_NS	((volatile uint32_t *)0xE002E110)

/*
 * ARM Cortex M33 Processor NVIC ICERx register address
 */
#define NVIC_ICER0		((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1		((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2		((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3		((volatile uint32_t *)0xE000E18C)
#define NVIC_ICER4		((volatile uint32_t *)0xE000E190)

#define NVIC_ICER0_NS	((volatile uint32_t *)0XE002E180)
#define NVIC_ICER1_NS	((volatile uint32_t *)0xE002E184)
#define NVIC_ICER2_NS	((volatile uint32_t *)0xE002E188)
#define NVIC_ICER3_NS	((volatile uint32_t *)0xE002E18C)
#define NVIC_ICER4_NS	((volatile uint32_t *)0xE002E190)


/*
 * ARM Cortex M33 Processor NVIC IPRx register base address
 */
#define NVIC_IPR_BASE_ADDRESS		((volatile uint32_t *)0xE000E400)
#define NVIC_IPR_NS_BASE_ADDRESS	((volatile uint32_t *)0xE002E400)

#define NUM_PRIORITY_BITS_IMPLEMENTED		4

// Base addresses of Flash and SRAM memories
#define FLASH_BASE_ADDRESS		(uint32_t)0x08000000

#define SRAM1_BASE_ADDRESS		(uint32_t)0x20000000		// 256 kB
#define SRAM2_BASE_ADDRESS		(uint32_t)0x20040000		// 64 kB
#define SRAM3_BASE_ADDRESS		(uint32_t)0x20050000		// 320 kB

#define ROM_BASE_ADDRESS		(uint32_t)0x0BF80000		// 128 kB

// AHBx and APBx Bus Peripheral base addresses
#define PERIPHERALS_BASE_ADDRESS		(uint32_t)0x50000000
#define APB1_BUS_BASE_ADDRESS			(uint32_t)PERIPHERALS_BASE_ADDRESS
#define APB2_BUS_BASE_ADDRESS			(uint32_t)0x50010000
#define APB3_BUS_BASE_ADDRESS			(uint32_t)0x5000000
#define AHB1_BUS_BASE_ADDRESS			(uint32_t)0x50020000
#define AHB2_BUS_BASE_ADDRESS			(uint32_t)0x52020000
#define AHB3_BUS_BASE_ADDRESS			(uint32_t)0x54020000
#define AHB4_BUS_BASE_ADDRESS			(uint32_t)0x56000000

// Base addresses of peripherals which are hanging on AHB2 bus
#define GPIOA_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x0000)
#define GPIOB_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x0400)
#define GPIOC_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x0800)
#define GPIOD_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x0C00)
#define GPIOE_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x1000)
#define GPIOF_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x1400)
#define GPIOG_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x1800)
#define GPIOH_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x1C00)
#define GPIOI_BASE_ADDRESS			(AHB2_BUS_BASE_ADDRESS + 0x2000)

// Base addresses of peripherals which are hanging on AHB3 bus
#define RCC_BASE_ADDRESS			(AHB3_BUS_BASE_ADDRESS + 0x0C00)
#define EXTI_BASE_ADDRESS			(AHB3_BUS_BASE_ADDRESS + 0x2000)

// Base addresses of peripherals which are hanging on APB1 bus
#define SPI2_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x3800)
#define SPI3_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x3C00)

#define USART2_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x4400)
#define USART3_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x4800)
#define USART6_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x6400)
#define USART10_BASE_ADDRESS		(APB1_BUS_BASE_ADDRESS + 0x6800)
#define USART11_BASE_ADDRESS		(APB1_BUS_BASE_ADDRESS + 0x6C00)

#define UART4_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x4C00)
#define UART5_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x5000)
#define UART7_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x7800)
#define UART8_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x7C00)
#define UART9_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x8000)
#define UART12_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x8400)

#define I2C1_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x5400)
#define I2C2_BASE_ADDRESS			(APB1_BUS_BASE_ADDRESS + 0x5800)

// Base addresses of peripherals which are hanging on APB2 bus
#define SPI1_BASE_ADDRESS			(APB2_BUS_BASE_ADDRESS + 0x3000)
#define SPI4_BASE_ADDRESS			(APB2_BUS_BASE_ADDRESS + 0x4C00)
#define SPI6_BASE_ADDRESS			(APB2_BUS_BASE_ADDRESS + 0x5000)

#define USART1_BASE_ADDRESS			(APB2_BUS_BASE_ADDRESS + 0x3800)

// Base addresses of peripherals which are hanging on APB3 bus
#define SPI5_BASE_ADDRESS			(APB3_BUS_BASE_ADDRESS + 0x2000)

#define I2C3_BASE_ADDRESS			(APB3_BUS_BASE_ADDRESS + 0x2800)
#define I2C4_BASE_ADDRESS			(APB3_BUS_BASE_ADDRESS + 0x2C00)

/*
 * Peripheral registers definition structure for GPIO
 */
typedef struct {
	volatile uint32_t MODER;			// GPIO port mode register,					Address offset: 0x00
	volatile uint32_t OTYPER;			// GPIO port output type register,			Address offset: 0x04
	volatile uint32_t OSPEEDR;			// GPIO port output speed register,			Address offset: 0x08
	volatile uint32_t PUPDR;			// GPIO port pull-up/pull-down register,	Address offset: 0x0C
	volatile uint32_t IDR;				// GPIO port input data register,			Address offset: 0x10
	volatile uint32_t ODR;				// GPIO port output data register,			Address offset: 0x14
	volatile uint32_t BSRR;				// GPIO port bit set/reset register,		Address offset: 0x18
	volatile uint32_t LCKR;				// GPIO port configuration lock register,	Address offset: 0x1C
	volatile uint32_t AFR[2];			// AFR[0]: GPIO port alternate function low register, Address offset: 0x20; AFR[1]: GPIO port alternate function high register, Address offset: 0x24
	volatile uint32_t BRR;				// GPIO port bit reset register,			Address offset: 0x28
	volatile uint32_t HSLVR;			// GPIO port high-speed low-voltage register, Address offset: 0x2C
	volatile uint32_t SECCFGR;			// GPIO port secure configuration register, Address offset: 0x30
} GPIO_RegDef_t;

/*
 * Peripheral registers definition structure for RCC
 */
typedef struct {
	volatile uint32_t CR;					// RCC clock control register,					Address offset: 0x000
	volatile uint32_t RESERVED0[3];			// Reserved 0x004, 0x008, 0x00C
	volatile uint32_t HSICFGR;				// RCC HSI calibration register,				Address offset: 0x010
	volatile uint32_t CRRCR;				// RCC clock recovery RC register,				Address offset: 0x014
	volatile uint32_t CSICFGR;				// RCC CSI calibration register,				Address offset: 0x018
	volatile uint32_t CFGR1;				// RCC clock configuration register 1,			Address offset: 0x01C
	volatile uint32_t CFGR2;				// RCC CPU domain clock configuration register 2, Address offset: 0x020
	uint32_t RESERVED1;						// Reserved 0x024
	volatile uint32_t PLL1CFGR;				// RCC PLL clock source selection register,	Address offset: 0x028
	volatile uint32_t PLL2CFGR;				// RCC PLL clock source selection register,	Address offset: 0x02C
	volatile uint32_t PLL3CFGR;				// RCC PLL clock source selection register,	Address offset: 0x030
	volatile uint32_t PLL1DIVR;				// RCC PLL1 dividers register,					Address offset: 0x034
	volatile uint32_t PLL1FRACR;			// RCC PLL1 fractional divider register,		Address offset: 0x038
	volatile uint32_t PLL2DIVR;				// RCC PLL2 dividers register,					Address offset: 0x03C
	volatile uint32_t PLL2FRACR;			// RCC PLL2 fractional divider register,		Address offset: 0x040
	volatile uint32_t PLL3DIVR;				// RCC PLL3 dividers register,					Address offset: 0x044
	volatile uint32_t PLL3FRACR;			// RCC PLL3 fractional divider register,		Address offset: 0x048
	uint32_t RESERVED2;						// Reserved 0x04C
	volatile uint32_t CIER;					// RCC clock source interrupt enable register, Address offset: 0x050
	uint32_t RESERVED3;						// Reserved 0x054
	volatile uint32_t CICR;					// RCC clock source interrupt clear register,	Address offset: 0x058
	uint32_t RESERVED4;						// Reserved 0x05C
	volatile uint32_t AHB1RSTR;				// RCC AHB1 reset register,					Address offset: 0x060
	volatile uint32_t AHB2RSTR;				// RCC AHB2 reset register,					Address offset: 0x064
	uint32_t RESERVED5;						// Reserved 0x068
	volatile uint32_t AHB4RSTR;				// RCC AHB4 reset register,					Address offset: 0x06C
	uint32_t RESERVED6;						// Reserved 0x070
	volatile uint32_t APB1RSTR[2];			// APB1RSTR[0]: APB1 peripheral low reset register, Address offset: 0x074, APB1RSTR[1]: APB1 peripheral high reset register, Address offset: 0x078
	volatile uint32_t APB2RSTR;				// RCC APB2 reset register,					Address offset: 0x07C
	volatile uint32_t APB3RSTR;				// RCC APB3 reset register,					Address offset: 0x080
	uint32_t RESERVED7;						// Reserved 0x084
	volatile uint32_t AHB1ENR;				// RCC AHB1 clock register,					Address offset: 0x088
	volatile uint32_t AHB2ENR;				// RCC AHB2 clock register,					Address offset: 0x08C
	uint32_t RESERVED8;						// Reserved 0x090
	volatile uint32_t AHB4ENR;				// RCC AHB4 clock register,					Address offset: 0x094
	uint32_t RESERVED9;						// Reserved 0x098
	volatile uint32_t APB1ENR[2];			// APB1ENR[0]: APB1 low clock register, Address offset: 0x09C, APB1ENR[1]: APB1 high clock register, Address offset: 0x0A0
	volatile uint32_t APB2ENR;				// RCC APB1 clock register,					Address offset: 0x0A4
	volatile uint32_t APB3ENR;				// RCC APB3 clock register,					Address offset: 0x0A8
	uint32_t RESERVED10;					// Reserved 0x0AC
	volatile uint32_t AHB1LPENR;			// RCC AHB1 sleep clock register,			Address offset: 0x0B0
	volatile uint32_t AHB2LPENR;			// RCC AHB2 sleep clock register,			Address offset: 0x0B4
	uint32_t RESERVED11;					// Reserved 0x0B8
	volatile uint32_t AHB4LPENR;			// RCC AHB4 sleep clock register,			Address offset: 0x0BC
	uint32_t RESERVED12;					// Reserved 0xC0
	volatile uint32_t APB1LPENR[2];			// APB1LPENR[0]: RCC APB1 low sleep clock register, Address offset: 0x0C4; APB1LPENR[1]: RCC APB1 hight sleep clock register, Address offset: 0x0C8
	volatile uint32_t APB2LPENR;			// RCC APB2 sleep clock register,			Address offset: 0x0CC
	volatile uint32_t APB3LPENR;			// RCC APB3 sleep clock register,			Address offset: 0x0D0
	uint32_t RESERVED13;					// Reserved 0x0D4
	volatile uint32_t CCIPR1;				// RCC kernel clock configuration register 1, Address offset: 0x0D8
	volatile uint32_t CCIPR2;				// RCC kernel clock configuration register 2, Address offset: 0x0DC
	volatile uint32_t CCIPR3;				// RCC kernel clock configuration register 3, Address offset: 0x0E0
	volatile uint32_t CCIPR4;				// RCC kernel clock configuration register 4, Address offset: 0x0E4
	volatile uint32_t CCIPR5;				// RCC kernel clock configuration register 5, Address offset: 0x0E8
	uint32_t RESERVED14;					// Reserved 0x0EC
	volatile uint32_t BDCR;					// RCC backup domain control register,		Address offset: 0x0F0
	volatile uint32_t RSR;					// RCC reset status register,				Address offset: 0x0F4
	uint32_t RESERVED15[6];					// Reserved 0x0F8 - 0x10C
	volatile uint32_t SECCFGR;				// RCC secure configuration register,		Address offset: 0x110
	volatile uint32_t PRIVCFGR;				// RCC privilege configuration register,	Address offset: 0x114
} RCC_RegDef_t;

/*
 * Peripheral registers definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t RTSR1;		// EXTI rising trigger selection register 1,		Address offset: 0x000
	volatile uint32_t FTSR1;		// EXTI falling trigger selection register 1,		Address offset: 0x004
	volatile uint32_t SWIER1;	// EXTI software interrupt event register 1,		Address offset: 0x008
	volatile uint32_t RPR1;		// EXTI rising edge pending register 1,				Address offset: 0x00C
	volatile uint32_t FPR1;		// EXTI falling edge pending register 1,			Address offset: 0x010
	volatile uint32_t SECCFGR1;	// EXTI security configuration register 1,			Address offset: 0x014
	volatile uint32_t PRIVCFGR1;	// EXTI privilege configuration register 1,			Address offset: 0x018
	volatile uint32_t RESERVED0;	// Reserved 0x01C - EXTI Register Map in Reference Manual miss this RESERVED
	volatile uint32_t RTSR2;		// EXTI rising trigger selection register 2,		Address offset: 0x020
	volatile uint32_t FTSR2;		// EXTI falling trigger selection register 2,		Address offset: 0x024
	volatile uint32_t SWIER2;	// EXTI software interrupt event register 2,		Address offset: 0x028
	volatile uint32_t RPR2;		// EXTI rising edge pending register 2,				Address offset: 0x02C
	volatile uint32_t FPR2;		// EXTI falling edge pending register 2,			Address offset: 0x030
	volatile uint32_t SECCFGR2;	// EXTI security configuration register 2,			Address offset: 0x034
	volatile uint32_t PRIVCFGR2;	// EXTI privilege configuration register 2,			Address offset: 0x038
	volatile uint32_t RESERVED1[9];	// Reserved from 0x03C to 0x05C
	volatile uint32_t EXTICR[4];	// EXTI external interrupt selection register,		Address offset: 0x060 - 0x064 - 0x068 - 0x06C
	volatile uint32_t LOCKR;		// EXTI lock register,								Address offset: 0x070
	volatile uint32_t RESERVED2[3];		// Reserved from 0x074 to 0x07C
	volatile uint32_t IMR1;		// EXTI CPU wake-up with interrupt mask register,	Address offset: 0x080
	volatile uint32_t EMR1;		// EXTI CPU wake-up with event mask register,		Address offset: 0x084
	volatile uint32_t RESERVED3[2];		// Reserved from 0x088 to 0x08C
	volatile uint32_t IMR2;		// EXTI CPU wake-up with interrupt mask register,	Address offset: 0x090
	volatile uint32_t EMR2;		// EXTI CPU wake-up with event mask register,		Address offset: 0x094
} EXTI_RegDef_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t
 */
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASE_ADDRESS)

#define RCC				((RCC_RegDef_t*)RCC_BASE_ADDRESS)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)

/*
 * IRQ (Interrupt Request) Number of STM32NUCLEO-H563ZI MCU
 */
#define IRQ_NUM_EXTI0		11
#define IRQ_NUM_EXTI1		12
#define IRQ_NUM_EXTI2		13
#define IRQ_NUM_EXTI3		14
#define IRQ_NUM_EXTI4		15
#define IRQ_NUM_EXTI5		16
#define IRQ_NUM_EXTI6		17
#define IRQ_NUM_EXTI7		18
#define IRQ_NUM_EXTI8		19
#define IRQ_NUM_EXTI9		20
#define IRQ_NUM_EXTI10		21
#define IRQ_NUM_EXTI11		22
#define IRQ_NUM_EXTI12		23
#define IRQ_NUM_EXTI13		24
#define IRQ_NUM_EXTI14		25
#define IRQ_NUM_EXTI15		26

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
 * Macros to enable peripheral clock of AHB2 bus
 */
#define GPIOA_PCLK_EN()			(RCC->AHB2ENR |= 1 << 0)
#define GPIOB_PCLK_EN()			(RCC->AHB2ENR |= 1 << 1)
#define GPIOC_PCLK_EN()			(RCC->AHB2ENR |= 1 << 2)
#define GPIOD_PCLK_EN()			(RCC->AHB2ENR |= 1 << 3)
#define GPIOE_PCLK_EN()			(RCC->AHB2ENR |= 1 << 4)
#define GPIOF_PCLK_EN()			(RCC->AHB2ENR |= 1 << 5)
#define GPIOG_PCLK_EN()			(RCC->AHB2ENR |= 1 << 6)
#define GPIOH_PCLK_EN()			(RCC->AHB2ENR |= 1 << 7)
#define GPIOI_PCLK_EN()			(RCC->AHB2ENR |= 1 << 8)

/*
 * Macros to disable peripheral clock of AHB2 bus
 */
#define GPIOA_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 8))

/*
 * Macros to reset peripheral of AHB2 bus
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 0));		(RCC->AHB2RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 1));		(RCC->AHB2RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 2));		(RCC->AHB2RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 3));		(RCC->AHB2RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 4));		(RCC->AHB2RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 5));		(RCC->AHB2RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 6));		(RCC->AHB2RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 7));		(RCC->AHB2RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB2RSTR |= (1 << 8));		(RCC->AHB2RSTR &= ~(1 << 8));}while(0)


/*
 * Macros to enable peripheral clock of APB1 bus
 */
#define SPI2_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 14)
#define SPI3_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 15)

#define USART2_PCLK_EN()		(RCC->APB1ENR[0] |= 1 << 17)
#define USART3_PCLK_EN()		(RCC->APB1ENR[0] |= 1 << 18)
#define UART4_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 19)
#define UART5_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 20)
#define USART6_PCLK_EN()		(RCC->APB1ENR[0] |= 1 << 25)
#define UART7_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 30)
#define UART8_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 31)
#define USART10_PCLK_EN()		(RCC->APB1ENR[0] |= 1 << 26)
#define USART11_PCLK_EN()		(RCC->APB1ENR[0] |= 1 << 27)

#define I2C1_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 21)
#define I2C2_PCLK_EN()			(RCC->APB1ENR[0] |= 1 << 22)

#define UART9_PCLK_EN()			(RCC->APB1ENR[1] |= 1 << 0)
#define UART12_PCLK_EN()		(RCC->APB1ENR[1] |= 1 << 1)

/*
 * Macros to disable peripheral clock of APB1 bus
 */
#define SPI2_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 15))

#define USART2_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 20))
#define USART6_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 25))
#define UART7_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 30))
#define UART8_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 31))
#define USART10_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 26))
#define USART11_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 27))

#define I2C1_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR[0] &= ~(1 << 22))

#define UART9_PCLK_DI()			(RCC->APB1ENR[1] &= ~(1 << 0))
#define UART12_PCLK_DI()		(RCC->APB1ENR[1] &= ~(1 << 1))

/*
 * Macros to enable peripheral clock of APB2 bus
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= 1 << 12)
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= 1 << 19)
#define SPI6_PCLK_EN()			(RCC->APB2ENR |= 1 << 20)
#define USART1_PCLK_EN()		(RCC->APB2ENR |= 1 << 14)

/*
 * Macros to disable peripheral clock of APB2 bus
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 19))
#define SPI6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 20))
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to enable peripheral clock of APB3 bus
 */
#define SPI5_PCLK_EN()			(RCC->APB3ENR |= 1 << 5)
#define I2C3_PCLK_EN()			(RCC->APB3ENR |= 1 << 7)
#define I2C4_PCLK_EN()			(RCC->APB3ENR |= 1 << 8)

/*
 * Macros to disable peripheral clock of APB3 bus
 */
#define SPI5_PCLK_DI()			(RCC->APB3ENR &= ~(1 << 5))
#define I2C3_PCLK_DI()			(RCC->APB3ENR &= ~(1 << 7))
#define I2C4_PCLK_DI()			(RCC->APB3ENR &= ~(1 << 8))


/*
 * Some generic Macros
 */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define GPIO_BASE_ADDRESS_TO_CODE(x)	(	(x == GPIOA) ? 0 : \
											(x == GPIOB) ? 1 : \
											(x == GPIOC) ? 2 : \
											(x == GPIOD) ? 3 : \
											(x == GPIOE) ? 4 : \
											(x == GPIOF) ? 5 : \
											(x == GPIOG) ? 6 : \
											(x == GPIOH) ? 7 : \
											(x == GPIOI) ? 8 : 0 \
										)

#endif /* INC_STM32NUCLEO_H563ZI_H_ */
