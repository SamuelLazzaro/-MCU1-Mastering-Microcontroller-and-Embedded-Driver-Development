/*
 * stm32nucleo_h563zi_gpio_driver.c
 *
 *  Created on: Dec 14, 2024
 *      Author: Samuel.Lazzaro
 */

#include "../Inc/stm32nucleo_h563zi_gpio_driver.h"

/*****************************************************************************
* FuncName: GPIO_init
*
* @brief
*
* @details
*
* @param[in]
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {14/12/2024}
******************************************************************************/
void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// It is NOT an interrupt mode
		// Calculate value to use in order to set a GPIO pin mode for a specific pin of a GPIOx port
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		// Clearing bit values: 0x3 = 11 -> ~(0x3) = 00
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		// Interrupt mode
//		EXTI->SECCFGR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the FTSR1 (Falling Trigger Selection Register 1)
			EXTI->FTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding RTSR1 bit
			EXTI->RTSR1 &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the RTSR (Rising Trigger Selection Register)
			EXTI->RTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding RTSR1 bit
			EXTI->FTSR1 &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure the FTSR and RTSR (Falling / Rising Trigger Selection Registers)
			EXTI->FTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t EXTI_CR_number = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t EXTI_CR_bit_position = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASE_ADDRESS_TO_CODE(pGPIOHandle->pGPIOx);

		// Su questo microcontrollore non vi e' esplicitamente il registro SYSCFG e di conseguenza il suo peripheral clock potrebbe essere gia' abilitato di default
		EXTI->EXTICR[EXTI_CR_number] &= ~(0xFF << (EXTI_CR_bit_position * 8));
		EXTI->EXTICR[EXTI_CR_number] |= portCode << (EXTI_CR_bit_position * 8);		// * 8 in quanto ogni pin occupa 8 bit in EXTICRx

		// 3. Enable the EXTI interrupt delivery using IMR1
		EXTI->IMR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	}

	// 2. Configure the speed of GPIO pin
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));			// Clearing bit values: 0x3 = 11 -> ~(0x3) = 00
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. Configure the pull-up / pull-down of GPIO pin
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));			// Clearing bit values: 0x3 = 11 -> ~(0x3) = 00
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. Configure the output type of GPIO pin
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);				// Clearing bit values: 0x1 = 1 -> ~(0x1) = 0
		pGPIOHandle->pGPIOx->OTYPER |= temp;
	}

	// 5. Configure the Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Configure the Alternate Function registers
		// If GPIO mode is Alternate Function Mode, then we can configure the Alternate Function registers, otherwise the GPIO_PinAltFunMode has no meaning
		uint8_t lowOrHightAltFnReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t bitPositionAltFnReg = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);

		pGPIOHandle->pGPIOx->AFR[lowOrHightAltFnReg] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));			// Clearing bit values: 0xF = 1111 -> ~(0xF) = 0000
		pGPIOHandle->pGPIOx->AFR[lowOrHightAltFnReg] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << bitPositionAltFnReg;
	}

	return;
}


/*****************************************************************************
* FuncName: GPIO_deinit
*
* @brief
*
* @details
*
* @param[in]
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {14/12/2024}
******************************************************************************/
void GPIO_deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

	return;
}

/*****************************************************************************
* FuncName: GPIO_peripheralClockControl
*
* @brief This function enables or disables peripheral clock for the given GPIO port
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : base address of the GPIO peripheral
* @param[in] uint8_t enable : ENABLE or DISABLE macro
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {16/11/2024}
******************************************************************************/
void GPIO_peripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable)
{
	if(enable == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}

	return;
}

uint8_t GPIO_readFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value = 0;

	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);

	return value;
}

uint16_t GPIO_readFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = 0;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

void GPIO_writeToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}

	return;
}

void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;

	return;
}

void GPIO_toggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);

	return;
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable)
{
	if(enable == ENABLE)
	{
		// This microcontroller has 130 IRQ Numbers (see vector table)
		if(IRQNumber <= 31)
		{
			// Program NVIC_ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
			// Perche' viene modificato di conseguenza anche il valore nel registro NVIC_ICER0 il quale determina un disable dell'interrupt???
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			// Program NVIC_ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// Program NVIC_ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
		else if(IRQNumber > 95 && IRQNumber <= 127)
		{
			// Program NVIC_ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}
		else if(IRQNumber > 127 && IRQNumber <= 130)
		{
			// Program NVIC_ISER4 register
			*NVIC_ISER4 |= (1 << (IRQNumber % 128));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			// Program NVIC_ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			// Program NVIC_ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// Program NVIC_ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
		else if(IRQNumber > 95 && IRQNumber <= 127)
		{
			// Program NVIC_ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));
		}
		else if(IRQNumber > 127 && IRQNumber <= 130)
		{
			// Program NVIC_ICER4 register
			*NVIC_ICER4 |= (1 << (IRQNumber % 128));
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. Find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NUM_PRIORITY_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDRESS + IPRx) |= (IRQPriority << shift_amount);
//	*(NVIC_IPR_NS_BASE_ADDRESS + IPRx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t pinMode, uint8_t pinNumber)
{
	// Clear the EXTI PR register corresponding to the pin number
	if(pinMode == GPIO_MODE_IT_FT)
	{
		// Falling trigger
		if(EXTI->FPR1 & (1 << pinNumber))
		{
			// Clear that Pending register bit. This bit is cleared by writing 1 to it.
			EXTI->FPR1 |= (1 << pinNumber);
		}
	}
	else if(pinMode == GPIO_MODE_IT_RT)
	{
		// Rising trigger
		if(EXTI->RPR1 & (1 << pinNumber))
		{
			// Clear that Pending register bit. This bit is cleared by writing 1 to it.
			EXTI->RPR1 |= (1 << pinNumber);
		}
	}
	else if(pinMode == GPIO_MODE_IT_RFT)
	{
		// Falling / Rising trigger
		if(EXTI->FPR1 & (1 << pinNumber))
		{
			// Clear that Pending register bit. This bit is cleared by writing 1 to it.
			EXTI->FPR1 |= (1 << pinNumber);
		}
		if(EXTI->RPR1 & (1 << pinNumber))
		{
			// Clear that Pending register bit. This bit is cleared by writing 1 to it.
			EXTI->RPR1 |= (1 << pinNumber);
		}
	}
}
