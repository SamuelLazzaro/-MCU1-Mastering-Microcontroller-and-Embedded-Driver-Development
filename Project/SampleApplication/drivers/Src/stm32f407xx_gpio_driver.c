/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Samuel.Lazzaro
 */

#include "stm32f407xx_gpio_driver.h"

/*****************************************************************************
* FuncName: GPIO_PeripheralClockControl
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
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable)
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
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
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
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}
	}
	return;
}


/*****************************************************************************
* FuncName: GPIO_init
*
* @brief This function initialize GPIO peripheral
*
* @details
*
* @param[in] GPIO_Handle_t *pGPIOHandle : handle structure of GPIO peripheral
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {16/11/2024}
******************************************************************************/
void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing. 3 = binary 11
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		// We will code this part later (interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the FTSR (Falling Trigger Selection Register)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the RTSR (Rising Trigger Selection Register)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure both the FTSR and RTSR (Falling / Rising Trigger Selection Registers)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASE_ADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << ( temp2 * 4 );


		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	// 2. Configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing. 3 = binary 11
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. Configure the pull-up / pull-down settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing. 3 = binary 11
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. Configure the Output Type
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing. 1 = binary 1
		pGPIOHandle->pGPIOx->OTYPER |= temp;
	}

	// 5. Configure the Alternate Functionality
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN))
	{
		// Configure the Alternate Function registers
		uint32_t temp1, temp2;

		// 8 is the number of pin for each Alternate Functionality Mode Register (High and Low)
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

	return;
}


/*****************************************************************************
* FuncName: GPIO_deinit
*
* @brief This function de-initialize GPIO peripheral
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : pointer to base address of GPIOx
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {22/11/2024}
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
	else if(pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
	return;
}


/*****************************************************************************
* FuncName: GPIO_readFromInputPin
*
* @brief This function read from input pin
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : pointer to base address of GPIOx
* @param[in] uint8_t pinNumber : pin number
* @param[out]
*
*
* @return[OK] : uint8_t value
*
*
* @Author  {slazzaro},  @date  {22/11/2024}
******************************************************************************/
uint8_t GPIO_readFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);

	return value;
}


/*****************************************************************************
* FuncName: GPIO_readFromInputPort
*
* @brief This function read from input port
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : pointer to base address of GPIOx
* @param[out]
*
*
* @return[OK] : uint16_t value
*
*
* @Author  {slazzaro},  @date  {22/11/2024}
******************************************************************************/
uint16_t GPIO_readFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/*****************************************************************************
* FuncName: GPIO_writeToOutputPin
*
* @brief This function write to output pin
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : pointer to base address of GPIOx
* @param[in] uint8_t pinNumber : pin number
* @param[in] uint8_t value : GPIO_PIN_SET or GPIO_PIN_RESET
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {22/11/2024}
******************************************************************************/
void GPIO_writeToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << pinNumber );
	}
	else
	{
		// Write 0
		pGPIOx->ODR &= ~( 1 << pinNumber );
	}
}


/*****************************************************************************
* FuncName: GPIO_writeToOutputPort
*
* @brief This function write to output port
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : pointer to base address of GPIOx
* @param[in] uint8_t value : GPIO_PIN_SET or GPIO_PIN_RESET
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {22/11/2024}
******************************************************************************/
void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}


/*****************************************************************************
* FuncName: GPIO_toggleOutputPin
*
* @brief
*
* @details
*
* @param[in] GPIO_RegDef_t *pGPIOx : pointer to base address of GPIOx
* @param[in] uint8_t pinNumber : pin number
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {22/11/2024}
******************************************************************************/
void GPIO_toggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= ( 1 << pinNumber );
}


/*****************************************************************************
* FuncName: GPIO_IRQInterruptConfig
*
* @brief
*
* @details
*
* @param[in] uint8_t IRQNumber : IRQ number
* @param[out] uint8_t enable : enable / disable
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {24/11/2024}
******************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable)
{
	if(enable == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program NVIC_ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program NVIC_ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program NVIC_ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			// Program NVIC_ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program NVIC_ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program NVIC_ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64));
		}
	}
}


/*****************************************************************************
* FuncName: GPIO_IRQPriorityConfig
*
* @brief
*
* @details
*
* @param[in] uint8_t IRQNumber : IRQ number
* @param[in] uint8_t IRQPriority : IRQ priority
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {24/11/2024}
******************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. Find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * IPRx_section ) + ( 8 - NUM_PRIORITY_BITS_IMPLEMENTED );
	*(NVIC_PRIORITY_BASE_ADDRESS + IPRx) |= ( IRQPriority << shift_amount );
}


/*****************************************************************************
* FuncName: GPIO_IRQHandling
*
* @brief
*
* @details
*
* @param[in] uint8_t pinNumber : pin number
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {24/11/2024}
******************************************************************************/
void GPIO_IRQHandling(uint8_t pinNumber)
{
	// Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << pinNumber ))
	{
		// Clear that Pending Register bit
		EXTI->PR |= ( 1 << pinNumber );
	}
}

