/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 12, 2025
 *      Author: Samuel.Lazzaro
 */

#include "stm32f407xx_spi_driver.h"

/*****************************************************************************
* FuncName: SPI_PeripheralClockControl
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
* @Author  {slazzaro},  @date  {12/01/2025}
******************************************************************************/
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t enable)
{
	if(enable == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
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
void SPI_init(SPI_Handle_t *pSPIHandle)
{
	// Configure the SPI_CR1 register
	uint32_t temp_reg = 0;

	// 1. Configure the Device Mode (MSTR)
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the Bus Config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		// BIDIMODe should be cleared
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		// BIDIMODE should be set
		temp_reg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		// BIDIMODE should be cleared
		// RXONLY bit must be set
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
		temp_reg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI Serial Clock Speed (baud rate)
	temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF (Data Frame Format)
	temp_reg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// Initialize SPI_CR1 register: I can use = operator because I'm initializing this register
	pSPIHandle->pSPIx->CR1 = temp_reg;
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
void SPI_deinit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}




/*****************************************************************************
* FuncName: SPI_GetFlagStatus
*
* @brief
*
* @details
*
* @param[in] SPI_RegDef_t *pSPIx
* @param[in] uint32_t flagName
* @param[out]
*
*
* @return[OK] : none
*
*
* @Author  {slazzaro},  @date  {18/01/2025}
******************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


/*****************************************************************************
* FuncName: SPI_SendData
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
* @Author  {slazzaro},  @date  {24/11/2024}
******************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// 1. Load the data into the DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			// Increment pointer in order to make it point to the next data item
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			// 1. Load the data into the DR register
			pSPIx->DR = *pTxBuffer;
			length--;
			// Increment pointer in order to make it point to the next data item
			pTxBuffer++;
		}
	}
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{

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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable)
{

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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}
