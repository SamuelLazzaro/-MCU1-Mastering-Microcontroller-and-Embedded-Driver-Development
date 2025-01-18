/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 12, 2025
 *      Author: Samuel.Lazzaro
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t SPI_DeviceMode;		// possible values at @SPI_DEVICE_MODE
	uint8_t SPI_BusConfig;		// possible values at @SPI_BUS_CONFIG
	uint8_t SPI_SclkSpeed;		// possible values at @SPI_CLOCK_SPEED
	uint8_t SPI_DFF;			// possible values at @SPI_DFF
	uint8_t SPI_CPOL;			// possible values at @SPI_CPOL
	uint8_t SPI_CPHA;			// possible values at @SPI_CPHA
	uint8_t SPI_SSM;			// possible values at @SPI_SSM
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;	// Pointer to the base address of SPIx
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BUS_CONFIG
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX						1	// Full Duplex / Simplex TX only
#define SPI_BUS_CONFIG_HALF_DUPLEX						2	// Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY					3	// Simplex RX only


/*
 * @SPI_CLOCK_SPEED
 */
#define SPI_SCLK_SPEED_DIVIDED_BY_2				0
#define SPI_SCLK_SPEED_DIVIDED_BY_4				1
#define SPI_SCLK_SPEED_DIVIDED_BY_8				2
#define SPI_SCLK_SPEED_DIVIDED_BY_16			3
#define SPI_SCLK_SPEED_DIVIDED_BY_32			4
#define SPI_SCLK_SPEED_DIVIDED_BY_64			5
#define SPI_SCLK_SPEED_DIVIDED_BY_128			6
#define SPI_SCLK_SPEED_DIVIDED_BY_256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8_BITS		0		// default value
#define SPI_DFF_16_BITS		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN			0		// default value
#define SPI_SSM_DI			1


/*
 * SPI related status flags definition
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/*******************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *******************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t enable);

/*
 * Init and de-init
 */
void SPI_init(SPI_Handle_t *pSPIHandle);
void SPI_deinit(SPI_RegDef_t *pSPIx);			// To reset the SPI port settings to the reset state

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable);			// API used to manage interrupts
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);							// API used to process an interrupt when an interrupt occurs



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
