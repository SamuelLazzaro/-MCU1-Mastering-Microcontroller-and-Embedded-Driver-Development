/*
 * stm32nucleo_h563zi_gpio_driver.h
 *
 *  Created on: Dec 14, 2024
 *      Author: Samuel.Lazzaro
 */

#ifndef INC_STM32NUCLEO_H563ZI_GPIO_DRIVER_H_
#define INC_STM32NUCLEO_H563ZI_GPIO_DRIVER_H_

#include "stm32nucleo_h563zi.h"

// Structure for GPIO Pin Configuration: all these settings can be configurable by the User Application
typedef struct {
	uint8_t GPIO_PinNumber;			// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// Possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;	// Possible values from @GPIO_PIN_PU_PD
	uint8_t GPIO_PinOPType;			// Possible values from @GPIO_PIN_OUTPUT
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;				// Pointer to hold the base address of the GPIO peripheral port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// It holds GPIO pin configuration settings
} GPIO_Handle_t;

/************************************************************************
 * 						APIs supported by this driver
 * For more information about the APIs check the function definitions
 ***********************************************************************/
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_deinit(GPIO_RegDef_t *pGPIOx);													// To reset the GPIO port settings to the reset state
void GPIO_peripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable);
uint8_t GPIO_readFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_readFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_writeToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_toggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable);							// API used to manage interrupts
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinMode, uint8_t pinNumber);						// API used to process an interrupt when an interrupt occurs



/*
 * GPIO pin possible modes
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// GPIO mode input falling edge
#define GPIO_MODE_IT_RT		5		// GPIO mode input rising edge
#define GPIO_MODE_IT_RFT	6		// GPIO mode input rising / falling edge

/*
 * GPIO pin possible output types
 * @GPIO_PIN_OUTPUT
 */
#define GPIO_OP_TYPE_PP		0		// GPIO mode output push-pull
#define GPIO_OP_TYPE_OD		1		// GPIO mode output open-drain

/*
 * GPIO pin possible output speeds
 * @GPIO_PIN_SPEEDS
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3

/*
 * GPIO pin pull-up and pull-down configuration macros
 * @GPIO_PIN_PU_PD
 */
#define GPIO_NO_PU_PD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * GPIO pin number
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NUM_0		0
#define GPIO_PIN_NUM_1		1
#define GPIO_PIN_NUM_2		2
#define GPIO_PIN_NUM_3		3
#define GPIO_PIN_NUM_4		4
#define GPIO_PIN_NUM_5		5
#define GPIO_PIN_NUM_6		6
#define GPIO_PIN_NUM_7		7
#define GPIO_PIN_NUM_8		8
#define GPIO_PIN_NUM_9		9
#define GPIO_PIN_NUM_10		10
#define GPIO_PIN_NUM_11		11
#define GPIO_PIN_NUM_12		12
#define GPIO_PIN_NUM_13		13
#define GPIO_PIN_NUM_14		14
#define GPIO_PIN_NUM_15		15

#endif /* INC_STM32NUCLEO_H563ZI_GPIO_DRIVER_H_ */
