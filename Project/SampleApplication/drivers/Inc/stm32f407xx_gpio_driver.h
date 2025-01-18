/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 16, 2024
 *      Author: Samuel.Lazzaro
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;				// Pointer to hold the base address of the GPIO peripheral port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// It holds GPIO pin configuration settings

} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// GPIO mode input falling edge
#define GPIO_MODE_IT_RT		5		// GPIO mode input rising edge
#define GPIO_MODE_IT_RFT	6		// GPIO mode input rising / falling edge trigger


/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define	GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_MODES
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PU_PD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*********************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 ********************************************************************************/
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable);

void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_deinit(GPIO_RegDef_t *pGPIOx);			// To reset the GPIO port settings to the reset state

uint8_t GPIO_readFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_readFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_writeToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_toggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable);		// API used to manage interrupts
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);	// API used to process an interrupt when an interrupt occurs





#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
