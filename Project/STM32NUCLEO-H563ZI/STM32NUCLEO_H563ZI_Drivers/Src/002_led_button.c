/*
 * 002_led_button.c
 *
 *  Created on: Jan 2, 2025
 *      Author: Samuel.Lazzaro
 */

#include <stdint.h>
#include "../Drivers/Inc/stm32nucleo_h563zi.h"
#include "../Drivers/Inc/stm32nucleo_h563zi_gpio_driver.h"

#define PUSH_PULL_CONFIGURATION		0
#define OPEN_DRAIN_CONFIGURATION	1

#define PIN_OUTPUT_TYPE				PUSH_PULL_CONFIGURATION

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	// User LED1 -> PB0 with SB43
	GPIO_Handle_t gpio_led1 = {0};
	// User LED2 -> PF4
	GPIO_Handle_t gpio_led2 = {0};
	// User LED3 -> PG4
	GPIO_Handle_t gpio_led3 = {0};

	// User button
	GPIO_Handle_t gpio_btn = {0};

	// User LED1
	gpio_led1.pGPIOx = GPIOB;
	gpio_led1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	gpio_led1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
#if(PIN_OUTPUT_TYPE == PUSH_PULL_CONFIGURATION)
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		// push-pull configuration
	gpio_led1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;	// No pull-up or pull-down
#elif (PIN_OUTPUT_TYPE == OPEN_DRAIN_CONFIGURATION)
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;		// open-drain configuration
	gpio_led1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;		// Pull-up resistor
#endif
	gpio_led1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// User LED2
	gpio_led2.pGPIOx = GPIOF;
	gpio_led2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_4;
	gpio_led2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
#if(PIN_OUTPUT_TYPE == PUSH_PULL_CONFIGURATION)
	gpio_led2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;	// No pull-up or pull-down
#elif (PIN_OUTPUT_TYPE == OPEN_DRAIN_CONFIGURATION)
	gpio_led2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;		// open-drain configuration
	gpio_led2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;		// Pull-up resistor
#endif
	gpio_led2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// User LED3
	gpio_led3.pGPIOx = GPIOG;
	gpio_led3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_4;
	gpio_led3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
#if(PIN_OUTPUT_TYPE == PUSH_PULL_CONFIGURATION)
	gpio_led3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;	// No pull-up or pull-down
#elif (PIN_OUTPUT_TYPE == OPEN_DRAIN_CONFIGURATION)
	gpio_led3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;		// open-drain configuration
	gpio_led3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;		// Pull-up resistor
#endif
	gpio_led3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// User button -> PC13
	gpio_btn.pGPIOx = GPIOC;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	// Enable peripherals clock and init GPIO
	GPIO_peripheralClockControl(gpio_led1.pGPIOx, ENABLE);
	GPIO_peripheralClockControl(gpio_led2.pGPIOx, ENABLE);
	GPIO_peripheralClockControl(gpio_led3.pGPIOx, ENABLE);
	GPIO_peripheralClockControl(gpio_btn.pGPIOx, ENABLE);
	GPIO_init(&gpio_led1);
	GPIO_init(&gpio_led2);
	GPIO_init(&gpio_led3);
	GPIO_init(&gpio_btn);

	GPIO_writeToOutputPin(gpio_led2.pGPIOx, gpio_led2.GPIO_PinConfig.GPIO_PinNumber, 1);

	while(1)
	{
		if(GPIO_readFromInputPin(gpio_btn.pGPIOx, gpio_btn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			delay();		// A delay is necessary in order to see the LED toggling (button/pin debouncing)
			GPIO_toggleOutputPin(gpio_led1.pGPIOx, gpio_led1.GPIO_PinConfig.GPIO_PinNumber);
			GPIO_toggleOutputPin(gpio_led2.pGPIOx, gpio_led2.GPIO_PinConfig.GPIO_PinNumber);
			GPIO_toggleOutputPin(gpio_led3.pGPIOx, gpio_led3.GPIO_PinConfig.GPIO_PinNumber);
		}
	}

	return 0;
}
