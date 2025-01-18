/*
 * 002_led_button.c
 *
 *  Created on: Nov 23, 2024
 *      Author: Samuel.Lazzaro
 */


/*
 * 001_led_toggle.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Samuel.Lazzaro
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define PUSH_PULL_CONFIGURATION		0
#define OPEN_DRAIN_CONFIGURATION	1

#define PIN_OUTPUT_MODE				PUSH_PULL_CONFIGURATION

#define LOW				0
#define HIGH			1
#define BTN_PRESSED		LOW

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t gpio_led;	// led
	GPIO_Handle_t gpio_btn;	// button

	// Led GPIO configuration
	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
#if(PIN_OUTPUT_MODE == PUSH_PULL_CONFIGURATION)
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
#elif(PIN_OUTPUT_MODE == OPEN_DRAIN_CONFIGURATION)
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;	// activate pull-up resistor
#endif

	GPIO_PeripheralClockControl(gpio_led.pGPIOx, ENABLE);
	GPIO_init(&gpio_led);

	// Button GPIO configuration
	gpio_btn.pGPIOx = GPIOB;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeripheralClockControl(gpio_btn.pGPIOx, ENABLE);
	GPIO_init(&gpio_btn);

	while(1)
	{
		if(GPIO_readFromInputPin(gpio_btn.pGPIOx, gpio_btn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			// delay();	// A small delay is necessary in order to see the LED toggling
			GPIO_toggleOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNumber);
		}
	}

	return 0;
}
