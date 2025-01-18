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

#define PIN_OUTPUT_MODE				OPEN_DRAIN_CONFIGURATION

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t gpio_led;
	gpio_led.pGPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
#if(PIN_OUTPUT_MODE == PUSH_PULL_CONFIGURATION)
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
#elif(PIN_OUTPUT_MODE == OPEN_DRAIN_CONFIGURATION)
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;	// activate pull-up resistor
#endif

	GPIO_PeripheralClockControl(GPIOD, ENABLE);
	GPIO_init(&gpio_led);

	while(1)
	{
		GPIO_toggleOutputPin(GPIOD, 12);
		delay();
	}

	return 0;
}
