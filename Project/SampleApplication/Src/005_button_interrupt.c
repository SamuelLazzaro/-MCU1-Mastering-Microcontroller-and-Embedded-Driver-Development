/*
 * 005_button_interrupt.c
 *
 *  Created on: Jan 3, 2025
 *      Author: Samuel.Lazzaro
 */


/*
 * 002_led_button.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Samuel.Lazzaro
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define PUSH_PULL_CONFIGURATION		0
#define OPEN_DRAIN_CONFIGURATION	1

#define PIN_OUTPUT_MODE				PUSH_PULL_CONFIGURATION

#define BTN_PRESSED		1

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t gpio_led = {0};	// led
	GPIO_Handle_t gpio_btn = {0};	// button

	// Led GPIO configuration
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

	GPIO_PeripheralClockControl(gpio_led.pGPIOx, ENABLE);
	GPIO_init(&gpio_led);

	GPIO_writeToOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);

	// Button GPIO configuration
	gpio_btn.pGPIOx = GPIOD;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeripheralClockControl(gpio_btn.pGPIOx, ENABLE);
	GPIO_init(&gpio_btn);

	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIORITY15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();	// To avoid button debouncing
	GPIO_IRQHandling(GPIO_PIN_NO_5); // Clear the pending event from EXTI Line
	GPIO_toggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
