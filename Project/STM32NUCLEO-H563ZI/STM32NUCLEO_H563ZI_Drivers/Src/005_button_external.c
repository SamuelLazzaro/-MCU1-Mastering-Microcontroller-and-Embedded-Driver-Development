/*
 * 005_button_external.c
 *
 *  Created on: Jan 3, 2025
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
	// External LED : PA12
	GPIO_Handle_t gpio_led1 = {0};

	// External button : PB9
	GPIO_Handle_t gpio_btn = {0};

	// External LED -> PA12
	gpio_led1.pGPIOx = GPIOA;
	gpio_led1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	gpio_led1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
#if(PIN_OUTPUT_TYPE == PUSH_PULL_CONFIGURATION)
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		// push-pull configuration
	gpio_led1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;	// No pull-up or pull-down
#elif (PIN_OUTPUT_TYPE == OPEN_DRAIN_CONFIGURATION)
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;		// open-drain configuration
	gpio_led1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;		// Pull-up resistor
#endif
	gpio_led1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// External button -> PB9
	gpio_btn.pGPIOx = GPIOB;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;		// GPIO_NO_PU_PD: an external pull-up resistor of 22kOhm must be used in order to avoid floating state. GPIO_PIN_PU: no external pull-up resistor needed.


	// Enable peripherals clock and init GPIO
	GPIO_peripheralClockControl(gpio_led1.pGPIOx, ENABLE);
	GPIO_peripheralClockControl(gpio_btn.pGPIOx, ENABLE);

	GPIO_init(&gpio_led1);
	GPIO_init(&gpio_btn);

	// IRQ Configurations
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI9, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI9, NVIC_IRQ_PRIORITY3);

	while(1);

	return 0;
}

void EXTI9_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_MODE_IT_FT, GPIO_PIN_NUM_9);
	GPIO_toggleOutputPin(GPIOA, GPIO_PIN_NUM_12);
}
