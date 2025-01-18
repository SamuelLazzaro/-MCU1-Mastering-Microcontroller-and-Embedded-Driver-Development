/*
 * main.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Samuel.Lazzaro
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

int main(void)
{

	return 0;
}

void EXTI0_IRQHandler(void)
{
	// Handle the interrupt
	uint8_t pinNumber = 0;
	GPIO_IRQHandling(pinNumber);
}


