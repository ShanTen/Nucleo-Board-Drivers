/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Sep 19, 2023
 *      Author: jojoh
 */

#include "stm32f446xx_gpio_driver.h"

/**********************************************************************************
 * Driver API provides the following functionality
 * ------------------------------------------------
 * 1. GPIOx Initialization (Initialize GPIO ports of MCU)
 * 2. Enable/Disable GPIOx Clock
 * 3. Read From GPIO pin
 * 4. Write to GPIO Pin
 * 5. Configure Alternate Functionality
 * 6. Interrupt Handling
 **********************************************************************************/

//Given a GPIO base address enable or disable it's clock
void GPIO_PeriClockControl(GPIO_RegDef_t * pGPIOX, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOX == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOX == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOX == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOX == GPIOD)
			GPIOD_PCLK_EN();
		else if (pGPIOX == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOX == GPIOF)
			GPIOF_PCLK_EN();
		else if (pGPIOX == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOX == GPIOH)
			GPIOH_PCLK_EN();
	}
	else
	{
		if(pGPIOX == GPIOA)
			GPIOA_PCLK_DI();
		else if (pGPIOX == GPIOB)
			GPIOB_PCLK_DI();
		else if (pGPIOX == GPIOC)
			GPIOC_PCLK_DI();
		else if (pGPIOX == GPIOD)
			GPIOD_PCLK_DI();
		else if (pGPIOX == GPIOE)
			GPIOE_PCLK_DI();
		else if (pGPIOX == GPIOF)
			GPIOF_PCLK_DI();
		else if (pGPIOX == GPIOG)
			GPIOG_PCLK_DI();
		else if (pGPIOX == GPIOH)
			GPIOH_PCLK_DI();
	}
};

//Given a GPIO base address initialize or de-initialize its registers
//takes pointer to GPIO handle
void GPIO_Init(GPIO_Handle_t * pGPIOHandle)
{
	//1. Configure the mode of the GPIO driver 
    uint32_t temp = 0; //temp register

    if(pGPIOHandle->GPIOpinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        //non interrupt mode
        temp = (pGPIOHandle->GPIOpinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIOpinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIOpinConfig.GPIO_PinNumber); //clearing
        pGPIOHandle->pGPIOx->MODER |= temp; //setting
    }
    else
    {
        //later 
    }

    temp = 0;

	//2. Configure the speed 
    temp = (pGPIOHandle->GPIOpinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIOpinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIOpinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the pull-up/pull-down settings
	temp = (pGPIOHandle->GPIOpinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIOpinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIOpinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the output type
	temp = (pGPIOHandle->GPIOpinConfig.GPIO_PinOPType << (pGPIOHandle->GPIOpinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIOpinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the alternate functionality (if any)
	if(pGPIOHandle->GPIOpinConfig.GPIO_PinMode == GPIO_MODE_ALT)
	{
		//configure the alternate function registers
		uint8_t temp1, temp2;
		//For pins 0-7, AFR[0] is used, for pins 8-15, AFR[1] is used
		temp1 = pGPIOHandle->GPIOpinConfig.GPIO_PinNumber / 8; 
		temp2 = pGPIOHandle->GPIOpinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIOpinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting
	}

}

//deinit => set rest bit to 1 so we use base peripheral reset register
void GPIO_DeInit(GPIO_RegDef_t * pGPIOX)
{
	if(pGPIOX == GPIOA)
		GPIOA_REG_RESET();
	else if (pGPIOX == GPIOB)
		GPIOB_REG_RESET();
	else if (pGPIOX == GPIOC)
		GPIOC_REG_RESET();
	else if (pGPIOX == GPIOD)
		GPIOD_REG_RESET();
	else if (pGPIOX == GPIOE)
		GPIOE_REG_RESET();
	else if (pGPIOX == GPIOF)
		GPIOF_REG_RESET();
	else if (pGPIOX == GPIOG)
		GPIOG_REG_RESET();
	else if (pGPIOX == GPIOH)
		GPIOH_REG_RESET();
}

//Read from GPIO
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOX, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOX->IDR >> PinNumber) & 0x00000001); //shift to the right by pin number and mask with 1
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOX)
{
	uint16_t value;
	value = (uint16_t)(pGPIOX->IDR); //read from input data register
	return value;
}

//Write to GPIO
void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIOX, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOX->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOX->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIOX, uint16_t Value)
{
	pGPIOX->ODR = Value;
}

//Toggle Pin
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOX, uint8_t PinNumber)
{
	pGPIOX->ODR ^= (1 << PinNumber); //XOR with 1 to toggle
}

//IRQ config and interrupt handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber); //should know from which pin it is triggered

/***********************************************************************************/
