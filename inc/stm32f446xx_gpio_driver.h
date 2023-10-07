/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Sep 19, 2023
 *      Author: jojoh
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h" //MCU Specific Data -- We use this to create a wrapper/API to program GPIO ports

/*******	 GPIO Pin Possible Modes	*******/
//@GPIO_PIN_MODES
#define GPIO_MODE_IN		0
#define GPIO_MODE_OU		1
#define GPIO_MODE_ALT		2 
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4 //Interrupt Falling Edge Trigger
#define GPIO_MODE_IT_RT		5 //Interrupt Rising Edge Trigger
#define GPIO_MODE_IT_RFT	6 //Interrupt Rising and Falling Edge Trigger

/*******	 GPIO Pin Possible Output Types	*******/
//@GPIO_PIN_OP_TYPES
#define GPIO_OP_TYPE_PP		0 //Push Pull
#define GPIO_OP_TYPE_OD		1 //Open Drain

/*******	 GPIO Pin Possible Output Speeds	*******/
//@GPIO_PIN_SPEEDS
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2

/*******	 GPIO Pin Possible Pull Up/Pull Down Configurations	*******/
//@GPIO_PIN_PUPD
#define GPIO_NO_PUPD		0	//No Pull Up or Pull Down
#define GPIO_PIN_PU			1	//Pull Up
#define GPIO_PIN_PD			2	//Pull Down

/*******	 GPIO Pin Numbers	*******/
//@GPIO_PIN_NUMBERS
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

/////////////////////// Handle Structures ///////////////////////

/*******	 This is a handle structure for a GPIO pin	*******/

typedef struct
{
	uint8_t GPIO_PinNumber; //Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode; //Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; //Possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl; //Possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType; //Possible values from @GPIO_PIN_OP_TYPES
	uint8_t GPIO_PinAltFunMode; //Possible values from @GPIO_PIN_ALT_FUN_MODES

} GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx; //Holds base address of the GPIO port we want to access -- A, B, C.. D (defined in MCU specific header file)
	GPIO_PinConfig_t GPIOpinConfig; //holds the pin configuration in settings

} GPIO_Handle_t;

/////////////////////// API Function definitions ///////////////////////

void GPIO_PeriClockControl(GPIO_RegDef_t * pGPIOX, uint8_t EnOrDi);
void GPIO_Init(GPIO_Handle_t * pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t * pGPIOX);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOX, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOX);
void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIOX, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIOX, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOX, uint8_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber); //should know from which pin it is triggered

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
