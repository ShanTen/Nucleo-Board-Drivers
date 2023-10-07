/*
 * stm32f446xx.h
 *
 *  Created on: Sep 19, 2023
 *      Author: jojoh
 *
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#include <stdint.h>

//Short Hand Declarations
#define __volInt32 volatile uint32_t

/*
 Base addresses of Flash and SRAM memories
*/
#define FLASH_BASEADDR                             0x08000000U //Base address of FLASH memory
#define SRAM1_BASEADDR                             0x20000000U //Base address of SRAM1 (112KB)
#define SRAM2_BASEADDR                             0x2001C000U //Base address of SRAM2 (Base SRAM1 + 112KB)
#define ROM										   0x1FFF0000U //Base address of System Memory (i.e ROM)
#define OTP 									   0x1FFF7800U //Base address for One Time Programmable Read Only Memory (OTPROM)
#define SRAM SRAM1_BASEADDR									   //Main memory

/*
 Base Addresses of all Busses
*/
#define PERIPH_BASEADDR 							0x40000000U
#define APB1PERIPH_BASEADDR							PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR							0x40010000U
#define AHB2PERIPH_BASEADDR							0x50000000U
#define AHB1PERIPH_BASEADDR							0x40020000U

//#define AHB1PERIPH_BASEADDR							0x40020000U

/*
 Base Addresses of all peripherals on AHB1
*/

#define GPIOA_BASEADDR								(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR								(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR								(AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR								(AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR								(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR								(AHB1PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR								(AHB1PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR								(AHB1PERIPH_BASEADDR+0x1C00)
#define RCC_BASEADDR								(AHB1PERIPH_BASEADDR+0x3800)

/*
 Base Addresses of all peripherals on APB1
*/

#define I2C1_BASEADDR								(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR								(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR								(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR								(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR								(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR								(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR								(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR								(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR								(APB1PERIPH_BASEADDR + 0x5000)

/*
 Base Addresses of all peripherals on APB2
*/

#define SPI1										(APB2PERIPH_BASEADDR + 0x3000)
#define USART1										(APB2PERIPH_BASEADDR + 0x1000)
#define USART6										(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI										(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG										(APB2PERIPH_BASEADDR + 0x3800)

/*Peripheral Register Definition Structures (LET ME COOK) */

/* Some C Syntax Decoding For readable code:
 * Usually we do
 * struct someStruct{
 * 		...
 * }
 *
 * typedef someStruct someStruct_t;
 *
 *
 * we can skip the redundant variable someStruct by typedef-ing it at initialization as follows
 *
 *
 * typedef struct {
 * 		bing;
 * 		bang;
 * 		pow;
 * } someStruct_t;
 * */

//Generic Structure for GPIO Pins
//each is offset by 0x04 i.e 4 bytes

typedef struct {
	__volInt32 MODER;
	__volInt32 OTYPER;
	__volInt32 OSPEEDR;
	__volInt32 PUPDR;
	__volInt32 IDR;
	__volInt32 ODR;
	__volInt32 BSRR;
	__volInt32 LCKR;
	__volInt32 AFR[2]; //Alt Function High, Low registers. array of 8 bytes (2 elements each 32 bit unsigned integers) (instead of creating AFRegister low, AFR High we have this array)
} GPIO_RegDef_t;

//Specific For All GPIO Pins
//all the ones below are casted to pointers
#define GPIOA           ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB           ((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC           ((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD           ((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE           ((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF           ((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG           ((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH           ((GPIO_RegDef_t *) GPIOH_BASEADDR)

//example
// #GPIO_RegDef_t * pGPIOA = GPIOA;
/*Peripheral Register Definition Structure For RCC Clock*/

typedef struct {
	__volInt32 CR;
	__volInt32 PLLCFGR;
	__volInt32 CFGR;
	__volInt32 CIR;
	__volInt32 AHB1RSTR;
	__volInt32 AHB2RSTR;
	__volInt32 AHB3RSTR;
	uint32_t RESERVED0;
	__volInt32 APB1RSTR;
	__volInt32 APB2RSTR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	__volInt32 AHB1LPENR;
	__volInt32 AHB2LPENR;
	__volInt32 AHB3LPENR;
	uint32_t RESERVED3;
	__volInt32 APB1LPENR;
	__volInt32 APB2LPENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	__volInt32 BDCR;
	__volInt32 CSR;
	uint32_t RESERVED6;
	uint32_t RESERVED7;
	__volInt32 SSCGR;
	__volInt32 PLLI2SCFGR;
	__volInt32 PLLSAICFGR;
	__volInt32 DCKCFGR;
	__volInt32 CKGATENR;
	__volInt32 DCKCFGR2;
} RCC_RegDef_t;


#define RCC				((RCC_RegDef_t *)RCC_BASEADDR) //RCC_BASEADDR is an address, RCC is a pointer to the struct starting at RCC_BASEADDR

//GPIOx Peripheral Clock Enable Macros
//(newState << bitPosition)
#define GPIOA_PCLK_EN()		(RCC->AHB1LPENR |= (1 << 0)) //Enable GPIOA's peripheral clock
#define GPIOB_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 2))
#define GPIOD_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 4))
#define GPIOF_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 5))
#define GPIOG_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 6))
#define GPIOH_PCLK_EN() 	(RCC->AHB1LPENR |= (1 << 7))

//I2Cx Peripheral Clock Enable Disable Macros
#define I2C1_PCLK_EN()		(RCC->APB1LPENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1LPENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1LPENR |= (1 << 23))

//SPIx Peripheral Clock Enable Macros
#define SPI2_PCLK_EN()		(RCC->APB1LPENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1LPENR |= (1 << 15))

//USARTx Peripheral Clock Enable Macros
#define USART2_PCLK_EN()		(RCC->APB1LPENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1LPENR |= (1 << 18))

//SYSCFG Peripheral Clock Enable Macros
#define SYSCFG_PCLK_EN()		(RCC->APB2LPENR |= (1 << 14))

/****** Peripheral Clock Disable Macros *******/

//GPIOx Peripheral Clock Disable Macros
#define GPIOA_PCLK_DI()		(RCC->AHB1LPENR &= ~ (1 << 0)) //Disable GPIOA's peripheral clock
#define GPIOB_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 4))
#define GPIOF_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1LPENR &= ~ (1 << 7))

//I2Cx Peripheral Clock Disable Macros
#define I2C1_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 23))

//SPIx Peripheral Clock Disable Macros
#define SPI2_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 15))

//USARTx Peripheral Clock Disable Macros
#define USART2_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1LPENR &= ~ (1 << 18))

//SYSCFG Peripheral Clock Disable Macros
#define SYSCFG_PCLK_DI()		(RCC->APB2LPENR &= ~ (1 << 14))

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0) //do while(0) is a trick to make it a single line macro
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)

//Some General Purpose Macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

/*
//////////// I2C related details /////////////
- I2C peripheral register definition structure 
- I2Cx base address macros 
- I2Cx peripheral defintion macros 
- enable and disable clock macros of each I2Cx clock
- Bit Position of I2C peripheral status register 
*/

//Generic Structure for I2C Pins
//each is offset by 0x04 i.e 4 bytes
typedef struct {
	__volInt32 CR1;
	__volInt32 CR2;
	__volInt32 OAR1;
	__volInt32 OAR2;
	__volInt32 DR;
	__volInt32 SR1;
	__volInt32 SR2;
	__volInt32 CCR;
	__volInt32 TRISE;
	__volInt32 FLTR;	
} I2C_RegDef_t;

//I2C Peripheral defnition macros 
#define I2C1				((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t *)I2C3_BASEADDR)

//////////////////////////////////////////////////////////////////////////////////
//////////////////// Bit Position definitions of I2C register ////////////////////
//////////////////////////////////////////////////////////////////////////////////

/* To Define:
- Cr1 -- done
- cr2 -- done
- oar1 -- done
- sr1 -- done
- sr2 
- ccr 
*/

// For I2C_CR1 register
#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

// For I2C_CR2 register
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

// For I2C_OAR1 register
#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

// For I2C_SR1 register
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

// For I2C_SR2 register
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

// For I2C_CCR register
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15




#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_I2C_driver.h"

#endif /* INC_STM32F446XX_H_ */
