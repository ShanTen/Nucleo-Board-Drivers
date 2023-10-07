/*
 * stm32f446xx_I2C_driver.h
 *
 *  Created on: 28-Sep-2023
 *      Author: jojoh
 */


#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/////////////////////// Handle Structures ///////////////////////

// I2C Configuration Structure
typedef struct
{
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress; //This is filled by the user so no need to initialize options
    uint8_t I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

// I2C Handle Structure
typedef struct
{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
}I2C_Handle_t;

/////////////////////// Options  ///////////////////////

// @I2C_SCLSpeed
#define I2C_SCL_SPEED_SM	100000 //100kHz
#define I2C_SCL_SPEED_FM2K	200000 //200kHz
#define I2C_SCL_SPEED_FM4K	400000 //400kHz

// @I2C_ACKControl
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

// @I2C_FMDutyCycle
//DutyCycle is only applicable in Fast Mode (FM) I2C
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/////////////////////// APIs Supported by this driver ///////////////////////

// Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Init and De-Init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

// IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// Other Peripheral Control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);

// Application Callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
