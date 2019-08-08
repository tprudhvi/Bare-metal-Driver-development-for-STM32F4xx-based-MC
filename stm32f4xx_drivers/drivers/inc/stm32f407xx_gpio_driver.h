/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 31, 2019
 *      Author: prudh
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_pinNumber;	//values from @GPIO_PIN
	uint8_t GPIO_pinMode;	//values from @GPIO_MODE
	uint8_t GPIO_pinSpeed;	//values from @GPIO_SPEED
	uint8_t GPIO_pinPuPd;	//values from @GPIO_PUPD
	uint8_t GPIO_pinOPType;	//values from @GPIO_OUTPUT_MODE
	uint8_t GPIO_pinAltFunc;
}GPIO_Config_t;

//Handle structure of GPIO

typedef struct
{
	GPIO_RegDef_t *pGPIOx; 			// It holds the base address of GPIO  port
	GPIO_Config_t GPIO_PinConfig; 	//It holds the pin configuration settings

}GPIO_Handle_t;

//macros for pin numbers
//@GPIO_PIN
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15


//macros for GPIO modes
//@GPIO_MODE
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALT 		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_INT_FT 	4
#define GPIO_MODE_INT_RT 	5
#define GPIO_MODE_INT_FRT 	6

//macros for output mode
//@GPIO_OUTPUT_MODE

#define GPIO_OUT_PUP 	0
#define GPIO_OUT_OD 	1

//macros for speed of gpio
//@GPIO_SPEED

#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MED 		1
#define GPIO_SPEED_HIGH 	2
#define GPIO_SPEED_VHIGH 	3

//macros for pullup and pull down configurations
//@GPIO_PUPD

#define GPIO_NOPUPD			0
#define GPIO_PULLUP			1
#define GPIO_PULLDW			2

/********************API Prototypes *********************************
********************Function Definitions****************************/

//GPIO peripheral clock control
void GPIO_clkcontrol(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi);

//GPIO Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data read and write
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx );
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx , uint16_t value);
void GPIO_Togglepin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);

//GPIO interrupt config and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriority(uint8_t IRQNumber, uint32_t Priority);
void GPIO_IRQHandling( uint8_t PinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
