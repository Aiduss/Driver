/*
 * stm32g431xx_gpio.h
 *
 *  Created on: Nov 3, 2024
 *      Author: eximusstudio
 */

#ifndef INC_STM32G431XX_GPIO_H_
#define INC_STM32G431XX_GPIO_H_


#include <stdint.h>
#include "stm32g431xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber;			/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinA;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct
{
	// pointer to hold address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;				/* THis holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/* GPIO Pin numbers */

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


/* ---[ INTERNAL MACROS ]--- */
/* ----------------------------------------------------------------------------- */
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALTF		2
#define GPIO_MODE_ANALOG	3

#define GPIO_MODE_IT_FT		4	// todo interrupt modes input falling edge
#define GPIO_MODE_IT_RT		5	// todo input rising edge
#define GPIO_MODE_IT_RFT	6	// todo rising edge / falling edge

#define GPIO_OP_TYPE_PP		0	// push pull
#define GPIO_OP_TYPE_OD		1	// open drain

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2




/* *********************************
 *   APIs supported by this driver
 * *********************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t State);

void GPIO_Init (GPIO_Handle_t *pGPIOHandle);
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t State);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32G431XX_GPIO_H_ */
