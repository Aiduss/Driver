/*
 * stm32g431xx_gpio.c
 *
 *  Created on: Nov 3, 2024
 *      Author: eximusstudio
 */

#include "stm32g431xx_gpio.h"


/**
 * @fn			GPIO Peripheral Clock Control
 * @brief		This function enables or disables peripheral clok for the given GPIO port
 * @param[in]	base address of the GPIO peripheral
 * @param[in]	ENABLE or DISABLE macros
 * @return		void
 * @note		none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t State)
{
	if (State == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{	GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{	GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA)
		{	GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{	GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
	}
};



/**
 * @fn			GPIO INITIALIZATION
 * @brief		This function congigures pin mode, speed, pupd , optype and alternate functionality
 * @param[in]	base address of the GPIO peripheral configuration
 * @return		void
 * @note		todo : check with debug for proper values
 */
void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	// CONFIGURE : PIN mode
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)											// checks if pin mode is interrupt or non interrupt mode, if greater or equal to 3
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// pin number multiplied by 2 gets bytes shifted to left
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );				// ensure that bitfields are clear clear bits before set operation
		pGPIOHandle->pGPIOx->MODER |= temp;																		// set bits at only given address
	} else
	{
		// todo : interrupt mode
	}

	// CONFIGURE : PIN speed
	temp = 0;
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// CONFIGURE : PIN pull up and pull down
	temp = 0;
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// CONFIGURE : PIN output type
	temp = 0;
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// CONFIGURE : PIN alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTF)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;													// gives whole numbers as result
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;													// gives todo
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ));												// set AFR regoster
		pGPIOHandle->pGPIOx->AFR[temp1] |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));
	}
};


/**
 * @fn			GPIO DEINITIALIZATION
 * @brief		This function resets GPIO peripheral
 * @param[in]	base address of the GPIO port
 * @return		void
 * @note		todo : check with debug for proper values
 */
void GPIO_Reset(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{	GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{	GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	}
};


/**
 * @fn			GPIO READ FROM PIN
 * @brief		This function reads from given GPIOx pin
 * @param[in]	base address of the GPIO port
 * @param[in]	pin number
 * @return		1 or 0
 * @note		todo : check with debug for proper values
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = ( uint8_t )( (pGPIOx->IDR >> PinNumber) & 0x00000001 ); // IDR value shifted by amount PinNumber ot extract LSB and mask everything else
	return value;
};


/**
 * @fn			GPIO READ FROM PORT
 * @brief		This function reads from given GPIOx port
 * @param[in]	base address of the GPIO port
 * @return		16 bit value of IDR register status
 * @note		todo : check with debug for proper values
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = ( uint16_t ) pGPIOx->IDR;
	return value;
};


/**
 * @fn			WRITE TO PIN
 * @brief		This function writes to output pin
 * @param[in]	base address of the GPIO port
 * @param[in]	pin number
 * @param[in]	input value
 * @return		none
 * @note		todo : check with debug for proper values
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= ( 1 << PinNumber );	// write 1 to the output data register at the bit field corresponding to the pin number
	} else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber );	// write 0 to the output data register at the bit field corresponding to the pin number
	}
};


/**
 * @fn			WRITE TO PORT
 * @brief		This function writes to output port
 * @param[in]	base address of the GPIO port
 * @param[in]	input value
 * @return		none
 * @note		todo : check with debug for proper values
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
};


/**
 * @fn			TOGGLE OUTPUT PIN
 * @brief		This function enables and disables pin
 * @param[in]	base address of the GPIO port
 * @param[in]	pin number
 * @return		none
 * @note		todo : check with debug for proper values
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber );	// Bitwise XOR implementation to toogle pin
};


/**
 * @fn			IRQ CONFIG
 * @brief		This function writes to output pin
 * @param[in]	base address of the GPIO port
 * @param[in]	pin number
 * @param[in]	input value
 * @return		none
 * @note		todo : check with debug for proper values
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t State)
{

};

void GPIO_IRQHandling(uint8_t PinNumber)
{

};
