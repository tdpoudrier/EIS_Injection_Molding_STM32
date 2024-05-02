/*
 * encoder.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Tevin Poudrier
 *      Description: Driver for encoder sensor with STM32 timer set to encoder mode
 */

#include "encoder.h"
#include "main.h"

/**
 * Initalize encoder timer and GPIO switch, expects switch to have a pull down resistor
 * @param encoder Pointer to encoder handle
 * @param timer Pointer to timer handle set to encoder mode in ioc file / CubeMX
 * @param port Pointer to GPIO port defined as GPIOx
 * @param pin GPIO pin defined as GPIO_PIN_x
 */
void ENC_Init (ENC_Handle* encoder, TIM_HandleTypeDef* timer, GPIO_TypeDef* port, uint16_t pin) {
	encoder->htim = timer->Instance;
	encoder->port = port;
	encoder->pin = pin;

	encoder->prevState = 0;
	encoder->prevCount = 0;

	HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL);
}

/**
 * Read the value of the encoder switch. Returns true when pressed and prevents debounce
 * @param encoder Pointer to encoder handle
 * @returns GPIO_PIN_SET (1) when encoder is pressed, GPIO_PIN_RESET (0) otherwise
 */
uint8_t ENC_ReadSwitch (ENC_Handle* encoder) {
	GPIO_PinState state = HAL_GPIO_ReadPin (encoder->port, encoder->pin);

	if (state != encoder->prevState) {
		encoder->prevState = state;
		return state;
	}
	else {
		return 0;
	}
}

/**
 * Get the current direction of encoder
 * @param encoder Pointer to encoder handle
 * @return direction of the encoder, either 0 or 1
 */
uint8_t ENC_GetDirection (ENC_Handle* encoder) {
	return (encoder->htim->CR1 & 0x0010) >> 4;
}

/**
 * Get the current count of the encoder
 * @param encoder Pointer to encoder handle
 * @return count of encoder
 */
uint32_t ENC_GetCount (ENC_Handle* encoder) {
	return encoder->htim->CNT;
}

/**
 * Check if encoder counter has changed since last check
 * @param encoder Pointer to encoder handle
 * @return 1 if counter has changed, 0 otherwise
 */
uint8_t ENC_CountChange (ENC_Handle* encoder) {
	uint32_t count = ENC_GetCount(encoder);

	if (count - encoder->prevCount > ENC_THRESHOLD) {
		encoder->prevCount = count;
		return 1;
	}
	else {
		return 0;
	}
}

