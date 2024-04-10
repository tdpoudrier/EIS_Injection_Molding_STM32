/*
 * encoder.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Tevin Poudrier
 */

#include "encoder.h"
#include "main.h"

void ENC_Init (ENC_Handle* encoder, TIM_HandleTypeDef* timer, GPIO_TypeDef* port, uint16_t pin) {
	encoder->htim = timer->Instance;
	encoder->port = port;
	encoder->pin = pin;

	encoder->prevState = 0;
	encoder->prevCount = 0;

	HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL);
}

/**
 * Only check once per super loop execution
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

uint8_t ENC_GetDirection (ENC_Handle* encoder) {
	return (encoder->htim->CR1 & 0x0010) >> 4;
}

uint32_t ENC_GetCount (ENC_Handle* encoder) {
	uint32_t count = encoder->htim->CNT;

	 if (count - encoder->prevCount > 3 || encoder->prevCount - count > 3) {
		 encoder->prevCount = count;
		 return count;
	 }
	 else {
		 return encoder->prevCount;
	 }
}

