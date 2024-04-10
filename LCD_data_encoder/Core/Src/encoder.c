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

	HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL);
}

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
	return (TIM3->CR1 & 0x0010) >> 4;
}

uint32_t ENC_GetCount (ENC_Handle* encoder) {
	return encoder->htim->CNT;
}

