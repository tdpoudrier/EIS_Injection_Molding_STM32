/*
 * encoder.h
 *
 *  Created on: Apr 9, 2024
 *      Author: Tevin Poudrier
 *      Description: Driver for encoder sensor with STM32 timer set to encoder mode
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

#define ENC_THRESHOLD 3

/**
 * ENC_Handle struct
 * Stores the timer handles that is set to encoder mode and the GPIO for the switch
 * Uses prevCount and prevState for debouncing
 */
typedef struct __encoder_Handle {
	TIM_TypeDef* htim;
	uint16_t pin;
	GPIO_TypeDef* port;
	GPIO_PinState prevState;
	uint32_t prevCount;
} ENC_Handle;

void ENC_Init (ENC_Handle* encoder, TIM_HandleTypeDef* timer, GPIO_TypeDef* port, uint16_t pin);

uint8_t ENC_ReadSwitch (ENC_Handle* encoder);

uint8_t ENC_GetDirection (ENC_Handle* encoder);

uint32_t ENC_GetCount (ENC_Handle* encoder);

uint8_t ENC_CountChange (ENC_Handle* encoder);




#endif /* INC_ENCODER_H_ */
