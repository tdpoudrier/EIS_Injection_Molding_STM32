/*
 * MAX31855.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Tevin Poudrier
 *      Description: Driver for MAX31855 thermocouple amplifier
 */

#ifndef INC_MAX31855_H_
#define INC_MAX31855_H_

#define MAX31855_THERMOCOUPLE_MASK 0xFFFC0000

/**
 * MAX31855_HandleTypeDef struct
 * Stores the SPI handle and the CS GPIO data to communicate over SPI
 */
typedef struct __MAX31855_struct {
	SPI_HandleTypeDef *hspi;
	uint16_t csPin;
	GPIO_TypeDef *csPort;
} MAX31855_HandleTypeDef;

void MAX_Init (MAX31855_HandleTypeDef *hmax,SPI_HandleTypeDef *hspi, uint16_t csPin, GPIO_TypeDef *csPort);

int16_t MAX_GetCelcius (MAX31855_HandleTypeDef * maxPtr);


#endif /* INC_MAX31855_H_ */
