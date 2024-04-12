/*
 * MAX3166.h
 *
 *  Created on: Mar 8, 2024
 *      Author: User1
 */

#ifndef INC_MAX31855_H_
#define INC_MAX31855_H_

#define MAX31855_THERMOCOUPLE_MASK 0xFFFC0000

typedef struct __MAX31855_struct {
	SPI_HandleTypeDef *hspi;
	uint16_t csPin;
	GPIO_TypeDef *csPort;
} MAX31855_HandleTypeDef;

void MAX_Init (MAX31855_HandleTypeDef *hmax,SPI_HandleTypeDef *hspi, uint16_t csPin, GPIO_TypeDef *csPort);

int16_t MAX_GetCelcius (MAX31855_HandleTypeDef * maxPtr);


#endif /* INC_MAX31855_H_ */
