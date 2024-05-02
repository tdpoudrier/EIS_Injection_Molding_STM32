/*
 * MAX31855.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Tevin Poudrier
 *      Description: Driver for MAX31855 thermocouple amplifier
 */
#include "main.h"
#include "MAX31855.h"

/**
 *  Initialize MAX31855 by storing SPI communication and writing CS high
 *  @param hmax Pointer to max handle
 *  @param hspi Pointer to spi handle
 *  @param csPin The GPIO pin defined as GPIO_PIN_x
 *  @param csPort The GPIO port defined as GPIOx
 */
void MAX_Init (MAX31855_HandleTypeDef *hmax,SPI_HandleTypeDef *hspi, uint16_t csPin, GPIO_TypeDef *csPort) {

	hmax->csPin = csPin;
	hmax->csPort = csPort;
	hmax->hspi = hspi;

	//CS pin normally HIGH
	HAL_GPIO_WritePin(hmax->csPort, hmax->csPin, GPIO_PIN_SET);

}

/**
 * Get celcius temperature data from the MAX31855
 * @param maxPtr Pointer to MAX31855 handle
 */
int16_t MAX_GetCelcius (MAX31855_HandleTypeDef * maxPtr) {

	uint8_t spi_buf[4] = {0};

	HAL_StatusTypeDef status = HAL_OK;

	HAL_GPIO_WritePin(maxPtr->csPort, maxPtr->csPin, GPIO_PIN_RESET);
	status = HAL_SPI_Receive(maxPtr->hspi, (uint8_t *) spi_buf, 4, 100);
	HAL_GPIO_WritePin(maxPtr->csPort, maxPtr->csPin, GPIO_PIN_SET);

	if (status != HAL_OK) {
		while(1);
	}

	//Assemble data into one int variable
	uint32_t data = 0x00000000;
	for(int i = 0, j = 3; i < 4; i++) {
		data = data | (spi_buf[j] << (i * 8));
		j--;
	}

	//Convert binary data to celcius
	uint32_t tempBitData = (data >> 18);
	float celcius = tempBitData * 0.25;

	return (int16_t) (celcius);
}




