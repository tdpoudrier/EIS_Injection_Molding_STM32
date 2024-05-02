/*
 * MAX3166.c
 *
 *  Created on: Mar 8, 2024
 *      Author: User1
 */
#include "main.h"
#include "MAX31855.h"

void MAX_Init (MAX31855_HandleTypeDef *hmax,SPI_HandleTypeDef *hspi, uint16_t csPin, GPIO_TypeDef *csPort) {

	hmax->csPin = csPin;
	hmax->csPort = csPort;
	hmax->hspi = hspi;

	//CS pin normally HIGH
	HAL_GPIO_WritePin(hmax->csPort, hmax->csPin, GPIO_PIN_SET);

}

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

//Read 32 bits from MAX3166




