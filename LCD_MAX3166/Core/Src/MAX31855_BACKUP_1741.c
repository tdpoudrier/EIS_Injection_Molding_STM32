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

int MAX_GetCelcius (MAX31855_HandleTypeDef * maxPtr) {

<<<<<<< HEAD
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t spi_buf[4] = {0};

	HAL_GPIO_WritePin(maxPtr->csPort, maxPtr->csPin, GPIO_PIN_RESET);
	status = HAL_SPI_Receive(maxPtr->hspi, (uint8_t *) spi_buf, 4, 100);
	if (status != HAL_OK) {
		while (1);
	}
=======
	uint8_t spi_buf[4] = {0};

	HAL_GPIO_WritePin(maxPtr->csPort, maxPtr->csPin, GPIO_PIN_RESET);
	HAL_SPI_Receive(maxPtr->hspi, (uint8_t *) spi_buf, 4, 100);
>>>>>>> 31744601834f8d16a78f35f6e5d5f74846a72531
	HAL_GPIO_WritePin(maxPtr->csPort, maxPtr->csPin, GPIO_PIN_SET);

	//Assemble data into one int variable
	uint32_t data = 0x00000000;
	for(int i = 0, j = 3; i < 4; i++) {
		data = data | (spi_buf[j] << (i * 8));
		j--;
	}

	//Convert binary data to celcius
	float celcius = (data >> 18) * 0.25;

	return (int) (celcius * 100);
}

//Read 32 bits from MAX3166




